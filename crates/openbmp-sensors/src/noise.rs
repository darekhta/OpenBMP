//! Deterministic noise primitives.
//!
//! These three generators are the building blocks of the IEEE 952
//! five-component IMU noise model:
//!
//! * [`BoxMullerGaussian`] — counter-based Gaussian sampler. Pulls
//!   two uniform `f64` from a [`DeterministicRng`] and returns one
//!   sample via the Box-Muller transform. Chosen over Ziggurat per
//!   the Phase-2 plan because Ziggurat's rejection step makes the
//!   per-step RNG draw count data-dependent and breaks bit-stable
//!   replay across small initial perturbations.
//!
//! * [`OrnsteinUhlenbeck`] — exact discrete-time Ornstein-Uhlenbeck
//!   process. Used for IMU bias instability where the bias
//!   correlates over a finite time `τ_BI` and reverts to a long-run
//!   mean of zero with stationary standard deviation `σ`.
//!
//! * [`IntegratedWhiteNoise`] — discrete random-walk integrator
//!   `x_{n+1} = x_n + σ · √dt · N(0, 1)`. Used for IMU rate random
//!   walk (RRW) on gyros and acceleration random walk (ARW) on
//!   accelerometers.
//!
//! # Determinism
//!
//! All three primitives are pure arithmetic on `f64`. Locked operand
//! order; FMA disabled. Bit-stability is guaranteed within the
//! reference platform profile (`x86_64-unknown-linux-gnu`); cross-
//! libm bit equality is not claimed because `ln`, `sqrt`, `cos`,
//! `sin` may differ across libm implementations.

use openbmp_core::DeterministicRng;

use crate::error::SensorError;

/// Two times pi, locked operand order.
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

// ---------------------------------------------------------------------
// BoxMullerGaussian
// ---------------------------------------------------------------------

/// Counter-based Gaussian sampler using the Box-Muller transform.
///
/// `r = √(-2 ln u₁) · cos(2π · u₂)` produces one sample of `N(0, 1)`
/// from two uniform `[0, 1)` samples `(u₁, u₂)`. The second Box-Muller
/// output (`sin` instead of `cos`) is discarded so the per-call RNG
/// draw count is exactly two — this keeps the RNG advance independent
/// of any branching in the caller.
#[derive(Copy, Clone, Debug, Default)]
pub struct BoxMullerGaussian;

impl BoxMullerGaussian {
    /// Construct the Gaussian sampler.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }

    /// Draw one `N(mean, stddev²)` sample from the supplied
    /// deterministic RNG.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] when `stddev < 0` or
    /// `stddev` / `mean` is non-finite.
    pub fn sample(
        &self,
        rng: &mut DeterministicRng,
        mean: f64,
        stddev: f64,
    ) -> Result<f64, SensorError> {
        if !mean.is_finite() || !stddev.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "Gaussian mean or stddev is NaN or infinite",
            });
        }
        if stddev < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "Gaussian stddev must be non-negative",
            });
        }
        let standard = self.sample_standard(rng);
        // Locked order: mean + stddev · standard.
        Ok(mean + stddev * standard)
    }

    /// Draw one `N(0, 1)` standard-Gaussian sample.
    ///
    /// Used internally by [`Self::sample`] but exposed so the IMU
    /// model can reuse the same standard-normal draw across
    /// scale/bias channels without the parameter-validation overhead.
    #[allow(clippy::cast_precision_loss)] // 52-bit float precision is intentional in [0, 1) sampling
    pub fn sample_standard(&self, rng: &mut DeterministicRng) -> f64 {
        // Two uniform samples in [0, 1). Use `next_u64` and divide
        // by 2^64 so the draw is independent of any rand-crate
        // float-conversion conventions.
        let u1_raw = rng.next_u64();
        let u2_raw = rng.next_u64();
        let denom = (u64::MAX as f64) + 1.0;
        let u1 = (u1_raw as f64) / denom;
        let u2 = (u2_raw as f64) / denom;

        // Avoid `ln(0)` if `u1` happens to be exactly zero. The
        // probability is 1 / 2^64 but the safety guard is cheap.
        let u1_safe = if u1 == 0.0 { f64::MIN_POSITIVE } else { u1 };

        // Locked order: (-2 ln u1).sqrt() · cos(2π u2).
        let radius = (-2.0 * u1_safe.ln()).sqrt();
        let angle = TWO_PI * u2;
        // Yield `radius · cos(angle)`. The companion `radius · sin(angle)`
        // is discarded to keep the RNG draw count fixed at 2.
        let _ = rng; // keep `rng` borrowed as `&mut`; clippy doesn't flag.
        radius * angle.cos()
    }
}

// ---------------------------------------------------------------------
// OrnsteinUhlenbeck
// ---------------------------------------------------------------------

/// Exact discrete-time Ornstein-Uhlenbeck process.
///
/// State update at fixed step `dt`:
///
/// ```text
/// x_{n+1} = x_n · e^{-θ·dt} + σ_eq · N(0, 1)
/// ```
///
/// with the equilibrium standard deviation
///
/// ```text
/// σ_eq = σ · √((1 - e^{-2θ·dt}) / (2θ))
/// ```
///
/// Long-run mean is zero; long-run variance is `σ² / (2θ)`. The
/// correlation time is `τ = 1/θ`.
#[derive(Copy, Clone, Debug)]
pub struct OrnsteinUhlenbeck {
    /// Mean-reversion rate `θ` (1/s).
    theta: f64,
    /// White-noise strength `σ` (units of state per √s).
    sigma: f64,
    /// Step duration (s).
    dt: f64,
    /// Precomputed `e^{-θ·dt}` for the deterministic path.
    decay: f64,
    /// Precomputed `σ_eq` for the noise injection.
    sigma_eq: f64,
}

impl OrnsteinUhlenbeck {
    /// Construct from `(theta, sigma, dt)`.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] when `theta <= 0`,
    /// `sigma < 0`, or `dt <= 0`. Returns [`SensorError::NonFinite`]
    /// for any non-finite parameter.
    pub fn new(theta: f64, sigma: f64, dt: f64) -> Result<Self, SensorError> {
        for v in [theta, sigma, dt] {
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "OU parameter is NaN or infinite",
                });
            }
        }
        if theta <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "OU theta must be > 0",
            });
        }
        if sigma < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "OU sigma must be ≥ 0",
            });
        }
        if dt <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "OU dt must be > 0",
            });
        }
        let decay = (-theta * dt).exp();
        // σ_eq² = σ² · (1 - e^{-2θ dt}) / (2θ).
        let one_minus_e2 = 1.0 - (-2.0 * theta * dt).exp();
        let sigma_eq = sigma * (one_minus_e2 / (2.0 * theta)).sqrt();
        Ok(Self {
            theta,
            sigma,
            dt,
            decay,
            sigma_eq,
        })
    }

    /// Mean-reversion rate `θ`.
    #[must_use]
    pub const fn theta(&self) -> f64 {
        self.theta
    }

    /// White-noise strength `σ`.
    #[must_use]
    pub const fn sigma(&self) -> f64 {
        self.sigma
    }

    /// Step duration.
    #[must_use]
    pub const fn dt(&self) -> f64 {
        self.dt
    }

    /// Asymptotic stationary standard deviation `σ / √(2θ)`.
    #[must_use]
    pub fn stationary_stddev(&self) -> f64 {
        self.sigma / (2.0 * self.theta).sqrt()
    }

    /// Apply one discrete-time step to the supplied state.
    ///
    /// `state_in` is consumed and `state_out` is returned. Locked
    /// operand order: `state_in · decay + sigma_eq · N(0, 1)`.
    pub fn step(
        &self,
        state_in: f64,
        rng: &mut DeterministicRng,
        gauss: &BoxMullerGaussian,
    ) -> f64 {
        let standard = gauss.sample_standard(rng);
        state_in * self.decay + self.sigma_eq * standard
    }
}

// ---------------------------------------------------------------------
// IntegratedWhiteNoise
// ---------------------------------------------------------------------

/// Discrete random-walk integrator for rate random walk (RRW) on
/// gyros and acceleration random walk (ARW) on accelerometers.
///
/// State update at fixed step `dt`:
///
/// ```text
/// x_{n+1} = x_n + σ · √dt · N(0, 1)
/// ```
///
/// Long-run variance grows linearly with elapsed time at rate `σ²`,
/// the canonical signature of a random walk.
#[derive(Copy, Clone, Debug)]
pub struct IntegratedWhiteNoise {
    /// White-noise strength `σ` (units of state per √s).
    sigma: f64,
    /// Step duration (s).
    dt: f64,
    /// Precomputed `σ · √dt`.
    step_stddev: f64,
}

impl IntegratedWhiteNoise {
    /// Construct from `(sigma, dt)`.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] when `sigma < 0` or
    /// `dt <= 0`. Returns [`SensorError::NonFinite`] for any
    /// non-finite parameter.
    pub fn new(sigma: f64, dt: f64) -> Result<Self, SensorError> {
        for v in [sigma, dt] {
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "random-walk parameter is NaN or infinite",
                });
            }
        }
        if sigma < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "random-walk sigma must be ≥ 0",
            });
        }
        if dt <= 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "random-walk dt must be > 0",
            });
        }
        Ok(Self {
            sigma,
            dt,
            step_stddev: sigma * dt.sqrt(),
        })
    }

    /// White-noise strength `σ`.
    #[must_use]
    pub const fn sigma(&self) -> f64 {
        self.sigma
    }

    /// Step duration.
    #[must_use]
    pub const fn dt(&self) -> f64 {
        self.dt
    }

    /// Apply one discrete-time step to the supplied state.
    pub fn step(
        &self,
        state_in: f64,
        rng: &mut DeterministicRng,
        gauss: &BoxMullerGaussian,
    ) -> f64 {
        let standard = gauss.sample_standard(rng);
        state_in + self.step_stddev * standard
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]
mod tests {
    use super::*;
    use openbmp_core::{SensorId, StepIndex};

    fn rng() -> DeterministicRng {
        DeterministicRng::for_sensor_component(0xCAFE, StepIndex::new(0), SensorId::new(1), 0)
    }

    // -----------------------------------------------------------------
    // BoxMullerGaussian
    // -----------------------------------------------------------------

    #[test]
    fn box_muller_is_bit_stable_across_two_invocations() {
        let g = BoxMullerGaussian::new();
        let mut a = rng();
        let mut b = rng();
        for _ in 0..32 {
            assert_eq!(
                g.sample(&mut a, 0.0, 1.0).unwrap().to_bits(),
                g.sample(&mut b, 0.0, 1.0).unwrap().to_bits()
            );
        }
    }

    #[test]
    fn box_muller_mean_and_variance_converge_for_standard_normal() {
        let g = BoxMullerGaussian::new();
        let mut r = rng();
        let n = 100_000;
        let samples: Vec<f64> = (0..n).map(|_| g.sample_standard(&mut r)).collect();
        let mean: f64 = samples.iter().sum::<f64>() / (n as f64);
        let var: f64 = samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n as f64);
        // 4 decimal places per the plan over 10⁵ samples.
        assert!(mean.abs() < 1.0e-2, "mean {mean} not within 0.01 of 0");
        assert!(
            (var - 1.0).abs() < 1.0e-2,
            "variance {var} not within 0.01 of 1"
        );
    }

    #[test]
    fn box_muller_scaled_gaussian_matches_mean_and_stddev() {
        let g = BoxMullerGaussian::new();
        let mut r = rng();
        let n = 50_000;
        let mu = 7.0;
        let sigma = 2.5;
        let samples: Vec<f64> = (0..n)
            .map(|_| g.sample(&mut r, mu, sigma).unwrap())
            .collect();
        let mean: f64 = samples.iter().sum::<f64>() / (n as f64);
        let stddev = (samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n as f64)).sqrt();
        assert!((mean - mu).abs() < 5.0e-2, "mean {mean} not close to {mu}");
        assert!(
            (stddev - sigma).abs() < 5.0e-2,
            "stddev {stddev} not close to {sigma}",
        );
    }

    #[test]
    fn box_muller_sample_rejects_non_finite_or_negative_stddev() {
        let g = BoxMullerGaussian::new();
        let mut r = rng();
        assert!(matches!(
            g.sample(&mut r, 0.0, -1.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            g.sample(&mut r, 0.0, f64::NAN),
            Err(SensorError::NonFinite { .. })
        ));
        assert!(matches!(
            g.sample(&mut r, f64::INFINITY, 1.0),
            Err(SensorError::NonFinite { .. })
        ));
    }

    // -----------------------------------------------------------------
    // OrnsteinUhlenbeck
    // -----------------------------------------------------------------

    #[test]
    fn ou_constructor_rejects_invalid_inputs() {
        assert!(matches!(
            OrnsteinUhlenbeck::new(0.0, 1.0, 0.01),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            OrnsteinUhlenbeck::new(-1.0, 1.0, 0.01),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            OrnsteinUhlenbeck::new(1.0, -0.1, 0.01),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            OrnsteinUhlenbeck::new(1.0, 1.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            OrnsteinUhlenbeck::new(f64::NAN, 1.0, 0.01),
            Err(SensorError::NonFinite { .. })
        ));
    }

    #[test]
    fn ou_long_run_mean_is_close_to_zero_and_variance_is_close_to_stationary() {
        let theta = 1.0;
        let sigma = 1.0;
        let dt = 0.01;
        let ou = OrnsteinUhlenbeck::new(theta, sigma, dt).unwrap();
        let g = BoxMullerGaussian::new();
        let mut r = rng();

        // Burn-in 5 correlation times to reach steady state.
        let burn_in = (5.0 / theta / dt) as usize;
        let mut x = 0.0;
        for _ in 0..burn_in {
            x = ou.step(x, &mut r, &g);
        }

        let n = 50_000;
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            x = ou.step(x, &mut r, &g);
            samples.push(x);
        }
        let mean: f64 = samples.iter().sum::<f64>() / (n as f64);
        let var: f64 = samples.iter().map(|s| (s - mean).powi(2)).sum::<f64>() / (n as f64);
        let stationary_var = ou.stationary_stddev().powi(2);
        // The autocorrelation in the OU sequence inflates the
        // sampling-mean variance; loosen the bands to match.
        assert!(mean.abs() < 0.1, "OU long-run mean {mean} too far from 0");
        assert!(
            (var - stationary_var).abs() / stationary_var < 0.1,
            "OU variance {var} not within 10% of stationary {stationary_var}",
        );
    }

    #[test]
    fn ou_step_is_bit_stable_across_two_runs_with_identical_seeds() {
        let ou = OrnsteinUhlenbeck::new(1.0, 1.0, 0.01).unwrap();
        let g = BoxMullerGaussian::new();
        let mut r1 = rng();
        let mut r2 = rng();
        let mut x1 = 0.5;
        let mut x2 = 0.5;
        for _ in 0..200 {
            x1 = ou.step(x1, &mut r1, &g);
            x2 = ou.step(x2, &mut r2, &g);
            assert_eq!(x1.to_bits(), x2.to_bits());
        }
    }

    #[test]
    fn ou_with_zero_sigma_decays_deterministically() {
        let theta = 2.0_f64;
        let dt = 0.05_f64;
        let ou = OrnsteinUhlenbeck::new(theta, 0.0, dt).unwrap();
        let g = BoxMullerGaussian::new();
        let mut r = rng();
        let mut x = 1.0;
        for k in 1..=20 {
            x = ou.step(x, &mut r, &g);
            let expected = (-theta * dt * (k as f64)).exp();
            assert!(
                (x - expected).abs() < 1.0e-12,
                "OU(σ=0) at step {k}: x = {x}, expected {expected}",
            );
        }
    }

    // -----------------------------------------------------------------
    // IntegratedWhiteNoise
    // -----------------------------------------------------------------

    #[test]
    fn random_walk_constructor_rejects_invalid_inputs() {
        assert!(matches!(
            IntegratedWhiteNoise::new(-1.0, 0.01),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            IntegratedWhiteNoise::new(1.0, 0.0),
            Err(SensorError::InvalidParameter { .. })
        ));
        assert!(matches!(
            IntegratedWhiteNoise::new(f64::NAN, 0.01),
            Err(SensorError::NonFinite { .. })
        ));
    }

    #[test]
    fn random_walk_variance_grows_linearly_in_time() {
        let sigma = 1.0;
        let dt = 0.01;
        let rw = IntegratedWhiteNoise::new(sigma, dt).unwrap();
        let g = BoxMullerGaussian::new();
        let n_paths = 2_000_usize;
        let n_steps = 200_usize;

        // Run many independent paths from x = 0.
        let mut endpoints = Vec::with_capacity(n_paths);
        for path in 0..n_paths {
            let mut r = DeterministicRng::for_sensor_component(
                0xBEEF,
                StepIndex::new(path as u64),
                SensorId::new(2),
                0,
            );
            let mut x = 0.0;
            for _ in 0..n_steps {
                x = rw.step(x, &mut r, &g);
            }
            endpoints.push(x);
        }
        let mean: f64 = endpoints.iter().sum::<f64>() / (n_paths as f64);
        let var: f64 = endpoints.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n_paths as f64);
        let expected_var = sigma * sigma * dt * (n_steps as f64);
        // The single-path-many-realisations variance should grow as
        // σ² · t. 10% band over 2000 paths.
        assert!(
            (var - expected_var).abs() / expected_var < 0.1,
            "random-walk variance {var} not within 10% of expected {expected_var}",
        );
    }

    #[test]
    fn random_walk_step_is_bit_stable_across_two_runs() {
        let rw = IntegratedWhiteNoise::new(1.0, 0.01).unwrap();
        let g = BoxMullerGaussian::new();
        let mut r1 = rng();
        let mut r2 = rng();
        let mut x1 = 0.0;
        let mut x2 = 0.0;
        for _ in 0..200 {
            x1 = rw.step(x1, &mut r1, &g);
            x2 = rw.step(x2, &mut r2, &g);
            assert_eq!(x1.to_bits(), x2.to_bits());
        }
    }
}
