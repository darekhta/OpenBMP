//! Willsky windowed-mean-shift GLRT detector.
//!
//! For an i.i.d. sequence of whitened innovations
//! `ν̃_i ~ N(0, I_d)` under H₀, the test asks whether at some
//! candidate jump time τ the post-jump samples have a non-zero
//! constant mean shift `b`:
//!
//! ```text
//!   H₀: ν̃_i ~ N(0, I_d)              for all i
//!   H₁(τ, b): ν̃_i ~ N(b, I_d)          for i ≥ τ,
//!             ν̃_i ~ N(0, I_d)          for i < τ.
//! ```
//!
//! Maximizing the likelihood ratio over `b` gives the per-τ statistic
//!
//! ```text
//!   b̂(τ) = mean(ν̃_τ, …, ν̃_k)
//!   Λ(τ) = N_τ · ‖b̂(τ)‖² = ‖Σ ν̃_i‖² / N_τ,    N_τ = k − τ + 1.
//! ```
//!
//! Each `Λ(τ)` is `χ²(d)`-distributed under H₀ for fixed τ. To control
//! the family-wise false-alarm rate across the W candidate τ values
//! the detector tracks, the trip threshold is
//! `χ²⁻¹(1 − α / W, d)` (Bonferroni-corrected). The detector trips
//! when `max_τ Λ(τ) > threshold`.
//!
//! # Reference
//!
//! Willsky, A. S. and Jones, H. L., *A generalized likelihood ratio
//! approach to the detection and estimation of jumps in linear
//! systems*, IEEE TAC 21(1), 108-112, 1976.
//!
//! # Determinism
//!
//! The detector consumes per-step whitened innovations and emits
//! either `NotTripped` or `Tripped { estimated_jump_step, statistic,
//! threshold }`. All arithmetic is `f64` with explicit parentheses on
//! the per-step running sum; no FMA; iteration order is fixed (`n`
//! from 1 to `count`) so two detectors fed an identical stream
//! produce bit-identical outputs across reruns.

use openbmp_physics::statistics::chi_square_inverse_cdf_wilson_hilferty;

/// Outcome of a single `step()` call.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum StepOutcome {
    /// `max_τ Λ(τ) ≤ threshold`. Detector remains armed.
    NotTripped,
    /// `max_τ Λ(τ) > threshold`. Detector reports the maximizer.
    Tripped {
        /// Step index of the candidate jump time `τ̂` that maximised
        /// `Λ(τ)`. Guaranteed `≤ last_step` and within the window.
        estimated_jump_step: u64,
        /// Maximised statistic value.
        statistic: f64,
        /// Bonferroni-corrected trip threshold.
        threshold: f64,
    },
}

/// Errors returned by [`WindowedMeanShiftGlrt::new`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GlrtError {
    /// `window_size` was zero.
    InvalidWindowSize,
    /// `false_alarm_rate` was outside the open interval `(0, 1)` or
    /// non-finite.
    InvalidFalseAlarmRate,
}

/// Windowed-mean-shift GLRT detector for a single sensor lane with
/// fixed innovation dimension `D`.
///
/// Holds a circular buffer of the most recent `window_size` whitened
/// innovation vectors. Each `step()` call pushes the new sample,
/// evaluates `Λ(τ)` for every candidate `τ` in the window, and trips
/// when the maximum exceeds the Bonferroni-corrected `χ²(1 − α/W, D)`
/// threshold.
#[derive(Clone, Debug)]
pub struct WindowedMeanShiftGlrt<const D: usize> {
    buffer: Vec<[f64; D]>,
    /// Position of the next write in `buffer`. Wraps modulo
    /// `window_size`.
    write_pos: usize,
    /// Number of valid samples currently in the buffer (`≤ window_size`).
    count: usize,
    window_size: usize,
    trip_threshold: f64,
    /// Step index of the most recent push. Only set after the first
    /// `step()` call.
    last_step: u64,
}

impl<const D: usize> WindowedMeanShiftGlrt<D> {
    /// Construct a detector with the given window size and false-alarm
    /// rate.
    ///
    /// The trip threshold is pre-computed as `χ²⁻¹(1 − α / W, D)` —
    /// Bonferroni-corrected over the `W` candidate jump times the
    /// detector inspects on each step.
    ///
    /// # Errors
    ///
    /// Returns [`GlrtError::InvalidWindowSize`] when `window_size`
    /// is `0`, or [`GlrtError::InvalidFalseAlarmRate`] when
    /// `false_alarm_rate` is non-finite or outside `(0, 1)`.
    #[allow(clippy::cast_precision_loss)] // window_size and D are bounded small (≤64 / ≤16)
    pub fn new(window_size: usize, false_alarm_rate: f64) -> Result<Self, GlrtError> {
        if window_size == 0 {
            return Err(GlrtError::InvalidWindowSize);
        }
        if !false_alarm_rate.is_finite() || false_alarm_rate <= 0.0 || false_alarm_rate >= 1.0 {
            return Err(GlrtError::InvalidFalseAlarmRate);
        }
        // Bonferroni-corrected per-τ false-alarm rate.
        let alpha_per_tau = false_alarm_rate / (window_size as f64);
        let trip_threshold = chi_square_inverse_cdf_wilson_hilferty(1.0 - alpha_per_tau, D as f64);
        Ok(Self {
            buffer: vec![[0.0; D]; window_size],
            write_pos: 0,
            count: 0,
            window_size,
            trip_threshold,
            last_step: 0,
        })
    }

    /// Configured window size (number of past samples retained).
    #[must_use]
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Configured trip threshold.
    #[must_use]
    pub fn trip_threshold(&self) -> f64 {
        self.trip_threshold
    }

    /// Number of samples currently in the window (`≤ window_size`).
    #[must_use]
    pub fn count(&self) -> usize {
        self.count
    }

    /// Reset the detector's internal buffer and step counter. The
    /// trip threshold is preserved.
    pub fn reset(&mut self) {
        for slot in &mut self.buffer {
            *slot = [0.0; D];
        }
        self.write_pos = 0;
        self.count = 0;
        self.last_step = 0;
    }

    /// Push a new whitened-innovation sample taken at `step_index`
    /// (`1`-based, monotonic) and report the test outcome. The
    /// per-step iteration cost is `O(D · W)` where `W` is the window
    /// size.
    #[allow(clippy::cast_precision_loss)] // `n` is bounded by `window_size` ≤ 2^32
    pub fn step(&mut self, whitened: [f64; D], step_index: u64) -> StepOutcome {
        self.buffer[self.write_pos] = whitened;
        self.write_pos = (self.write_pos + 1) % self.window_size;
        if self.count < self.window_size {
            self.count += 1;
        }
        self.last_step = step_index;

        // Iterate over candidate jump times in step-index space.
        // For `n` ranging 1..=count, we accumulate the sum of the
        // last `n` samples (running-sum trick), then evaluate
        //   Λ(τ) = ‖Σ ν̃_i‖² / N_τ,   N_τ = n
        // for that candidate. The argmax is the best estimate of τ̂.
        //
        // DETERMINISM CONTRACT: locked iteration order on every loop;
        // explicit parentheses on the squared-norm sum; no FMA. The
        // `+=` operator on `f64` is identity-equivalent to `a = a + b`
        // at the IEEE 754 level (a single rounding) and matches the
        // determinism contract verbatim.
        let mut running_sum = [0.0_f64; D];
        let mut max_lambda = 0.0_f64;
        let mut argmax_n: usize = 0;
        for n in 1..=self.count {
            // Index of the `n`-th most recent sample. The most recent
            // sample lives at `(write_pos − 1) mod W` (we've already
            // advanced write_pos past it above).
            let idx = (self.write_pos + self.window_size - n) % self.window_size;
            let sample = &self.buffer[idx];
            for (running, sample_value) in running_sum.iter_mut().zip(sample.iter()) {
                *running += *sample_value;
            }
            // Squared L₂ norm of the running sum, locked-order sum.
            let mut sum_norm_sq = 0.0_f64;
            for &component in &running_sum {
                sum_norm_sq += component * component;
            }
            let lambda = sum_norm_sq / (n as f64);
            if lambda > max_lambda {
                max_lambda = lambda;
                argmax_n = n;
            }
        }

        if max_lambda > self.trip_threshold {
            // Estimated jump step is the argmax_n-th sample back from
            // last_step (inclusive of last_step itself when n = 1).
            let estimated_jump_step = self.last_step - (argmax_n as u64) + 1;
            StepOutcome::Tripped {
                estimated_jump_step,
                statistic: max_lambda,
                threshold: self.trip_threshold,
            }
        } else {
            StepOutcome::NotTripped
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::needless_range_loop,
    clippy::doc_markdown
)]
mod tests {
    use super::*;

    /// Deterministic 64-bit linear congruential generator for tests.
    /// Same constants as Numerical Recipes' "ranqd1"; suitable for
    /// repeatable pseudo-random tests, not for cryptographic use.
    struct TestRng {
        state: u64,
    }

    impl TestRng {
        const fn new(seed: u64) -> Self {
            Self { state: seed }
        }

        fn next_u64(&mut self) -> u64 {
            self.state = self
                .state
                .wrapping_mul(1_664_525)
                .wrapping_add(1_013_904_223);
            self.state
        }

        /// Uniform `[0, 1)` from the top 53 bits of next_u64.
        fn next_unit(&mut self) -> f64 {
            let bits = (self.next_u64() >> 11) as f64;
            bits / ((1_u64 << 53) as f64)
        }

        /// Box-Muller standard normal sample.
        fn next_normal(&mut self) -> f64 {
            let u1 = self.next_unit().max(1.0e-308); // avoid ln(0)
            let u2 = self.next_unit();
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * std::f64::consts::PI * u2;
            r * theta.cos()
        }
    }

    #[test]
    fn constructor_rejects_zero_window_size() {
        assert!(matches!(
            WindowedMeanShiftGlrt::<3>::new(0, 0.01),
            Err(GlrtError::InvalidWindowSize)
        ));
    }

    #[test]
    fn constructor_rejects_invalid_false_alarm_rate() {
        for bad in [0.0, 1.0, -0.1, 1.1, f64::NAN, f64::INFINITY] {
            assert!(
                matches!(
                    WindowedMeanShiftGlrt::<3>::new(64, bad),
                    Err(GlrtError::InvalidFalseAlarmRate)
                ),
                "false_alarm_rate = {bad} should be rejected",
            );
        }
    }

    #[test]
    fn empty_step_after_construction_returns_not_tripped() {
        let mut det = WindowedMeanShiftGlrt::<3>::new(16, 0.01).unwrap();
        // Single zero sample produces Λ = 0 < threshold.
        let out = det.step([0.0, 0.0, 0.0], 1);
        assert_eq!(out, StepOutcome::NotTripped);
        assert_eq!(det.count(), 1);
    }

    #[test]
    fn pure_h0_does_not_trip_over_long_run() {
        // 200 samples of N(0, I_3) with α = 0.01 and W = 32. Bonferroni
        // per-τ rate is 3.1e-4; the chi-square(3) 1−3.1e-4 quantile
        // is not crossed by this deterministic seeded H₀ sequence.
        let mut det = WindowedMeanShiftGlrt::<3>::new(32, 0.01).unwrap();
        let mut rng = TestRng::new(0xdead_beef_2026_0503);
        let mut tripped = false;
        for step in 1..=200_u64 {
            let s = [rng.next_normal(), rng.next_normal(), rng.next_normal()];
            if matches!(det.step(s, step), StepOutcome::Tripped { .. }) {
                tripped = true;
                break;
            }
        }
        assert!(!tripped, "pure-H₀ run should not trip over 200 samples");
    }

    #[test]
    fn step_injection_trips_within_window_resolution() {
        let window = 32_usize;
        let det_far = 0.001;
        let mut det = WindowedMeanShiftGlrt::<3>::new(window, det_far).unwrap();
        let mut rng = TestRng::new(0x4f70_656e_4243_5234);
        // Burn in 32 zero-mean samples so the window is fully populated.
        for step in 1..=window as u64 {
            let s = [rng.next_normal(), rng.next_normal(), rng.next_normal()];
            assert_eq!(det.step(s, step), StepOutcome::NotTripped);
        }
        // Inject a constant 4σ shift on component 0 starting at step 33.
        let injection_start: u64 = (window as u64) + 1;
        let mut tripped_at = None;
        let mut tripped_jump = None;
        for step in injection_start..injection_start + 16 {
            let s = [
                rng.next_normal() + 4.0,
                rng.next_normal(),
                rng.next_normal(),
            ];
            if let StepOutcome::Tripped {
                estimated_jump_step,
                ..
            } = det.step(s, step)
            {
                tripped_at = Some(step);
                tripped_jump = Some(estimated_jump_step);
                break;
            }
        }
        let trip_step = tripped_at.expect("4σ shift must trip the detector within 16 samples");
        let jump_estimate = tripped_jump.unwrap();
        // Estimated jump time within ±2 samples of the true τ.
        let true_jump = injection_start as i64;
        assert!(
            (jump_estimate as i64 - true_jump).abs() <= 2,
            "estimated jump {jump_estimate} not within ±2 of true {true_jump} (tripped at {trip_step})",
        );
    }

    #[test]
    fn determinism_byte_stable_across_two_constructions() {
        let mut a = WindowedMeanShiftGlrt::<6>::new(20, 0.01).unwrap();
        let mut b = WindowedMeanShiftGlrt::<6>::new(20, 0.01).unwrap();
        let mut rng = TestRng::new(0x4f42_4d50_4754_5230);
        for step in 1..=50_u64 {
            let mut sample = [0.0; 6];
            for i in 0..6 {
                sample[i] = rng.next_normal();
            }
            // Inject a step at step = 30.
            if step >= 30 {
                sample[0] += 2.5;
            }
            let oa = a.step(sample, step);
            let ob = b.step(sample, step);
            match (oa, ob) {
                (StepOutcome::NotTripped, StepOutcome::NotTripped) => {}
                (
                    StepOutcome::Tripped {
                        estimated_jump_step: ja,
                        statistic: sa,
                        threshold: ta,
                    },
                    StepOutcome::Tripped {
                        estimated_jump_step: jb,
                        statistic: sb,
                        threshold: tb,
                    },
                ) => {
                    assert_eq!(ja, jb, "estimated_jump_step diverged at step {step}");
                    assert_eq!(
                        sa.to_bits(),
                        sb.to_bits(),
                        "statistic not bit-stable at step {step}",
                    );
                    assert_eq!(ta.to_bits(), tb.to_bits());
                }
                _ => panic!("trip-vs-not-trip divergence at step {step}: a={oa:?}, b={ob:?}"),
            }
        }
    }

    #[test]
    fn reset_clears_buffer_and_step_history() {
        let mut det = WindowedMeanShiftGlrt::<3>::new(8, 0.001).unwrap();
        // Trip the detector with an extreme shift.
        for step in 1..=10_u64 {
            let _ = det.step([10.0, 10.0, 10.0], step);
        }
        det.reset();
        assert_eq!(det.count(), 0);
        // After reset, a single zero sample should not trip.
        assert_eq!(det.step([0.0, 0.0, 0.0], 1), StepOutcome::NotTripped);
    }

    #[test]
    fn scalar_baro_detector_compiles_and_trips_on_step_injection() {
        // Sanity check on the D = 1 specialisation — the baro path uses
        // a single-component whitened innovation.
        let mut det = WindowedMeanShiftGlrt::<1>::new(16, 0.001).unwrap();
        let mut rng = TestRng::new(0x4261_726f_4744_3132);
        for step in 1..=16_u64 {
            let _ = det.step([rng.next_normal()], step);
        }
        let mut tripped = false;
        for step in 17..=40_u64 {
            if matches!(
                det.step([rng.next_normal() + 5.0], step),
                StepOutcome::Tripped { .. }
            ) {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "scalar 5σ shift must trip the D=1 detector");
    }

    /// Single-sample sanity: with W = 1 and a single non-zero sample
    /// of magnitude `m`, Λ = m². Trip iff m² > χ²⁻¹(1 − α, D).
    #[test]
    fn single_sample_window_collapses_to_single_sample_chi_square_test() {
        let det = WindowedMeanShiftGlrt::<3>::new(1, 0.001).unwrap();
        let threshold = det.trip_threshold();
        let mut det = det;
        // Sample at threshold + epsilon should trip.
        let m = (threshold + 0.1).sqrt();
        let outcome = det.step([m, 0.0, 0.0], 1);
        match outcome {
            StepOutcome::Tripped {
                statistic,
                estimated_jump_step,
                ..
            } => {
                assert!(
                    (statistic - (m * m)).abs() < 1.0e-12,
                    "statistic should equal m² at W=1: got {statistic}",
                );
                assert_eq!(estimated_jump_step, 1);
            }
            StepOutcome::NotTripped => panic!("threshold + 0.1 should trip"),
        }
    }
}
