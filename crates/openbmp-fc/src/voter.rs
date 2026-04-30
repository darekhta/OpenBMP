//! Sensor voter trait and reference implementations.
//!
//! Phase 4.2: simplex pass-through, triplex mid-value-select, and
//! weighted-mean voters. Even when an OpenBMP scenario uses one IMU /
//! one barometer / one GNSS, the voter seam exists so a downstream
//! HAL adopter wiring redundant lanes is a configuration change, not
//! a refactor.

use std::cmp::Ordering;

/// Result of a [`Voter::vote`] call.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VotedReading<T> {
    /// Voted value.
    pub value: T,
    /// `true` if any input sample disagreed with the voted value
    /// beyond the voter's tolerance.
    pub divergent: bool,
    /// Number of input samples that contributed to the vote.
    pub contributed: usize,
}

/// N-of-M voter for redundant sensor samples.
pub trait Voter<T: Copy> {
    /// Votes across `samples`. The semantic depends on the
    /// implementation — see [`PassThroughVoter`],
    /// [`MidValueSelectVoter`], [`WeightedMeanVoter`].
    fn vote(&self, samples: &[T]) -> Option<VotedReading<T>>;
}

/// Simplex / pass-through voter. Always selects the first sample.
#[derive(Copy, Clone, Debug, Default)]
pub struct PassThroughVoter;

impl<T: Copy> Voter<T> for PassThroughVoter {
    fn vote(&self, samples: &[T]) -> Option<VotedReading<T>> {
        samples.first().copied().map(|value| VotedReading {
            value,
            divergent: false,
            contributed: samples.len().min(1),
        })
    }
}

/// Mid-value-select voter. Selects the median of three or more
/// scalar samples; for arrays / vectors, see [`MidValueSelectScalar`].
#[derive(Copy, Clone, Debug, Default)]
pub struct MidValueSelectScalar {
    /// Maximum allowed deviation (in absolute units) between the
    /// voted value and any contributing sample before flagging
    /// divergence.
    pub divergence_tol: f64,
}

impl Voter<f64> for MidValueSelectScalar {
    fn vote(&self, samples: &[f64]) -> Option<VotedReading<f64>> {
        if samples.is_empty() {
            return None;
        }
        let mut sorted = samples.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let value = sorted[sorted.len() / 2];
        let divergent = samples
            .iter()
            .any(|s| (*s - value).abs() > self.divergence_tol);
        Some(VotedReading {
            value,
            divergent,
            contributed: samples.len(),
        })
    }
}

/// Weighted-mean voter for scalar samples. The first weight is the
/// midpoint weight; the remaining weights are equally distributed
/// across other samples. Useful when one sensor is a higher-grade
/// reference.
#[derive(Copy, Clone, Debug)]
pub struct WeightedMeanScalar {
    /// Weight of the first / midpoint sample.
    pub primary_weight: f64,
    /// Weight assigned to each non-primary sample.
    pub others_weight: f64,
    /// Maximum allowed deviation of any sample from the weighted mean
    /// before flagging divergence.
    pub divergence_tol: f64,
}

impl Voter<f64> for WeightedMeanScalar {
    fn vote(&self, samples: &[f64]) -> Option<VotedReading<f64>> {
        if samples.is_empty() {
            return None;
        }
        // Sample counts are intentionally cast to f64 for the weight
        // sum; voter inputs are bounded at simplex/triplex sizes well
        // under f64's 52-bit mantissa, so the precision loss is moot.
        #[allow(clippy::cast_precision_loss)]
        let total_weight = self.primary_weight + self.others_weight * (samples.len() - 1) as f64;
        if total_weight <= 0.0 {
            return None;
        }
        let mut weighted_sum = self.primary_weight * samples[0];
        for s in &samples[1..] {
            weighted_sum += self.others_weight * s;
        }
        let value = weighted_sum / total_weight;
        let divergent = samples
            .iter()
            .any(|s| (s - value).abs() > self.divergence_tol);
        Some(VotedReading {
            value,
            divergent,
            contributed: samples.len(),
        })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn pass_through_returns_first_sample() {
        let v = PassThroughVoter;
        let r = v.vote(&[1.0_f64, 2.0, 3.0]).unwrap();
        assert!((r.value - 1.0).abs() < f64::EPSILON);
        assert!(!r.divergent);
        assert_eq!(r.contributed, 1);
    }

    #[test]
    fn pass_through_empty_returns_none() {
        let v = PassThroughVoter;
        let r: Option<VotedReading<f64>> = v.vote(&[]);
        assert!(r.is_none());
    }

    #[test]
    fn midvalue_returns_median_of_three() {
        let v = MidValueSelectScalar {
            divergence_tol: 1.5,
        };
        let r = v.vote(&[1.0, 3.0, 2.0]).unwrap();
        assert!((r.value - 2.0).abs() < f64::EPSILON);
        assert!(!r.divergent);
    }

    #[test]
    fn midvalue_flags_divergence() {
        let v = MidValueSelectScalar {
            divergence_tol: 0.5,
        };
        // 0.5 vs median 2.0 -> diverges (delta 1.5 > 0.5)
        let r = v.vote(&[0.5, 2.0, 3.0]).unwrap();
        assert!((r.value - 2.0).abs() < f64::EPSILON);
        assert!(r.divergent);
    }

    #[test]
    fn weighted_mean_uses_primary_weight() {
        let v = WeightedMeanScalar {
            primary_weight: 0.5,
            others_weight: 0.25,
            divergence_tol: 1.0,
        };
        // 0.5*10 + 0.25*0 + 0.25*0 = 5; total weight 1.0; mean = 5
        let r = v.vote(&[10.0, 0.0, 0.0]).unwrap();
        assert!((r.value - 5.0).abs() < f64::EPSILON);
    }
}
