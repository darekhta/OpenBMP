//! Synthetic star tracker.
//!
//! The star tracker reports the body-to-ECI attitude. Per axis the
//! measurement transformation is a small-angle perturbation:
//!
//! ```text
//! θ_x, θ_y, θ_z ∼ N(0, σ_per_axis_rad)
//! q_perturb = normalize(1, θ_x/2, θ_y/2, θ_z/2)   (small-angle)
//! q_meas    = q_truth ⊗ q_perturb                  (body-frame perturb)
//! ```
//!
//! The small-angle form `q ≈ (1, θ/2)` is valid for typical
//! academic star-tracker noise budgets (`σ ≪ 1 rad`) — Liebe 1995
//! and Crassidis 2007 both cite arc-second-level noise (1″ ≈ 4.85
//! µrad). The constructor rejects budgets with `σ > 0.01 rad`
//! (~34′ ≈ 0.57°) where the small-angle approximation becomes
//! noticeably inaccurate; for those budgets the audit can promote
//! the model to a full quaternion-exponential form.
//!
//! # Component-id layout
//!
//! 3 reserved component slots: one per perturbation axis.
//!
//! ```text
//! channel        component_id
//! theta_x        0
//! theta_y        1
//! theta_z        2
//! ```
//!
//! # Determinism
//!
//! Pure `f64`; locked operand order on the perturbation
//! quaternion build and the quaternion-multiplication. Per-
//! component RNG sub-streams via
//! `DeterministicRng::for_sensor_component`. Truth-bypass mode
//! (`σ = 0`) returns the truth quaternion bit-equal.

use nalgebra::{Quaternion, UnitQuaternion};
use openbmp_core::{DeterministicRng, SensorId, StepIndex};

use crate::error::SensorError;
use crate::noise::BoxMullerGaussian;
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

const SUB_THETA_X: u32 = 0;
const SUB_THETA_Y: u32 = 1;
const SUB_THETA_Z: u32 = 2;

const THETA_IDS: [u32; 3] = [SUB_THETA_X, SUB_THETA_Y, SUB_THETA_Z];

/// Maximum per-axis sigma (rad) above which the small-angle
/// approximation `q ≈ (1, θ/2)` starts to drift noticeably from
/// the exact quaternion-exponential form.
pub const MAX_SMALL_ANGLE_SIGMA_RAD: f64 = 0.01;

/// One arc-second in radians (= π / 648 000). Used by the
/// data-file loader to convert from the human-friendly arc-second
/// budget unit into radians.
pub const ARCSEC_TO_RAD: f64 = std::f64::consts::PI / 648_000.0;

/// Isotropic per-axis noise budget for the synthetic star tracker.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StarTrackerNoiseBudget {
    /// Per-axis Gaussian-noise stddev on the rotation-vector
    /// perturbation (rad). Isotropic; matches the typical academic
    /// star-tracker noise model in Crassidis 2007 §3.2.
    pub sigma_per_axis_rad: f64,
}

impl StarTrackerNoiseBudget {
    /// Construct after validating the parameter.
    ///
    /// # Errors
    ///
    /// - [`SensorError::NonFinite`] if `sigma_per_axis_rad` is NaN
    ///   or infinite.
    /// - [`SensorError::InvalidParameter`] if it is negative or
    ///   strictly greater than [`MAX_SMALL_ANGLE_SIGMA_RAD`] —
    ///   above which the small-angle quaternion form diverges
    ///   noticeably.
    pub fn new(sigma_per_axis_rad: f64) -> Result<Self, SensorError> {
        if !sigma_per_axis_rad.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "star tracker sigma_per_axis_rad is NaN or infinite",
            });
        }
        if sigma_per_axis_rad < 0.0 {
            return Err(SensorError::InvalidParameter {
                reason: "star tracker sigma_per_axis_rad must be non-negative",
            });
        }
        if sigma_per_axis_rad > MAX_SMALL_ANGLE_SIGMA_RAD {
            return Err(SensorError::InvalidParameter {
                reason: "star tracker sigma_per_axis_rad exceeds the small-angle range (≤ 0.01 rad)",
            });
        }
        Ok(Self { sigma_per_axis_rad })
    }

    /// Construct from the human-friendly arc-second unit.
    ///
    /// # Errors
    ///
    /// Same as [`Self::new`] after the arc-second → rad conversion.
    pub fn from_arcsec(sigma_per_axis_arcsec: f64) -> Result<Self, SensorError> {
        Self::new(sigma_per_axis_arcsec * ARCSEC_TO_RAD)
    }
}

/// Synthetic star tracker.
#[derive(Copy, Clone, Debug)]
pub struct SyntheticStarTracker {
    sensor_id: SensorId,
    budget: StarTrackerNoiseBudget,
    gauss: BoxMullerGaussian,
}

impl SyntheticStarTracker {
    /// Construct from a sensor id and a fully-validated noise
    /// budget.
    #[must_use]
    pub fn new(sensor_id: SensorId, budget: StarTrackerNoiseBudget) -> Self {
        Self {
            sensor_id,
            budget,
            gauss: BoxMullerGaussian::new(),
        }
    }

    /// Read-only access to the budget.
    #[must_use]
    pub const fn budget(&self) -> &StarTrackerNoiseBudget {
        &self.budget
    }
}

impl SyntheticSensor for SyntheticStarTracker {
    fn sensor_id(&self) -> SensorId {
        self.sensor_id
    }

    fn measure(
        &mut self,
        truth: &SensorTruth,
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError> {
        require_truth_finite(truth)?;
        let sigma = self.budget.sigma_per_axis_rad;
        // Sample three Gaussian rotation-vector components.
        let mut theta = [0.0_f64; 3];
        for axis in 0..3 {
            let mut rng = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                THETA_IDS[axis],
            );
            theta[axis] = self.gauss.sample(&mut rng, 0.0, sigma)?;
        }
        // Small-angle quaternion: q_pert = (1, θ_x/2, θ_y/2, θ_z/2)
        // then normalize. Locked operand order so reproducibility
        // is bit-stable. The exact (sin, cos) form would diverge
        // from the linearised form by ~θ³/48; for σ ≤ 0.01 rad
        // this is below 1e-7 of unit-norm — well inside f64
        // precision but the budget gate keeps us on the safe side.
        let raw = Quaternion::new(1.0, theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
        let norm = raw.norm();
        if !norm.is_finite() || norm == 0.0 {
            return Err(SensorError::NonFinite {
                reason: "star tracker perturbation quaternion has non-finite norm",
            });
        }
        let unit_pert = UnitQuaternion::from_quaternion(raw / norm);
        let q_meas = truth.attitude_eci_to_body * unit_pert;
        Ok(SensorMeasurement::StarTracker {
            attitude_eci_to_body: q_meas,
        })
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::{Position3, SimTime, Velocity3};

    fn truth_with_attitude(q: UnitQuaternion<f64>) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: q,
            angular_velocity_body_rad_s: Vector3::zeros(),
            angular_acceleration_body_rad_s2: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: 0.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: SimTime::ZERO,
        }
    }

    #[test]
    fn rejects_negative_sigma() {
        assert!(matches!(
            StarTrackerNoiseBudget::new(-1.0e-6),
            Err(SensorError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_non_finite_sigma() {
        assert!(matches!(
            StarTrackerNoiseBudget::new(f64::NAN),
            Err(SensorError::NonFinite { .. })
        ));
    }

    #[test]
    fn rejects_sigma_above_small_angle_limit() {
        // 0.01 rad is the boundary; anything strictly larger fails.
        assert!(StarTrackerNoiseBudget::new(MAX_SMALL_ANGLE_SIGMA_RAD).is_ok());
        assert!(matches!(
            StarTrackerNoiseBudget::new(MAX_SMALL_ANGLE_SIGMA_RAD + 1.0e-9),
            Err(SensorError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn arcsec_constructor_round_trips() {
        // 5 arc-seconds (Crassidis 2007 textbook value).
        let b = StarTrackerNoiseBudget::from_arcsec(5.0).unwrap();
        let expected = 5.0 * ARCSEC_TO_RAD;
        assert!((b.sigma_per_axis_rad - expected).abs() < 1.0e-15);
    }

    #[test]
    fn truth_bypass_returns_attitude_bit_equal() {
        let id = SensorId::from_path("sensors.star.test");
        let mut s = SyntheticStarTracker::new(id, StarTrackerNoiseBudget::new(0.0).unwrap());
        let q_truth = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), 0.5);
        let truth = truth_with_attitude(q_truth);
        let r = s.measure(&truth, StepIndex::new(0), 0xC0FF_EE00).unwrap();
        let SensorMeasurement::StarTracker {
            attitude_eci_to_body,
        } = r
        else {
            panic!("expected StarTracker");
        };
        let q_in = q_truth.into_inner();
        let q_out = attitude_eci_to_body.into_inner();
        assert_eq!(q_out.w.to_bits(), q_in.w.to_bits());
        assert_eq!(q_out.i.to_bits(), q_in.i.to_bits());
        assert_eq!(q_out.j.to_bits(), q_in.j.to_bits());
        assert_eq!(q_out.k.to_bits(), q_in.k.to_bits());
    }

    #[test]
    fn measurement_quaternion_is_unit_norm_for_random_seeds() {
        let id = SensorId::from_path("sensors.star.unit_norm");
        let mut s = SyntheticStarTracker::new(
            id,
            StarTrackerNoiseBudget::from_arcsec(20.0).unwrap(), // wide budget
        );
        let q_truth = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), 0.7);
        let truth = truth_with_attitude(q_truth);
        for step in 0..1_000_u64 {
            let r = s
                .measure(&truth, StepIndex::new(step), 0xBABE_F00D)
                .unwrap();
            let SensorMeasurement::StarTracker {
                attitude_eci_to_body: q,
            } = r
            else {
                panic!();
            };
            let qq = q.into_inner();
            let n = (qq.w * qq.w + qq.i * qq.i + qq.j * qq.j + qq.k * qq.k).sqrt();
            assert!(
                (n - 1.0).abs() < 1.0e-12,
                "quaternion not unit norm: {n} at step {step}",
            );
        }
    }

    #[test]
    fn empirical_axis_std_within_5_percent_of_budget() {
        let id = SensorId::from_path("sensors.star.long_run");
        let sigma_arcsec = 50.0;
        let budget = StarTrackerNoiseBudget::from_arcsec(sigma_arcsec).unwrap();
        let sigma = budget.sigma_per_axis_rad;
        let mut s = SyntheticStarTracker::new(id, budget);
        let q_truth = UnitQuaternion::identity();
        let truth = truth_with_attitude(q_truth);
        let n = 10_000_u64;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for step in 0..n {
            let r = s
                .measure(&truth, StepIndex::new(step), 0x55AA_55AA)
                .unwrap();
            let SensorMeasurement::StarTracker {
                attitude_eci_to_body: q,
            } = r
            else {
                panic!();
            };
            // Recover θ_x from the perturbation quaternion: for small
            // angles, q.i ≈ θ_x / 2.
            let theta_x = 2.0 * q.into_inner().i;
            sum += theta_x;
            sum_sq += theta_x * theta_x;
        }
        let n_f = n as f64;
        let mean = sum / n_f;
        let variance = sum_sq / n_f - mean * mean;
        let stddev = variance.sqrt();
        let rel_err = (stddev - sigma).abs() / sigma;
        assert!(
            rel_err < 0.05,
            "empirical std {stddev} differs from budget {sigma} by {rel_err}",
        );
    }

    #[test]
    fn determinism_byte_stable_across_two_runs() {
        let id = SensorId::from_path("sensors.star.replay");
        let budget = StarTrackerNoiseBudget::from_arcsec(5.0).unwrap();
        let mut a = SyntheticStarTracker::new(id, budget);
        let mut b = SyntheticStarTracker::new(id, budget);
        let q_truth = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), 0.3);
        let truth = truth_with_attitude(q_truth);
        for step in 0..200_u64 {
            let ra = a
                .measure(&truth, StepIndex::new(step), 0xDEAD_BEEF)
                .unwrap();
            let rb = b
                .measure(&truth, StepIndex::new(step), 0xDEAD_BEEF)
                .unwrap();
            let SensorMeasurement::StarTracker {
                attitude_eci_to_body: qa,
            } = ra
            else {
                panic!();
            };
            let SensorMeasurement::StarTracker {
                attitude_eci_to_body: qb,
            } = rb
            else {
                panic!();
            };
            let qa = qa.into_inner();
            let qb = qb.into_inner();
            assert_eq!(qa.w.to_bits(), qb.w.to_bits());
            assert_eq!(qa.i.to_bits(), qb.i.to_bits());
            assert_eq!(qa.j.to_bits(), qb.j.to_bits());
            assert_eq!(qa.k.to_bits(), qb.k.to_bits());
        }
    }
}
