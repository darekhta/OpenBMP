//! Synthetic magnetometer.
//!
//! Per body axis the measurement transformation is:
//!
//! ```text
//! field_biased = soft_iron · field_body_truth + hard_iron
//! field_meas   = field_biased + N(0, σ_body)
//! ```
//!
//! `field_body_truth` is the WMM-truth geodetic-NED field at the
//! vehicle's position and time, rotated into the body frame by the
//! truth attitude. The runner-side adapter does that
//! pre-rotation and packs the result into
//! [`SensorTruth::magnetic_field_body_nt`]. The magnetometer then
//! applies its constant soft-iron / hard-iron biases plus per-axis
//! Gaussian receiver noise.
//!
//! # Component-id layout
//!
//! 3 reserved component slots per magnetometer: one per body axis.
//!
//! ```text
//! channel        component_id
//! noise_x        0
//! noise_y        1
//! noise_z        2
//! ```
//!
//! No OU bias drift on the body axes — the soft-
//! iron and hard-iron biases are constants. Temperature drift,
//! spin-induced bias, and full hysteresis are deferred to a later
//! phase.
//!
//! # Determinism
//!
//! Pure `f64`; locked operand order on the soft-iron matrix-vector
//! product and the hard-iron + noise sum. Per-component RNG
//! sub-streams via `DeterministicRng::for_sensor_component`.
//! Truth-bypass mode (`σ = 0`, identity soft-iron, zero hard-
//! iron) returns the truth-rotated body-frame field bit-equal.

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{DeterministicRng, SensorId, StepIndex};

use crate::error::SensorError;
use crate::noise::BoxMullerGaussian;
use crate::sensor::{SensorMeasurement, SensorTruth, SyntheticSensor, require_truth_finite};

const SUB_NOISE_X: u32 = 0;
const SUB_NOISE_Y: u32 = 1;
const SUB_NOISE_Z: u32 = 2;

const NOISE_IDS: [u32; 3] = [SUB_NOISE_X, SUB_NOISE_Y, SUB_NOISE_Z];

/// Per-axis isotropic noise budget for the magnetometer.
///
/// All values are in nT (or dimensionless for `soft_iron_body`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MagnetometerNoiseBudget {
    /// Per-axis Gaussian-noise stddev on the body-frame field
    /// magnitude (nT).
    pub sigma_body_nt: [f64; 3],
    /// Constant body-frame hard-iron offset (nT).
    pub hard_iron_body_nt: [f64; 3],
    /// Constant body-frame soft-iron 3×3 matrix (dimensionless).
    /// Identity = no soft-iron distortion. Row-major: row `i` is
    /// the body-frame component-`i` linear combination of the
    /// truth components.
    pub soft_iron_body: [[f64; 3]; 3],
}

impl MagnetometerNoiseBudget {
    /// Construct after validating the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidParameter`] for negative
    /// sigma; [`SensorError::NonFinite`] for any non-finite value.
    pub fn new(
        sigma_body_nt: [f64; 3],
        hard_iron_body_nt: [f64; 3],
        soft_iron_body: [[f64; 3]; 3],
    ) -> Result<Self, SensorError> {
        for v in sigma_body_nt.iter().chain(hard_iron_body_nt.iter()) {
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "magnetometer noise budget contains a NaN or infinite value",
                });
            }
        }
        for row in soft_iron_body {
            for v in row {
                if !v.is_finite() {
                    return Err(SensorError::NonFinite {
                        reason: "magnetometer soft_iron_body contains a NaN or infinite value",
                    });
                }
            }
        }
        for v in sigma_body_nt {
            if v < 0.0 {
                return Err(SensorError::InvalidParameter {
                    reason: "magnetometer sigma_body_nt must be non-negative",
                });
            }
        }
        Ok(Self {
            sigma_body_nt,
            hard_iron_body_nt,
            soft_iron_body,
        })
    }

    /// Identity-soft-iron, zero-hard-iron, zero-noise budget. Used
    /// by the truth-bypass tests.
    #[must_use]
    pub fn truth_bypass() -> Self {
        Self {
            sigma_body_nt: [0.0; 3],
            hard_iron_body_nt: [0.0; 3],
            soft_iron_body: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

/// Synthetic magnetometer.
#[derive(Clone, Debug)]
pub struct SyntheticMagnetometer {
    sensor_id: SensorId,
    budget: MagnetometerNoiseBudget,
    soft_iron: Matrix3<f64>,
    hard_iron: Vector3<f64>,
    gauss: BoxMullerGaussian,
}

impl SyntheticMagnetometer {
    /// Construct from a sensor id and a fully-validated noise
    /// budget.
    #[must_use]
    pub fn new(sensor_id: SensorId, budget: MagnetometerNoiseBudget) -> Self {
        let s = budget.soft_iron_body;
        let soft_iron = Matrix3::new(
            s[0][0], s[0][1], s[0][2], s[1][0], s[1][1], s[1][2], s[2][0], s[2][1], s[2][2],
        );
        let hard_iron = Vector3::new(
            budget.hard_iron_body_nt[0],
            budget.hard_iron_body_nt[1],
            budget.hard_iron_body_nt[2],
        );
        Self {
            sensor_id,
            budget,
            soft_iron,
            hard_iron,
            gauss: BoxMullerGaussian::new(),
        }
    }

    /// Read-only access to the budget.
    #[must_use]
    pub const fn budget(&self) -> &MagnetometerNoiseBudget {
        &self.budget
    }
}

impl SyntheticSensor for SyntheticMagnetometer {
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
        let truth_body = truth.magnetic_field_body_nt;
        // Soft-iron: linear mix of truth components (locked-order
        // matrix-vector product). Hard-iron: constant offset.
        let biased = self.soft_iron * truth_body + self.hard_iron;
        let mut reported = [0.0_f64; 3];
        for axis in 0..3 {
            let mut rng = DeterministicRng::for_sensor_component(
                scenario_seed,
                step,
                self.sensor_id,
                NOISE_IDS[axis],
            );
            let noise = self
                .gauss
                .sample(&mut rng, 0.0, self.budget.sigma_body_nt[axis])?;
            let v = biased[axis] + noise;
            if !v.is_finite() {
                return Err(SensorError::NonFinite {
                    reason: "magnetometer reported component is non-finite",
                });
            }
            reported[axis] = v;
        }
        Ok(SensorMeasurement::Magnetometer {
            field_body_nt: Vector3::new(reported[0], reported[1], reported[2]),
            hard_iron_body_nt: self.hard_iron,
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
    use openbmp_core::{Position3, SimTime, Velocity3};

    fn truth_at(field_body_nt: Vector3<f64>) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: nalgebra::UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: 101_325.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: field_body_nt,
            time: SimTime::ZERO,
        }
    }

    #[test]
    fn rejects_negative_sigma() {
        let res = MagnetometerNoiseBudget::new(
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        );
        assert!(matches!(res, Err(SensorError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_non_finite_soft_iron() {
        let res = MagnetometerNoiseBudget::new(
            [0.0; 3],
            [0.0; 3],
            [[1.0, f64::NAN, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        );
        assert!(matches!(res, Err(SensorError::NonFinite { .. })));
    }

    #[test]
    fn truth_bypass_returns_field_bit_equal() {
        let id = SensorId::from_path("sensors.mag.test");
        let mut m = SyntheticMagnetometer::new(id, MagnetometerNoiseBudget::truth_bypass());
        let truth = truth_at(Vector3::new(20_000.0, -3_000.0, 45_000.0));
        let r = m.measure(&truth, StepIndex::new(0), 0xC0FF_EE00).unwrap();
        let SensorMeasurement::Magnetometer { field_body_nt, .. } = r else {
            panic!("expected Magnetometer");
        };
        assert_eq!(field_body_nt.x.to_bits(), 20_000.0_f64.to_bits());
        assert_eq!(field_body_nt.y.to_bits(), (-3_000.0_f64).to_bits());
        assert_eq!(field_body_nt.z.to_bits(), 45_000.0_f64.to_bits());
    }

    #[test]
    fn hard_iron_only_offsets_truth_by_constant() {
        let id = SensorId::from_path("sensors.mag.hard_iron");
        let budget = MagnetometerNoiseBudget::new(
            [0.0; 3],
            [100.0, -50.0, 30.0],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let mut m = SyntheticMagnetometer::new(id, budget);
        let truth = truth_at(Vector3::new(20_000.0, 0.0, 45_000.0));
        let r = m.measure(&truth, StepIndex::new(0), 0).unwrap();
        let SensorMeasurement::Magnetometer {
            field_body_nt,
            hard_iron_body_nt,
        } = r
        else {
            panic!("expected Magnetometer");
        };
        assert_eq!(field_body_nt.x.to_bits(), 20_100.0_f64.to_bits());
        assert_eq!(field_body_nt.y.to_bits(), (-50.0_f64).to_bits());
        assert_eq!(field_body_nt.z.to_bits(), 45_030.0_f64.to_bits());
        assert_eq!(hard_iron_body_nt.x.to_bits(), 100.0_f64.to_bits());
    }

    #[test]
    fn soft_iron_2x_scales_truth() {
        let id = SensorId::from_path("sensors.mag.soft_iron");
        let budget = MagnetometerNoiseBudget::new(
            [0.0; 3],
            [0.0; 3],
            [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
        )
        .unwrap();
        let mut m = SyntheticMagnetometer::new(id, budget);
        let truth = truth_at(Vector3::new(100.0, -200.0, 300.0));
        let r = m.measure(&truth, StepIndex::new(0), 0).unwrap();
        let SensorMeasurement::Magnetometer { field_body_nt, .. } = r else {
            panic!("expected Magnetometer");
        };
        assert_eq!(field_body_nt.x.to_bits(), 200.0_f64.to_bits());
        assert_eq!(field_body_nt.y.to_bits(), (-400.0_f64).to_bits());
        assert_eq!(field_body_nt.z.to_bits(), 600.0_f64.to_bits());
    }

    #[test]
    fn empirical_std_within_5_percent_of_budget() {
        let id = SensorId::from_path("sensors.mag.long_run");
        let sigma = 50.0;
        let budget = MagnetometerNoiseBudget::new(
            [sigma, sigma, sigma],
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let mut m = SyntheticMagnetometer::new(id, budget);
        let truth = truth_at(Vector3::new(20_000.0, 0.0, 45_000.0));
        let n = 10_000_u64;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for step in 0..n {
            let r = m
                .measure(&truth, StepIndex::new(step), 0xBABE_CAFE)
                .unwrap();
            let SensorMeasurement::Magnetometer { field_body_nt, .. } = r else {
                panic!("expected Magnetometer");
            };
            let dx = field_body_nt.x - 20_000.0;
            sum += dx;
            sum_sq += dx * dx;
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
        let id = SensorId::from_path("sensors.mag.replay");
        let budget = MagnetometerNoiseBudget::new(
            [50.0, 50.0, 50.0],
            [100.0, -50.0, 30.0],
            [[1.0, 0.01, 0.0], [-0.01, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let mut a = SyntheticMagnetometer::new(id, budget);
        let mut b = SyntheticMagnetometer::new(id, budget);
        let truth = truth_at(Vector3::new(20_000.0, 0.0, 45_000.0));
        for step in 0..200_u64 {
            let ra = a
                .measure(&truth, StepIndex::new(step), 0xDEAD_BEEF)
                .unwrap();
            let rb = b
                .measure(&truth, StepIndex::new(step), 0xDEAD_BEEF)
                .unwrap();
            let SensorMeasurement::Magnetometer {
                field_body_nt: fa, ..
            } = ra
            else {
                panic!("expected");
            };
            let SensorMeasurement::Magnetometer {
                field_body_nt: fb, ..
            } = rb
            else {
                panic!("expected");
            };
            assert_eq!(fa.x.to_bits(), fb.x.to_bits());
            assert_eq!(fa.y.to_bits(), fb.y.to_bits());
            assert_eq!(fa.z.to_bits(), fb.z.to_bits());
        }
    }

    #[test]
    fn sigma_axis_y_does_not_shift_axis_x_stream() {
        // Per-component RNG independence.
        let id = SensorId::from_path("sensors.mag.indep");
        let budget_a = MagnetometerNoiseBudget::new(
            [50.0, 50.0, 50.0],
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let budget_b = MagnetometerNoiseBudget::new(
            [50.0, 999.0, 50.0],
            [0.0; 3],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        .unwrap();
        let mut a = SyntheticMagnetometer::new(id, budget_a);
        let mut b = SyntheticMagnetometer::new(id, budget_b);
        let truth = truth_at(Vector3::new(20_000.0, 0.0, 45_000.0));
        for step in 0..50_u64 {
            let ra = a
                .measure(&truth, StepIndex::new(step), 0x55AA_55AA)
                .unwrap();
            let rb = b
                .measure(&truth, StepIndex::new(step), 0x55AA_55AA)
                .unwrap();
            let SensorMeasurement::Magnetometer {
                field_body_nt: fa, ..
            } = ra
            else {
                panic!();
            };
            let SensorMeasurement::Magnetometer {
                field_body_nt: fb, ..
            } = rb
            else {
                panic!();
            };
            assert_eq!(fa.x.to_bits(), fb.x.to_bits());
        }
    }
}
