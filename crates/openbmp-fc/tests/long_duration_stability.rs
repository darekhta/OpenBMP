//! Long-duration estimator stability checks.

#![allow(clippy::expect_used, clippy::unwrap_used)]

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::SimTime;
use openbmp_fc::estimator::{Ekf, EkfParams, Estimator};
use openbmp_fc::topics::{BarometerSample, GnssSample, ImuSample, MagnetometerSample};
use openbmp_physics::atmosphere::USSA76_SEA_LEVEL_PRESSURE_PA;
use openbmp_physics::gravity::STANDARD_GRAVITY_M_S2;

#[test]
fn ekf_sixty_second_synthetic_truth_run_stays_bounded() {
    let mut ekf = Ekf::new(EkfParams::default());
    ekf.seed(
        Vector3::zeros(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
    );
    let initial_cov = ekf.covariance_max_diag();
    let dt_s = 0.01;
    let ticks = 6_000_u64;

    for k in 0..ticks {
        let now = SimTime::from_seconds(f64::from(u32::try_from(k).unwrap()) * dt_s);
        ekf.update_imu(&ImuSample {
            time: now,
            gyro_rad_s: Vector3::zeros(),
            accel_m_s2: Vector3::new(0.0, 0.0, STANDARD_GRAVITY_M_S2),
            healthy: true,
        })
        .unwrap();
        ekf.predict(dt_s).unwrap();
        if k % 10 == 0 {
            ekf.update_gnss(&GnssSample {
                time: now,
                position_eci_m: Vector3::zeros(),
                velocity_eci_m_s: Vector3::zeros(),
                position_bias_eci_m: Vector3::zeros(),
                healthy: true,
            })
            .unwrap();
            ekf.update_baro(&BarometerSample {
                time: now,
                pressure_pa: USSA76_SEA_LEVEL_PRESSURE_PA,
                bias_pa: 0.0,
                healthy: true,
            })
            .unwrap();
            ekf.update_mag(&MagnetometerSample {
                time: now,
                field_body_nt: Vector3::new(0.0, 0.0, 30_000.0),
                hard_iron_body_nt: Vector3::zeros(),
                healthy: true,
            })
            .unwrap();
        }

        let position = ekf.position();
        assert!(position.position_eci_m.iter().all(|v| v.is_finite()));
        assert!(position.velocity_eci_m_s.iter().all(|v| v.is_finite()));
        assert!(ekf.covariance_max_diag() <= initial_cov * 100.0);
    }

    let attitude = ekf.attitude();
    let attitude_error_rad = 2.0
        * (attitude.q_body_to_eci_xyzw[0] * attitude.q_body_to_eci_xyzw[0]
            + attitude.q_body_to_eci_xyzw[1] * attitude.q_body_to_eci_xyzw[1]
            + attitude.q_body_to_eci_xyzw[2] * attitude.q_body_to_eci_xyzw[2])
            .sqrt();
    assert!(attitude_error_rad < 0.1);
}
