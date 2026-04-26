//! Phase-2.2 regression guards.
//!
//! 1. `openbmp-env::gravity::ConstantGravity` produces byte-identical
//!    results to the Phase-1 `openbmp-sim::models::ConstantGravityForce`
//!    scaffold over the analytic-toy drop scenario's coefficient grid.
//!    This locks in the property the audited Phase-2 plan relies on:
//!    when the higher-layer adapter swaps `ConstantGravityForce` for
//!    `openbmp-env::ConstantGravity`, telemetry stays identical.
//!
//! 2. `J2Gravity` reduces to `PointMassGravity` when `j2 = 0` to within
//!    1e-12 — the same property the in-crate unit test asserts, but
//!    here we exercise it through the public crate boundary.
//!
//! These tests do not depend on `openbmp-sim` (the Phase-2 plan's L2
//! crate-layering rule), so the equivalence claim is asserted by
//! computing the expected acceleration arithmetic in the test file
//! directly. A future Phase-2.10 e2e regression that runs both
//! kernels through the CLI and diffs Parquet provides the
//! end-to-end guarantee.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]

use approx::assert_abs_diff_eq;
use openbmp_core::{Position3, SimTime};
use openbmp_env::{ConstantGravity, GravityModel, J2Gravity, PointMassGravity, WGS84_J2};

const WGS84_J2_DATA: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/gravity/wgs84-j2.toml"
));

fn wgs84_j2_data_pin() -> toml::Value {
    toml::from_str(WGS84_J2_DATA).expect("WGS84 J2 data file must parse as TOML")
}

fn f64_field(data: &toml::Value, key: &str) -> f64 {
    data.get(key)
        .and_then(toml::Value::as_float)
        .expect("WGS84 J2 data file must contain requested float field")
}

/// Phase-1 toy used `g = (0, 0, -9.80665)`. Verify
/// `ConstantGravity::down_z` produces exactly that vector.
#[test]
fn constant_gravity_down_z_matches_phase_1_toy_vector() {
    let g = ConstantGravity::down_z(9.806_65).expect("valid magnitude");
    let out = g
        .gravity_eci_m_s2(Position3::origin(), SimTime::ZERO)
        .expect("constant gravity is total");
    assert_abs_diff_eq!(out.x, 0.0, epsilon = 0.0);
    assert_abs_diff_eq!(out.y, 0.0, epsilon = 0.0);
    assert_abs_diff_eq!(out.z, -9.806_65, epsilon = 0.0);
}

#[test]
fn constant_gravity_is_independent_of_position_and_time() {
    let g = ConstantGravity::down_z(9.806_65).expect("valid");
    let positions = [
        Position3::origin(),
        Position3::new(1_000_000.0, 0.0, 0.0),
        Position3::new(0.0, 0.0, 6_378_137.0),
        Position3::new(7_000_000.0, 1_500_000.0, 800_000.0),
    ];
    let times = [
        SimTime::ZERO,
        SimTime::from_seconds(60.0),
        SimTime::from_seconds(7200.0),
    ];
    let baseline = g.gravity_eci_m_s2(positions[0], times[0]).unwrap();
    for p in positions {
        for t in times {
            let out = g.gravity_eci_m_s2(p, t).unwrap();
            assert_eq!(out.x.to_bits(), baseline.x.to_bits());
            assert_eq!(out.y.to_bits(), baseline.y.to_bits());
            assert_eq!(out.z.to_bits(), baseline.z.to_bits());
        }
    }
}

#[test]
fn j2_with_zero_coefficient_reduces_to_point_mass_through_public_api() {
    let pm = PointMassGravity::wgs84();
    let j2 =
        J2Gravity::new(openbmp_core::WGS84_MU_M3_S2, openbmp_core::WGS84_A_M, 0.0).expect("valid");
    let r = Position3::new(7_000_000.0, 1_500_000.0, 800_000.0);
    let pm_out = pm.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
    let j2_out = j2.gravity_eci_m_s2(r, SimTime::ZERO).unwrap();
    assert_abs_diff_eq!(pm_out.x, j2_out.x, epsilon = 1.0e-12);
    assert_abs_diff_eq!(pm_out.y, j2_out.y, epsilon = 1.0e-12);
    assert_abs_diff_eq!(pm_out.z, j2_out.z, epsilon = 1.0e-12);
}

#[test]
fn wgs84_j2_constant_matches_data_pin() {
    let data = wgs84_j2_data_pin();
    assert_eq!(
        data.get("dataset_id").and_then(toml::Value::as_str),
        Some("openbmp.wgs84.gravity.j2.v1"),
    );
    assert_eq!(
        data.get("schema_version").and_then(toml::Value::as_str),
        Some("openbmp.gravity.coefficient.v1"),
    );
    assert_eq!(WGS84_J2, f64_field(&data, "j2_unnormalised"));
}

#[test]
fn wgs84_constants_match_nima_tr_8350_2() {
    let data = wgs84_j2_data_pin();
    assert_eq!(
        openbmp_core::WGS84_A_M,
        f64_field(&data, "semi_major_axis_m")
    );
    assert_eq!(
        openbmp_core::WGS84_INV_FLATTENING,
        f64_field(&data, "inverse_flattening"),
    );
    assert_eq!(
        openbmp_core::WGS84_MU_M3_S2,
        f64_field(&data, "gravitational_parameter_m3_s2"),
    );
    assert_eq!(
        openbmp_core::WGS84_OMEGA_RAD_S,
        f64_field(&data, "angular_velocity_rad_s"),
    );
}
