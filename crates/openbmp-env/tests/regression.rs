//! Phase-2.2 + 2.3 regression guards.
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
//! 3. The compile-time WGS84 J2 + USSA76 constants and the USSA76
//!    layer table match their TOML data pins under
//!    `data/gravity/wgs84-j2.toml` and
//!    `data/atmosphere/us_standard_1976.toml`. This is the same shape
//!    the Phase-2.10 `openbmp check-provenance` walk will perform; we
//!    do it locally now so a typo in either source flips a CI gate.
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
use openbmp_env::{
    ConstantGravity, GravityModel, J2Gravity, PointMassGravity, UsStandard1976, WGS84_J2,
    atmosphere::us_standard_1976::{
        USSA76_G0_M_S2, USSA76_GAMMA_AIR, USSA76_MAX_GEOMETRIC_M, USSA76_MAX_GEOPOTENTIAL_M,
        USSA76_MOLAR_MASS_AIR_KG_KMOL, USSA76_REFERENCE_RADIUS_M, USSA76_UNIVERSAL_GAS_CONSTANT,
    },
};

const WGS84_J2_DATA: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/gravity/wgs84-j2.toml"
));

const USSA76_DATA: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/atmosphere/us_standard_1976.toml"
));

fn wgs84_j2_data_pin() -> toml::Value {
    toml::from_str(WGS84_J2_DATA).expect("WGS84 J2 data file must parse as TOML")
}

fn ussa76_data_pin() -> toml::Value {
    toml::from_str(USSA76_DATA).expect("USSA76 data file must parse as TOML")
}

fn f64_field(data: &toml::Value, key: &str) -> f64 {
    data.get(key)
        .and_then(toml::Value::as_float)
        .expect("data file must contain requested float field")
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

// ---------------------------------------------------------------------
// USSA76 data-pin regression
// ---------------------------------------------------------------------

#[test]
fn ussa76_dataset_id_and_schema_version_match() {
    let data = ussa76_data_pin();
    assert_eq!(
        data.get("dataset_id").and_then(toml::Value::as_str),
        Some("openbmp.atmosphere.us_standard_1976.v1"),
    );
    assert_eq!(
        data.get("schema_version").and_then(toml::Value::as_str),
        Some("openbmp.atmosphere.layered_model.v1"),
    );
}

#[test]
fn ussa76_constants_match_noaa_st_76_1562() {
    let data = ussa76_data_pin();
    assert_eq!(USSA76_G0_M_S2, f64_field(&data, "standard_gravity_m_s2"));
    assert_eq!(
        USSA76_UNIVERSAL_GAS_CONSTANT,
        f64_field(&data, "universal_gas_constant_j_kmol_k"),
    );
    assert_eq!(
        USSA76_MOLAR_MASS_AIR_KG_KMOL,
        f64_field(&data, "mean_molecular_weight_air_kg_kmol"),
    );
    assert_eq!(
        USSA76_GAMMA_AIR,
        f64_field(&data, "ratio_of_specific_heats_air"),
    );
    assert_eq!(
        USSA76_REFERENCE_RADIUS_M,
        f64_field(&data, "reference_radius_m"),
    );
    assert_eq!(
        USSA76_MAX_GEOMETRIC_M,
        f64_field(&data, "max_geometric_altitude_m"),
    );
    assert_eq!(
        USSA76_MAX_GEOPOTENTIAL_M,
        f64_field(&data, "max_geopotential_altitude_m"),
    );
}

/// Verify each of the seven layer-base entries in the TOML pin
/// matches what the in-source layer table produces. The private
/// `LAYERS` array is exercised through the public
/// `sample_at_geopotential` API: at `h = h_b` the model returns
/// exactly `(T_b, p_b)` from the standard, and the layer's lapse
/// rate is recovered from a small offset query.
#[test]
fn ussa76_layer_table_matches_data_pin() {
    let data = ussa76_data_pin();
    let layers = data
        .get("layers")
        .and_then(toml::Value::as_array)
        .expect("USSA76 data pin must contain a `layers` array");
    assert_eq!(layers.len(), 7, "USSA76 has 7 geopotential layers");

    let atm = UsStandard1976::new();
    for (expected_index, layer) in layers.iter().enumerate() {
        let pin_index = layer
            .get("index")
            .and_then(toml::Value::as_integer)
            .expect("each layer must have an `index`");
        let pin_index_usize =
            usize::try_from(pin_index).expect("layer `index` must be a non-negative integer");
        assert_eq!(
            pin_index_usize, expected_index,
            "layer indices must be sequential starting at 0",
        );

        let base_geopotential_m = f64_field(layer, "base_geopotential_m");
        let base_temperature_k = f64_field(layer, "base_temperature_k");
        let base_pressure_pa = f64_field(layer, "base_pressure_pa");
        let lapse_rate_k_per_m = f64_field(layer, "lapse_rate_k_per_m");

        // 1. T(h_b) and p(h_b) match the published constants exactly
        //    (within f64 representation precision).
        let s = atm
            .sample_at_geopotential(base_geopotential_m)
            .expect("layer base is inside USSA76 envelope");
        assert_eq!(
            s.temperature_k.to_bits(),
            base_temperature_k.to_bits(),
            "T_b mismatch at layer {expected_index} (h_b = {base_geopotential_m})",
        );
        assert_abs_diff_eq!(s.pressure_pa, base_pressure_pa, epsilon = 1.0e-9);

        // 2. The layer's lapse rate is recoverable from a 100-m
        //    offset query inside the same layer (every USSA76 layer
        //    is at least 4 km thick, so base + 100 stays in-layer).
        let offset_m = 100.0;
        let s_off = atm
            .sample_at_geopotential(base_geopotential_m + offset_m)
            .expect("base + 100 m is inside layer");
        let observed_lapse = (s_off.temperature_k - s.temperature_k) / offset_m;
        assert_abs_diff_eq!(observed_lapse, lapse_rate_k_per_m, epsilon = 1.0e-12);
    }
}
