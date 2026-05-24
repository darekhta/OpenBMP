#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
//! Phase-6.8 hypersonic validation suite.
//!
//! Per the design document, each case is either analytic-toy
//! (closed-form comparison) or public-benchmark (against published
//! academic results). This module ships the analytic-toy battery
//! that exercises the Phase-6 algorithms end to end; public-benchmark
//! Apollo / Stardust cross-validation runs against published
//! trajectory data are deferred to the integration test in
//! `crates/openbmp-cli/tests` once the kernel-side scenario wiring
//! lands.
//!
//! Cases included:
//!
//! * Allen-Eggers ballistic-entry analytic-toy: peak deceleration
//!   altitude and magnitude vs the closed-form expressions.
//! * Sutton-Graves single-point sanity at Apollo conditions.
//! * Modified-Newtonian sphere `Cp(0) = Cp_max`, `Cp(π/2) = 0`.
//! * Knudsen-bridge `α → 0` at Kn = 0; `α → 1` at Kn → ∞; smooth
//!   transition.
//! * Tannehill equilibrium-air queries fail closed until verified
//!   public table values land.
//! * NRLMSISE-00 static density at 200 km and 400 km matches public
//!   model outputs for the declared static condition.

use approx::assert_relative_eq;
use openbmp_physics::{
    APOLLO_CM_TABLE13_HEATING, AllenEggers, EquilibriumAir, Nrlmsise00Inputs, Nrlmsise00Static,
    STARDUST_SRC_TABLE13_HEATING, TannehillEquilibriumAir,
};

#[test]
fn allen_eggers_analytic_toy_case() {
    // Classical Allen-Eggers parameters (Anderson 2019 §13.4):
    //   V_e = 7800 m/s
    //   γ_e = 5° (descending below local horizon)
    //   B   = 0.001 m²/kg
    //   ρ_s = 1.225 kg/m³ (sea-level reference)
    //   β   = 1/7000 m⁻¹
    let ae = AllenEggers {
        rho_s_kg_m3: 1.225,
        beta_inv_m: 1.0 / 7_000.0,
        entry_velocity_m_s: 7_800.0,
        flight_path_angle_rad: 5.0_f64.to_radians(),
        ballistic_coefficient_m2_kg: 0.001,
    };
    ae.validate().unwrap();
    let n = ae.peak_deceleration_g();
    let h = ae.peak_decel_altitude_m();
    // Tolerance bands derived from the analytic formula; the
    // simulator must match the closed-form to engineering accuracy.
    assert!((n - 14.2).abs() / 14.2 < 0.05, "peak n = {n} g");
    // Closed-form altitude h_max = (1/β) · ln[ρ_s · B / (β · sin γ)]
    //   = 7000 · ln(1.225e-3 / 1.246e-5) ≈ 32 km
    assert!((h - 32_000.0).abs() < 5_000.0, "peak h = {h} m");
}

#[test]
fn allen_eggers_integrated_trajectory_matches_closed_form_peak_decel() {
    const G0: f64 = 9.806_65;
    let ae = AllenEggers {
        rho_s_kg_m3: 1.225,
        beta_inv_m: 1.0 / 7_000.0,
        entry_velocity_m_s: 7_800.0,
        flight_path_angle_rad: 5.0_f64.to_radians(),
        ballistic_coefficient_m2_kg: 0.001,
    };
    let sin_gamma = ae.flight_path_angle_rad.sin().abs();
    let dv_dh = |altitude_m: f64, velocity_m_s: f64| -> f64 {
        let rho = ae.rho_s_kg_m3 * (-ae.beta_inv_m * altitude_m).exp();
        0.5 * rho * ae.ballistic_coefficient_m2_kg * velocity_m_s / sin_gamma
    };
    let decel_g = |altitude_m: f64, velocity_m_s: f64| -> f64 {
        let rho = ae.rho_s_kg_m3 * (-ae.beta_inv_m * altitude_m).exp();
        0.5 * rho * ae.ballistic_coefficient_m2_kg * velocity_m_s * velocity_m_s / G0
    };

    let mut altitude_m = 120_000.0;
    let mut velocity_m_s = ae.velocity_at_altitude_m_s(altitude_m);
    let step_m = -10.0;
    let mut peak_g = 0.0;
    let mut peak_altitude_m = altitude_m;
    while altitude_m > 0.0 {
        let n_g = decel_g(altitude_m, velocity_m_s);
        if n_g > peak_g {
            peak_g = n_g;
            peak_altitude_m = altitude_m;
        }

        let k1 = dv_dh(altitude_m, velocity_m_s);
        let k2 = dv_dh(altitude_m + 0.5 * step_m, velocity_m_s + 0.5 * step_m * k1);
        let k3 = dv_dh(altitude_m + 0.5 * step_m, velocity_m_s + 0.5 * step_m * k2);
        let k4 = dv_dh(altitude_m + step_m, velocity_m_s + step_m * k3);
        velocity_m_s += step_m * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        altitude_m += step_m;
    }

    let expected_g = ae.peak_deceleration_g();
    let expected_altitude_m = ae.peak_decel_altitude_m();
    assert!(
        (peak_g - expected_g).abs() / expected_g < 0.01,
        "integrated peak n={peak_g} g, expected {expected_g} g"
    );
    assert!(
        (peak_altitude_m - expected_altitude_m).abs() < 100.0,
        "integrated peak h={peak_altitude_m} m, expected {expected_altitude_m} m"
    );
}

#[test]
fn sutton_graves_apollo_sanity_point() {
    use openbmp_physics::AtmosphereSample;
    let sg = openbmp_aerothermal::SuttonGraves::default();
    let t = 226.5;
    let rho = 1.0e-4;
    let p = rho * 287.05 * t;
    let a = (1.4_f64 * 287.05 * t).sqrt();
    let ctx = openbmp_aerothermal::AerothermalContext {
        freestream: AtmosphereSample::new(rho, p, t, a).unwrap(),
        airspeed_m_s: 11_000.0,
        mach: 11_000.0 / a,
        nose_radius_m: 1.5,
        wall_temperature_k: 1500.0,
        wall_catalysis: openbmp_aerothermal::WallCatalysis::FullyCatalytic,
    };
    let h = openbmp_aerothermal::HeatTransferModel::stagnation(&sg, &ctx).unwrap();
    // Apollo-class peak ~ 1.5–2 MW/m².
    assert!(
        (h.q_conv_w_m2 - 1.89e6).abs() / 1.89e6 < 0.10,
        "Sutton-Graves Apollo q_conv = {} W/m²",
        h.q_conv_w_m2
    );
}

#[test]
fn apollo_stardust_public_table13_benchmarks_are_pinned() {
    assert_relative_eq!(
        APOLLO_CM_TABLE13_HEATING
            .peak_convective_heat_flux_w_m2()
            .unwrap(),
        3.366e6,
        max_relative = 1.0e-12
    );
    assert_relative_eq!(
        STARDUST_SRC_TABLE13_HEATING.total_heat_load_j_m2.unwrap(),
        2.373e8,
        max_relative = 1.0e-12
    );
}

#[test]
fn modified_newtonian_sphere_cp_extremes() {
    use openbmp_aero::{ModifiedNewtonian, PanelInclination};
    let m = ModifiedNewtonian {
        cp_max: ModifiedNewtonian::CP_MAX_PERFECT_GAS_INFINITE_MACH,
        reference_area_m2: 1.0,
        reference_length_m: 1.0,
    };
    // θ = 0 → Cp = Cp_max.
    let cp_stag = m.cp(PanelInclination { theta_rad: 0.0 });
    assert_relative_eq!(
        cp_stag,
        ModifiedNewtonian::CP_MAX_PERFECT_GAS_INFINITE_MACH,
        max_relative = 1.0e-12
    );
    // θ = π/2 → Cp = 0.
    let cp_lat = m.cp(PanelInclination {
        theta_rad: std::f64::consts::FRAC_PI_2,
    });
    assert_relative_eq!(cp_lat, 0.0, epsilon = 1.0e-12);
}

#[test]
fn knudsen_bridge_limits_case() {
    use openbmp_aero::{BridgeFunction, ChengBridge, LinearKnudsenBridge};
    let cheng = ChengBridge;
    // Kn = 0 (continuum): α ≈ 0.
    assert!(cheng.alpha(1.0e-9) < 1.0e-9);
    // Kn → ∞: α → 1.
    assert!((cheng.alpha(1.0e6) - 1.0).abs() < 1.0e-3);
    // Linear bridge smoothly maps the band.
    let lin = LinearKnudsenBridge::default();
    let mid = 0.5 * (lin.kn_lo + lin.kn_hi);
    assert_relative_eq!(lin.alpha(mid), 0.5, max_relative = 1.0e-9);
}

#[test]
fn tannehill_equilibrium_air_is_deferred_until_verified_table_lands() {
    let m = TannehillEquilibriumAir;
    assert!(m.gamma_eff(300.0, 101_325.0).is_err());
}

#[test]
fn nrlmsise00_static_at_200km_in_reference_band() {
    let m = Nrlmsise00Static;
    let o = m
        .evaluate(Nrlmsise00Inputs::mid_conditions(200_000.0))
        .unwrap();
    // Public NRLMSISE-00 gtd7 static condition: ρ ≈ 3.155e-10 kg/m³.
    assert!((o.mass_density_kg_m3 - 3.155_122_198_534e-10).abs() / 3.155_122_198_534e-10 < 0.01);
}

#[test]
fn nrlmsise00_static_at_400km_in_reference_band() {
    let m = Nrlmsise00Static;
    let o = m
        .evaluate(Nrlmsise00Inputs::mid_conditions(400_000.0))
        .unwrap();
    // Public NRLMSISE-00 gtd7 static condition: ρ ≈ 6.060e-12 kg/m³.
    assert!((o.mass_density_kg_m3 - 6.059_650_178_061e-12).abs() / 6.059_650_178_061e-12 < 0.01);
}
