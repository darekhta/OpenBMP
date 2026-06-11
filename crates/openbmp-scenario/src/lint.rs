//! TOML linting that runs *before* serde deserialisation so the user
//! sees the most user-friendly error first:
//!
//! 1. Dimensional fields without an explicit unit suffix
//!    (`docs/scenario-format.md § Units`).
//! 2. 3-tuple numeric fields without an explicit frame infix
//!    (`docs/scenario-format.md § Frames`).
//!
//! Both checks fold into a single recursive walk over the parsed
//! `toml::Value`.

use crate::error::ScenarioError;

const UNIT_SUFFIXES: &[&str] = &[
    "_dt_s",
    "_m_s2",
    "_m_s",
    "_m3_s2",
    "_n_m",
    "_n_s",
    "_rad2_s2",
    "_rad_s",
    "_hz",
    "_kg_m3",
    "_kg_m2",
    "_kg_per_s",
    "_per_s",
    "_kg",
    "_w_m2",
    "_m3",
    "_m2",
    "_m",
    "_n",
    "_pa_s",
    "_pa",
    "_sfu",
    "_rad",
    "_h",
    "_s",
    "_k",
    "_deg",
    "_g",
];

const FRAME_INFIXES: &[&str] = &["_eci_", "_ecef_", "_ned_", "_enu_", "_body_"];

/// Dimensionless suffixes that declare component ordering rather than
/// physical units.
const DIMENSIONLESS_COMPONENT_ORDER_SUFFIXES: &[&str] = &["_xyzw"];

/// Top-level lint entry point.
///
/// # Errors
///
/// Returns the first unit / frame suffix failure encountered.
pub(crate) fn lint(value: &toml::Value) -> Result<(), ScenarioError> {
    walk("$", None, value)
}

fn walk(path: &str, parent_key: Option<&str>, value: &toml::Value) -> Result<(), ScenarioError> {
    if let Some(key) = parent_key {
        check_rare_event_limit_state_terms(path, key, value)?;
        check_dimensional_field(path, key, value)?;
    }
    match value {
        toml::Value::Table(table) => {
            for (key, child) in table {
                let child_path = format!("{path}.{key}");
                walk(&child_path, Some(key), child)?;
            }
        }
        toml::Value::Array(values) => {
            for (index, child) in values.iter().enumerate() {
                walk(&format!("{path}[{index}]"), None, child)?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn check_rare_event_limit_state_terms(
    path: &str,
    key: &str,
    value: &toml::Value,
) -> Result<(), ScenarioError> {
    if !path.starts_with("$.monte_carlo.limit_state") {
        return Ok(());
    }
    if contains_banned_rare_event_term(key) {
        return Err(ScenarioError::UnsupportedValue {
            field: path.to_owned(),
            value: key.to_owned(),
        });
    }
    if let toml::Value::String(text) = value
        && contains_banned_rare_event_term(text)
    {
        return Err(ScenarioError::UnsupportedValue {
            field: path.to_owned(),
            value: text.clone(),
        });
    }
    Ok(())
}

fn contains_banned_rare_event_term(text: &str) -> bool {
    let normalized = text.to_ascii_lowercase().replace('_', "-");
    [
        "ground-aimpoint",
        "aimpoint",
        "target",
        "cep",
        "impact",
        "miss-distance",
    ]
    .iter()
    .any(|term| normalized.contains(term))
}

fn check_dimensional_field(
    path: &str,
    key: &str,
    value: &toml::Value,
) -> Result<(), ScenarioError> {
    // `[fc]` block — typed validation in
    // `FcConfig::validate` covers the FC tuning scalars and per-axis
    // gain triples. Bypass the workspace-level unit/frame lints for
    // any path under `$.fc`.
    if path.starts_with("$.fc") {
        return Ok(());
    }
    // v3-only `[schedule]` and `[multi_body]` blocks — typed
    // validation in `ScheduleConfig::validate` /
    // `MultiBodyConfig::validate` covers field-internal invariants
    // (rate-group divisor, momentum-conservation flag, etc.). The
    // workspace-level unit/frame lint is bypassed for the same reason
    // it is bypassed for `$.fc`: the typed validate methods are the
    // authority for these blocks.
    if path.starts_with("$.schedule") || path.starts_with("$.multi_body") {
        return Ok(());
    }
    if is_numeric_value(value)
        && !is_dimensionless_key(path, key)
        && !key_has_unit_suffix(key)
        && !key_has_dimensionless_component_order_suffix(key)
    {
        return Err(ScenarioError::MissingUnitSuffix {
            field: path.to_owned(),
        });
    }
    if is_numeric_3vector(value) && !key_has_frame_infix(key) && !is_frame_exempt_3vector(path, key)
    {
        return Err(ScenarioError::MissingFrameSuffix {
            field: path.to_owned(),
        });
    }
    if is_numeric_4vector(value)
        && !key_has_dimensionless_component_order_suffix(key)
        && !key_has_frame_infix(key)
    {
        return Err(ScenarioError::MissingFrameSuffix {
            field: path.to_owned(),
        });
    }
    Ok(())
}

fn is_numeric_value(value: &toml::Value) -> bool {
    match value {
        toml::Value::Integer(_) | toml::Value::Float(_) => true,
        toml::Value::Array(values) => !values.is_empty() && values.iter().all(is_numeric_value),
        _ => false,
    }
}

fn is_numeric_3vector(value: &toml::Value) -> bool {
    matches!(value, toml::Value::Array(values)
        if values.len() == 3 && values.iter().all(is_numeric_value))
}

/// Whether the dimensional 3-vector at `(path, key)` is exempt from
/// the frame-infix check. The `[wind]` block carries `intensity_m_s` and
/// `length_scale_m` arrays, both indexed by Dryden
/// axis (u, v, w) rather than a spatial frame; the frame-infix
/// requirement does not apply.
fn is_frame_exempt_3vector(path: &str, key: &str) -> bool {
    matches!(
        (path, key),
        ("$.wind.intensity_m_s", "intensity_m_s") | ("$.wind.length_scale_m", "length_scale_m")
    ) || (path.starts_with("$.landing_footprint.monte_carlo") && key == "confidence_levels")
        || (path.starts_with("$.propulsion.feed_network")
            && (path.contains(".oxidizer_pump") || path.contains(".fuel_pump"))
            && matches!(key, "head_coefficients" | "efficiency_coefficients"))
}

fn is_numeric_4vector(value: &toml::Value) -> bool {
    matches!(value, toml::Value::Array(values)
        if values.len() == 4 && values.iter().all(is_numeric_value))
}

fn is_dimensionless_key(path: &str, key: &str) -> bool {
    if matches!(
        key,
        "scenario"
            | "seed"
            | "worker_index"
            | "worker_count"
            | "chemistry_substeps"
            | "material_substeps"
            | "nonlinear_max_iter"
            | "rtol"
            | "atol"
            | "j2"
            | "year"
            | "day_of_year"
            | "ap_average"
            | "ap_current_3h"
            // Mission-block trigger fields: a mass fraction
            // ratio in [0, 1].
            | "remaining"
    ) {
        return true;
    }

    // Bending-mode fields: `damping_ratio` is a dimensionless modal damping
    // ratio; `slope_at_engine` / `slope_at_gyro` are dimensionless mode-shape
    // slopes (the frequency and modal mass carry `_hz` / `_kg` suffixes).
    if path.starts_with("$.vehicle.bending")
        && matches!(key, "damping_ratio" | "slope_at_engine" | "slope_at_gyro")
    {
        return true;
    }

    // Effector-block fields are unit-agnostic command
    // magnitudes: their concrete unit depends on the effector kind
    // (rad, m, fraction, etc.). Keep this exemption path-scoped so
    // arbitrary extension tables do not accidentally accept unlabeled
    // dimensional fields named `min`, `max`, or `value`.
    if path.starts_with("$.vehicle.assembly.effectors")
        && matches!(
            key,
            "before"
                | "after"
                | "start"
                | "end"
                | "value"
                | "factor"
                | "min"
                | "max"
                | "deadband"
                | "at"
                | "to"
                | "initial_position"
        )
    {
        return true;
    }

    // Engine-block fields. `throttle_unit` is dimensionless
    // by convention; `at_throttle` is the same scalar; `factor` is a
    // dimensionless multiplier on thrust.
    if path.starts_with("$.vehicle.assembly.engines")
        && matches!(
            key,
            "throttle_unit"
                | "at_throttle"
                | "factor"
                | "min_throttle_unit"
                | "isp_throttle_falloff"
                | "oxidizer_fuel_ratio"
        )
    {
        return true;
    }

    // Tank-block fields. `initial_fill_fraction` is a
    // ratio in [0, 1]; `damping_ratio_zeta`, `base_damping_ratio_zeta`,
    // and `damping_increment_zeta` are dimensionless damping ratios.
    if path.starts_with("$.vehicle.assembly.tanks")
        && matches!(
            key,
            "initial_fill_fraction"
                | "damping_ratio_zeta"
                | "base_damping_ratio_zeta"
                | "damping_increment_zeta"
                | "gas_gamma"
        )
    {
        return true;
    }

    if path.starts_with("$.propulsion.motor.grain")
        && matches!(
            key,
            "segments" | "expansion_ratio" | "burn_rate_a" | "burn_rate_n" | "gamma" | "web_steps"
        )
    {
        return true;
    }

    if path.starts_with("$.propulsion.thermochem") && key == "mixture_ratio" {
        return true;
    }

    if path.starts_with("$.propulsion.feed_network")
        && matches!(key, "valve_discharge_coefficient" | "valve_open_fraction")
    {
        return true;
    }
    if path.starts_with("$.propulsion.feed_network")
        && matches!(
            key,
            "oxidizer_valve_discharge_coefficient"
                | "fuel_valve_discharge_coefficient"
                | "oxidizer_open_fraction"
                | "fuel_open_fraction"
        )
    {
        return true;
    }
    if path.starts_with("$.propulsion.feed_network")
        && path.contains(".controller")
        && matches!(
            key,
            "target_mixture_ratio"
                | "pressure_proportional_gain_per_pa"
                | "pressure_integral_gain_per_pa_s"
                | "pressure_integral_limit_pa_s"
                | "mixture_proportional_gain"
                | "mixture_integral_gain_per_s"
                | "mixture_integral_limit_s"
                | "min_open_fraction"
                | "max_open_fraction"
                | "max_open_fraction_slew_per_s"
        )
    {
        return true;
    }
    if path.starts_with("$.propulsion.feed_network")
        && (path.contains(".oxidizer_pump") || path.contains(".fuel_pump"))
        && matches!(
            key,
            "design_efficiency"
                | "specific_speed"
                | "head_coefficients"
                | "efficiency_coefficients"
                | "cavitation_head_multiplier"
        )
    {
        return true;
    }
    if path.starts_with("$.propulsion.feed_network")
        && (path.contains(".oxidizer_line") || path.contains(".fuel_line"))
        && key == "segment_count"
    {
        return true;
    }
    if path.starts_with("$.propulsion.pogo") && key == "mode_damping_ratio" {
        return true;
    }
    if path.starts_with("$.propulsion.faults.rules") && key == "start_step" {
        return true;
    }
    if path.starts_with("$.propulsion.faults.mixture_ratio_runaway_rules") && key == "start_step" {
        return true;
    }
    if (path.starts_with("$.propulsion.faults.rules")
        || path.starts_with("$.propulsion.faults.cavitation_rules"))
        && path.contains(".fault")
        && matches!(key, "at_throttle" | "factor")
    {
        return true;
    }

    if path.starts_with("$.staging_analysis") && matches!(key, "structural_coefficient") {
        return true;
    }

    // Aero buildup grids and shape parameters. The grid bounds are
    // dimensionless for Mach and degrees by the parent table name for
    // alpha; typed validation owns the ranges.
    if path.starts_with("$.aero.buildup")
        && matches!(
            key,
            "min" | "max" | "steps" | "fineness" | "count" | "thickness_ratio"
        )
    {
        return true;
    }
    if path.starts_with("$.aero.method")
        && matches!(
            key,
            "cp_max"
                | "gamma"
                | "accommodation_normal"
                | "accommodation_tangential"
                | "mach_handoff"
                | "mach_lo"
                | "mach_hi"
                | "sigma"
                | "kn_lo"
                | "kn_hi"
        )
    {
        return true;
    }

    if path.starts_with("$.aerothermal")
        && matches!(key, "n_nodes" | "gas_yield_fraction" | "lewis_number")
    {
        return true;
    }

    // Contact-block fields. Coulomb friction coefficient is
    // dimensionless; the smoothing speed carries `_m_s`.
    if path.starts_with("$.contact") && matches!(key, "friction_coefficient" | "substeps") {
        return true;
    }

    // Recovery-block fields. `c_d`, `drogue_c_d`, and
    // `main_c_d` are dimensionless drag coefficients per Knacke
    // 1992 Chapter 5.
    if path.starts_with("$.vehicle.assembly.recovery")
        && matches!(key, "c_d" | "drogue_c_d" | "main_c_d")
    {
        return true;
    }

    // Entry-profile fields. `lift_to_drag_ratio` is dimensionless
    // aerodynamic L/D by convention.
    if path.starts_with("$.entry_profile") && key == "lift_to_drag_ratio" {
        return true;
    }

    // Realtime target factor is dimensionless:
    // simulation seconds per wall-clock second.
    if path.starts_with("$.realtime") && key == "target_rtf" {
        return true;
    }

    // Landing-footprint Monte Carlo fields. Sample counts,
    // confidence levels, and multiplicative wind-scale uncertainty are
    // dimensionless; nested threshold units are defined by the selected
    // metric, and typed validation owns their ranges.
    if path.starts_with("$.landing_footprint.monte_carlo")
        && matches!(
            key,
            "samples"
                | "confidence_levels"
                | "speed_scale_sigma"
                | "uncertainty_class"
                | "epistemic_samples"
                | "aleatory_samples"
                | "metric"
                | "threshold"
                | "minimum_probability"
        )
    {
        return true;
    }

    // Top-level rare-event Monte-Carlo manifests are synthetic-only
    // metadata. Typed validation owns the supported ranges and the
    // consumer-agreement lint above owns forbidden target vocabulary.
    if path.starts_with("$.monte_carlo")
        && matches!(
            key,
            "seed"
                | "kind"
                | "label"
                | "dimension"
                | "beta"
                | "samples_per_level"
                | "conditional_probability"
                | "max_levels"
                | "proposal_sigma"
                | "dimension_id"
                | "samples"
                | "elite_fraction"
                | "iterations"
                | "smoothing"
                | "min_std_dev"
        )
    {
        return true;
    }

    // Mission/scenario-script effector overrides use the same
    // unit-agnostic command scalar as the target effector.
    if is_event_path(path) && path.ends_with(".action.command") && key == "command" {
        return true;
    }

    // Mission/scenario-script engine commands carry a typed payload
    // including a dimensionless `throttle_unit` and lifecycle bools
    // (the bools never trip this lint, but `throttle_unit` would
    // without an exemption).
    is_event_path(path)
        && path
            .strip_suffix(".throttle_unit")
            .is_some_and(|parent| parent.ends_with(".action.command"))
        && key == "throttle_unit"
}

fn is_event_path(path: &str) -> bool {
    path.starts_with("$.mission.events") || path.starts_with("$.scenario_script.events")
}

fn key_has_unit_suffix(key: &str) -> bool {
    UNIT_SUFFIXES.iter().any(|suffix| key.ends_with(suffix))
}

fn key_has_frame_infix(key: &str) -> bool {
    FRAME_INFIXES.iter().any(|infix| key.contains(infix))
}

fn key_has_dimensionless_component_order_suffix(key: &str) -> bool {
    DIMENSIONLESS_COMPONENT_ORDER_SUFFIXES
        .iter()
        .any(|suffix| key.ends_with(suffix))
}
