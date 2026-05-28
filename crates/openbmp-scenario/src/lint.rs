//! TOML linting that runs *before* serde deserialisation so the user
//! sees the most user-friendly error first:
//!
//! 1. Safety-limited terms in keys or short string values
//!    (`docs/safety-boundaries.md § Naming Rules`).
//! 2. Dimensional fields without an explicit unit suffix
//!    (`docs/scenario-format.md § Units`).
//! 3. 3-tuple numeric fields without an explicit frame infix
//!    (`docs/scenario-format.md § Frames`).
//!
//! All three checks fold into a single recursive walk over the parsed
//! `toml::Value`. Safety-name violations have global priority: the
//! walk records the first dimensional error but keeps scanning for a
//! later safety error before returning it.

use crate::error::ScenarioError;

const FORBIDDEN_SAFETY_TERMS: &[ForbiddenTerm] = &[
    ForbiddenTerm::new("target", "target"),
    ForbiddenTerm::new("seeker", "seeker"),
    ForbiddenTerm::new("warhead", "warhead"),
    ForbiddenTerm::new("strike", "strike"),
    ForbiddenTerm::new("interceptor", "interceptor"),
    ForbiddenTerm::new("kill", "kill"),
    ForbiddenTerm::new("threat", "threat"),
    ForbiddenTerm::new("engagement", "engagement"),
    ForbiddenTerm::new("terminalhoming", "terminal-homing"),
    ForbiddenTerm::new("terminalwaypoint", "terminal-waypoint"),
    ForbiddenTerm::new("impactpoint", "impact-point"),
    ForbiddenTerm::new("weapon", "weapon"),
    // Mission-vocabulary rejections (see
    // `docs/mission-states-vocabulary.md § Rejected Vocabulary`).
    ForbiddenTerm::new("midcourse", "midcourse"),
    ForbiddenTerm::new("endgame", "endgame"),
    ForbiddenTerm::new("decoy", "decoy"),
    ForbiddenTerm::new("penaid", "pen-aid"),
    ForbiddenTerm::new("blackoutevasion", "blackout-evasion"),
    // Flight-profile guardrails (see
    // `docs/profile-vocabulary-and-guardrails.md § Rejected operational
    // vocabulary`). Trajectory-mechanics terms (ballistic, boost, coast,
    // ascent, apogee, entry, descent, footprint, dispersion, downrange)
    // stay accepted; only targeting / engagement vocabulary is rejected.
    ForbiddenTerm::new("aimpoint", "aimpoint"),
    ForbiddenTerm::new("missdistance", "miss-distance"),
    ForbiddenTerm::new("terminalguidance", "terminal-guidance"),
    ForbiddenTerm::new("banktoturn", "bank-to-turn"),
    ForbiddenTerm::new("skidtoturn", "skid-to-turn"),
    ForbiddenTerm::new("intercept", "intercept"),
    ForbiddenTerm::new("reentryvehicle", "reentry-vehicle"),
    ForbiddenTerm::new("circularerror", "circular-error"),
    // Propulsion/staging guardrails: vehicle-intrinsic staging
    // analysis is accepted, range/target optimization vocabulary is not.
    ForbiddenTerm::new("maxrange", "max-range"),
    ForbiddenTerm::new("rangemax", "range-max"),
    ForbiddenTerm::new("targetrange", "target-range"),
    ForbiddenTerm::new("throwweight", "throw-weight"),
    ForbiddenTerm::new("impactenergy", "impact-energy"),
    // Aerodynamics guardrails: drag curves are accepted, gunnery
    // range-table products are not.
    ForbiddenTerm::new("firingtable", "firing-table"),
    ForbiddenTerm::new("rangetable", "range-table"),
    ForbiddenTerm::new("ballisticmatch", "ballistic-match"),
];

struct ForbiddenTerm {
    /// Lowercased, alphanumeric-only needle.
    needle: &'static str,
    /// Human-readable label written into the error.
    label: &'static str,
}

impl ForbiddenTerm {
    const fn new(needle: &'static str, label: &'static str) -> Self {
        Self { needle, label }
    }
}

const UNIT_SUFFIXES: &[&str] = &[
    "_dt_s",
    "_m_s2",
    "_m_s",
    "_m3_s2",
    "_n_m",
    "_n_s",
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
/// Returns the first safety-name failure if one exists; otherwise
/// returns the first unit / frame suffix failure encountered.
pub(crate) fn lint(value: &toml::Value) -> Result<(), ScenarioError> {
    let mut first_dimensional_error = None;
    walk("$", None, value, &mut first_dimensional_error)?;
    match first_dimensional_error {
        Some(error) => Err(error),
        None => Ok(()),
    }
}

fn walk(
    path: &str,
    parent_key: Option<&str>,
    value: &toml::Value,
    first_dimensional_error: &mut Option<ScenarioError>,
) -> Result<(), ScenarioError> {
    if let Some(key) = parent_key {
        check_key_for_safety_term(path, key)?;
        if first_dimensional_error.is_none()
            && let Err(error) = check_dimensional_field(path, key, value)
        {
            *first_dimensional_error = Some(error);
        }
    }
    match value {
        toml::Value::Table(table) => {
            for (key, child) in table {
                let child_path = format!("{path}.{key}");
                walk(&child_path, Some(key), child, first_dimensional_error)?;
            }
        }
        toml::Value::Array(values) => {
            for (index, child) in values.iter().enumerate() {
                walk(
                    &format!("{path}[{index}]"),
                    None,
                    child,
                    first_dimensional_error,
                )?;
            }
        }
        toml::Value::String(text) if string_value_is_lintable(path) => {
            check_string_value_for_safety_term(path, text)?;
        }
        _ => {}
    }
    Ok(())
}

/// Whether a leaf string value at `path` should be checked for
/// safety-limited terms.
///
/// Free-form descriptive fields and any value containing path
/// separators are excluded; everything else is treated as a
/// model-name-shaped value where safety vocabulary is forbidden.
fn string_value_is_lintable(path: &str) -> bool {
    const SKIPPED_SUFFIXES: &[&str] = &[".description", ".provenance"];
    const SKIPPED_PREFIXES: &[&str] = &["$.telemetry.output.", "$.data_packages."];
    const SKIPPED_LEAVES: &[&str] = &[".manifest", ".leap_second_table"];

    if SKIPPED_SUFFIXES.iter().any(|s| path.ends_with(s)) {
        return false;
    }
    if SKIPPED_PREFIXES.iter().any(|p| path.starts_with(p)) {
        return false;
    }
    if SKIPPED_LEAVES.iter().any(|s| path.ends_with(s)) {
        return false;
    }
    true
}

fn check_key_for_safety_term(path: &str, key: &str) -> Result<(), ScenarioError> {
    check_for_safety_term(path, key, key)
}

fn check_string_value_for_safety_term(path: &str, value: &str) -> Result<(), ScenarioError> {
    if value.contains('/') || value.contains('\\') {
        return Ok(());
    }
    check_for_safety_term(path, value, value)
}

fn check_for_safety_term(path: &str, original: &str, raw: &str) -> Result<(), ScenarioError> {
    let normalised: String = raw
        .chars()
        .filter(char::is_ascii_alphanumeric)
        .flat_map(char::to_lowercase)
        .collect();
    for term in FORBIDDEN_SAFETY_TERMS {
        if normalised.contains(term.needle) {
            return Err(ScenarioError::SafetyName {
                path: path.to_owned(),
                value: original.to_owned(),
                term: term.label,
            });
        }
    }
    Ok(())
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
            "cp_max" | "gamma" | "accommodation_normal" | "accommodation_tangential"
        )
    {
        return true;
    }

    if path.starts_with("$.aerothermal")
        && matches!(key, "n_nodes" | "gas_yield_fraction" | "lewis_number")
    {
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

    // Landing-footprint Monte Carlo fields. Sample counts,
    // confidence levels, and multiplicative wind-scale uncertainty are
    // dimensionless; typed validation owns their ranges.
    if path.starts_with("$.landing_footprint.monte_carlo")
        && matches!(key, "samples" | "confidence_levels" | "speed_scale_sigma")
    {
        return true;
    }

    // Mission effector overrides use the same unit-agnostic command
    // scalar as the target effector.
    if path.starts_with("$.mission.events") && path.ends_with(".action.command") && key == "command"
    {
        return true;
    }

    // Mission engine commands carry a typed payload
    // including a dimensionless `throttle_unit` and lifecycle bools
    // (the bools never trip this lint, but `throttle_unit` would
    // without an exemption).
    path.starts_with("$.mission.events")
        && path
            .strip_suffix(".throttle_unit")
            .is_some_and(|parent| parent.ends_with(".action.command"))
        && key == "throttle_unit"
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
