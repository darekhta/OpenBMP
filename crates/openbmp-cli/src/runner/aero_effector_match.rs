//! Phase-3.5.C runner-side matcher between schema-2 aero deck axes
//! and scenario-declared effectors.
//!
//! For each schema-2 deck axis (e.g. `delta_e_deg`) the runner must
//! pair the axis with a scenario effector by name and assert the
//! effector's `unit` field matches the suffix on the axis name.
//! Mismatches fail closed at runner build time with
//! `CliError::AeroEffectorMismatch`,
//! before any kernel step is taken.
//!
//! Schema-1 decks (no effector axes) and scenarios without aero
//! decks both produce an empty `Vec<DeckAxisBinding>` — the runner
//! gates every per-step snapshot push on `!bindings.is_empty()` to
//! preserve byte-identical legacy behaviour.

use openbmp_aero::AeroDeck;
use openbmp_scenario::ScenarioDocument;

use crate::error::CliError;

/// Resolved binding from a schema-2 deck axis to a scenario effector.
///
/// `effector_index` indexes into the effector rack's
/// scenario-declared order (matching `EffectorRack::scenario_ids()`
/// and `EffectorRack::snapshot()`). `deck_axis_name` is the deck's
/// own name for the axis (with unit suffix), used as the
/// `BTreeMap<String, f64>` key the kernel snapshot is built with.
#[derive(Clone, Debug, PartialEq)]
pub struct DeckAxisBinding {
    /// Index into the scenario-declared effector list.
    pub effector_index: usize,
    /// Deck axis name (e.g. `"delta_e_deg"`). Carried as-is into
    /// the kernel's effector-actuals snapshot map.
    pub deck_axis_name: String,
}

/// Match every schema-2 deck axis against scenario effectors.
///
/// Returns an empty vector when:
/// - the deck is `None` (no `[aero]` block in the scenario), or
/// - the deck is schema-1 (no effector axes declared).
///
/// Otherwise returns one [`DeckAxisBinding`] per declared effector
/// axis, in deck-declared order.
///
/// # Errors
///
/// Returns [`CliError::AeroEffectorMismatch`] when:
/// - a deck axis name has no matching scenario effector (after
///   stripping the unit suffix);
/// - the matching effector's declared `unit` does not equal the
///   suffix on the deck axis name.
pub fn assert_axes_match_effectors(
    deck: Option<&AeroDeck>,
    document: &ScenarioDocument,
) -> Result<Vec<DeckAxisBinding>, CliError> {
    let Some(deck) = deck else {
        return Ok(Vec::new());
    };
    let axis_names = deck.effector_axis_names();
    if axis_names.is_empty() {
        return Ok(Vec::new());
    }
    let effectors = document.vehicle.assembly.effectors.as_slice();

    let mut bindings = Vec::with_capacity(axis_names.len());
    let mut seen_bare_axis_names: std::collections::BTreeSet<&str> =
        std::collections::BTreeSet::new();
    for axis_name in axis_names {
        let (bare_name, suffix) =
            split_unit_suffix(axis_name).ok_or(CliError::AeroEffectorMismatch {
                field: format!("aero.deck.axis_order[{axis_name}]"),
                reason: format!(
                    "deck axis name `{axis_name}` lacks a `_deg` or `_rad` unit suffix; \
                 schema-2 axes are declared with explicit units"
                ),
            })?;
        if !seen_bare_axis_names.insert(bare_name) {
            return Err(CliError::AeroEffectorMismatch {
                field: format!("aero.deck.axis_order[{axis_name}]"),
                reason: format!(
                    "deck declares more than one effector axis for `{bare_name}` after stripping \
                     `_deg` / `_rad`; declare a single axis in the same unit as the scenario \
                     effector"
                ),
            });
        }
        let (index, config) = effectors
            .iter()
            .enumerate()
            .find(|(_, c)| c.id == bare_name)
            .ok_or_else(|| CliError::AeroEffectorMismatch {
                field: format!("vehicle.assembly.effectors.{bare_name}"),
                reason: format!(
                    "deck declares effector axis `{axis_name}` but no scenario effector with id \
                     `{bare_name}` is declared"
                ),
            })?;
        let declared_unit = config.unit.as_deref().unwrap_or("");
        if declared_unit != suffix {
            return Err(CliError::AeroEffectorMismatch {
                field: format!("vehicle.assembly.effectors.{bare_name}.unit"),
                reason: format!(
                    "deck axis `{axis_name}` requires effector unit `{suffix}`, but scenario \
                     effector `{bare_name}` declares unit `{declared_unit}`"
                ),
            });
        }
        bindings.push(DeckAxisBinding {
            effector_index: index,
            deck_axis_name: axis_name.clone(),
        });
    }
    Ok(bindings)
}

/// Build the per-step snapshot map the kernel consumes via
/// `set_effector_actuals`. The rack snapshot is in scenario-declared
/// order; the bindings carry the index. The output `BTreeMap` is
/// keyed by deck axis name (with unit suffix), valued by the
/// effector's `EffectorState.actual` at the time the snapshot was
/// taken.
///
/// Empty `bindings` produces an empty map. The runner's per-step
/// guard on `!bindings.is_empty()` should also short-circuit the
/// `kernel.set_effector_actuals` call so legacy scenarios stay
/// byte-identical.
#[must_use]
pub fn build_snapshot_map(
    bindings: &[DeckAxisBinding],
    rack_snapshot: &[openbmp_vehicle::EffectorState],
) -> std::collections::BTreeMap<String, f64> {
    let mut out = std::collections::BTreeMap::new();
    for binding in bindings {
        if let Some(state) = rack_snapshot.get(binding.effector_index) {
            out.insert(binding.deck_axis_name.clone(), state.actual);
        }
    }
    out
}

/// Strip the `_deg` or `_rad` suffix from a deck axis name. Returns
/// `(bare_name, suffix)` on match, `None` otherwise.
fn split_unit_suffix(name: &str) -> Option<(&str, &str)> {
    if let Some(bare) = name.strip_suffix("_deg") {
        Some((bare, "deg"))
    } else {
        name.strip_suffix("_rad").map(|bare| (bare, "rad"))
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_aero::AeroDeck;
    use openbmp_scenario::Scenario;

    /// Schema-2 deck with one elevon axis.
    fn elevon_4d_deck() -> AeroDeck {
        let mach = vec![0.0, 1.0];
        let alpha = vec![0.0, 1.0];
        let beta = vec![0.0];
        let delta = vec![-20.0, 0.0, 20.0];
        let mut cn = Vec::new();
        let mut cd = Vec::new();
        let mut cm = Vec::new();
        for m in &mach {
            for a in &alpha {
                for _b in &beta {
                    for d in &delta {
                        cn.push(m + a + 0.1 * d);
                        cd.push(0.0);
                        cm.push(0.0);
                    }
                }
            }
        }
        AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
            ],
            vec![mach, alpha, beta, delta],
            cn,
            cd,
            cm,
            1.0,
            1.0,
        )
        .unwrap()
    }

    const ASSEMBLY_WITH_EFFECTOR: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../crates/openbmp-scenario/tests/fixtures/assembly-with-effector.toml"
    ));

    fn elevon_scenario_with_unit(unit: Option<&str>) -> Scenario {
        let toml = match unit {
            Some(u) => ASSEMBLY_WITH_EFFECTOR.replace(
                "unit             = \"rad\"",
                &format!("unit             = \"{u}\""),
            ),
            None => ASSEMBLY_WITH_EFFECTOR.replace("unit             = \"rad\"\n", ""),
        };
        Scenario::from_toml_str(&toml).expect("scenario parses")
    }

    #[test]
    fn match_succeeds_for_no_deck() {
        let scenario = elevon_scenario_with_unit(Some("rad"));
        let bindings = assert_axes_match_effectors(None, &scenario.document).unwrap();
        assert!(bindings.is_empty());
    }

    #[test]
    fn match_succeeds_for_schema1_deck_with_no_effectors() {
        let mach = vec![0.0, 1.0];
        let alpha = vec![0.0, 1.0];
        let beta = vec![0.0];
        let cn = vec![0.0; 4];
        let deck = AeroDeck::new(mach, alpha, beta, cn.clone(), cn.clone(), cn, 1.0, 1.0).unwrap();
        let scenario = elevon_scenario_with_unit(Some("rad"));
        let bindings = assert_axes_match_effectors(Some(&deck), &scenario.document).unwrap();
        assert!(bindings.is_empty());
    }

    #[test]
    fn match_succeeds_for_schema2_with_matching_unit() {
        let deck = elevon_4d_deck();
        let scenario = elevon_scenario_with_unit(Some("deg"));
        let bindings = assert_axes_match_effectors(Some(&deck), &scenario.document).unwrap();
        assert_eq!(bindings.len(), 1);
        assert_eq!(bindings[0].effector_index, 0);
        assert_eq!(bindings[0].deck_axis_name, "delta_e_deg");
    }

    #[test]
    fn match_fails_when_no_scenario_effector_with_matching_id() {
        let deck = elevon_4d_deck();
        // Scenario uses id="delta_e", deck wants matching name. Replace
        // the scenario id to break the match.
        let toml = ASSEMBLY_WITH_EFFECTOR.replace(
            "id               = \"delta_e\"",
            "id               = \"delta_a\"",
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let err = assert_axes_match_effectors(Some(&deck), &scenario.document).unwrap_err();
        assert!(matches!(err, CliError::AeroEffectorMismatch { .. }));
    }

    #[test]
    fn match_fails_when_unit_mismatch() {
        let deck = elevon_4d_deck();
        // Deck axis is `delta_e_deg` but scenario declares `unit = "rad"`.
        let scenario = elevon_scenario_with_unit(Some("rad"));
        let err = assert_axes_match_effectors(Some(&deck), &scenario.document).unwrap_err();
        assert!(matches!(err, CliError::AeroEffectorMismatch { .. }));
    }

    #[test]
    fn match_fails_when_two_deck_axes_strip_to_same_effector_id() {
        let mach = vec![0.0, 1.0];
        let alpha = vec![0.0, 1.0];
        let beta = vec![0.0];
        let delta_deg = vec![-20.0, 0.0, 20.0];
        let delta_rad = vec![-0.3, 0.0, 0.3];
        let table =
            vec![0.0; mach.len() * alpha.len() * beta.len() * delta_deg.len() * delta_rad.len()];
        let deck = AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
                "delta_e_rad".to_string(),
            ],
            vec![mach, alpha, beta, delta_deg, delta_rad],
            table.clone(),
            table.clone(),
            table,
            1.0,
            1.0,
        )
        .unwrap();
        let scenario = elevon_scenario_with_unit(Some("deg"));
        let err = assert_axes_match_effectors(Some(&deck), &scenario.document).unwrap_err();
        assert!(matches!(err, CliError::AeroEffectorMismatch { .. }));
    }

    #[test]
    fn build_snapshot_map_pairs_bindings_with_rack_snapshot() {
        use openbmp_vehicle::EffectorState;
        let bindings = vec![DeckAxisBinding {
            effector_index: 0,
            deck_axis_name: "delta_e_deg".to_string(),
        }];
        let snapshot = vec![EffectorState::at_rest(7.5)];
        let map = build_snapshot_map(&bindings, &snapshot);
        assert_eq!(map.get("delta_e_deg").copied(), Some(7.5));
    }

    #[test]
    fn build_snapshot_map_with_empty_bindings_is_empty() {
        let map = build_snapshot_map(&[], &[]);
        assert!(map.is_empty());
    }
}
