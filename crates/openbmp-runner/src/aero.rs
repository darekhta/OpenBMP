//! Runner-side aerodynamic coefficient source loading.

use std::collections::BTreeMap;

use openbmp_aero::{
    AccommodationCoeffs, AeroDeck, AeroError, AeroMethod, Afterbody, BodyGeometry, BuildupGrid,
    ChengBridge, ComponentBuildup, DeckLookup, DragBuildupModel, ErfcBridge, FinSet,
    FreeMolecularAero, HybridAeroMethod, LinearKnudsenBridge, LinearMachBridge, ModifiedNewtonian,
    NoseShape, ProtuberanceSet, TangentCone, TangentWedge,
};
use openbmp_aerodb::{
    CorrelatedDeckDrawSettings, RunMatrix, correlated_deck_draw_required_dimension_count,
    draw_correlated_aero_deck,
};
use openbmp_core::SimTime;
use openbmp_physics::AtmosphereModel;
use openbmp_scenario::{
    AeroBuildupConfig, AeroBuildupNoseConfig, AeroFreeMolecularConfig, AeroKnudsenBridgeConfig,
    AeroMethodConfig, ResolvedFile, ScenarioDocument,
};
use openbmp_uq::{CorrelatedErrorBudget, CredibilityLevel, aero_coefficient_margin_source};

use crate::atmosphere::build_document_runtime_atmosphere;
use crate::error::RunnerError;

const HYPERSONIC_DECK_GUARD_MACH: f64 = 5.0;

/// Built aerodynamic method plus reference length (m), or `None`
/// when the scenario has no aerodynamic model.
pub type BuiltAeroMethod = Option<(Box<dyn AeroMethod>, f64)>;

/// Load or bake the aerodynamic deck declared by `[aero]`.
///
/// # Errors
///
/// Returns [`RunnerError`] when the referenced deck file is missing
/// from the already-verified file map, cannot be read as UTF-8, fails
/// deck validation, or the buildup geometry cannot be baked.
pub fn load_aero_deck(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<AeroDeck>, RunnerError> {
    let Some(aero) = &document.aero else {
        return Ok(None);
    };
    let deck = if aero.deck.is_some() {
        Some(load_deck_file(resolved_files)?)
    } else if let Some(buildup) = &aero.buildup {
        Some(bake_buildup_deck(document, buildup)?)
    } else {
        None
    };
    deck.map(|deck| apply_deck_draw_if_declared(document, deck))
        .transpose()
}

/// Return the deterministic RNG dimension count required by the resolved aero
/// deck's correlated-draw tables.
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario's declared deck/buildup source
/// cannot be loaded or baked.
pub fn correlated_deck_draw_required_dimensions(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<u32>, RunnerError> {
    load_aero_deck(document, resolved_files).map(|deck| {
        deck.as_ref()
            .map(correlated_deck_draw_required_dimension_count)
    })
}

/// Load a schema-declared offline aero-database run matrix.
///
/// This is a file-fed database handoff helper only. It does not make the run
/// matrix an aerodynamic force source and does not launch an external solver.
///
/// # Errors
///
/// Returns [`RunnerError`] if the resolved-file set lacks the declared asset, if
/// it is not UTF-8, or if `openbmp-aerodb` rejects the strict condition-only
/// run-matrix TOML.
pub fn load_declared_aero_run_matrix(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<RunMatrix>, RunnerError> {
    if document
        .aero
        .as_ref()
        .and_then(|aero| aero.run_matrix.as_ref())
        .is_none()
    {
        return Ok(None);
    }
    let resolved = resolved_files.get("aero.run_matrix.file").ok_or_else(|| {
        RunnerError::UnsupportedScenario {
            what: "declared aero.run_matrix.file was not present in resolved scenario files"
                .to_owned(),
        }
    })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("aero.run_matrix.file is not UTF-8: {err}"),
        })?;
    Ok(Some(RunMatrix::load_toml_str(text)?))
}

/// Build the live [`AeroMethod`] selected by `[aero.method]`.
///
/// Returns `Ok(None)` when the scenario has no `[aero]` block.
/// Absent `[aero.method]` or `kind = "deck"` wraps the loaded/baked
/// deck as [`DeckLookup`].
pub fn build_aero_method(
    document: &ScenarioDocument,
    deck: Option<AeroDeck>,
) -> Result<BuiltAeroMethod, RunnerError> {
    let Some(aero) = &document.aero else {
        return Ok(None);
    };
    let Some(config) = aero.method.as_ref() else {
        let deck = deck.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "aero method kind `deck` requires [aero].deck or [aero.buildup]".to_owned(),
        })?;
        let reference_length_m = deck.reference_length_m();
        return Ok(Some((Box::new(DeckLookup::new(deck)), reference_length_m)));
    };
    build_configured_aero_method(config, deck).map(Some)
}

/// Gather source-tagged UQ declared by `[aero.uq]`.
///
/// # Errors
///
/// Returns [`RunnerError`] if the credibility level or coefficient band cannot
/// be converted into a validated UQ source.
pub fn upstream_uq_budget(
    document: &ScenarioDocument,
) -> Result<CorrelatedErrorBudget, RunnerError> {
    let Some(uq) = document.aero.as_ref().and_then(|aero| aero.uq.as_ref()) else {
        return Ok(CorrelatedErrorBudget::default());
    };
    let level = CredibilityLevel::from_value(uq.credibility_level).ok_or_else(|| {
        RunnerError::UnsupportedScenario {
            what: format!(
                "[aero.uq] credibility_level {} is outside 0..=4",
                uq.credibility_level
            ),
        }
    })?;
    let source = aero_coefficient_margin_source(
        uq.deck_id.clone(),
        uq.coefficient_id.clone(),
        uq.coefficient_lower,
        uq.coefficient_nominal,
        uq.coefficient_upper,
        level,
        uq.evidence.clone(),
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[aero.uq] invalid: {err}"),
    })?;
    Ok(CorrelatedErrorBudget {
        sources: vec![source],
        correlation: None,
    })
}

fn apply_deck_draw_if_declared(
    document: &ScenarioDocument,
    deck: AeroDeck,
) -> Result<AeroDeck, RunnerError> {
    let Some(draw) = document
        .aero
        .as_ref()
        .and_then(|aero| aero.deck_draw.as_ref())
    else {
        return Ok(deck);
    };
    let settings = CorrelatedDeckDrawSettings::new_with_deck_default_and_axis_correlation_lengths(
        draw.campaign_seed,
        draw.sample_index,
        draw.dimension_id_base,
        draw.mach_correlation_length,
        draw.alpha_correlation_length_deg,
        draw.beta_correlation_length_deg,
        &deck,
    )?;
    Ok(draw_correlated_aero_deck(&deck, settings)?)
}

/// Reject deck-only aerodynamic use when the initial state is already
/// hypersonic and outside the deck's Mach envelope.
///
/// This does not police mild launch-vehicle clamp policies near the
/// edge of low-Mach decks. It catches the re-entry failure mode where a
/// Mach-20 trajectory would either fail closed immediately or clamp a
/// launch deck far beyond its intended range. Hybrid/live aero methods
/// are exempt because they do not rely on a single low-Mach deck over
/// the full flight envelope.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] with guidance to use a
/// hybrid method when deck-only aero starts hypersonic above the deck's
/// maximum Mach.
pub fn reject_hypersonic_deck_only_out_of_envelope(
    document: &ScenarioDocument,
    deck: Option<&AeroDeck>,
) -> Result<(), RunnerError> {
    let Some(aero) = &document.aero else {
        return Ok(());
    };
    let method_kind = aero
        .method
        .as_ref()
        .map_or("deck", |method| method.kind.as_str());
    if method_kind != "deck" {
        return Ok(());
    }
    let Some(deck) = deck else {
        return Ok(());
    };
    let Some(max_deck_mach) = deck.mach_grid().last().copied() else {
        return Ok(());
    };

    let [vx, vy, vz] = document.vehicle.initial_velocity_eci_m_s;
    let speed_m_s = (vx * vx + vy * vy + vz * vz).sqrt();
    if !speed_m_s.is_finite() || speed_m_s <= 0.0 {
        return Ok(());
    }
    let atmosphere = build_document_runtime_atmosphere(document)?;
    let altitude_m = document.vehicle.initial_position_eci_m[2].max(0.0);
    let sample = atmosphere
        .sample(altitude_m, SimTime::from_seconds(document.time.start_s))
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("initial aero deck Mach check could not sample atmosphere: {err}"),
        })?;
    if sample.speed_of_sound_m_s <= 0.0 || !sample.speed_of_sound_m_s.is_finite() {
        return Ok(());
    }
    let initial_mach = speed_m_s / sample.speed_of_sound_m_s;
    if initial_mach > max_deck_mach && initial_mach >= HYPERSONIC_DECK_GUARD_MACH {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "deck-only aero initial Mach {initial_mach:.3} exceeds deck max Mach \
                 {max_deck_mach:.3}; use [aero.method] kind = \"hybrid\" with a \
                 hypersonic continuum method and free_molecular branch for re-entry"
            ),
        });
    }
    Ok(())
}

fn build_configured_aero_method(
    config: &AeroMethodConfig,
    deck: Option<AeroDeck>,
) -> Result<(Box<dyn AeroMethod>, f64), RunnerError> {
    let method: Box<dyn AeroMethod> = match config.kind.as_str() {
        "deck" => {
            let deck = deck.ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "aero method kind `deck` requires [aero].deck or [aero.buildup]".to_owned(),
            })?;
            let reference_length_m = deck.reference_length_m();
            return Ok((Box::new(DeckLookup::new(deck)), reference_length_m));
        }
        "modified_newtonian" => {
            let cfg = selected_method(Some(config), "modified_newtonian")?
                .modified_newtonian
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "aero.method.kind = `modified_newtonian` requires [aero.method.modified_newtonian]".to_owned(),
                })?;
            Box::new(ModifiedNewtonian {
                cp_max: cfg.cp_max,
                reference_area_m2: cfg.reference_area_m2,
                reference_length_m: cfg.reference_length_m,
            })
        }
        "tangent_cone" => {
            let cfg = selected_method(Some(config), "tangent_cone")?
                .tangent_cone
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "aero.method.kind = `tangent_cone` requires [aero.method.tangent_cone]"
                        .to_owned(),
                })?;
            Box::new(TangentCone {
                cone_half_angle_rad: cfg.cone_half_angle_rad,
                reference_area_m2: cfg.reference_area_m2,
                reference_length_m: cfg.reference_length_m,
                gamma: cfg.gamma,
            })
        }
        "tangent_wedge" => {
            let cfg = selected_method(Some(config), "tangent_wedge")?
                .tangent_wedge
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "aero.method.kind = `tangent_wedge` requires [aero.method.tangent_wedge]"
                        .to_owned(),
                })?;
            Box::new(TangentWedge {
                wedge_half_angle_rad: cfg.wedge_half_angle_rad,
                reference_area_m2: cfg.reference_area_m2,
                gamma: cfg.gamma,
            })
        }
        "free_molecular" => {
            let cfg = selected_method(Some(config), "free_molecular")?
                .free_molecular
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what:
                        "aero.method.kind = `free_molecular` requires [aero.method.free_molecular]"
                            .to_owned(),
                })?;
            Box::new(free_molecular_method(cfg))
        }
        "hybrid" => {
            let cfg = selected_method(Some(config), "hybrid")?
                .hybrid
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "aero.method.kind = `hybrid` requires [aero.method.hybrid]".to_owned(),
                })?;
            let (continuum_low_mach, _) =
                build_configured_aero_method(&cfg.continuum_low_mach, deck.clone())?;
            let (continuum_high_mach, _) =
                build_configured_aero_method(&cfg.continuum_high_mach, deck)?;
            #[allow(deprecated)]
            let hybrid = HybridAeroMethod {
                continuum_low_mach,
                continuum_high_mach,
                free_molecular: Box::new(free_molecular_method(&cfg.free_molecular)),
                mach_handoff: cfg.mach_handoff,
                mach_bridge: cfg.mach_bridge.as_ref().map(|bridge| LinearMachBridge {
                    mach_lo: bridge.mach_lo,
                    mach_hi: bridge.mach_hi,
                }),
                bridge: knudsen_bridge(&cfg.bridge)?,
                knudsen: 0.0,
            };
            Box::new(hybrid)
        }
        other => {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("unsupported aero.method.kind `{other}`"),
            });
        }
    };
    let reference_length_m = aero_method_reference_length(config)?;
    Ok((method, reference_length_m))
}

fn free_molecular_method(config: &AeroFreeMolecularConfig) -> FreeMolecularAero {
    FreeMolecularAero {
        accommodation: AccommodationCoeffs {
            normal: config.accommodation_normal,
            tangential: config.accommodation_tangential,
        },
        reference_area_m2: config.reference_area_m2,
    }
}

fn knudsen_bridge(
    config: &AeroKnudsenBridgeConfig,
) -> Result<Box<dyn openbmp_aero::BridgeFunction>, RunnerError> {
    match config.kind.as_str() {
        "cheng" => Ok(Box::new(ChengBridge)),
        "erfc" => Ok(Box::new(ErfcBridge {
            sigma: config.sigma,
        })),
        "linear" => Ok(Box::new(LinearKnudsenBridge {
            kn_lo: config.kn_lo,
            kn_hi: config.kn_hi,
        })),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("unsupported hybrid aero Knudsen bridge `{other}`"),
        }),
    }
}

fn selected_method<'a>(
    method: Option<&'a AeroMethodConfig>,
    kind: &str,
) -> Result<&'a AeroMethodConfig, RunnerError> {
    method.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("aero.method.kind = `{kind}` requires [aero.method]"),
    })
}

fn aero_method_reference_length(config: &AeroMethodConfig) -> Result<f64, RunnerError> {
    match config.kind.as_str() {
        "modified_newtonian" => config
            .modified_newtonian
            .as_ref()
            .map(|cfg| cfg.reference_length_m),
        "tangent_cone" => config
            .tangent_cone
            .as_ref()
            .map(|cfg| cfg.reference_length_m),
        "tangent_wedge" => config
            .tangent_wedge
            .as_ref()
            .map(|cfg| cfg.reference_area_m2.sqrt()),
        "free_molecular" => config
            .free_molecular
            .as_ref()
            .map(|cfg| cfg.reference_area_m2.sqrt()),
        "hybrid" => config.hybrid.as_ref().map(|cfg| cfg.reference_length_m),
        _ => None,
    }
    .ok_or_else(|| RunnerError::UnsupportedScenario {
        what: "aero method reference length is unavailable".to_owned(),
    })
}

fn load_deck_file(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<AeroDeck, RunnerError> {
    let resolved =
        resolved_files
            .get("aero.deck")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what:
                    "internal invariant: resolved file `aero.deck` missing after pin verification"
                        .to_owned(),
            })?;
    let text = std::str::from_utf8(&resolved.bytes).map_err(|e| {
        RunnerError::Aero(AeroError::Io {
            reason: format!(
                "could not read deck file {} as UTF-8: {e}",
                resolved.path.display()
            ),
        })
    })?;
    Ok(AeroDeck::load_from_str(text)?)
}

fn bake_buildup_deck(
    document: &ScenarioDocument,
    config: &AeroBuildupConfig,
) -> Result<AeroDeck, RunnerError> {
    let atmosphere = build_document_runtime_atmosphere(document)?;
    let sample = atmosphere.sample(config.reference_altitude_m, openbmp_core::SimTime::ZERO)?;
    if sample.density_kg_m3 <= 0.0
        || sample.speed_of_sound_m_s <= 0.0
        || sample.dynamic_viscosity_pa_s <= 0.0
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "aero buildup reference atmosphere must have positive density, speed of sound, and dynamic viscosity".to_owned(),
        });
    }
    let reference_reynolds_at_mach1 =
        sample.density_kg_m3 * sample.speed_of_sound_m_s * config.reference_length_m
            / sample.dynamic_viscosity_pa_s;
    let mach_steps =
        usize::try_from(config.mach_grid.steps).map_err(|_| RunnerError::UnsupportedScenario {
            what: "aero buildup mach_grid.steps overflows usize".to_owned(),
        })?;
    let alpha_steps = usize::try_from(config.alpha_grid_deg.steps).map_err(|_| {
        RunnerError::UnsupportedScenario {
            what: "aero buildup alpha_grid_deg.steps overflows usize".to_owned(),
        }
    })?;
    let grid = BuildupGrid::from_ranges(
        config.mach_grid.min,
        config.mach_grid.max,
        mach_steps,
        config.alpha_grid_deg.min,
        config.alpha_grid_deg.max,
        alpha_steps,
        reference_reynolds_at_mach1,
    )?;
    let geom = BodyGeometry {
        body_diameter_m: config.body_diameter_m,
        body_length_m: config.body_length_m,
        nose: scenario_nose(&config.nose)?,
        afterbody: config.afterbody.as_ref().map(|afterbody| Afterbody {
            exit_diameter_m: afterbody.exit_diameter_m,
            length_m: afterbody.length_m,
        }),
        surface_roughness_m: config.surface_roughness_m,
        fins: config.fins.as_ref().map(|fins| FinSet {
            count: fins.count,
            root_chord_m: fins.root_chord_m,
            tip_chord_m: fins.tip_chord_m,
            span_m: fins.span_m,
            thickness_ratio: fins.thickness_ratio,
            sweep_rad: fins.sweep_rad,
        }),
        protuberances: config
            .protuberances
            .as_ref()
            .map(|protuberances| ProtuberanceSet {
                count: protuberances.count,
                frontal_area_m2: protuberances.frontal_area_m2,
                drag_coefficient: protuberances.drag_coefficient,
                interference_factor: protuberances.interference_factor,
            }),
        reference_area_m2: config.reference_area_m2,
        reference_length_m: config.reference_length_m,
        center_of_gravity_from_nose_m: config.center_of_gravity_from_nose_m,
    };
    let legacy = ComponentBuildup::default();
    if config.datcom {
        Ok(legacy.datcom().bake_deck(&geom, &grid)?)
    } else {
        Ok(legacy.bake_deck(&geom, &grid)?)
    }
}

fn scenario_nose(config: &AeroBuildupNoseConfig) -> Result<NoseShape, RunnerError> {
    match config.shape.as_str() {
        "conical" => {
            let Some(half_angle_rad) = config.half_angle_rad else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "aero buildup conical nose missing half_angle_rad".to_owned(),
                });
            };
            Ok(NoseShape::Conical { half_angle_rad })
        }
        "ogive" => {
            let Some(fineness) = config.fineness else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "aero buildup ogive nose missing fineness".to_owned(),
                });
            };
            Ok(NoseShape::Ogive { fineness })
        }
        "von_karman" => {
            let Some(fineness) = config.fineness else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "aero buildup von_karman nose missing fineness".to_owned(),
                });
            };
            Ok(NoseShape::VonKarman { fineness })
        }
        "hemispherical" => Ok(NoseShape::Hemispherical),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("unsupported aero buildup nose shape `{other}`"),
        }),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_scenario::{ResolvedFile, Scenario};

    const BUILDUP_SCENARIO: &str = r#"
openbmp.scenario = 2

[meta]
name = "buildup-smoke"
description = "buildup smoke test"
validation = "experimental"
provenance = "synthetic"

[time]
start_s = 0.0
stop_s = 0.02
dt_s = 0.01
seed = 1

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 50.0]

[vehicle.assembly]
id = "body"

[[vehicle.assembly.bodies]]
id = "main"
geometry = { kind = "reference", length_m = 2.4, area_m2 = 0.031415926535897934 }
dry_mass_kg = 1.0

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "us_standard_1976"
wind = "none"

[forces]
models = ["gravity", "aero"]

[aero]

[aero.buildup]
body_diameter_m = 0.2
body_length_m = 2.4
surface_roughness_m = 6.0e-5
reference_area_m2 = 0.031415926535897934
reference_length_m = 0.2
mach_grid = { min = 0.0, max = 2.0, steps = 5 }
alpha_grid_deg = { min = 0.0, max = 6.0, steps = 4 }
reference_altitude_m = 0.0

[aero.buildup.nose]
shape = "ogive"
fineness = 3.5

[telemetry]
output.csv = "out/buildup-smoke.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const SIGMA_DECK_TOML: &str = r#"
openbmp.aero_deck = 1

reference.area_m2 = 1.0
reference.length_m = 1.0
provenance = "synthetic sigma deck"
validation = "validated-toy"

[grid]
mach = [1.0, 2.0]
alpha_deg = [0.0]
beta_deg = [0.0]

[coefficients.cn]
data = [0.1, 0.2]

[coefficients.cd]
data = [0.3, 0.4]

[coefficients.cm]
data = [0.0, 0.0]

[uncertainty.cn]
data = [0.05, 0.05]

[uncertainty.cd]
data = [0.0, 0.0]

[uncertainty.cm]
data = [0.0, 0.0]

[uncertainty.correlation]
mach_length = 1.5
"#;

    const SIGMA_GRID_DECK_TOML: &str = r#"
openbmp.aero_deck = 1

reference.area_m2 = 1.0
reference.length_m = 1.0
provenance = "synthetic sigma grid deck"
validation = "validated-toy"

[grid]
mach = [1.0, 2.0]
alpha_deg = [-5.0, 5.0]
beta_deg = [0.0, 3.0]

[coefficients.cn]
data = [0.10, 0.11, 0.12, 0.13, 0.20, 0.21, 0.22, 0.23]

[coefficients.cd]
data = [0.30, 0.31, 0.32, 0.33, 0.40, 0.41, 0.42, 0.43]

[coefficients.cm]
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

[uncertainty.cn]
data = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]

[uncertainty.cd]
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

[uncertainty.cm]
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

[uncertainty.correlation]
mach_length = 1.5
"#;

    const SIX_SIGMA_DECK_TOML: &str = r#"
openbmp.aero_deck = 1

reference.area_m2 = 1.0
reference.length_m = 1.0
provenance = "synthetic six sigma deck"
validation = "validated-toy"

[grid]
mach = [1.0, 2.0]
alpha_deg = [0.0]
beta_deg = [0.0]

[coefficients.cn]
data = [0.1, 0.2]

[coefficients.cd]
data = [0.3, 0.4]

[coefficients.cm]
data = [0.0, 0.0]

[six_coefficients.cx]
data = [0.11, 0.12]

[six_coefficients.cy]
data = [0.21, 0.22]

[six_coefficients.cz]
data = [0.31, 0.32]

[six_coefficients.cl]
data = [0.01, 0.02]

[six_coefficients.cm]
data = [0.03, 0.04]

[six_coefficients.cn]
data = [0.05, 0.06]

[uncertainty.cn]
data = [0.0, 0.0]

[uncertainty.cd]
data = [0.0, 0.0]

[uncertainty.cm]
data = [0.0, 0.0]

[uncertainty.six_coefficients.cx]
data = [0.05, 0.05]

[uncertainty.six_coefficients.cy]
data = [0.05, 0.05]

[uncertainty.six_coefficients.cz]
data = [0.05, 0.05]

[uncertainty.six_coefficients.cl]
data = [0.05, 0.05]

[uncertainty.six_coefficients.cm]
data = [0.05, 0.05]

[uncertainty.six_coefficients.cn]
data = [0.05, 0.05]

[uncertainty.correlation]
mach_length = 1.5
"#;

    const RUN_MATRIX_TOML: &str = r#"
openbmp.aerodb_run_matrix = 1
seed = 99

[grid]
mach = [0.5, 1.0]
alpha_deg = [0.0]
beta_deg = [0.0]
reynolds_length = [1000000.0]
knudsen = [0.0]
dynamic_pressure_pa = [100.0]
"#;

    fn deck_draw_scenario(sample_index: u64) -> String {
        format!(
            r#"
openbmp.scenario = 3

[meta]
name = "deck-draw-smoke"
description = "deck draw smoke test"
validation = "experimental"
provenance = "synthetic"

[time]
start_s = 0.0
stop_s = 0.02
dt_s = 0.01
seed = 1

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 50.0]

[vehicle.assembly]
id = "body"

[[vehicle.assembly.bodies]]
id = "main"
geometry = {{ kind = "reference", length_m = 2.4, area_m2 = 0.031415926535897934 }}
dry_mass_kg = 1.0

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "us_standard_1976"
wind = "none"

[forces]
models = ["gravity", "aero"]

[aero]
deck = "deck.toml"

[aero.deck_draw]
campaign_seed = 123
sample_index = {sample_index}
dimension_id_base = 20
mach_correlation_length = 1.5

[telemetry]
output.csv = "out/deck-draw-smoke.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#
        )
    }

    fn run_matrix_scenario() -> String {
        BUILDUP_SCENARIO
            .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
            .replace(
                "[aero]\n\n[aero.buildup]",
                "[aero]\n\n[aero.run_matrix]\nfile = \"matrix.toml\"\n\n[aero.buildup]",
            )
    }

    fn sigma_deck_files() -> BTreeMap<String, ResolvedFile> {
        BTreeMap::from([(
            "aero.deck".to_owned(),
            ResolvedFile::from_bytes("deck.toml", SIGMA_DECK_TOML.as_bytes().to_vec()),
        )])
    }

    fn six_sigma_deck_files() -> BTreeMap<String, ResolvedFile> {
        BTreeMap::from([(
            "aero.deck".to_owned(),
            ResolvedFile::from_bytes("deck.toml", SIX_SIGMA_DECK_TOML.as_bytes().to_vec()),
        )])
    }

    fn grid_sigma_deck_files() -> BTreeMap<String, ResolvedFile> {
        BTreeMap::from([(
            "aero.deck".to_owned(),
            ResolvedFile::from_bytes("deck.toml", SIGMA_GRID_DECK_TOML.as_bytes().to_vec()),
        )])
    }

    fn deck_draw_scenario_using_deck_correlation(sample_index: u64) -> String {
        deck_draw_scenario(sample_index).replace("mach_correlation_length = 1.5\n", "")
    }

    fn deck_draw_grid_scenario(sample_index: u64) -> String {
        deck_draw_scenario(sample_index).replace(
            "mach_correlation_length = 1.5\n",
            "mach_correlation_length = 1.5\nalpha_correlation_length_deg = 4.0\nbeta_correlation_length_deg = 3.0\n",
        )
    }

    #[test]
    fn load_aero_deck_bakes_buildup_without_resolved_file() {
        let scenario = Scenario::from_toml_str(BUILDUP_SCENARIO).unwrap();
        let deck = load_aero_deck(&scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();
        assert_eq!(deck.mach_grid(), &[0.0, 0.5, 1.0, 1.5, 2.0]);
        assert_eq!(deck.alpha_grid_deg(), &[0.0, 2.0, 4.0, 6.0]);
        assert_eq!(deck.beta_grid_deg(), &[0.0]);
        let coefficients = deck.lookup(0.5, 2.0, 0.0, &BTreeMap::new()).unwrap();
        assert!(coefficients.cd > 0.0);
    }

    #[test]
    fn load_aero_deck_bakes_datcom_buildup_when_enabled() {
        let legacy_scenario = Scenario::from_toml_str(BUILDUP_SCENARIO).unwrap();
        let datcom_toml =
            BUILDUP_SCENARIO.replace("[aero.buildup]\n", "[aero.buildup]\ndatcom = true\n");
        let datcom_scenario = Scenario::from_toml_str(&datcom_toml).unwrap();
        let legacy = load_aero_deck(&legacy_scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();
        let datcom = load_aero_deck(&datcom_scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();

        assert!(!legacy.has_dynamic_derivative_tables());
        assert!(datcom.has_dynamic_derivative_tables());
        let derivatives = datcom
            .lookup_dynamic_derivatives(2.0, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        assert!(derivatives.cm_q < 0.0);
        assert!(derivatives.cn_r < 0.0);
        assert!(derivatives.cl_p <= 0.0);

        let legacy_zero = legacy.lookup(2.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let datcom_zero = datcom.lookup(2.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert_eq!(legacy_zero.cn.to_bits(), datcom_zero.cn.to_bits());
        assert!(datcom_zero.cd < legacy_zero.cd);
        assert_eq!(legacy_zero.cm.to_bits(), datcom_zero.cm.to_bits());

        let legacy_subsonic_zero = legacy.lookup(0.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let datcom_subsonic_zero = datcom.lookup(0.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert_eq!(
            legacy_subsonic_zero.cd.to_bits(),
            datcom_subsonic_zero.cd.to_bits()
        );

        let legacy_alpha = legacy.lookup(2.0, 6.0, 0.0, &BTreeMap::new()).unwrap();
        let datcom_alpha = datcom.lookup(2.0, 6.0, 0.0, &BTreeMap::new()).unwrap();
        assert!(datcom_alpha.cn > legacy_alpha.cn);
        assert!(datcom_alpha.cd > legacy_alpha.cd);
    }

    #[test]
    fn load_aero_deck_applies_datcom_protuberance_drag_when_declared() {
        let datcom_toml =
            BUILDUP_SCENARIO.replace("[aero.buildup]\n", "[aero.buildup]\ndatcom = true\n");
        let protuberance_toml = datcom_toml.clone()
            + r#"

[aero.buildup.protuberances]
count = 2
frontal_area_m2 = 1.2e-4
drag_coefficient = 1.15
interference_factor = 1.1
"#;
        let datcom_scenario = Scenario::from_toml_str(&datcom_toml).unwrap();
        let protuberance_scenario = Scenario::from_toml_str(&protuberance_toml).unwrap();
        let datcom = load_aero_deck(&datcom_scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();
        let protuberance = load_aero_deck(&protuberance_scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();

        let datcom_zero = datcom.lookup(0.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let protuberance_zero = protuberance
            .lookup(0.5, 0.0, 0.0, &BTreeMap::new())
            .unwrap();
        assert_eq!(datcom_zero.cn.to_bits(), protuberance_zero.cn.to_bits());
        assert_eq!(datcom_zero.cm.to_bits(), protuberance_zero.cm.to_bits());
        assert!(protuberance_zero.cd > datcom_zero.cd);
    }

    #[test]
    fn load_declared_aero_run_matrix_parses_resolved_asset() {
        let scenario = Scenario::from_toml_str(&run_matrix_scenario()).unwrap();
        let files = BTreeMap::from([(
            "aero.run_matrix.file".to_owned(),
            ResolvedFile::from_bytes("matrix.toml", RUN_MATRIX_TOML.as_bytes().to_vec()),
        )]);

        let matrix = load_declared_aero_run_matrix(&scenario.document, &files)
            .unwrap()
            .unwrap();

        assert_eq!(matrix.seed(), 99);
        assert_eq!(matrix.nodes().len(), 2);
        assert_eq!(matrix.axes().mach, vec![0.5, 1.0]);
    }

    #[test]
    fn load_aero_deck_applies_declared_correlated_draw() {
        let nominal_toml = deck_draw_scenario(4).replace(
            "\n[aero.deck_draw]\ncampaign_seed = 123\nsample_index = 4\ndimension_id_base = 20\nmach_correlation_length = 1.5\n",
            "",
        );
        let nominal_scenario = Scenario::from_toml_str(&nominal_toml).unwrap();
        let drawn_scenario = Scenario::from_toml_str(&deck_draw_scenario(4)).unwrap();
        let deck_metadata_scenario =
            Scenario::from_toml_str(&deck_draw_scenario_using_deck_correlation(4)).unwrap();
        let different_scenario = Scenario::from_toml_str(&deck_draw_scenario(5)).unwrap();
        let files = sigma_deck_files();

        let nominal = load_aero_deck(&nominal_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_a = load_aero_deck(&drawn_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_b = load_aero_deck(&drawn_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_from_deck_metadata = load_aero_deck(&deck_metadata_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_c = load_aero_deck(&different_scenario.document, &files)
            .unwrap()
            .unwrap();

        assert_eq!(drawn_a, drawn_b);
        assert_eq!(drawn_a, drawn_from_deck_metadata);
        assert_ne!(drawn_a, drawn_c);
        assert!(drawn_a.has_sigma_tables());

        let nominal_coeffs = nominal.lookup(1.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let drawn_coeffs = drawn_a.lookup(1.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert!((drawn_coeffs.cn - nominal_coeffs.cn).abs() > 1.0e-12);

        let nominal_sigma = nominal
            .lookup_sigma(1.0, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        let drawn_sigma = drawn_a
            .lookup_sigma(1.0, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        assert_eq!(nominal_sigma, drawn_sigma);
    }

    #[test]
    fn load_aero_deck_applies_declared_alpha_beta_correlated_draw() {
        let nominal_toml = deck_draw_grid_scenario(4).replace(
            "\n[aero.deck_draw]\ncampaign_seed = 123\nsample_index = 4\ndimension_id_base = 20\nmach_correlation_length = 1.5\nalpha_correlation_length_deg = 4.0\nbeta_correlation_length_deg = 3.0\n",
            "",
        );
        let nominal_scenario = Scenario::from_toml_str(&nominal_toml).unwrap();
        let default_scenario = Scenario::from_toml_str(&deck_draw_scenario(4)).unwrap();
        let grid_scenario = Scenario::from_toml_str(&deck_draw_grid_scenario(4)).unwrap();
        let files = grid_sigma_deck_files();

        let nominal = load_aero_deck(&nominal_scenario.document, &files)
            .unwrap()
            .unwrap();
        let default_drawn = load_aero_deck(&default_scenario.document, &files)
            .unwrap()
            .unwrap();
        let grid_drawn_a = load_aero_deck(&grid_scenario.document, &files)
            .unwrap()
            .unwrap();
        let grid_drawn_b = load_aero_deck(&grid_scenario.document, &files)
            .unwrap()
            .unwrap();

        assert_eq!(grid_drawn_a, grid_drawn_b);

        let nominal_low = nominal.lookup(1.0, -5.0, 0.0, &BTreeMap::new()).unwrap();
        let nominal_high = nominal.lookup(1.0, 5.0, 3.0, &BTreeMap::new()).unwrap();
        let default_low = default_drawn
            .lookup(1.0, -5.0, 0.0, &BTreeMap::new())
            .unwrap();
        let default_high = default_drawn
            .lookup(1.0, 5.0, 3.0, &BTreeMap::new())
            .unwrap();
        let grid_low = grid_drawn_a
            .lookup(1.0, -5.0, 0.0, &BTreeMap::new())
            .unwrap();
        let grid_high = grid_drawn_a
            .lookup(1.0, 5.0, 3.0, &BTreeMap::new())
            .unwrap();

        let default_delta_low = default_low.cn - nominal_low.cn;
        let default_delta_high = default_high.cn - nominal_high.cn;
        let grid_delta_low = grid_low.cn - nominal_low.cn;
        let grid_delta_high = grid_high.cn - nominal_high.cn;

        assert!((default_delta_low - default_delta_high).abs() < 1.0e-14);
        assert_ne!(grid_delta_low.to_bits(), grid_delta_high.to_bits());
    }

    #[test]
    fn load_aero_deck_applies_declared_six_correlated_draw() {
        let nominal_toml = deck_draw_scenario(4).replace(
            "\n[aero.deck_draw]\ncampaign_seed = 123\nsample_index = 4\ndimension_id_base = 20\nmach_correlation_length = 1.5\n",
            "",
        );
        let nominal_scenario = Scenario::from_toml_str(&nominal_toml).unwrap();
        let drawn_scenario = Scenario::from_toml_str(&deck_draw_scenario(4)).unwrap();
        let files = six_sigma_deck_files();

        let nominal = load_aero_deck(&nominal_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_a = load_aero_deck(&drawn_scenario.document, &files)
            .unwrap()
            .unwrap();
        let drawn_b = load_aero_deck(&drawn_scenario.document, &files)
            .unwrap()
            .unwrap();

        assert_eq!(drawn_a, drawn_b);
        assert!(drawn_a.has_six_coefficient_tables());
        assert!(drawn_a.has_six_sigma_tables());

        let nominal_six = nominal
            .lookup_six(1.0, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        let drawn_six = drawn_a
            .lookup_six(1.0, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        assert!((drawn_six.cx - nominal_six.cx).abs() > 1.0e-12);
        assert!((drawn_six.cn - nominal_six.cn).abs() > 1.0e-12);

        let nominal_coeffs = nominal.lookup(1.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let drawn_coeffs = drawn_a.lookup(1.0, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert_eq!(nominal_coeffs, drawn_coeffs);
    }

    #[test]
    fn aero_upstream_uq_budget_reads_declared_coefficient_band() {
        let toml = BUILDUP_SCENARIO.replace(
            "[aero.buildup]\n",
            "[aero.uq]\n\
             deck_id = \"synthetic_finned_cylinder.v1\"\n\
             coefficient_id = \"cd\"\n\
             coefficient_lower = 0.18\n\
             coefficient_nominal = 0.20\n\
             coefficient_upper = 0.23\n\
             credibility_level = 2\n\
             evidence = \"data/aero/synthetic-finned-cylinder.toml#coefficient=cd\"\n\
             \n\
             [aero.buildup]\n",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let budget = upstream_uq_budget(&scenario.document).unwrap();

        assert_eq!(budget.sources.len(), 1);
        assert_eq!(
            budget.sources[0].source_id,
            "03.aero.synthetic_finned_cylinder.v1.cd"
        );
        assert_eq!(
            budget.sources[0].class,
            openbmp_uq::UncertaintyClass::Epistemic
        );
        assert_eq!(
            budget.sources[0].credibility.binding_level(),
            openbmp_uq::CredibilityLevel::L2
        );
        assert!((budget.sources[0].one_sigma - 0.03).abs() < 1.0e-15);
    }

    #[test]
    fn build_aero_method_wires_hybrid_knudsen_bridge() {
        let toml = format!(
            r#"{BUILDUP_SCENARIO}

[aero.method]
kind = "hybrid"

[aero.method.hybrid]
reference_length_m = 0.2
mach_handoff = 4.0

[aero.method.hybrid.continuum_low_mach]
kind = "deck"

[aero.method.hybrid.continuum_high_mach]
kind = "modified_newtonian"

[aero.method.hybrid.continuum_high_mach.modified_newtonian]
cp_max = 2.0
reference_area_m2 = 0.031415926535897934
reference_length_m = 0.2

[aero.method.hybrid.free_molecular]
reference_area_m2 = 0.031415926535897934
accommodation_normal = 1.0
accommodation_tangential = 1.0

[aero.method.hybrid.bridge]
kind = "linear"
kn_lo = 0.01
kn_hi = 10.0
"#
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let deck = load_aero_deck(&scenario.document, &BTreeMap::new())
            .unwrap()
            .unwrap();
        let (method, reference_length_m) = build_aero_method(&scenario.document, Some(deck))
            .unwrap()
            .unwrap();
        assert_eq!(reference_length_m.to_bits(), 0.2_f64.to_bits());

        let continuum_limit = method
            .aero_force_moment_body(&openbmp_aero::AeroContext {
                mach: 8.0,
                alpha_deg: 12.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 1_000.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
                knudsen: 1.0e-6,
                reynolds_length: 1.0e6,
            })
            .unwrap();
        let free_molecular_limit = method
            .aero_force_moment_body(&openbmp_aero::AeroContext {
                mach: 8.0,
                alpha_deg: 12.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 1_000.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
                knudsen: 100.0,
                reynolds_length: 1.0e6,
            })
            .unwrap();
        assert_ne!(
            continuum_limit.force_n_body.z.to_bits(),
            free_molecular_limit.force_n_body.z.to_bits(),
            "hybrid method should move between continuum and free-molecular limits"
        );
    }
}
