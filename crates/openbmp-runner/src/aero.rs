//! Runner-side aerodynamic coefficient source loading.

use std::collections::BTreeMap;

use openbmp_aero::{
    AccommodationCoeffs, AeroDeck, AeroError, AeroMethod, Afterbody, BodyGeometry, BuildupGrid,
    ChengBridge, ComponentBuildup, DeckLookup, DragBuildupModel, ErfcBridge, FinSet,
    FreeMolecularAero, HybridAeroMethod, LinearKnudsenBridge, LinearMachBridge, ModifiedNewtonian,
    NoseShape, TangentCone, TangentWedge,
};
use openbmp_core::SimTime;
use openbmp_physics::AtmosphereModel;
use openbmp_scenario::{
    AeroBuildupConfig, AeroBuildupNoseConfig, AeroFreeMolecularConfig, AeroKnudsenBridgeConfig,
    AeroMethodConfig, ResolvedFile, ScenarioDocument,
};

use crate::atmosphere::build_document_runtime_atmosphere;
use crate::error::RunnerError;

const HYPERSONIC_DECK_GUARD_MACH: f64 = 5.0;

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
    if aero.deck.is_some() {
        return load_deck_file(resolved_files).map(Some);
    }
    if let Some(buildup) = &aero.buildup {
        return bake_buildup_deck(document, buildup).map(Some);
    }
    Ok(None)
}

/// Build the live [`AeroMethod`] selected by `[aero.method]`.
///
/// Returns `Ok(None)` when the scenario has no `[aero]` block.
/// Absent `[aero.method]` or `kind = "deck"` wraps the loaded/baked
/// deck as [`DeckLookup`].
pub fn build_aero_method(
    document: &ScenarioDocument,
    deck: Option<AeroDeck>,
) -> Result<Option<(Box<dyn AeroMethod>, f64)>, RunnerError> {
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
        reference_area_m2: config.reference_area_m2,
        reference_length_m: config.reference_length_m,
        center_of_gravity_from_nose_m: config.center_of_gravity_from_nose_m,
    };
    Ok(ComponentBuildup::default().bake_deck(&geom, &grid)?)
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
    use openbmp_scenario::Scenario;

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
