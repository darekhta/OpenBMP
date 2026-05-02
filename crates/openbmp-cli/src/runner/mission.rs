//! Phase-3.2 scenario → kernel mission-block conversion helpers.
//!
//! Bridges [`openbmp_scenario::MissionConfig`] (parsed from the
//! scenario `[mission]` block) to the [`openbmp_sim::EventBinding`]
//! list and [`openbmp_sim::MissionPhaseGraph`] the kernel's
//! `with_mission` builder consumes.
//!
//! Identifier convention: each phase / event id is path-derived from
//! the canonical path (`mission.phases.<id>` /
//! `mission.events.<id>`). Scenario files may provide either the bare
//! id (`ascent`) or the canonical path (`mission.phases.ascent`);
//! both forms resolve to the same stable id. Reordering the
//! `[[mission.phases]]` / `[[mission.events]]` blocks does not shift
//! any id; this is the load-bearing invariant for declaration-order-
//! independent determinism.

use std::collections::BTreeMap;

use openbmp_scenario::{
    EventActionConfig, EventConfig, EventTriggerConfig, MissionConfig, PhaseConfig,
    PhaseTransitionConfig,
};
use openbmp_sim::{
    BuiltInEventTrigger, EventAction, EventBinding, EventId, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition,
};

use crate::error::CliError;

/// Convert a parsed [`MissionConfig`] into the runtime types the
/// kernel consumes: a list of [`EventBinding`]s plus a validated
/// [`MissionPhaseGraph`].
///
/// # Errors
///
/// Returns [`CliError::Scenario`] when:
/// - A transition references an unknown phase or event id.
/// - The phase graph contains a cycle, an unreachable phase, or
///   duplicate phase ids.
/// - `mission.initial_phase` references an unknown id.
pub fn build_mission_runtime(
    mission: &MissionConfig,
) -> Result<(Vec<EventBinding>, MissionPhaseGraph), CliError> {
    let phase_id_lookup: BTreeMap<&str, PhaseId> = mission
        .phases
        .iter()
        .map(|p| (p.id.as_str(), phase_id(&p.id)))
        .collect();
    let event_id_lookup: BTreeMap<&str, EventId> = mission
        .events
        .iter()
        .map(|e| (e.id.as_str(), event_id(&e.id)))
        .collect();

    let phases: Vec<Phase> = mission.phases.iter().map(build_phase).collect();
    let mut event_bindings: Vec<EventBinding> = mission
        .events
        .iter()
        .map(|e| build_event_binding(e, &phase_id_lookup))
        .collect::<Result<_, _>>()?;
    event_bindings.sort_by_key(|event| event.id.value());
    let transitions: Vec<PhaseTransition> = mission
        .transitions
        .iter()
        .enumerate()
        .map(|(i, t)| build_transition(i, t, &phase_id_lookup, &event_id_lookup))
        .collect::<Result<_, _>>()?;

    let initial = phase_id_lookup
        .get(mission.initial_phase.as_str())
        .copied()
        .ok_or_else(|| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "mission.initial_phase = `{}` does not match any declared phase",
                    mission.initial_phase,
                ),
            })
        })?;

    let declared_event_ids: Vec<EventId> = event_bindings.iter().map(|b| b.id).collect();
    let graph = MissionPhaseGraph::new(phases, transitions, initial, &declared_event_ids).map_err(
        |err| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: err.to_string(),
            })
        },
    )?;

    Ok((event_bindings, graph))
}

fn phase_id(id: &str) -> PhaseId {
    if id.starts_with("mission.phases.") {
        PhaseId::from_path(id)
    } else {
        PhaseId::from_path(&format!("mission.phases.{id}"))
    }
}

fn event_id(id: &str) -> EventId {
    if id.starts_with("mission.events.") {
        EventId::from_path(id)
    } else {
        EventId::from_path(&format!("mission.events.{id}"))
    }
}

fn build_phase(config: &PhaseConfig) -> Phase {
    Phase {
        id: phase_id(&config.id),
        label: config.label.clone(),
        allowed_effectors: config.allowed_effectors.clone(),
        allowed_engines: config.allowed_engines.clone(),
    }
}

fn build_event_binding(
    config: &EventConfig,
    phase_id_lookup: &BTreeMap<&str, PhaseId>,
) -> Result<EventBinding, CliError> {
    Ok(EventBinding {
        id: event_id(&config.id),
        trigger: build_trigger(&config.trigger)?,
        action: build_action(&config.action, phase_id_lookup)?,
        once: config.once,
    })
}

fn build_trigger(config: &EventTriggerConfig) -> Result<BuiltInEventTrigger, CliError> {
    Ok(match config {
        EventTriggerConfig::AtTime { time_s } => BuiltInEventTrigger::AtTime { time_s: *time_s },
        EventTriggerConfig::AtAltitudeAscending { altitude_m } => {
            BuiltInEventTrigger::AtAltitudeAscending {
                meters: *altitude_m,
            }
        }
        EventTriggerConfig::AtAltitudeDescending { altitude_m } => {
            BuiltInEventTrigger::AtAltitudeDescending {
                meters: *altitude_m,
            }
        }
        EventTriggerConfig::AtApogee => BuiltInEventTrigger::AtApogee,
        EventTriggerConfig::AtMassFraction { remaining } => BuiltInEventTrigger::AtMassFraction {
            remaining: *remaining,
        },
        EventTriggerConfig::AtDynamicPressure { .. } => {
            return Err(CliError::Scenario(
                openbmp_scenario::ScenarioError::UnsupportedTriggerKind {
                    kind: "at_dynamic_pressure".to_owned(),
                    reason: "dynamic-pressure triggers ship in Phase 3.4 when atmosphere is wired into event evaluation".to_owned(),
                },
            ));
        }
        EventTriggerConfig::Scripted => {
            return Err(CliError::Scenario(
                openbmp_scenario::ScenarioError::UnsupportedTriggerKind {
                    kind: "scripted".to_owned(),
                    reason: "scripted triggers are deferred to a later Phase-3 sub-phase; use effector command_schedule for deterministic actuator scripts".to_owned(),
                },
            ));
        }
    })
}

fn build_action(
    config: &EventActionConfig,
    phase_id_lookup: &BTreeMap<&str, PhaseId>,
) -> Result<EventAction, CliError> {
    Ok(match config {
        EventActionConfig::EnterPhase { phase } => {
            let id = phase_id_lookup
                .get(phase.as_str())
                .copied()
                .ok_or_else(|| {
                    CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                        reason: format!("action.enter_phase references unknown phase `{phase}`"),
                    })
                })?;
            EventAction::EnterPhase(id)
        }
        EventActionConfig::EmitTelemetryMarker { tag } => {
            EventAction::EmitTelemetryMarker { tag: tag.clone() }
        }
        EventActionConfig::Stop { label } => EventAction::Stop {
            label: label.clone(),
        },
        // Phase-3.4: effector override resolves the scenario-text id
        // to a stable `EffectorId` (FNV of canonical effector path).
        EventActionConfig::EffectorOverride { id, command } => EventAction::EffectorOverride {
            id: openbmp_core::EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}")),
            command: *command,
        },
        // Phase-3.6: engine command resolves the scenario-text id to
        // a stable `EngineId` (FNV of canonical engine path) and
        // forwards the scenario `EngineCommandConfig` scalar fields.
        // Phase-3.15.C: the mission graph carries the scalar payload
        // directly; the runner-side rack constructs the typed
        // `openbmp_propulsion::EngineCommand` at apply time so the
        // mission graph crate has zero dependency on actuator-domain
        // crates.
        EventActionConfig::EngineCommand { id, command } => EventAction::EngineCommand {
            id: openbmp_core::EngineId::from_path(&format!("vehicle.assembly.engines.{id}")),
            throttle_unit: command.throttle_unit,
            gimbal_pitch_rad: command.gimbal_pitch_rad,
            gimbal_yaw_rad: command.gimbal_yaw_rad,
            ignite: command.ignite,
            shutdown: command.shutdown,
        },
        // Parser-rejected variants — defensively map to a stop-like
        // no-op. The runner does not normally reach these arms; if
        // they were to appear, the kernel's match for reserved
        // variants is a no-op.
        EventActionConfig::Separation => EventAction::Separation,
        // Phase-3.9: recovery-deploy resolves the scenario-text id to
        // a stable `RecoveryId` (FNV of canonical recovery path) and
        // forwards the canonical command-name string to the
        // runner-side `RecoveryRack::apply_deploys`.
        EventActionConfig::DeployRecovery { id, command } => EventAction::DeployRecovery {
            id: openbmp_core::RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}")),
            command: command.clone(),
        },
    })
}

fn build_transition(
    index: usize,
    config: &PhaseTransitionConfig,
    phase_id_lookup: &BTreeMap<&str, PhaseId>,
    event_id_lookup: &BTreeMap<&str, EventId>,
) -> Result<PhaseTransition, CliError> {
    let from = phase_id_lookup
        .get(config.from.as_str())
        .copied()
        .ok_or_else(|| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "transition #{index}.from = `{}` is not a declared phase",
                    config.from,
                ),
            })
        })?;
    let to = phase_id_lookup
        .get(config.to.as_str())
        .copied()
        .ok_or_else(|| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "transition #{index}.to = `{}` is not a declared phase",
                    config.to,
                ),
            })
        })?;
    let event = event_id_lookup
        .get(config.event.as_str())
        .copied()
        .ok_or_else(|| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "transition #{index}.event = `{}` is not a declared event",
                    config.event,
                ),
            })
        })?;
    Ok(PhaseTransition { from, to, event })
}

/// Collect the set of telemetry-marker tags declared in a
/// [`MissionConfig`]. Returned in `BTreeMap` order — alphabetical by
/// tag — so channel allocation is deterministic regardless of
/// scenario-text declaration order.
#[must_use]
pub fn marker_tags(mission: &MissionConfig) -> Vec<String> {
    let mut tags: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for event in &mission.events {
        if let EventActionConfig::EmitTelemetryMarker { tag } = &event.action {
            tags.insert(tag.clone());
        }
    }
    tags.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_and_bare_phase_ids_match() {
        assert_eq!(phase_id("ascent"), phase_id("mission.phases.ascent"));
        assert_eq!(event_id("liftoff"), event_id("mission.events.liftoff"));
    }
}
