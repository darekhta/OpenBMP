//! Phase-3.2 scenario → kernel mission-block conversion helpers.
//!
//! Bridges [`openbmp_scenario::MissionConfig`] (parsed from the
//! scenario `[mission]` block) to typed [`openbmp_sim::EventBinding`]
//! lists and the [`openbmp_sim::MissionPhaseGraph`] the kernel
//! consumes.
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

use openbmp_mission::{HsmError, MissionState, MissionStateMachine};
use openbmp_scenario::{
    EventActionConfig, EventConfig, EventTriggerConfig, MissionConfig, PhaseConfig,
    PhaseTransitionConfig,
};
use openbmp_sim::{
    BuiltInEventTrigger, EventBinding, EventId, MissionAction, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, ScenarioScriptAction,
};

use crate::error::CliError;

/// Split mission runtime produced from a parsed scenario mission
/// block: FC-owned mission bindings, simulator-owned script bindings,
/// and the validated flat mission graph.
pub type MissionRuntime = (
    Vec<EventBinding<MissionAction>>,
    Vec<EventBinding<ScenarioScriptAction>>,
    MissionPhaseGraph,
);

enum RuntimeEventBinding {
    Mission(EventBinding<MissionAction>),
    Script(EventBinding<ScenarioScriptAction>),
}

/// Phase 5.X.F: v3 → v4 lifting pass. Builds a [`MissionStateMachine`]
/// from either the v3 `[[mission.phases]]` block (treated as a flat
/// depth-0 hierarchy where every state has no parent and empty
/// action lists) or, when populated, the v4
/// `[[mission.states]]` block with hierarchical `parent` fields.
///
/// Returns a `MissionStateMachine` that downstream subscribers
/// (Phase 5.X.B simulator subscriber, FC commander hierarchical
/// upgrade) consume.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] when the v3 phases / v4 states
/// fail HSM validation (duplicate ids, unknown parents, parent
/// cycles, unreachable states, missing initial).
pub fn lift_mission_state_machine(
    mission: &MissionConfig,
) -> Result<MissionStateMachine, CliError> {
    // v4 path: when `[[mission.states]]` is populated, prefer it.
    if !mission.states.is_empty() {
        let states: Vec<MissionState> = mission
            .states
            .iter()
            .map(|s| MissionState {
                id: phase_id(&s.id),
                label: s.label.clone(),
                parent: s.parent.as_deref().map(phase_id),
                on_entry: Vec::new(), // Phase 5.X.F.2 wires action lists
                on_exit: Vec::new(),
                on_active: Vec::new(),
                allowed_effectors: s.allowed_effectors.clone(),
                allowed_engines: s.allowed_engines.clone(),
            })
            .collect();
        let initial = phase_id(&mission.initial_phase);
        return MissionStateMachine::new(states, initial).map_err(|err| hsm_to_cli_error(&err));
    }

    // v3 lifting: every phase becomes a top-level (depth-0) state
    // with no parent and empty action lists. FNV-1a-64 ids are
    // identical to the v3 path because the canonical-path string is
    // the same.
    let states: Vec<MissionState> = mission
        .phases
        .iter()
        .map(|p| MissionState {
            id: phase_id(&p.id),
            label: p.label.clone(),
            parent: None,
            on_entry: Vec::new(),
            on_exit: Vec::new(),
            on_active: Vec::new(),
            allowed_effectors: p.allowed_effectors.clone(),
            allowed_engines: p.allowed_engines.clone(),
        })
        .collect();
    let initial = phase_id(&mission.initial_phase);
    MissionStateMachine::new(states, initial).map_err(|err| hsm_to_cli_error(&err))
}

fn hsm_to_cli_error(err: &HsmError) -> CliError {
    CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
        reason: err.to_string(),
    })
}

/// Convert a parsed [`MissionConfig`] into the typed runtime values
/// consumed by the kernel and FC commander.
///
/// Returns the FC-owned mission bindings, the simulator-owned
/// scenario-script bindings, and the validated mission graph. Both
/// binding lists are id-sorted.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] when a transition references an
/// unknown phase or event id, the phase graph is invalid, or
/// `mission.initial_phase` references an unknown id.
pub fn build_mission_runtime_typed(mission: &MissionConfig) -> Result<MissionRuntime, CliError> {
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
    let runtime_bindings: Vec<RuntimeEventBinding> = mission
        .events
        .iter()
        .map(|event| build_event_binding(event, &phase_id_lookup))
        .collect::<Result<_, _>>()?;
    let mut mission_bindings = Vec::new();
    let mut script_bindings = Vec::new();
    let mut declared_event_ids = Vec::with_capacity(runtime_bindings.len());
    for binding in runtime_bindings {
        match binding {
            RuntimeEventBinding::Mission(binding) => {
                declared_event_ids.push(binding.id);
                mission_bindings.push(binding);
            }
            RuntimeEventBinding::Script(binding) => {
                declared_event_ids.push(binding.id);
                script_bindings.push(binding);
            }
        }
    }
    mission_bindings.sort_by_key(|event| event.id.value());
    script_bindings.sort_by_key(|event| event.id.value());
    declared_event_ids.sort_by_key(|event| event.value());

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

    let graph = MissionPhaseGraph::new(phases, transitions, initial, &declared_event_ids).map_err(
        |err| {
            CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: err.to_string(),
            })
        },
    )?;

    Ok((mission_bindings, script_bindings, graph))
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
) -> Result<RuntimeEventBinding, CliError> {
    let id = event_id(&config.id);
    let trigger = build_trigger(&config.trigger)?;
    Ok(match &config.action {
        EventActionConfig::EnterPhase { phase } => {
            let target = phase_id_lookup
                .get(phase.as_str())
                .copied()
                .ok_or_else(|| {
                    CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                        reason: format!("action.enter_phase references unknown phase `{phase}`"),
                    })
                })?;
            RuntimeEventBinding::Mission(EventBinding {
                id,
                trigger,
                action: MissionAction::EnterState(target),
                once: config.once,
            })
        }
        EventActionConfig::EmitTelemetryMarker { tag } => {
            RuntimeEventBinding::Mission(EventBinding {
                id,
                trigger,
                action: MissionAction::EmitTelemetryMarker { tag: tag.clone() },
                once: config.once,
            })
        }
        EventActionConfig::Stop { label } => RuntimeEventBinding::Mission(EventBinding {
            id,
            trigger,
            action: MissionAction::Stop {
                label: label.clone(),
            },
            once: config.once,
        }),
        EventActionConfig::EffectorOverride {
            id: effector,
            command,
        } => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::EffectorOverride {
                id: openbmp_core::EffectorId::from_path(&format!(
                    "vehicle.assembly.effectors.{effector}"
                )),
                command: *command,
            },
            once: config.once,
        }),
        EventActionConfig::EngineCommand {
            id: engine,
            command,
        } => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::EngineCommand {
                id: openbmp_core::EngineId::from_path(&format!(
                    "vehicle.assembly.engines.{engine}"
                )),
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            },
            once: config.once,
        }),
        EventActionConfig::Separation => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::Separation,
            once: config.once,
        }),
        EventActionConfig::DeployRecovery {
            id: recovery,
            command,
        } => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::DeployRecovery {
                id: openbmp_core::RecoveryId::from_path(&format!(
                    "vehicle.assembly.recovery.{recovery}"
                )),
                command: command.clone(),
            },
            once: config.once,
        }),
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
