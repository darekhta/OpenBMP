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
#[allow(deprecated)]
use openbmp_sim::EventAction;
use openbmp_sim::{
    BuiltInEventTrigger, EventBinding, EventId, MissionAction, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, ScenarioScriptAction,
};
use openbmp_mission::{MissionState, MissionStateMachine, HsmError};

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
/// Phase 5.X.A: typed split-binding constructor.
///
/// Returns the FC-owned mission bindings, the simulator-owned
/// scenario-script bindings, and the validated mission graph. Both
/// binding lists are id-sorted, preserving the canonical Phase 5
/// iteration order within each list.
///
/// Backward-compat: the legacy [`build_mission_runtime`] still
/// returns the unified `Vec<EventBinding<EventAction>>` consumed by
/// the kernel. Phase 5.X.B switches the kernel's `with_mission`
/// signature to consume the typed lists directly and retires the
/// legacy builder.
///
/// # Errors
///
/// Same conditions as [`build_mission_runtime`].
pub fn build_mission_runtime_typed(
    mission: &MissionConfig,
) -> Result<
    (
        Vec<EventBinding<MissionAction>>,
        Vec<EventBinding<ScenarioScriptAction>>,
        MissionPhaseGraph,
    ),
    CliError,
> {
    #[allow(deprecated)]
    let (unified, graph) = build_mission_runtime(mission)?;
    let mission_bindings = project_mission_bindings(&unified);
    let script_bindings = project_script_bindings(&unified);
    Ok((mission_bindings, script_bindings, graph))
}

/// Phase 5.X.F: v3 → v4 lifting pass. Builds a [`MissionStateMachine`]
/// from either the v3 `[[mission.phases]]` block (treated as a flat
/// depth-0 hierarchy where every state has no parent and empty
/// action lists) or, when populated, the v4
/// `[[mission.states]]` block with hierarchical `parent` fields.
///
/// Returns a `MissionStateMachine` that downstream subscribers
/// (Phase 5.X.B simulator subscriber, FC commander hierarchical
/// upgrade) consume. The flat-DAG [`MissionPhaseGraph`] returned by
/// [`build_mission_runtime`] continues to flow through the
/// simulator kernel during the migration window for byte-identical
/// determinism.
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
        return MissionStateMachine::new(states, initial).map_err(hsm_to_cli_error);
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
    MissionStateMachine::new(states, initial).map_err(hsm_to_cli_error)
}

fn hsm_to_cli_error(err: HsmError) -> CliError {
    CliError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
        reason: err.to_string(),
    })
}

/// Phase-3.2 legacy unified builder.
///
/// Returns the unified `Vec<EventBinding<EventAction>>` list and the
/// validated mission graph. Phase 5.X.A introduced
/// [`build_mission_runtime_typed`] which returns the split typed
/// lists; this builder remains during the migration window because
/// the simulator kernel still consumes the unified list. Phase 5.X.B
/// retires it once the kernel's `with_mission` switches to typed
/// inputs.
///
/// # Errors
///
/// Returns [`CliError::Scenario`] when:
/// - A transition references an unknown phase or event id.
/// - The phase graph contains a cycle, an unreachable phase, or
///   duplicate phase ids.
/// - `mission.initial_phase` references an unknown id.
#[allow(deprecated)]
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

#[allow(deprecated)]
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

/// Phase 5.X.A: project a legacy unified [`EventBinding<EventAction>`]
/// list down to the HAL-portable mission-action bindings only. The
/// commander owns these; the simulator-only physics-override
/// variants (engine / effector / separation / recovery) are dropped.
///
/// Iteration order is preserved (canonical id-sorted), so each binding's
/// position in the projected list matches its position in the source
/// list among mission-action bindings.
#[allow(deprecated)]
#[must_use]
pub fn project_mission_bindings(
    bindings: &[EventBinding],
) -> Vec<EventBinding<MissionAction>> {
    bindings
        .iter()
        .filter_map(|b| {
            let action = match &b.action {
                EventAction::EnterPhase(p) => MissionAction::EnterState(*p),
                EventAction::EmitTelemetryMarker { tag } => {
                    MissionAction::EmitTelemetryMarker { tag: tag.clone() }
                }
                EventAction::Stop { label } => MissionAction::Stop {
                    label: label.clone(),
                },
                EventAction::EngineCommand { .. }
                | EventAction::EffectorOverride { .. }
                | EventAction::Separation
                | EventAction::DeployRecovery { .. } => return None,
            };
            Some(EventBinding {
                id: b.id,
                trigger: b.trigger.clone(),
                action,
                once: b.once,
            })
        })
        .collect()
}

/// Phase 5.X.A: project a legacy unified [`EventBinding<EventAction>`]
/// list down to the simulator-only [`ScenarioScriptAction`] bindings.
/// The simulator kernel consumes these; HAL adopters do not link the
/// `openbmp-scenario-script` crate.
///
/// Iteration order is preserved (canonical id-sorted).
#[allow(deprecated)]
#[must_use]
pub fn project_script_bindings(
    bindings: &[EventBinding],
) -> Vec<EventBinding<ScenarioScriptAction>> {
    bindings
        .iter()
        .filter_map(|b| {
            let action = match &b.action {
                EventAction::EngineCommand {
                    id,
                    throttle_unit,
                    gimbal_pitch_rad,
                    gimbal_yaw_rad,
                    ignite,
                    shutdown,
                } => ScenarioScriptAction::EngineCommand {
                    id: *id,
                    throttle_unit: *throttle_unit,
                    gimbal_pitch_rad: *gimbal_pitch_rad,
                    gimbal_yaw_rad: *gimbal_yaw_rad,
                    ignite: *ignite,
                    shutdown: *shutdown,
                },
                EventAction::EffectorOverride { id, command } => {
                    ScenarioScriptAction::EffectorOverride {
                        id: *id,
                        command: *command,
                    }
                }
                EventAction::Separation => ScenarioScriptAction::Separation,
                EventAction::DeployRecovery { id, command } => {
                    ScenarioScriptAction::DeployRecovery {
                        id: *id,
                        command: command.clone(),
                    }
                }
                EventAction::EnterPhase(_)
                | EventAction::EmitTelemetryMarker { .. }
                | EventAction::Stop { .. } => return None,
            };
            Some(EventBinding {
                id: b.id,
                trigger: b.trigger.clone(),
                action,
                once: b.once,
            })
        })
        .collect()
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

#[allow(deprecated)]
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
