//! Phase-3.2 scenario → kernel mission-block conversion helpers.
//!
//! Bridges [`openbmp_scenario::MissionConfig`] (parsed from the
//! scenario `[mission]` block) to the [`openbmp_sim::EventBinding`]
//! list and [`openbmp_sim::MissionPhaseGraph`] the kernel's
//! `with_mission` builder consumes.
//!
//! Identifier convention: each phase / event id is path-derived via
//! `PhaseId::from_path("mission.phases.<id>")` and
//! `EventId::from_path("mission.events.<id>")`. Reordering the
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
    let event_bindings: Vec<EventBinding> = mission
        .events
        .iter()
        .map(|e| build_event_binding(e, &phase_id_lookup))
        .collect::<Result<_, _>>()?;
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
    PhaseId::from_path(&format!("mission.phases.{id}"))
}

fn event_id(id: &str) -> EventId {
    EventId::from_path(&format!("mission.events.{id}"))
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
        trigger: build_trigger(&config.trigger),
        action: build_action(&config.action, phase_id_lookup)?,
        once: config.once,
    })
}

fn build_trigger(config: &EventTriggerConfig) -> BuiltInEventTrigger {
    match config {
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
        EventTriggerConfig::AtDynamicPressure {
            pressure_pa,
            falling,
        } => BuiltInEventTrigger::AtDynamicPressure {
            pa: *pressure_pa,
            falling: *falling,
        },
        EventTriggerConfig::Scripted => {
            // Parser rejects this variant; kernel will never see it.
            // Defensive default: treat as never-firing AtApogee.
            BuiltInEventTrigger::AtApogee
        }
    }
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
        // Parser-rejected variants — defensively map to a stop-like
        // no-op. The runner does not normally reach these arms; if
        // they were to appear, the kernel's match for reserved
        // variants is a no-op.
        EventActionConfig::EngineCommand => EventAction::EngineCommand,
        EventActionConfig::EffectorOverride => EventAction::EffectorOverride,
        EventActionConfig::Separation => EventAction::Separation,
        EventActionConfig::DeployRecovery => EventAction::DeployRecovery,
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
