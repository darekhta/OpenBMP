//! Scenario → kernel mission-block conversion helpers.
//!
//! Bridges [`openbmp_scenario::MissionConfig`] (parsed from the
//! scenario `[mission]` block) to typed [`openbmp_sim::EventBinding`]
//! lists and the [`openbmp_sim::MissionPhaseGraph`] the kernel
//! consumes.
//!
//! Identifier convention: each phase / state / event id is
//! path-derived from its canonical path (`mission.phases.<id>`,
//! `mission.states.<id>`, or `mission.events.<id>`). Scenario files
//! may provide either the bare id (`ascent`) or the canonical path
//! (`mission.phases.ascent`); bare mission states intentionally keep
//! the `mission.phases.<id>` namespace so v3 → v4 flat
//! migrations can remain byte-identical. Reordering the
//! `[[mission.phases]]` / `[[mission.events]]` blocks does not shift
//! any id; this is the load-bearing invariant for declaration-order-
//! independent determinism.

use std::collections::BTreeMap;

use openbmp_mission::{
    CanonicalRegionStates, CanonicalRegions, HsmError, MissionState, MissionStateMachine, Region,
    RegionError, RegionSet,
};
use openbmp_scenario::{
    EventConfig, EventTriggerConfig, MissionConfig, PhaseConfig, PhaseTransitionConfig,
    RegionConfig, RegionStateConfig, ScenarioActionConfig, StateConfig,
};
use openbmp_sim::{
    AlarmCode, BuiltInEventTrigger, EventBinding, EventId, MissionAction, MissionPhaseGraph, Phase,
    PhaseId, PhaseTransition, RegionId, ScenarioScriptAction,
};

use crate::error::RunnerError;

/// Split mission runtime produced from a parsed scenario mission block.
#[derive(Clone, Debug)]
pub struct MissionRuntime {
    /// FC-owned mission bindings.
    pub mission_bindings: Vec<EventBinding<MissionAction>>,
    /// Simulator-owned script bindings.
    pub script_bindings: Vec<EventBinding<ScenarioScriptAction>>,
    /// Flat transition graph. Production code still uses this for
    /// transition edges, while the HSM supplies entry / exit chains.
    pub graph: MissionPhaseGraph,
    /// Lifted hierarchical state machine.
    pub hsm: MissionStateMachine,
    /// Canonical orthogonal regions.
    pub regions: RegionSet,
}

enum RuntimeEventBinding {
    Mission(EventBinding<MissionAction>),
    Script(EventBinding<ScenarioScriptAction>),
}

/// v3 → v4 lifting pass. Builds a [`MissionStateMachine`]
/// from either the v3 `[[mission.phases]]` block (treated as a flat
/// depth-0 hierarchy where every state has no parent and empty
/// action lists) or, when populated, the v4
/// `[[mission.states]]` block with hierarchical `parent` fields.
///
/// Returns a `MissionStateMachine` consumed by the FC commander and
/// pure-sim kernel for transition entry / exit chains.
///
/// # Errors
///
/// Returns [`RunnerError::Scenario`] when the v3 phases / v4 states
/// fail HSM validation (duplicate ids, unknown parents, parent
/// cycles, unreachable states, missing initial).
pub fn lift_mission_state_machine(
    mission: &MissionConfig,
) -> Result<MissionStateMachine, RunnerError> {
    // v4 path: when `[[mission.states]]` is populated, prefer it.
    if !mission.states.is_empty() {
        let states: Vec<MissionState> = mission
            .states
            .iter()
            .map(|s| {
                Ok(MissionState {
                    id: phase_id(&s.id),
                    label: s.label.clone(),
                    parent: s.parent.as_deref().map(phase_id),
                    on_entry: mission_actions(&s.on_entry, "mission.states[].on_entry")?,
                    on_exit: mission_actions(&s.on_exit, "mission.states[].on_exit")?,
                    on_active: mission_actions(&s.on_active, "mission.states[].on_active")?,
                    allowed_effectors: s.allowed_effectors.clone(),
                    allowed_engines: s.allowed_engines.clone(),
                })
            })
            .collect::<Result<_, RunnerError>>()?;
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

fn hsm_to_cli_error(err: &HsmError) -> RunnerError {
    RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
        reason: err.to_string(),
    })
}

fn region_to_cli_error(err: &RegionError) -> RunnerError {
    RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
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
/// Returns [`RunnerError::Scenario`] when a transition references an
/// unknown phase or event id, the phase graph is invalid, or
/// `mission.initial_phase` references an unknown id.
pub fn build_mission_runtime_typed(mission: &MissionConfig) -> Result<MissionRuntime, RunnerError> {
    let phase_id_lookup = phase_lookup(mission);
    let event_id_lookup: BTreeMap<&str, EventId> = mission
        .events
        .iter()
        .map(|e| (e.id.as_str(), event_id(&e.id)))
        .collect();

    let phases = build_phases(mission);
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
            RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "mission.initial_phase = `{}` does not match any declared phase/state",
                    mission.initial_phase,
                ),
            })
        })?;

    let hsm = lift_mission_state_machine(mission)?;
    let graph = MissionPhaseGraph::new(phases, transitions, initial, &declared_event_ids).map_err(
        |err| {
            RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
                reason: err.to_string(),
            })
        },
    )?;
    let regions = build_region_set(mission, &graph)?;

    Ok(MissionRuntime {
        mission_bindings,
        script_bindings,
        graph,
        hsm,
        regions,
    })
}

fn phase_id(id: &str) -> PhaseId {
    if id.starts_with("mission.phases.") || id.starts_with("mission.states.") {
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

fn build_phase_from_state(config: &StateConfig) -> Phase {
    Phase {
        id: phase_id(&config.id),
        label: if config.label.is_empty() {
            config.id.clone()
        } else {
            config.label.clone()
        },
        allowed_effectors: config.allowed_effectors.clone(),
        allowed_engines: config.allowed_engines.clone(),
    }
}

fn phase_lookup(mission: &MissionConfig) -> BTreeMap<&str, PhaseId> {
    if mission.states.is_empty() {
        mission
            .phases
            .iter()
            .map(|p| (p.id.as_str(), phase_id(&p.id)))
            .collect()
    } else {
        mission
            .states
            .iter()
            .map(|s| (s.id.as_str(), phase_id(&s.id)))
            .collect()
    }
}

fn build_phases(mission: &MissionConfig) -> Vec<Phase> {
    if mission.states.is_empty() {
        mission.phases.iter().map(build_phase).collect()
    } else {
        mission.states.iter().map(build_phase_from_state).collect()
    }
}

fn mission_actions(
    actions: &[ScenarioActionConfig],
    field: &'static str,
) -> Result<Vec<MissionAction>, RunnerError> {
    actions
        .iter()
        .map(|action| match action {
            ScenarioActionConfig::EnterPhase { phase } => {
                Ok(MissionAction::EnterState(phase_id(phase)))
            }
            ScenarioActionConfig::EmitTelemetryMarker { tag } => {
                Ok(MissionAction::EmitTelemetryMarker { tag: tag.clone() })
            }
            ScenarioActionConfig::Stop { label } => Ok(MissionAction::Stop {
                label: label.clone(),
            }),
            ScenarioActionConfig::RaiseHealthAlarm { region, alarm } => {
                Ok(MissionAction::RaiseHealthAlarm {
                    region: resolve_region_id(region.as_deref()),
                    alarm: AlarmCode::new(*alarm),
                })
            }
            ScenarioActionConfig::RequestSafeState { reason } => {
                Ok(MissionAction::RequestSafeState {
                    reason: reason.clone(),
                })
            }
            ScenarioActionConfig::EffectorOverride { .. }
            | ScenarioActionConfig::EngineCommand { .. }
            | ScenarioActionConfig::Separation
            | ScenarioActionConfig::JettisonStage { .. }
            | ScenarioActionConfig::SelectGuidanceProfile { .. }
            | ScenarioActionConfig::DeployRecovery { .. } => Err(RunnerError::Scenario(
                openbmp_scenario::ScenarioError::MissionGraph {
                    reason: format!("{field} may contain only HAL-portable mission actions"),
                },
            )),
        })
        .collect()
}

#[allow(clippy::too_many_lines)] // expanded with RaiseHealthAlarm / RequestSafeState branches.
fn build_event_binding(
    config: &EventConfig,
    phase_id_lookup: &BTreeMap<&str, PhaseId>,
) -> Result<RuntimeEventBinding, RunnerError> {
    let id = event_id(&config.id);
    let trigger = build_trigger(&config.trigger)?;
    Ok(match &config.action {
        ScenarioActionConfig::EnterPhase { phase } => {
            let target = phase_id_lookup
                .get(phase.as_str())
                .copied()
                .ok_or_else(|| {
                    RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
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
        ScenarioActionConfig::EmitTelemetryMarker { tag } => {
            RuntimeEventBinding::Mission(EventBinding {
                id,
                trigger,
                action: MissionAction::EmitTelemetryMarker { tag: tag.clone() },
                once: config.once,
            })
        }
        ScenarioActionConfig::Stop { label } => RuntimeEventBinding::Mission(EventBinding {
            id,
            trigger,
            action: MissionAction::Stop {
                label: label.clone(),
            },
            once: config.once,
        }),
        ScenarioActionConfig::EffectorOverride {
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
        ScenarioActionConfig::EngineCommand {
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
        ScenarioActionConfig::Separation => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::Separation,
            once: config.once,
        }),
        ScenarioActionConfig::DeployRecovery {
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
        ScenarioActionConfig::JettisonStage { body } => RuntimeEventBinding::Script(EventBinding {
            id,
            trigger,
            action: ScenarioScriptAction::JettisonStage {
                body: openbmp_core::BodyId::from_path(&format!("vehicle.assembly.bodies.{body}")),
            },
            once: config.once,
        }),
        ScenarioActionConfig::RaiseHealthAlarm { region, alarm } => {
            RuntimeEventBinding::Mission(EventBinding {
                id,
                trigger,
                action: MissionAction::RaiseHealthAlarm {
                    region: resolve_region_id(region.as_deref()),
                    alarm: AlarmCode::new(*alarm),
                },
                once: config.once,
            })
        }
        ScenarioActionConfig::RequestSafeState { reason } => {
            RuntimeEventBinding::Mission(EventBinding {
                id,
                trigger,
                action: MissionAction::RequestSafeState {
                    reason: reason.clone(),
                },
                once: config.once,
            })
        }
        // Deferred flight-profile action. Rejected at scenario
        // `validate()`; the runner never sees it, but the match is
        // kept exhaustive and fails closed defensively. See
        // docs/ascent-guidance.md.
        ScenarioActionConfig::SelectGuidanceProfile { .. } => {
            return Err(RunnerError::Scenario(
                openbmp_scenario::ScenarioError::UnsupportedActionKind {
                    kind: "select_guidance_profile".to_owned(),
                    missing_capability: "runtime guidance-profile switching".to_owned(),
                },
            ));
        }
    })
}

/// Resolve a scenario-text region id to a stable `RegionId`. Bare
/// names (e.g. `"health"`) resolve to the canonical region path
/// `"mission.regions.<id>"`; absent / empty values default to
/// `mission.regions.health` (the load-bearing target of
/// `raise_health_alarm`).
fn resolve_region_id(id: Option<&str>) -> RegionId {
    match id {
        Some(id) if id.starts_with("mission.regions.") => RegionId::from_path(id),
        Some(id) if !id.is_empty() => RegionId::from_path(&format!("mission.regions.{id}")),
        _ => openbmp_mission::CanonicalRegions::health(),
    }
}

fn build_trigger(config: &EventTriggerConfig) -> Result<BuiltInEventTrigger, RunnerError> {
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
            return Err(RunnerError::Scenario(
                openbmp_scenario::ScenarioError::UnsupportedTriggerKind {
                    kind: "at_dynamic_pressure".to_owned(),
                    reason: "dynamic-pressure triggers are not yet supported; they require atmosphere wired into event evaluation".to_owned(),
                },
            ));
        }
        EventTriggerConfig::Scripted => {
            return Err(RunnerError::Scenario(
                openbmp_scenario::ScenarioError::UnsupportedTriggerKind {
                    kind: "scripted".to_owned(),
                    reason: "scripted triggers are not yet supported; use effector command_schedule for deterministic actuator scripts".to_owned(),
                },
            ));
        }
    })
}

fn build_region_set(
    mission: &MissionConfig,
    graph: &MissionPhaseGraph,
) -> Result<RegionSet, RunnerError> {
    let mut regions = default_region_set(graph)?;
    for region in &mission.regions {
        let id = region_id(&region.id);
        if id == CanonicalRegions::mission() {
            // The mission region is the production mission graph.
            // `[[mission.regions]]` may mention it for documentation,
            // but it cannot replace the already-validated graph.
            continue;
        }
        regions.insert(build_region(region)?);
    }
    validate_required_canonical_region_states(&regions)?;
    Ok(regions)
}

fn default_region_set(graph: &MissionPhaseGraph) -> Result<RegionSet, RunnerError> {
    let mut regions = RegionSet::new();
    regions.insert(Region::new(CanonicalRegions::mission(), graph.clone()));
    regions.insert(
        Region::from_states(
            CanonicalRegions::health(),
            vec![
                region_phase(CanonicalRegionStates::health_nominal(), "nominal"),
                region_phase(CanonicalRegionStates::health_degraded(), "degraded"),
                region_phase(
                    CanonicalRegionStates::health_abort_requested(),
                    "abort_requested",
                ),
                region_phase(
                    CanonicalRegionStates::health_safed_on_fault(),
                    "safed_on_fault",
                ),
            ],
            CanonicalRegionStates::health_nominal(),
        )
        .map_err(|err| region_to_cli_error(&err))?,
    );
    regions.insert(
        Region::from_states(
            CanonicalRegions::comms(),
            vec![region_phase(
                CanonicalRegionStates::comms_linked(),
                "linked",
            )],
            CanonicalRegionStates::comms_linked(),
        )
        .map_err(|err| region_to_cli_error(&err))?,
    );
    regions.insert(
        Region::from_states(
            CanonicalRegions::estimator_regime(),
            vec![region_phase(
                CanonicalRegionStates::estimator_boost_mode(),
                "boost_mode",
            )],
            CanonicalRegionStates::estimator_boost_mode(),
        )
        .map_err(|err| region_to_cli_error(&err))?,
    );
    Ok(regions)
}

fn build_region(config: &RegionConfig) -> Result<Region, RunnerError> {
    let id = region_id(&config.id);
    let states: Vec<Phase> = config
        .states
        .iter()
        .map(|state| build_region_state(&config.id, state))
        .collect();
    let initial = region_state_id_by_name(&config.id, &config.initial_state);
    Region::from_states(id, states, initial).map_err(|err| region_to_cli_error(&err))
}

fn build_region_state(region_name: &str, config: &RegionStateConfig) -> Phase {
    Phase {
        id: region_state_id_by_name(region_name, &config.id),
        label: if config.label.is_empty() {
            config.id.clone()
        } else {
            config.label.clone()
        },
        allowed_effectors: Vec::new(),
        allowed_engines: Vec::new(),
    }
}

fn validate_required_canonical_region_states(regions: &RegionSet) -> Result<(), RunnerError> {
    require_region_state(
        regions,
        CanonicalRegions::mission(),
        "mission",
        None,
        "mission.regions.mission",
    )?;
    require_region_state(
        regions,
        CanonicalRegions::health(),
        "health",
        Some(CanonicalRegionStates::health_nominal()),
        "mission.regions.health.nominal",
    )?;
    require_region_state(
        regions,
        CanonicalRegions::health(),
        "health",
        Some(CanonicalRegionStates::health_abort_requested()),
        "mission.regions.health.abort_requested",
    )?;
    require_region_state(
        regions,
        CanonicalRegions::health(),
        "health",
        Some(CanonicalRegionStates::health_safed_on_fault()),
        "mission.regions.health.safed_on_fault",
    )?;
    require_region_state(
        regions,
        CanonicalRegions::comms(),
        "comms",
        Some(CanonicalRegionStates::comms_linked()),
        "mission.regions.comms.linked",
    )?;
    require_region_state(
        regions,
        CanonicalRegions::estimator_regime(),
        "estimator_regime",
        Some(CanonicalRegionStates::estimator_boost_mode()),
        "mission.regions.estimator_regime.boost_mode",
    )?;
    Ok(())
}

fn require_region_state(
    regions: &RegionSet,
    region_id: openbmp_sim::RegionId,
    region_name: &str,
    state: Option<PhaseId>,
    state_name: &str,
) -> Result<(), RunnerError> {
    let Some(region) = regions.regions.get(&region_id) else {
        return Err(RunnerError::Scenario(
            openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!("canonical region `{region_name}` is missing"),
            },
        ));
    };
    if let Some(state) = state
        && !region.contains_state(state)
    {
        return Err(RunnerError::Scenario(
            openbmp_scenario::ScenarioError::MissionGraph {
                reason: format!(
                    "canonical region `{region_name}` is missing required state `{state_name}`"
                ),
            },
        ));
    }
    Ok(())
}

fn region_phase(id: PhaseId, label: &str) -> Phase {
    Phase {
        id,
        label: label.to_owned(),
        allowed_effectors: Vec::new(),
        allowed_engines: Vec::new(),
    }
}

fn region_id(id: &str) -> openbmp_sim::RegionId {
    if id.starts_with("mission.regions.") {
        openbmp_sim::RegionId::from_path(id)
    } else {
        openbmp_sim::RegionId::from_path(&format!("mission.regions.{id}"))
    }
}

fn region_state_id_by_name(region_name: &str, id: &str) -> PhaseId {
    if id.starts_with("mission.regions.") {
        PhaseId::from_path(id)
    } else if region_name.starts_with("mission.regions.") {
        PhaseId::from_path(&format!("{region_name}.{id}"))
    } else {
        PhaseId::from_path(&format!("mission.regions.{region_name}.{id}"))
    }
}

fn build_transition(
    index: usize,
    config: &PhaseTransitionConfig,
    phase_id_lookup: &BTreeMap<&str, PhaseId>,
    event_id_lookup: &BTreeMap<&str, EventId>,
) -> Result<PhaseTransition, RunnerError> {
    let from = phase_id_lookup
        .get(config.from.as_str())
        .copied()
        .ok_or_else(|| {
            RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
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
            RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
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
            RunnerError::Scenario(openbmp_scenario::ScenarioError::MissionGraph {
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
        if let ScenarioActionConfig::EmitTelemetryMarker { tag } = &event.action {
            tags.insert(tag.clone());
        }
    }
    for state in &mission.states {
        collect_marker_tags(&state.on_entry, &mut tags);
        collect_marker_tags(&state.on_exit, &mut tags);
        collect_marker_tags(&state.on_active, &mut tags);
    }
    tags.into_iter().collect()
}

fn collect_marker_tags(
    actions: &[ScenarioActionConfig],
    tags: &mut std::collections::BTreeSet<String>,
) {
    for action in actions {
        if let ScenarioActionConfig::EmitTelemetryMarker { tag } = action {
            tags.insert(tag.clone());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_and_bare_phase_ids_match() {
        assert_eq!(phase_id("ascent"), phase_id("mission.phases.ascent"));
        assert_eq!(event_id("liftoff"), event_id("mission.events.liftoff"));
    }

    #[test]
    fn marker_tags_include_hsm_state_actions() {
        let mission = MissionConfig {
            initial_phase: "ascent".to_owned(),
            phases: Vec::new(),
            events: vec![EventConfig {
                id: "apogee".to_owned(),
                trigger: EventTriggerConfig::AtApogee,
                action: ScenarioActionConfig::EmitTelemetryMarker {
                    tag: "event_marker".to_owned(),
                },
                once: true,
            }],
            transitions: Vec::new(),
            states: vec![StateConfig {
                id: "ascent".to_owned(),
                label: "ascent".to_owned(),
                parent: None,
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
                on_entry: vec![ScenarioActionConfig::EmitTelemetryMarker {
                    tag: "entry_marker".to_owned(),
                }],
                on_exit: vec![ScenarioActionConfig::EmitTelemetryMarker {
                    tag: "exit_marker".to_owned(),
                }],
                on_active: Vec::new(),
            }],
            regions: Vec::new(),
            scope: None,
            test_only_state_override: false,
        };

        assert_eq!(
            marker_tags(&mission),
            vec!["entry_marker", "event_marker", "exit_marker"]
        );
    }

    #[test]
    fn jettison_stage_event_builds_script_action() -> Result<(), RunnerError> {
        let event = EventConfig {
            id: "stage_separation".to_owned(),
            trigger: EventTriggerConfig::AtTime { time_s: 1.0 },
            action: ScenarioActionConfig::JettisonStage {
                body: "lower".to_owned(),
            },
            once: true,
        };
        let phase_lookup = BTreeMap::new();

        let binding = build_event_binding(&event, &phase_lookup)?;
        let binding = match binding {
            RuntimeEventBinding::Script(binding) => binding,
            RuntimeEventBinding::Mission(_) => {
                return Err(RunnerError::UnsupportedScenario {
                    what: "jettison_stage must build a script binding".to_owned(),
                });
            }
        };

        assert_eq!(binding.id, event_id("stage_separation"));
        assert_eq!(
            binding.action,
            ScenarioScriptAction::JettisonStage {
                body: openbmp_core::BodyId::from_path("vehicle.assembly.bodies.lower")
            }
        );
        Ok(())
    }
}
