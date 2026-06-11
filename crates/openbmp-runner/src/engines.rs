//! Runner-side engine rack.
//!
//! The rack owns the cluster of `Box<dyn EngineModel>` resolved
//! from the scenario's `[[vehicle.assembly.engines]]` block, plus
//! the construction-time `dt` that's passed to each engine's
//! `step(dt)` call. It is stepped once per kernel base tick
//! **before** the kernel's `step()` so the engine snapshot
//! observable in the Parquet matches the kernel's view of the
//! world.
//!
//! The rack leaves the kernel's force / moment / mass evaluation
//! untouched when it is empty — legacy single-motor scenarios
//! produce byte-identical Parquet because the runner short-circuits
//! every rack-related operation on `is_empty()`.
//!
//! Mirrors the [`crate::effectors::EffectorRack`]
//! pattern: kernel records scenario-script engine-command firings;
//! runner drains and applies them via `apply_commands(&fired)`.
//!
//! # Determinism
//!
//! - Engines are stored in scenario-declared order on the
//!   propulsion-side [`openbmp_propulsion::EngineCluster`]; the
//!   rack iterates that order verbatim.
//! - `apply_commands` walks the fired-event slice in order and
//!   rejects multiple commands addressing the same engine in one step.
//!   This keeps same-step command bundles explicit instead of
//!   depending on event-id ordering.
//! - The kernel-pushed snapshot map is `BTreeMap<EngineId,
//!   EngineSnapshot>` (not `HashMap`) — deterministic iteration on
//!   macOS `SipHash` builds.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_core::{Body, BodyId, Duration, EngineId, Position3, StepIndex};
use openbmp_propulsion::{
    ClusterLayout as PropulsionClusterLayout, EngineCluster, EngineFault, EngineLimits,
    EngineModel, EngineState, LiquidEngine,
};
use openbmp_scenario::{
    ClusterLayoutConfig, EngineConfig, EngineFaultConfig, EngineKindConfig,
    PropulsionCavitationFaultLegConfig, ScenarioDocument,
};
use openbmp_sim::{EngineSnapshot, FiredEvent, ScenarioScriptAction};
use openbmp_vehicle::PropellantBudgetReport;

use crate::error::RunnerError;
use crate::feed_network::{FeedPumpCavitationEvent, FeedPumpLeg};

/// Runner-side engine rack. Built once per `openbmp run` invocation;
/// consumed by the per-step kernel loop.
#[derive(Debug)]
pub struct EngineRack {
    cluster: EngineCluster,
    dt: Duration,
    engine_owners: BTreeMap<EngineId, BodyId>,
    retired_bodies: BTreeSet<BodyId>,
    scheduled_faults: Vec<ScheduledEngineFault>,
    cavitation_faults: Vec<CavitationEngineFault>,
    applied_fault_ids: BTreeSet<String>,
}

#[derive(Clone, Debug)]
struct ScheduledEngineFault {
    id: String,
    engine_id: EngineId,
    start_step: u64,
    fault: EngineFault,
}

#[derive(Clone, Debug)]
struct CavitationEngineFault {
    id: String,
    engine_id: EngineId,
    leg: PropulsionCavitationFaultLegConfig,
    fault: EngineFault,
}

impl EngineRack {
    /// Build the rack from a parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.engines]]` blocks are
    /// declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when an engine config fails
    /// `LiquidEngine::new` (invalid limits) or when a load-time
    /// fault rejects against the engine's authority envelope.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut engines: Vec<Box<dyn EngineModel>> = Vec::new();
        let mut mount_points_body: Vec<Position3<Body>> = Vec::new();
        let mut engine_ids: Vec<EngineId> = Vec::new();
        let mut engine_owners: BTreeMap<EngineId, BodyId> = BTreeMap::new();
        let assembly = &document.vehicle.assembly;
        let layout = assembly.cluster_layout.unwrap_or_default();
        for (index, config) in assembly.engines.iter().enumerate() {
            let engine = build_engine(index, config)?;
            let id = engine.id();
            if let Some(owner) = config.mounted_to.as_deref() {
                engine_owners.insert(id, body_id_from_scenario_text(owner));
            }
            engines.push(Box::new(engine));
            mount_points_body.push(Position3::<Body>::new(
                config.mount_point_body_m[0],
                config.mount_point_body_m[1],
                config.mount_point_body_m[2],
            ));
            engine_ids.push(id);
        }

        let propulsion_layout = match layout {
            ClusterLayoutConfig::Axial => PropulsionClusterLayout::Axial,
            ClusterLayoutConfig::Ring => PropulsionClusterLayout::Ring,
            ClusterLayoutConfig::Octaweb => PropulsionClusterLayout::Octaweb,
            ClusterLayoutConfig::Custom => PropulsionClusterLayout::Custom,
        };

        let scheduled_faults = scheduled_faults(document);
        let cavitation_faults = cavitation_faults(document);
        let cluster = EngineCluster::new(engines, mount_points_body, engine_ids, propulsion_layout)
            .map_err(|err| RunnerError::Engine {
                field: "vehicle.assembly".to_owned(),
                reason: err.to_string(),
            })?;

        Ok(Self {
            cluster,
            dt,
            engine_owners,
            retired_bodies: BTreeSet::new(),
            scheduled_faults,
            cavitation_faults,
            applied_fault_ids: BTreeSet::new(),
        })
    }

    /// Number of engines in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.cluster.len()
    }

    /// `true` when the rack carries no engines. The runner gates
    /// every per-step rack operation on this so legacy scenarios
    /// never touch the rack code path.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.cluster.is_empty()
    }

    /// Engine ids in scenario-declared order.
    #[must_use]
    pub fn engine_ids(&self) -> &[EngineId] {
        self.cluster.engine_ids()
    }

    /// Body-frame mount points in scenario-declared order.
    #[must_use]
    pub fn mount_points_body(&self) -> &[Position3<Body>] {
        self.cluster.mount_points_body()
    }

    /// Replace the set of rigid-body lanes that have been retired
    /// after separated-body ground impact.
    pub fn set_retired_bodies<I>(&mut self, retired_bodies: I)
    where
        I: IntoIterator<Item = BodyId>,
    {
        self.retired_bodies = retired_bodies.into_iter().collect();
    }

    /// Force engines mounted to retired separated bodies into shutdown
    /// before publishing the next kernel snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] if the underlying propulsion
    /// cluster rejects the shutdown command.
    pub fn shutdown_retired_body_engines(&mut self) -> Result<(), RunnerError> {
        let retired_engines: Vec<EngineId> = self
            .cluster
            .engine_ids()
            .iter()
            .copied()
            .filter(|id| {
                self.engine_owners
                    .get(id)
                    .is_some_and(|owner| self.retired_bodies.contains(owner))
            })
            .collect();
        for id in retired_engines {
            self.cluster
                .apply_command(id, shutdown_command())
                .map_err(|err| RunnerError::Engine {
                    field: format!(
                        "vehicle.assembly.engines.{id_value}.retired_body_shutdown",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?;
        }
        Ok(())
    }

    /// Apply any scenario-script engine-command actions drained from
    /// the kernel's per-step fired-event queue. Multiple commands
    /// addressing the same engine in one step are rejected so the
    /// scenario author must resolve the command bundle explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when an event references an
    /// engine id that is not present in this rack, or when the
    /// engine's `apply_command` rejects (non-finite payload), or
    /// when more than one command targets the same engine in this
    /// rack tick.
    pub fn apply_commands(
        &mut self,
        fired: &[FiredEvent<ScenarioScriptAction>],
    ) -> Result<(), RunnerError> {
        let mut seen: BTreeSet<EngineId> = BTreeSet::new();
        for event in fired {
            if let ScenarioScriptAction::EngineCommand {
                id,
                throttle_unit,
                gimbal_pitch_rad,
                gimbal_yaw_rad,
                ignite,
                shutdown,
            } = event.action
            {
                if !seen.insert(id) {
                    return Err(RunnerError::Engine {
                        field: format!(
                            "mission.events[*].action.engine_command.{id_value}",
                            id_value = id.value()
                        ),
                        reason: "multiple `engine_command` events fired for the same engine in one step; resolve to a single command per engine per step".to_owned(),
                    });
                }
                // Construct the typed propulsion-side
                // `EngineCommand` here (the mission graph carries
                // only the scalar fields, decoupling
                // `openbmp-mission` from `openbmp-propulsion`).
                let command = openbmp_propulsion::EngineCommand {
                    throttle_unit,
                    gimbal_pitch_rad,
                    gimbal_yaw_rad,
                    ignite,
                    shutdown,
                };
                self.reject_retired_body_command(
                    id,
                    &command,
                    format!(
                        "mission.events[*].action.engine_command.{id_value}",
                        id_value = id.value()
                    ),
                )?;
                self.cluster
                    .apply_command(id, command)
                    .map_err(|err| RunnerError::Engine {
                        field: format!(
                            "mission.events[*].action.engine_command.{id_value}",
                            id_value = id.value()
                        ),
                        reason: err.to_string(),
                    })?;
            }
        }
        Ok(())
    }

    /// Apply one-tick FC engine commands keyed by `EngineId::value()`.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when the FC references an engine
    /// id outside this rack, or when the propulsion-side command is
    /// rejected.
    pub fn apply_fc_commands(
        &mut self,
        commands: &openbmp_fc::topics::EngineCommandSet,
    ) -> Result<(), RunnerError> {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let id = EngineId::new(command.engine_id);
            let payload = openbmp_propulsion::EngineCommand {
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            };
            self.reject_retired_body_command(
                id,
                &payload,
                format!("fc.actuator.engine_cmds.{id_value}", id_value = id.value()),
            )?;
            self.cluster
                .apply_command(id, payload)
                .map_err(|err| RunnerError::Engine {
                    field: "fc.actuator.engine_cmds".to_owned(),
                    reason: err.to_string(),
                })?;
        }
        Ok(())
    }

    /// Inject scheduled propulsion faults whose start step has arrived.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] if a scheduled rule targets an engine
    /// missing from this rack or the underlying engine rejects the fault payload.
    pub fn apply_scheduled_faults(&mut self, step: StepIndex) -> Result<(), RunnerError> {
        for fault in &self.scheduled_faults {
            if step.value() < fault.start_step || self.applied_fault_ids.contains(&fault.id) {
                continue;
            }
            self.cluster
                .inject_fault(fault.engine_id, fault.fault)
                .map_err(|err| RunnerError::Engine {
                    field: format!("propulsion.faults.rules.{}", fault.id),
                    reason: err.to_string(),
                })?;
            self.applied_fault_ids.insert(fault.id.clone());
        }
        Ok(())
    }

    /// Inject one-shot propulsion faults whose pump-cavitation trigger is active.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] if a triggered rule targets an engine
    /// missing from this rack or the underlying engine rejects the fault payload.
    pub fn apply_cavitation_faults(
        &mut self,
        events: &[FeedPumpCavitationEvent],
    ) -> Result<(), RunnerError> {
        for fault in &self.cavitation_faults {
            if self.applied_fault_ids.contains(&fault.id)
                || !events
                    .iter()
                    .any(|event| cavitation_rule_matches(fault, *event))
            {
                continue;
            }
            self.cluster
                .inject_fault(fault.engine_id, fault.fault)
                .map_err(|err| RunnerError::Engine {
                    field: format!("propulsion.faults.cavitation_rules.{}", fault.id),
                    reason: err.to_string(),
                })?;
            self.applied_fault_ids.insert(fault.id.clone());
        }
        Ok(())
    }

    fn reject_retired_body_command(
        &self,
        id: EngineId,
        command: &openbmp_propulsion::EngineCommand,
        field: String,
    ) -> Result<(), RunnerError> {
        let Some(owner) = self.engine_owners.get(&id) else {
            return Ok(());
        };
        if self.retired_bodies.contains(owner) && command_requests_activity(command) {
            return Err(RunnerError::Engine {
                field,
                reason: format!(
                    "active engine command targets retired separated body {body}",
                    body = owner.value()
                ),
            });
        }
        Ok(())
    }

    /// Step every engine in the rack by one kernel base tick.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] when an engine's `step()`
    /// rejects (non-finite output, internal numeric error).
    pub fn step(&mut self) -> Result<(), RunnerError> {
        self.cluster
            .step(self.dt)
            .map(|_| ())
            .map_err(|err| RunnerError::Engine {
                field: "vehicle.assembly.engines".to_owned(),
                reason: err.to_string(),
            })
    }

    /// Apply vehicle-side propellant-budget outputs before stepping
    /// the engines for this tick.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Engine`] if a referenced engine id is
    /// not present or the feed-pressure scalar is invalid.
    pub fn apply_propellant_budget(
        &mut self,
        report: &PropellantBudgetReport,
    ) -> Result<(), RunnerError> {
        for (id, scale) in &report.feed_pressure_scales {
            self.cluster
                .set_feed_pressure_scale(*id, *scale)
                .map_err(|err| RunnerError::Engine {
                    field: format!(
                        "vehicle.assembly.engines.{id_value}.propellant.feed",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?;
        }
        for id in &report.shutdown_engines {
            self.cluster
                .apply_command(
                    *id,
                    openbmp_propulsion::EngineCommand {
                        throttle_unit: 0.0,
                        gimbal_pitch_rad: 0.0,
                        gimbal_yaw_rad: 0.0,
                        ignite: false,
                        shutdown: true,
                    },
                )
                .map_err(|err| RunnerError::Engine {
                    field: format!(
                        "vehicle.assembly.engines.{id_value}.propellant",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?;
        }
        Ok(())
    }

    /// Propulsion-side snapshots keyed by engine id for vehicle-side
    /// propellant budget evaluation.
    #[must_use]
    pub fn propulsion_snapshot_map(
        &self,
    ) -> BTreeMap<EngineId, openbmp_propulsion::EngineSnapshot> {
        let mut out = BTreeMap::new();
        let snapshots = self.cluster.current_snapshot();
        for (id, snap) in self.cluster.engine_ids().iter().zip(snapshots.iter()) {
            out.insert(*id, *snap);
        }
        out
    }

    /// Produce the per-engine snapshot map the kernel consumes via
    /// `set_engine_snapshot`. Keys are `EngineId`; values are the
    /// engine's `current_snapshot()` (gimbal-applied body-frame
    /// thrust, mass-flow rate, integrated `consumed_kg`, current
    /// state).
    #[must_use]
    pub fn snapshot_map(&self) -> BTreeMap<EngineId, EngineSnapshot> {
        let mut out = BTreeMap::new();
        let snapshots = self.cluster.current_snapshot();
        for (id, snap) in self.cluster.engine_ids().iter().zip(snapshots.iter()) {
            out.insert(
                *id,
                EngineSnapshot {
                    thrust_body: snap.thrust_body,
                    mass_flow_kg_per_s: snap.mass_flow_kg_per_s,
                    consumed_kg: snap.consumed_kg,
                    lifecycle_state_index: engine_state_index(snap.state),
                },
            );
        }
        out
    }
}

fn body_id_from_scenario_text(id: &str) -> BodyId {
    BodyId::from_path(&format!("vehicle.assembly.bodies.{id}"))
}

fn shutdown_command() -> openbmp_propulsion::EngineCommand {
    openbmp_propulsion::EngineCommand {
        throttle_unit: 0.0,
        gimbal_pitch_rad: 0.0,
        gimbal_yaw_rad: 0.0,
        ignite: false,
        shutdown: true,
    }
}

fn command_requests_activity(command: &openbmp_propulsion::EngineCommand) -> bool {
    command.ignite
        || command.throttle_unit > 0.0
        || command.gimbal_pitch_rad != 0.0
        || command.gimbal_yaw_rad != 0.0
        || !command.shutdown
}

fn engine_state_index(state: EngineState) -> u8 {
    match state {
        EngineState::Idle => 0,
        EngineState::Igniting => 1,
        EngineState::Burning => 2,
        EngineState::Shutdown => 3,
        EngineState::Failed => 4,
    }
}

/// Engine resolver: scenario `EngineConfig` →
/// `LiquidEngine` (the only kind currently supported). Mounts the
/// optional load-time fault.
fn build_engine(index: usize, config: &EngineConfig) -> Result<LiquidEngine, RunnerError> {
    let id = EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = config.id));
    let limits = EngineLimits {
        max_thrust_n: config.limits.max_thrust_n,
        isp_s: config.limits.isp_s,
        ignition_transient_s: config.limits.ignition_transient_s,
        shutdown_transient_s: config.limits.shutdown_transient_s,
        max_gimbal_rad: config.limits.max_gimbal_rad,
        gimbal_slew_rad_per_s: config.limits.gimbal_slew_rad_per_s,
        throttle_slew_per_s: config.limits.throttle_slew_per_s,
        min_throttle_unit: config.limits.min_throttle_unit,
        isp_throttle_falloff: config.limits.isp_throttle_falloff,
    };
    let kind_ok = matches!(config.kind, EngineKindConfig::LiquidEngine);
    if !kind_ok {
        // Not currently reachable — `EngineKindConfig` only has
        // `LiquidEngine` — but the explicit match guards future
        // variants.
        return Err(RunnerError::Engine {
            field: format!("vehicle.assembly.engines[{index}].kind"),
            reason: "unsupported engine kind".to_owned(),
        });
    }
    let mut engine = LiquidEngine::new(id, limits)
        .map_err(|err| RunnerError::Engine {
            field: format!("vehicle.assembly.engines[{index}]"),
            reason: err.to_string(),
        })?
        .with_restart_policy(config.limits.restartable);
    if let Some(fault_config) = &config.fault {
        let fault = engine_fault_from_config(*fault_config);
        engine
            .inject_fault(fault)
            .map_err(|err| RunnerError::Engine {
                field: format!("vehicle.assembly.engines[{index}].fault"),
                reason: err.to_string(),
            })?;
    }
    Ok(engine)
}

fn scheduled_faults(document: &ScenarioDocument) -> Vec<ScheduledEngineFault> {
    document
        .propulsion
        .as_ref()
        .and_then(|propulsion| propulsion.faults.as_ref())
        .map(|faults| {
            faults
                .rules
                .iter()
                .map(|rule| ScheduledEngineFault {
                    id: rule.id.clone(),
                    engine_id: EngineId::from_path(&format!(
                        "vehicle.assembly.engines.{engine_id}",
                        engine_id = rule.engine_id
                    )),
                    start_step: rule.start_step,
                    fault: engine_fault_from_config(rule.fault),
                })
                .collect()
        })
        .unwrap_or_default()
}

fn cavitation_faults(document: &ScenarioDocument) -> Vec<CavitationEngineFault> {
    document
        .propulsion
        .as_ref()
        .and_then(|propulsion| propulsion.faults.as_ref())
        .map(|faults| {
            faults
                .cavitation_rules
                .iter()
                .map(|rule| CavitationEngineFault {
                    id: rule.id.clone(),
                    engine_id: EngineId::from_path(&format!(
                        "vehicle.assembly.engines.{engine_id}",
                        engine_id = rule.engine_id
                    )),
                    leg: rule.leg,
                    fault: engine_fault_from_config(rule.fault),
                })
                .collect()
        })
        .unwrap_or_default()
}

fn cavitation_rule_matches(rule: &CavitationEngineFault, event: FeedPumpCavitationEvent) -> bool {
    if rule.engine_id != event.engine_id {
        return false;
    }
    match rule.leg {
        PropulsionCavitationFaultLegConfig::Any => true,
        PropulsionCavitationFaultLegConfig::Oxidizer => event.leg == FeedPumpLeg::Oxidizer,
        PropulsionCavitationFaultLegConfig::Fuel => event.leg == FeedPumpLeg::Fuel,
    }
}

fn engine_fault_from_config(config: EngineFaultConfig) -> EngineFault {
    match config {
        EngineFaultConfig::Stuck { at_throttle } => EngineFault::Stuck { at_throttle },
        EngineFaultConfig::HardOff => EngineFault::HardOff,
        EngineFaultConfig::OverThrust { factor } => EngineFault::OverThrust { factor },
        EngineFaultConfig::HardStartOverpressure { factor, duration_s } => {
            EngineFault::HardStartOverpressure { factor, duration_s }
        }
        EngineFaultConfig::CavitationThrustLoss { factor } => {
            EngineFault::CavitationThrustLoss { factor }
        }
        EngineFaultConfig::GimbalLocked { pitch_rad, yaw_rad } => {
            EngineFault::GimbalLocked { pitch_rad, yaw_rad }
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{SimTime, StepIndex};
    use openbmp_propulsion::{EngineCluster, EngineCommand, EngineLimits};
    use openbmp_sim::EventId;

    fn test_limits() -> EngineLimits {
        EngineLimits {
            max_thrust_n: 1000.0,
            isp_s: 250.0,
            ignition_transient_s: 0.1,
            shutdown_transient_s: 0.1,
            max_gimbal_rad: 0.1,
            gimbal_slew_rad_per_s: f64::INFINITY,
            throttle_slew_per_s: f64::INFINITY,
            min_throttle_unit: 0.0,
            isp_throttle_falloff: 0.0,
        }
    }

    fn one_engine_rack() -> (EngineRack, EngineId) {
        let id = EngineId::from_path("vehicle.assembly.engines.engine_a");
        let engine = LiquidEngine::new(id, test_limits()).unwrap();
        let cluster = EngineCluster::new(
            vec![Box::new(engine)],
            vec![Position3::<Body>::new(0.0, 0.0, 0.0)],
            vec![id],
            PropulsionClusterLayout::Axial,
        )
        .unwrap();
        (
            EngineRack {
                cluster,
                dt: Duration::from_seconds(0.001),
                engine_owners: BTreeMap::new(),
                retired_bodies: BTreeSet::new(),
                scheduled_faults: Vec::new(),
                cavitation_faults: Vec::new(),
                applied_fault_ids: BTreeSet::new(),
            },
            id,
        )
    }

    fn engine_event(
        name: &str,
        id: EngineId,
        command: EngineCommand,
    ) -> FiredEvent<ScenarioScriptAction> {
        FiredEvent {
            binding_id: EventId::from_path(name),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            // Typed scenario-script action.
            action: ScenarioScriptAction::EngineCommand {
                id,
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            },
        }
    }

    #[test]
    fn duplicate_engine_commands_in_one_step_fail_closed() {
        let (mut rack, id) = one_engine_rack();
        let events = vec![
            engine_event(
                "mission.events.ignite_a",
                id,
                EngineCommand {
                    throttle_unit: 1.0,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    ignite: true,
                    shutdown: false,
                },
            ),
            engine_event(
                "mission.events.shutdown_a",
                id,
                EngineCommand {
                    throttle_unit: 0.0,
                    gimbal_pitch_rad: 0.0,
                    gimbal_yaw_rad: 0.0,
                    ignite: false,
                    shutdown: true,
                },
            ),
        ];

        let err = rack.apply_commands(&events).unwrap_err();
        assert!(
            matches!(err, RunnerError::Engine { .. }),
            "expected RunnerError::Engine, got {err:?}",
        );
    }

    #[test]
    fn snapshot_map_exposes_kernel_snapshot_without_propulsion_enum() {
        let (mut rack, id) = one_engine_rack();
        let initial = rack.snapshot_map();
        let initial_snapshot = initial.get(&id).unwrap();
        assert_eq!(initial_snapshot.lifecycle_state_index, 0);
        assert_eq!(initial_snapshot.thrust_body, nalgebra::Vector3::zeros());

        rack.apply_commands(&[engine_event(
            "mission.events.ignite_a",
            id,
            EngineCommand {
                throttle_unit: 0.5,
                gimbal_pitch_rad: 0.0,
                gimbal_yaw_rad: 0.0,
                ignite: true,
                shutdown: false,
            },
        )])
        .unwrap();
        rack.step().unwrap();

        let after_step = rack.snapshot_map();
        let snapshot = after_step.get(&id).unwrap();
        assert_eq!(snapshot.lifecycle_state_index, 1);
        assert!(snapshot.thrust_body.z > 0.0);
        assert!(snapshot.mass_flow_kg_per_s > 0.0);
    }

    #[test]
    fn scheduled_fault_injects_once_when_step_arrives() {
        let (mut rack, id) = one_engine_rack();
        rack.scheduled_faults.push(ScheduledEngineFault {
            id: "fail-main".to_owned(),
            engine_id: id,
            start_step: 2,
            fault: EngineFault::HardOff,
        });

        rack.apply_scheduled_faults(StepIndex::new(1)).unwrap();
        rack.step().unwrap();
        assert_eq!(rack.snapshot_map()[&id].lifecycle_state_index, 0);

        rack.apply_scheduled_faults(StepIndex::new(2)).unwrap();
        rack.apply_scheduled_faults(StepIndex::new(2)).unwrap();
        rack.step().unwrap();

        assert_eq!(rack.snapshot_map()[&id].lifecycle_state_index, 4);
        assert!(rack.applied_fault_ids.contains("fail-main"));
        assert_eq!(rack.applied_fault_ids.len(), 1);
    }

    #[test]
    fn engine_fault_config_maps_hard_start_overpressure() {
        let fault = engine_fault_from_config(EngineFaultConfig::HardStartOverpressure {
            factor: 1.75,
            duration_s: 0.08,
        });

        assert!(matches!(
            fault,
            EngineFault::HardStartOverpressure { factor, duration_s }
                if factor.to_bits() == 1.75_f64.to_bits()
                    && duration_s.to_bits() == 0.08_f64.to_bits()
        ));
    }

    #[test]
    fn cavitation_fault_injects_once_when_event_matches() {
        let (mut rack, id) = one_engine_rack();
        rack.cavitation_faults.push(CavitationEngineFault {
            id: "oxidizer-cavitation".to_owned(),
            engine_id: id,
            leg: PropulsionCavitationFaultLegConfig::Oxidizer,
            fault: EngineFault::CavitationThrustLoss { factor: 0.5 },
        });

        rack.apply_cavitation_faults(&[FeedPumpCavitationEvent {
            engine_id: id,
            leg: FeedPumpLeg::Fuel,
        }])
        .unwrap();
        assert!(rack.applied_fault_ids.is_empty());

        rack.apply_cavitation_faults(&[FeedPumpCavitationEvent {
            engine_id: id,
            leg: FeedPumpLeg::Oxidizer,
        }])
        .unwrap();
        rack.apply_cavitation_faults(&[FeedPumpCavitationEvent {
            engine_id: id,
            leg: FeedPumpLeg::Oxidizer,
        }])
        .unwrap();

        assert!(rack.applied_fault_ids.contains("oxidizer-cavitation"));
        assert_eq!(rack.applied_fault_ids.len(), 1);
    }

    #[test]
    fn engine_fault_config_maps_cavitation_thrust_loss() {
        let fault =
            engine_fault_from_config(EngineFaultConfig::CavitationThrustLoss { factor: 0.35 });

        assert!(matches!(
            fault,
            EngineFault::CavitationThrustLoss { factor }
                if factor.to_bits() == 0.35_f64.to_bits()
        ));
    }
}
