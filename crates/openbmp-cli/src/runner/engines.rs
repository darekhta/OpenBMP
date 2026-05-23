//! Phase-3.6 runner-side engine rack.
//!
//! The rack owns the cluster of `Box<dyn EngineModel>` resolved
//! from the scenario's `[[vehicle.assembly.engines]]` block, plus
//! the construction-time `dt` that's passed to each engine's
//! `step(dt)` call. It is stepped once per kernel base tick
//! **before** the kernel's `step()` so the engine snapshot
//! observable in the Parquet matches the kernel's view of the
//! world.
//!
//! Phase-3.6 leaves the kernel's force / moment / mass evaluation
//! untouched when the rack is empty — legacy single-motor scenarios
//! produce byte-identical Parquet because the runner short-circuits
//! every rack-related operation on `is_empty()`.
//!
//! Mirrors the Phase-3.4 [`crate::runner::effectors::EffectorRack`]
//! pattern: kernel records `EventAction::EngineCommand` firings;
//! runner drains and applies them via `apply_commands(&fired)`.
//!
//! # Determinism
//!
//! - Engines are stored in scenario-declared order on the
//!   propulsion-side [`openbmp_propulsion::EngineCluster`]; the
//!   rack iterates that order verbatim.
//! - `apply_commands` walks the fired-event slice in order and
//!   rejects multiple commands targeting the same engine in one step.
//!   This keeps same-step command bundles explicit instead of
//!   depending on event-id ordering.
//! - The kernel-pushed snapshot map is `BTreeMap<EngineId,
//!   EngineSnapshot>` (not `HashMap`) — deterministic iteration on
//!   macOS `SipHash` builds.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_core::{Body, Duration, EngineId, Position3};
use openbmp_propulsion::{
    ClusterLayout as PropulsionClusterLayout, EngineCluster, EngineFault, EngineLimits,
    EngineModel, EngineSnapshot, LiquidEngine,
};
use openbmp_scenario::{
    ClusterLayoutConfig, EngineConfig, EngineFaultConfig, EngineKindConfig, ScenarioDocument,
};
use openbmp_sim::{FiredEvent, ScenarioScriptAction};

use crate::error::CliError;

/// Runner-side engine rack. Built once per `openbmp run` invocation;
/// consumed by the per-step kernel loop.
#[derive(Debug)]
pub struct EngineRack {
    cluster: EngineCluster,
    dt: Duration,
}

impl EngineRack {
    /// Build the rack from a parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.engines]]` blocks are
    /// declared.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Engine`] when an engine config fails
    /// `LiquidEngine::new` (invalid limits) or when a load-time
    /// fault rejects against the engine's authority envelope.
    pub fn build(document: &ScenarioDocument) -> Result<Self, CliError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut engines: Vec<Box<dyn EngineModel>> = Vec::new();
        let mut mount_points_body: Vec<Position3<Body>> = Vec::new();
        let mut engine_ids: Vec<EngineId> = Vec::new();
        let assembly = &document.vehicle.assembly;
        let layout = assembly.cluster_layout.unwrap_or_default();
        for (index, config) in assembly.engines.iter().enumerate() {
            let engine = build_engine(index, config)?;
            let id = engine.id();
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

        let cluster = EngineCluster::new(engines, mount_points_body, engine_ids, propulsion_layout)
            .map_err(|err| CliError::Engine {
                field: "vehicle.assembly".to_owned(),
                reason: err.to_string(),
            })?;

        Ok(Self { cluster, dt })
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

    /// Apply any `EventAction::EngineCommand` actions drained from
    /// the kernel's per-step fired-event queue. Multiple commands
    /// targeting the same engine in one step are rejected so the
    /// scenario author must resolve the command bundle explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Engine`] when an event references an
    /// engine id that is not present in this rack, or when the
    /// engine's `apply_command` rejects (non-finite payload), or
    /// when more than one command targets the same engine in this
    /// rack tick.
    pub fn apply_commands(
        &mut self,
        fired: &[FiredEvent<ScenarioScriptAction>],
    ) -> Result<(), CliError> {
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
                    return Err(CliError::Engine {
                        field: format!(
                            "mission.events[*].action.engine_command.{id_value}",
                            id_value = id.value()
                        ),
                        reason: "multiple `engine_command` events fired for the same engine in one step; resolve to a single command per engine per step".to_owned(),
                    });
                }
                // Phase-3.15.C: construct the typed propulsion-side
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
                self.cluster
                    .apply_command(id, command)
                    .map_err(|err| CliError::Engine {
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
    /// Returns [`CliError::Engine`] when the FC references an engine
    /// id outside this rack, or when the propulsion-side command is
    /// rejected.
    pub fn apply_fc_commands(
        &mut self,
        commands: &openbmp_fc::topics::EngineCommandSet,
    ) -> Result<(), CliError> {
        for command in commands.commands.iter().take(usize::from(commands.count)) {
            let id = EngineId::new(command.engine_id);
            let payload = openbmp_propulsion::EngineCommand {
                throttle_unit: command.throttle_unit,
                gimbal_pitch_rad: command.gimbal_pitch_rad,
                gimbal_yaw_rad: command.gimbal_yaw_rad,
                ignite: command.ignite,
                shutdown: command.shutdown,
            };
            self.cluster
                .apply_command(id, payload)
                .map_err(|err| CliError::Engine {
                    field: "fc.actuator.engine_cmds".to_owned(),
                    reason: err.to_string(),
                })?;
        }
        Ok(())
    }

    /// Step every engine in the rack by one kernel base tick.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Engine`] when an engine's `step()`
    /// rejects (non-finite output, internal numeric error).
    pub fn step(&mut self) -> Result<(), CliError> {
        self.cluster
            .step(self.dt)
            .map(|_| ())
            .map_err(|err| CliError::Engine {
                field: "vehicle.assembly.engines".to_owned(),
                reason: err.to_string(),
            })
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
            out.insert(*id, *snap);
        }
        out
    }
}

/// Phase-3.6 engine resolver: scenario `EngineConfig` →
/// `LiquidEngine` (the only kind shipped in 3.6). Mounts the
/// optional load-time fault.
fn build_engine(index: usize, config: &EngineConfig) -> Result<LiquidEngine, CliError> {
    let id = EngineId::from_path(&format!("vehicle.assembly.engines.{id}", id = config.id));
    let limits = EngineLimits {
        max_thrust_n: config.limits.max_thrust_n,
        isp_s: config.limits.isp_s,
        ignition_transient_s: config.limits.ignition_transient_s,
        shutdown_transient_s: config.limits.shutdown_transient_s,
        max_gimbal_rad: config.limits.max_gimbal_rad,
    };
    let kind_ok = matches!(config.kind, EngineKindConfig::LiquidEngine);
    if !kind_ok {
        // Not currently reachable — `EngineKindConfig` only has
        // `LiquidEngine` — but the explicit match guards future
        // variants.
        return Err(CliError::Engine {
            field: format!("vehicle.assembly.engines[{index}].kind"),
            reason: "unsupported engine kind in Phase 3.6".to_owned(),
        });
    }
    let mut engine = LiquidEngine::new(id, limits).map_err(|err| CliError::Engine {
        field: format!("vehicle.assembly.engines[{index}]"),
        reason: err.to_string(),
    })?;
    if let Some(fault_config) = &config.fault {
        let fault = match *fault_config {
            EngineFaultConfig::Stuck { at_throttle } => EngineFault::Stuck { at_throttle },
            EngineFaultConfig::HardOff => EngineFault::HardOff,
            EngineFaultConfig::OverThrust { factor } => EngineFault::OverThrust { factor },
            EngineFaultConfig::GimbalLocked { pitch_rad, yaw_rad } => {
                EngineFault::GimbalLocked { pitch_rad, yaw_rad }
            }
        };
        engine.inject_fault(fault).map_err(|err| CliError::Engine {
            field: format!("vehicle.assembly.engines[{index}].fault"),
            reason: err.to_string(),
        })?;
    }
    Ok(engine)
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
            // Phase 5.X.E: typed scenario-script action.
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
            matches!(err, CliError::Engine { .. }),
            "expected CliError::Engine, got {err:?}",
        );
    }
}
