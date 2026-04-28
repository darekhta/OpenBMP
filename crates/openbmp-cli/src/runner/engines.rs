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
//! - `apply_commands` walks the fired-event slice in order;
//!   multiple commands targeting the same engine in one step are
//!   resolved last-write-wins by the order the kernel pushed them
//!   into `pending_events`.
//! - The kernel-pushed snapshot map is `BTreeMap<EngineId,
//!   EngineSnapshot>` (not `HashMap`) — deterministic iteration on
//!   macOS `SipHash` builds.

use std::collections::BTreeMap;

use openbmp_core::{Body, Duration, EngineId, Position3};
use openbmp_propulsion::{
    ClusterLayout as PropulsionClusterLayout, EngineCluster, EngineFault, EngineLimits,
    EngineModel, EngineSnapshot, LiquidEngine,
};
use openbmp_scenario::{
    ClusterLayoutConfig, EngineConfig, EngineFaultConfig, EngineKindConfig, ScenarioDocument,
};
use openbmp_sim::{EventAction, FiredEvent};

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
        let mut layout = ClusterLayoutConfig::default();

        if let Some(assembly) = &document.vehicle.assembly {
            layout = assembly.cluster_layout.unwrap_or_default();
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
    /// targeting the same engine in one step are applied in fired
    /// order — last write wins per field.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Engine`] when an event references an
    /// engine id that is not present in this rack, or when the
    /// engine's `apply_command` rejects (non-finite payload).
    pub fn apply_commands(&mut self, fired: &[FiredEvent]) -> Result<(), CliError> {
        for event in fired {
            if let EventAction::EngineCommand { id, command } = event.action {
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
