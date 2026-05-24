//! Phase-3.9 runner-side recovery rack.
//!
//! The rack owns a `BTreeMap<RecoveryId, Box<dyn RecoveryModel>>`
//! resolved from the scenario's `[[vehicle.assembly.recovery]]`
//! block. It mirrors the Phase-3.6 [`crate::engines::EngineRack`]
//! and Phase-3.7 [`crate::tanks::TankRack`] patterns:
//!
//! Each kernel base tick the runner:
//!
//! 1. Drains any scenario-script recovery-deploy firings via
//!    [`RecoveryRack::apply_deploys`]. Each event is mapped to the
//!    typed `openbmp_vehicle::RecoveryCommand` and forwarded to the
//!    target device's `apply_command(...)`. Multiple command firings
//!    for the same device in one step are rejected with
//!    [`RunnerError::Recovery`].
//! 2. Calls [`RecoveryRack::step`] to advance internal state. Phase
//!    3.9 instances are instantaneous-deploy and `step` is a no-op,
//!    but the call is part of the rack contract.
//! 3. Packs each device's `(phase, c_d, drag_area)` triple into a
//!    `BTreeMap<RecoveryId, RecoverySnapshot>` via
//!    [`RecoveryRack::snapshot_map`] and pushes it into the kernel
//!    via `set_recovery_snapshot(...)`.
//!
//! Phase-3.9 leaves the kernel hot path's force evaluation untouched
//! when the rack is empty — legacy scenarios produce byte-identical
//! Parquet because the runner short-circuits every rack-related
//! operation on `is_empty()`, the kernel's recovery snapshot map
//! stays empty, and the `RecoveryRackForceAdapter` never appears in
//! the breakdown vehicle's force-model list.
//!
//! # Determinism
//!
//! - Devices are stored in a `BTreeMap<RecoveryId, Box<dyn
//!   RecoveryModel>>` (defeats macOS `SipHash` randomisation);
//!   iteration is in `RecoveryId` order (deterministic across reruns).
//! - The kernel-pushed snapshot map is also keyed by `RecoveryId`.
//! - State transitions are event-driven only — no clock or RNG —
//!   so `step` is a no-op and replay-stable by construction.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_core::RecoveryId;
use openbmp_scenario::{RecoveryConfig, RecoveryKindConfig, ScenarioDocument};
use openbmp_sim::{FiredEvent, RecoverySnapshot, ScenarioScriptAction};
use openbmp_vehicle::{
    DragDevice, DrogueMainRecovery, ParachuteDrag, RecoveryCommand, RecoveryModel,
};

use crate::error::RunnerError;

/// Runner-side recovery rack. Built once per `openbmp run`
/// invocation; consumed by the per-step kernel loop.
pub struct RecoveryRack {
    devices: BTreeMap<RecoveryId, Box<dyn RecoveryModel>>,
    /// Scenario-text id strings indexed by `RecoveryId`. Used in
    /// telemetry-channel naming.
    scenario_ids: BTreeMap<RecoveryId, String>,
}

impl std::fmt::Debug for RecoveryRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecoveryRack")
            .field("device_count", &self.devices.len())
            .field("scenario_ids", &self.scenario_ids)
            .finish()
    }
}

impl RecoveryRack {
    /// Build the rack from a parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.recovery]]` block is
    /// declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] when a recovery config fails
    /// the kind-specific constructor (non-finite or non-positive
    /// `c_d` / area parameters).
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let mut devices: BTreeMap<RecoveryId, Box<dyn RecoveryModel>> = BTreeMap::new();
        let mut scenario_ids: BTreeMap<RecoveryId, String> = BTreeMap::new();

        for config in &document.vehicle.assembly.recovery {
            let id = recovery_id_from_config(config);
            let device = build_device(id, config)?;
            if devices.insert(id, device).is_some() {
                return Err(RunnerError::Recovery {
                    field: format!("vehicle.assembly.recovery.{id_text}", id_text = config.id),
                    reason: "duplicate recovery id (collision in fnv1a-64 hash)".to_owned(),
                });
            }
            scenario_ids.insert(id, config.id.clone());
        }

        Ok(Self {
            devices,
            scenario_ids,
        })
    }

    /// `true` when the rack carries no recovery devices.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.devices.is_empty()
    }

    /// Number of recovery devices in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.devices.len()
    }

    /// Recovery-device ids in `BTreeMap` (sorted) order.
    #[must_use]
    pub fn recovery_ids(&self) -> Vec<RecoveryId> {
        self.devices.keys().copied().collect()
    }

    /// Scenario-text id for a given `RecoveryId`. Used by telemetry
    /// channel registration.
    #[must_use]
    pub fn scenario_id(&self, id: RecoveryId) -> Option<&str> {
        self.scenario_ids.get(&id).map(String::as_str)
    }

    /// Apply any scenario-script recovery-deploy actions drained from
    /// the kernel's per-step fired-event queue.
    ///
    /// Multiple command firings for the same device in one step are
    /// rejected; the scenario author must resolve the command bundle
    /// explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] when:
    ///
    /// - An event targets a recovery id that is not present in this
    ///   rack (defensive — scenario validation should catch this).
    /// - The device rejects the command (kind-incompatible
    ///   transition; e.g. `deploy_drogue` against a `parachute_drag`).
    /// - Multiple commands target the same device in this rack tick.
    pub fn apply_deploys(
        &mut self,
        fired: &[FiredEvent<ScenarioScriptAction>],
    ) -> Result<(), RunnerError> {
        let mut seen: BTreeSet<RecoveryId> = BTreeSet::new();
        for event in fired {
            if let ScenarioScriptAction::DeployRecovery { id, command } = &event.action {
                if !seen.insert(*id) {
                    return Err(RunnerError::Recovery {
                        field: format!(
                            "mission.events[*].action.deploy_recovery.{id_value}",
                            id_value = id.value()
                        ),
                        reason: "multiple `deploy_recovery` events fired for the same device in one step; resolve to a single command per device per step".to_owned(),
                    });
                }
                let typed_command =
                    parse_recovery_command(command).ok_or_else(|| RunnerError::Recovery {
                        field: format!(
                            "mission.events[*].action.deploy_recovery.{id_value}.command",
                            id_value = id.value()
                        ),
                        reason: format!(
                            "unknown recovery command `{command}`; expected one of \
                             `deploy`, `deploy_drogue`, `deploy_main`, `stow`",
                        ),
                    })?;
                let device = self.devices.get_mut(id).ok_or_else(|| RunnerError::Recovery {
                    field: format!(
                        "mission.events[*].action.deploy_recovery.{id_value}",
                        id_value = id.value()
                    ),
                    reason: "deploy_recovery event targets an unknown recovery id".to_owned(),
                })?;
                device
                    .apply_command(typed_command)
                    .map_err(|err| RunnerError::Recovery {
                        field: format!(
                            "mission.events[*].action.deploy_recovery.{id_value}",
                            id_value = id.value()
                        ),
                        reason: err.to_string(),
                    })?;
            }
        }
        Ok(())
    }

    /// Advance every recovery device's internal state by one kernel
    /// base tick. Phase-3.9 instances are instantaneous-deploy and
    /// `step` is a no-op, but the call is part of the rack contract
    /// for future extensions (canopy inflation transients, brake
    /// rate-limiting, etc.).
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] when a device's `step()`
    /// rejects.
    pub fn step(&mut self, dt_s: f64) -> Result<(), RunnerError> {
        for (id, device) in &mut self.devices {
            device.step(dt_s).map_err(|err| RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}",
                    id_value = id.value()
                ),
                reason: err.to_string(),
            })?;
        }
        Ok(())
    }

    /// Produce the per-recovery snapshot map the kernel consumes via
    /// `set_recovery_snapshot`. Keys are `RecoveryId`; values are
    /// the device's current phase, drag area, and drag coefficient.
    #[must_use]
    pub fn snapshot_map(&self) -> BTreeMap<RecoveryId, RecoverySnapshot> {
        let mut out = BTreeMap::new();
        for (id, device) in &self.devices {
            let phase = device.phase();
            let snap = RecoverySnapshot {
                deployed: phase.is_deployed(),
                phase_index: phase.index(),
                c_d: device.current_c_d(),
                drag_area_m2: device.current_drag_area_m2(),
            };
            out.insert(*id, snap);
        }
        out
    }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

fn recovery_id_from_config(config: &RecoveryConfig) -> RecoveryId {
    RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = config.id))
}

fn build_device(
    id: RecoveryId,
    config: &RecoveryConfig,
) -> Result<Box<dyn RecoveryModel>, RunnerError> {
    let field = format!("vehicle.assembly.recovery.{id_text}", id_text = config.id);
    match config.kind {
        RecoveryKindConfig::ParachuteDrag {
            c_d,
            area_inflated_m2,
        } => ParachuteDrag::new(id, c_d, area_inflated_m2)
            .map(|m| Box::new(m) as Box<dyn RecoveryModel>)
            .map_err(|err| RunnerError::Recovery {
                field,
                reason: err.to_string(),
            }),
        RecoveryKindConfig::DrogueMain {
            drogue_c_d,
            drogue_area_m2,
            main_c_d,
            main_area_m2,
        } => DrogueMainRecovery::new(id, drogue_c_d, drogue_area_m2, main_c_d, main_area_m2)
            .map(|m| Box::new(m) as Box<dyn RecoveryModel>)
            .map_err(|err| RunnerError::Recovery {
                field,
                reason: err.to_string(),
            }),
        RecoveryKindConfig::DragDevice {
            c_d,
            area_deployed_m2,
        } => DragDevice::new(id, c_d, area_deployed_m2)
            .map(|m| Box::new(m) as Box<dyn RecoveryModel>)
            .map_err(|err| RunnerError::Recovery {
                field,
                reason: err.to_string(),
            }),
    }
}

fn parse_recovery_command(command: &str) -> Option<RecoveryCommand> {
    match command {
        "deploy" => Some(RecoveryCommand::Deploy),
        "deploy_drogue" => Some(RecoveryCommand::DeployDrogue),
        "deploy_main" => Some(RecoveryCommand::DeployMain),
        "stow" => Some(RecoveryCommand::Stow),
        _ => None,
    }
}
