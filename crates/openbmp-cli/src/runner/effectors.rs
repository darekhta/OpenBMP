//! Phase-3.4 runner-side effector rack.
//!
//! The rack owns the `Vec<Box<dyn ControlEffector>>` resolved from
//! the scenario's `[[vehicle.assembly.effectors]]` block, plus the
//! per-effector deterministic command schedules and one-shot
//! override map. It is stepped once per kernel base tick **before**
//! the kernel's `step()` so the effector telemetry observable in
//! the Parquet matches the kernel's view of the world.
//!
//! Phase-3.4 leaves the kernel's force / moment evaluation untouched
//! when the rack is empty — legacy scenarios produce byte-identical
//! Parquet because the runner short-circuits every rack-related
//! operation on `is_empty()`.
//!
//! Aero-deck schema-2 consumption of effector deflections lands in
//! Phase 3.5; in 3.4 the rack snapshot is read only by the
//! telemetry layer.

use std::collections::BTreeMap;

use openbmp_core::{Duration, EffectorId, SimTime};
use openbmp_scenario::{
    EffectorCommandScheduleConfig, EffectorConfig, EffectorFaultConfig, EffectorKindConfig,
    ScenarioDocument,
};
use openbmp_sim::{EventAction, FiredEvent};
use openbmp_vehicle::{
    ControlEffector, EffectorFault, EffectorLimits, EffectorState, LinearActuator,
};

use crate::error::CliError;

/// Runner-side effector container. Built once per `openbmp run`
/// invocation; consumed by the per-step kernel loop.
pub struct EffectorRack {
    /// Effectors in scenario-declared order.
    effectors: Vec<Box<dyn ControlEffector>>,
    /// Telemetry-friendly id-string copies of each effector's
    /// scenario-declared id. Same iteration order as `effectors`.
    string_ids: Vec<String>,
    /// Lookup from `EffectorId` to the `effectors` index.
    /// `BTreeMap`, not `HashMap`, to defeat macOS `SipHash`
    /// randomisation when the runner iterates the override map.
    id_index: BTreeMap<EffectorId, usize>,
    /// Per-effector command schedules. Indexed by effector index;
    /// `None` when the scenario declared no schedule for that
    /// effector (defaults to commanding the initial position
    /// indefinitely).
    schedules: Vec<Option<EffectorSchedule>>,
    /// One-shot command overrides drained from the kernel's
    /// `EventAction::EffectorOverride` events. Cleared after each
    /// `step()`.
    overrides: BTreeMap<EffectorId, f64>,
    /// Construction-time `dt`; stepped at exactly this rate.
    dt: Duration,
}

impl std::fmt::Debug for EffectorRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EffectorRack")
            .field("effector_count", &self.effectors.len())
            .field(
                "schedule_count",
                &self.schedules.iter().filter(|s| s.is_some()).count(),
            )
            .field("override_count", &self.overrides.len())
            .finish_non_exhaustive()
    }
}

/// Per-effector deterministic command schedule.
#[derive(Clone, Debug)]
enum EffectorSchedule {
    Constant(f64),
    StepAt {
        time_s: f64,
        before: f64,
        after: f64,
    },
    LinearRamp {
        start_time_s: f64,
        end_time_s: f64,
        start: f64,
        end: f64,
    },
}

impl EffectorSchedule {
    fn from_config(config: &EffectorCommandScheduleConfig) -> Self {
        match *config {
            EffectorCommandScheduleConfig::Constant { value } => Self::Constant(value),
            EffectorCommandScheduleConfig::StepAt {
                time_s,
                before,
                after,
            } => Self::StepAt {
                time_s,
                before,
                after,
            },
            EffectorCommandScheduleConfig::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => Self::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            },
        }
    }

    fn command_at(&self, time: SimTime) -> f64 {
        let t = time.as_seconds();
        match *self {
            Self::Constant(value) => value,
            Self::StepAt {
                time_s,
                before,
                after,
            } => {
                if t < time_s {
                    before
                } else {
                    after
                }
            }
            Self::LinearRamp {
                start_time_s,
                end_time_s,
                start,
                end,
            } => {
                if t <= start_time_s {
                    start
                } else if t >= end_time_s {
                    end
                } else {
                    let alpha = (t - start_time_s) / (end_time_s - start_time_s);
                    start + alpha * (end - start)
                }
            }
        }
    }
}

impl EffectorRack {
    /// Build the rack from the parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.effectors]]` blocks
    /// are declared.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Assembly`] when a `LinearActuator::new`
    /// call rejects the scenario-declared limits / initial position
    /// / latency / `tau_s`.
    pub fn build(document: &ScenarioDocument) -> Result<Self, CliError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut effectors: Vec<Box<dyn ControlEffector>> = Vec::new();
        let mut string_ids: Vec<String> = Vec::new();
        let mut id_index: BTreeMap<EffectorId, usize> = BTreeMap::new();
        let mut schedules: Vec<Option<EffectorSchedule>> = Vec::new();

        if let Some(assembly) = &document.vehicle.assembly {
            for (index, config) in assembly.effectors.iter().enumerate() {
                let actuator = build_effector(index, config, dt)?;
                let id = actuator.id();
                effectors.push(Box::new(actuator));
                string_ids.push(config.id.clone());
                id_index.insert(id, index);
                schedules.push(
                    config
                        .command_schedule
                        .as_ref()
                        .map(EffectorSchedule::from_config),
                );
            }
        }

        Ok(Self {
            effectors,
            string_ids,
            id_index,
            schedules,
            overrides: BTreeMap::new(),
            dt,
        })
    }

    /// Number of effectors in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.effectors.len()
    }

    /// `true` when the rack carries no effectors. The runner gates
    /// every per-step operation on this so legacy scenarios never
    /// touch the rack code path.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.effectors.is_empty()
    }

    /// Scenario-declared id strings, in declared order.
    #[must_use]
    pub fn scenario_ids(&self) -> &[String] {
        &self.string_ids
    }

    /// Apply any `EventAction::EffectorOverride` actions drained
    /// from the kernel's per-step fired-event queue. Override values
    /// take precedence over the schedule for the next step only.
    pub fn apply_overrides(&mut self, fired: &[FiredEvent]) {
        for event in fired {
            if let EventAction::EffectorOverride { id, command } = event.action
                && self.id_index.contains_key(&id)
            {
                self.overrides.insert(id, command);
            }
        }
    }

    /// Step every effector by one kernel base tick.
    ///
    /// Command resolution: override (if present) wins; else the
    /// schedule's command at `time` (if a schedule is declared);
    /// else the effector's `current_state().commanded` (hold).
    /// Overrides are cleared after the step.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Effector`] when `LinearActuator::step`
    /// fails (dt mismatch, non-finite command, etc.).
    pub fn step(&mut self, time: SimTime) -> Result<(), CliError> {
        for (index, effector) in self.effectors.iter_mut().enumerate() {
            let id = effector.id();
            let cmd = if let Some(&override_cmd) = self.overrides.get(&id) {
                override_cmd
            } else if let Some(schedule) = &self.schedules[index] {
                schedule.command_at(time)
            } else {
                effector.current_state().commanded
            };
            effector
                .step(cmd, self.dt)
                .map_err(|err| CliError::Effector {
                    field: format!("vehicle.assembly.effectors[{index}]"),
                    reason: err.to_string(),
                })?;
        }
        self.overrides.clear();
        Ok(())
    }

    /// Snapshot every effector's most recent `EffectorState` for
    /// telemetry, in declared order.
    #[must_use]
    pub fn snapshot(&self) -> Vec<EffectorState> {
        self.effectors.iter().map(|e| e.current_state()).collect()
    }
}

/// Phase-3.4 effector resolver: scenario `EffectorConfig` →
/// `LinearActuator` (the only kind shipped in 3.4). Mounts the
/// optional load-time fault.
fn build_effector(
    index: usize,
    config: &EffectorConfig,
    dt: Duration,
) -> Result<LinearActuator, CliError> {
    let id = EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}", id = config.id));
    let limits = EffectorLimits {
        min: config.limits.min,
        max: config.limits.max,
        max_rate_per_s: config.limits.max_rate_per_s,
        deadband: config.limits.deadband,
        latency: Duration::from_seconds(config.limits.latency_s),
    };
    let initial_position = config.initial_position.unwrap_or(0.0);
    let tau_s = match config.kind {
        EffectorKindConfig::LinearActuator { tau_s } => tau_s.unwrap_or(0.0),
    };
    let mut actuator =
        LinearActuator::new(id, limits, dt, initial_position, tau_s).map_err(|err| {
            CliError::Assembly {
                field: format!("vehicle.assembly.effectors[{index}]"),
                reason: err.to_string(),
            }
        })?;
    if let Some(fault_config) = &config.fault {
        let fault = match *fault_config {
            EffectorFaultConfig::Jam { at } => EffectorFault::Jam { at },
            EffectorFaultConfig::Runaway { rate_per_s } => EffectorFault::Runaway { rate_per_s },
            EffectorFaultConfig::ReducedRate { factor } => EffectorFault::ReducedRate { factor },
            EffectorFaultConfig::Hardover { to } => EffectorFault::Hardover { to },
        };
        actuator.inject_fault(fault);
    }
    Ok(actuator)
}
