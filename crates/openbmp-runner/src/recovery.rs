//! Runner-side recovery rack.
//!
//! The rack owns a `BTreeMap<RecoveryId, Box<dyn RecoveryModel>>`
//! resolved from the scenario's `[[vehicle.assembly.recovery]]`
//! block. It mirrors the [`crate::engines::EngineRack`]
//! and [`crate::tanks::TankRack`] patterns:
//!
//! Each kernel base tick the runner:
//!
//! 1. Drains any scenario-script recovery-deploy firings via
//!    [`RecoveryRack::apply_deploys`]. Each event is mapped to the
//!    typed `openbmp_vehicle::RecoveryCommand` and forwarded to the
//!    target device's `apply_command(...)`. Multiple command firings
//!    for the same device in one step are rejected with
//!    [`RunnerError::Recovery`].
//! 2. Calls [`RecoveryRack::step`] to advance internal state. The
//!    shipped instances are instantaneous-deploy and `step` is a no-op,
//!    but the call is part of the rack contract.
//! 3. Packs each device's `(phase, c_d, drag_area)` state and any
//!    active inflation opening-load diagnostics into a
//!    `BTreeMap<RecoveryId, RecoverySnapshot>` via
//!    `RecoveryRack::snapshot_map_with_opening_load_context` and
//!    pushes it into the kernel via `set_recovery_snapshot(...)`.
//!
//! The rack leaves the kernel hot path's force evaluation untouched
//! when it is empty — legacy scenarios produce byte-identical
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

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use openbmp_core::{BodyId, RecoveryId};
use openbmp_scenario::{
    RecoveryConfig, RecoveryInflationConfig, RecoveryInflationGrowthKindConfig, RecoveryKindConfig,
    RecoveryPflanzReductionConfig, RecoveryRiserCanopyConfig, RecoveryRiserConfig,
    RecoverySequenceConfig, ResolvedFile, ScenarioDocument,
};
use openbmp_sim::{EnvironmentSample, FiredEvent, RecoverySnapshot, ScenarioScriptAction};
use openbmp_vehicle::recovery::{
    RiserEndpointState, RiserLineElement, RiserSlackTautEvent, riser_attach_moment_body_n_m,
};
use openbmp_vehicle::{
    CanopyGrowthShape, DragDevice, DrogueMainRecovery, InflationProfile, ParachuteDrag,
    PflanzReductionTable, RecoveryCommand, RecoveryModel, RecoveryPhase, canopy_apparent_mass_kg,
    canopy_apparent_mass_rate_kg_s, canopy_fill_time_s, pflanz_ballistic_parameter_a,
    pflanz_finite_mass_peak_load_n,
};
use serde::Deserialize;

use crate::error::RunnerError;

#[derive(Clone, Copy, Debug)]
pub(crate) struct RecoveryFrameAlias {
    pub(crate) rot_owner_from_body: Matrix3<f64>,
    pub(crate) translation_owner_from_body_m: Vector3<f64>,
}

/// Runner-side recovery rack. Built once per `openbmp run`
/// invocation; consumed by the per-step kernel loop.
pub struct RecoveryRack {
    devices: BTreeMap<RecoveryId, Box<dyn RecoveryModel>>,
    finite_mass_opening_loads: BTreeMap<RecoveryId, RecoveryStageModels<PflanzOpeningLoadModel>>,
    apparent_mass_models: BTreeMap<RecoveryId, RecoveryStageModels<CanopyApparentMassModel>>,
    sequence_diagnostics: BTreeMap<RecoveryId, RecoverySequenceDiagnostics>,
    risers: BTreeMap<RecoveryId, RecoveryRiserSet>,
    /// Scenario-text id strings indexed by `RecoveryId`. Used in
    /// telemetry-channel naming.
    scenario_ids: BTreeMap<RecoveryId, String>,
}

impl std::fmt::Debug for RecoveryRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecoveryRack")
            .field("device_count", &self.devices.len())
            .field(
                "finite_mass_opening_load_count",
                &self.finite_mass_opening_loads.len(),
            )
            .field("apparent_mass_count", &self.apparent_mass_models.len())
            .field(
                "sequence_diagnostic_count",
                &self.sequence_diagnostics.len(),
            )
            .field("riser_device_count", &self.risers.len())
            .field("scenario_ids", &self.scenario_ids)
            .finish()
    }
}

#[derive(Debug)]
struct RecoveryInflationRuntime {
    profile: InflationProfile,
    finite_mass_opening_load: Option<PflanzOpeningLoadModel>,
    apparent_mass: Option<CanopyApparentMassModel>,
}

#[derive(Clone, Debug)]
enum RecoveryStageModels<T> {
    Single(T),
    DrogueMain { drogue: Option<T>, main: Option<T> },
}

impl<T> RecoveryStageModels<T> {
    fn active(&self, phase: RecoveryPhase) -> Option<&T> {
        match self {
            Self::Single(model) if phase.is_deployed() => Some(model),
            Self::Single(_) => None,
            Self::DrogueMain { drogue, main } => match phase {
                RecoveryPhase::Stowed => None,
                RecoveryPhase::Drogue => drogue.as_ref(),
                RecoveryPhase::Main => main.as_ref(),
            },
        }
    }
}

#[derive(Clone, Debug)]
struct PflanzOpeningLoadModel {
    full_c_d: f64,
    full_drag_area_m2: f64,
    fill_time_s: f64,
    exponent_j: f64,
    table: PflanzReductionTable,
}

#[derive(Clone, Debug)]
struct CanopyApparentMassModel {
    apparent_mass_coeff_k_a: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoverySequenceDiagnostics {
    snatch: RecoverySnatchDiagnostic,
    mortar: Option<RecoveryMortarImpulseDiagnostic>,
    pending_mortar_impulse_body_n_s: Option<Vector3<f64>>,
    active_mortar_impulse_body_n_s: Vector3<f64>,
    active_mortar_force_body_n: Vector3<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct RecoverySnatchDiagnostic {
    delta_v_m_s: f64,
    stiffness_n_m: f64,
    effective_mass_kg: f64,
    pack_drag_load_n: f64,
    peak_load_n: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct RecoveryMortarImpulseDiagnostic {
    pack_mass_kg: f64,
    pack_velocity_body_m_s: Vector3<f64>,
    impulse_body_n_s: Vector3<f64>,
}

impl RecoverySequenceDiagnostics {
    fn new(
        snatch: RecoverySnatchDiagnostic,
        mortar: Option<RecoveryMortarImpulseDiagnostic>,
    ) -> Self {
        Self {
            snatch,
            mortar,
            pending_mortar_impulse_body_n_s: None,
            active_mortar_impulse_body_n_s: Vector3::zeros(),
            active_mortar_force_body_n: Vector3::zeros(),
        }
    }

    fn arm_mortar_impulse(&mut self) {
        if let Some(mortar) = self.mortar {
            self.pending_mortar_impulse_body_n_s = Some(mortar.impulse_body_n_s);
        }
    }

    fn step(&mut self, id: RecoveryId, phase: RecoveryPhase, dt_s: f64) -> Result<(), RunnerError> {
        self.active_mortar_impulse_body_n_s = Vector3::zeros();
        self.active_mortar_force_body_n = Vector3::zeros();
        if !phase.is_deployed() {
            self.pending_mortar_impulse_body_n_s = None;
            return Ok(());
        }
        let Some(impulse_body_n_s) = self.pending_mortar_impulse_body_n_s.take() else {
            return Ok(());
        };
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.sequence",
                    id_value = id.value()
                ),
                reason: "mortar impulse force path requires a finite positive time step".to_owned(),
            });
        }
        let force_body_n = impulse_body_n_s / dt_s;
        if !(is_finite_vec3(impulse_body_n_s) && is_finite_vec3(force_body_n)) {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.sequence",
                    id_value = id.value()
                ),
                reason: "mortar impulse force path produced a non-finite force".to_owned(),
            });
        }
        self.active_mortar_impulse_body_n_s = impulse_body_n_s;
        self.active_mortar_force_body_n = force_body_n;
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
struct RecoveryRiserRuntime {
    line: RiserLineElement,
    vehicle_attach_body_m: Vector3<f64>,
    initial_canopy_attach_body_m: Vector3<f64>,
    initial_canopy_velocity_body_m_s: Vector3<f64>,
    canopy_mass_kg: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoveryRiserState {
    canopy_attach_body_m: Vector3<f64>,
    canopy_velocity_body_m_s: Vector3<f64>,
    last_slack_taut_event_time_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoverySharedCanopyRuntime {
    initial_origin_body_m: Vector3<f64>,
    initial_velocity_body_m_s: Vector3<f64>,
    mass_kg: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoverySharedCanopyState {
    origin_body_m: Vector3<f64>,
    velocity_body_m_s: Vector3<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoverySharedCanopy {
    runtime: RecoverySharedCanopyRuntime,
    state: RecoverySharedCanopyState,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoveryRiserSet {
    lines: Vec<RecoveryRiserRuntime>,
    states: Vec<RecoveryRiserState>,
    shared_canopy: Option<RecoverySharedCanopy>,
    aggregate: RecoveryRiserAggregate,
}

impl RecoveryRiserSet {
    fn new(
        id: RecoveryId,
        lines: Vec<RecoveryRiserRuntime>,
        shared_canopy_runtime: Option<RecoverySharedCanopyRuntime>,
    ) -> Result<Self, RunnerError> {
        let shared_canopy = shared_canopy_runtime.map(|runtime| RecoverySharedCanopy {
            state: RecoverySharedCanopyState {
                origin_body_m: runtime.initial_origin_body_m,
                velocity_body_m_s: runtime.initial_velocity_body_m_s,
            },
            runtime,
        });
        let states = initial_riser_states(&lines, shared_canopy.as_ref());
        let aggregate = evaluate_declared_risers(id, &lines, &states)?;
        Ok(Self {
            lines,
            states,
            shared_canopy,
            aggregate,
        })
    }

    fn step(&mut self, id: RecoveryId, phase: RecoveryPhase, dt_s: f64) -> Result<(), RunnerError> {
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.risers",
                    id_value = id.value()
                ),
                reason: "riser time step must be finite and non-negative".to_owned(),
            });
        }
        if !phase.is_deployed() {
            self.reset(id)?;
            return Ok(());
        }
        if dt_s == 0.0 {
            self.aggregate = evaluate_declared_risers(id, &self.lines, &self.states)?;
            return Ok(());
        }
        if self.shared_canopy.is_some() {
            self.step_shared_canopy(id, dt_s)?;
            self.aggregate = evaluate_declared_risers(id, &self.lines, &self.states)?;
            return Ok(());
        }
        for (line, state) in self.lines.iter().zip(self.states.iter_mut()) {
            let anchor = RiserEndpointState::new(line.vehicle_attach_body_m, Vector3::zeros());
            let canopy =
                RiserEndpointState::new(state.canopy_attach_body_m, state.canopy_velocity_body_m_s);
            let event = line
                .line
                .slack_taut_event(anchor, canopy, dt_s)
                .map_err(|err| RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery.{id_value}.risers",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?;
            state.last_slack_taut_event_time_s = event.map(|event| event.time_s);
            let advanced = if let Some(canopy_mass_kg) = line.canopy_mass_kg {
                advance_force_integrated_riser_endpoint(
                    id,
                    line,
                    canopy,
                    event,
                    dt_s,
                    canopy_mass_kg,
                )?
            } else {
                canopy
                    .advance_by(dt_s)
                    .map_err(|err| RunnerError::Recovery {
                        field: format!(
                            "vehicle.assembly.recovery.{id_value}.risers",
                            id_value = id.value()
                        ),
                        reason: err.to_string(),
                    })?
            };
            state.canopy_attach_body_m = advanced.position_m;
            state.canopy_velocity_body_m_s = advanced.velocity_m_s;
        }
        self.aggregate = evaluate_declared_risers(id, &self.lines, &self.states)?;
        Ok(())
    }

    fn reset(&mut self, id: RecoveryId) -> Result<(), RunnerError> {
        if let Some(shared_canopy) = &mut self.shared_canopy {
            shared_canopy.state.origin_body_m = shared_canopy.runtime.initial_origin_body_m;
            shared_canopy.state.velocity_body_m_s = shared_canopy.runtime.initial_velocity_body_m_s;
            self.states = initial_riser_states(&self.lines, self.shared_canopy.as_ref());
            self.aggregate = evaluate_declared_risers(id, &self.lines, &self.states)?;
            return Ok(());
        }
        for (line, state) in self.lines.iter().zip(self.states.iter_mut()) {
            state.canopy_attach_body_m = line.initial_canopy_attach_body_m;
            state.canopy_velocity_body_m_s = line.initial_canopy_velocity_body_m_s;
            state.last_slack_taut_event_time_s = None;
        }
        self.aggregate = evaluate_declared_risers(id, &self.lines, &self.states)?;
        Ok(())
    }

    fn step_shared_canopy(&mut self, id: RecoveryId, dt_s: f64) -> Result<(), RunnerError> {
        let Some(shared_canopy) = self.shared_canopy.as_mut() else {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.riser_canopy",
                    id_value = id.value()
                ),
                reason: "shared riser canopy state is missing".to_owned(),
            });
        };
        validate_shared_canopy_mass(id, shared_canopy.runtime.mass_kg)?;

        let mut events: Vec<Option<RiserSlackTautEvent>> = Vec::with_capacity(self.lines.len());
        let mut first_event_time_s: Option<f64> = None;
        for line in &self.lines {
            let anchor = RiserEndpointState::new(line.vehicle_attach_body_m, Vector3::zeros());
            let canopy = RiserEndpointState::new(
                shared_canopy.state.origin_body_m + line.initial_canopy_attach_body_m,
                shared_canopy.state.velocity_body_m_s,
            );
            let event = line
                .line
                .slack_taut_event(anchor, canopy, dt_s)
                .map_err(|err| RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery.{id_value}.riser_canopy",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?;
            if let Some(event) = event {
                first_event_time_s = Some(
                    first_event_time_s.map_or(event.time_s, |current| current.min(event.time_s)),
                );
                events.push(Some(event));
            } else {
                events.push(None);
            }
        }

        let Some(first_event_time_s) = first_event_time_s else {
            shared_canopy.state.origin_body_m += shared_canopy.state.velocity_body_m_s * dt_s;
            validate_shared_canopy_state(id, &shared_canopy.state)?;
            sync_shared_canopy_riser_states(&self.lines, &mut self.states, shared_canopy, &events);
            return Ok(());
        };

        let active_dt_s = dt_s - first_event_time_s;
        if !active_dt_s.is_finite() || active_dt_s < 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.riser_canopy",
                    id_value = id.value()
                ),
                reason: "shared riser canopy produced an invalid integration interval".to_owned(),
            });
        }
        let event_origin_body_m = shared_canopy.state.origin_body_m
            + shared_canopy.state.velocity_body_m_s * first_event_time_s;
        if active_dt_s > 0.0 {
            let mut force_on_canopy_body_n = Vector3::zeros();
            for line in &self.lines {
                let anchor = RiserEndpointState::new(line.vehicle_attach_body_m, Vector3::zeros());
                let canopy = RiserEndpointState::new(
                    event_origin_body_m + line.initial_canopy_attach_body_m,
                    shared_canopy.state.velocity_body_m_s,
                );
                let sample =
                    line.line
                        .sample(anchor, canopy)
                        .map_err(|err| RunnerError::Recovery {
                            field: format!(
                                "vehicle.assembly.recovery.{id_value}.riser_canopy",
                                id_value = id.value()
                            ),
                            reason: err.to_string(),
                        })?;
                force_on_canopy_body_n += sample.force_on_canopy_n;
            }
            let acceleration_body_m_s2 = force_on_canopy_body_n / shared_canopy.runtime.mass_kg;
            if !is_finite_vec3(acceleration_body_m_s2) {
                return Err(RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery.{id_value}.riser_canopy",
                        id_value = id.value()
                    ),
                    reason: "shared riser canopy acceleration was non-finite".to_owned(),
                });
            }
            shared_canopy.state.velocity_body_m_s += acceleration_body_m_s2 * active_dt_s;
        }
        shared_canopy.state.origin_body_m =
            event_origin_body_m + shared_canopy.state.velocity_body_m_s * active_dt_s;
        validate_shared_canopy_state(id, &shared_canopy.state)?;
        sync_shared_canopy_riser_states(&self.lines, &mut self.states, shared_canopy, &events);
        Ok(())
    }
}

fn initial_riser_states(
    lines: &[RecoveryRiserRuntime],
    shared_canopy: Option<&RecoverySharedCanopy>,
) -> Vec<RecoveryRiserState> {
    lines
        .iter()
        .map(|line| {
            if let Some(shared_canopy) = shared_canopy {
                RecoveryRiserState {
                    canopy_attach_body_m: shared_canopy.state.origin_body_m
                        + line.initial_canopy_attach_body_m,
                    canopy_velocity_body_m_s: shared_canopy.state.velocity_body_m_s,
                    last_slack_taut_event_time_s: None,
                }
            } else {
                RecoveryRiserState {
                    canopy_attach_body_m: line.initial_canopy_attach_body_m,
                    canopy_velocity_body_m_s: line.initial_canopy_velocity_body_m_s,
                    last_slack_taut_event_time_s: None,
                }
            }
        })
        .collect()
}

fn sync_shared_canopy_riser_states(
    lines: &[RecoveryRiserRuntime],
    states: &mut [RecoveryRiserState],
    shared_canopy: &RecoverySharedCanopy,
    events: &[Option<RiserSlackTautEvent>],
) {
    for ((line, state), event) in lines.iter().zip(states.iter_mut()).zip(events.iter()) {
        state.canopy_attach_body_m =
            shared_canopy.state.origin_body_m + line.initial_canopy_attach_body_m;
        state.canopy_velocity_body_m_s = shared_canopy.state.velocity_body_m_s;
        state.last_slack_taut_event_time_s = event.map(|event| event.time_s);
    }
}

fn validate_shared_canopy_mass(id: RecoveryId, mass_kg: f64) -> Result<(), RunnerError> {
    if !mass_kg.is_finite() || mass_kg <= 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.riser_canopy.mass_kg",
                id_value = id.value()
            ),
            reason: "shared riser canopy mass must be finite and positive".to_owned(),
        });
    }
    Ok(())
}

fn validate_shared_canopy_state(
    id: RecoveryId,
    state: &RecoverySharedCanopyState,
) -> Result<(), RunnerError> {
    if !(is_finite_vec3(state.origin_body_m) && is_finite_vec3(state.velocity_body_m_s)) {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.riser_canopy",
                id_value = id.value()
            ),
            reason: "shared riser canopy produced a non-finite state".to_owned(),
        });
    }
    Ok(())
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct RecoveryRiserLoadShare {
    attach_point_body_m: Vector3<f64>,
    share: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct RecoveryRiserAggregate {
    tension_sum_n: f64,
    tension_max_n: f64,
    force_body_n: Vector3<f64>,
    moment_body_n_m: Vector3<f64>,
    elastic_energy_j: f64,
    load_shares: Vec<RecoveryRiserLoadShare>,
}

const RECOVERY_PFLANZ_DECK_FORMAT: &str = "openbmp.recovery.pflanz_x1.v1";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RecoveryPflanzDeckDocument {
    format: String,
    provenance: String,
    ballistic_parameter_a: Vec<f64>,
    exponent_j: Vec<f64>,
    reduction_factors: Vec<f64>,
}

/// Inputs used to evaluate recovery opening-load diagnostics for a base tick.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct RecoveryOpeningLoadContext {
    /// Dynamic pressure from the pre-step air-relative velocity.
    pub dynamic_pressure_pa: f64,
    /// Atmosphere density used by the recovery force adapter.
    pub density_kg_m3: f64,
    /// Air-relative vehicle velocity in ECI, m/s.
    pub air_relative_velocity_eci_m_s: Vector3<f64>,
    /// Live body-to-ECI attitude used to express transmitted drag in body axes.
    pub body_to_eci: UnitQuaternion<f64>,
    /// Air-relative speed used as the line-stretch speed estimate.
    pub line_stretch_speed_m_s: f64,
    /// Current vehicle system mass used in the Pflanz ballistic parameter.
    pub system_mass_kg: f64,
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
    pub fn build(
        document: &ScenarioDocument,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Self, RunnerError> {
        Self::build_with_frame_aliases(document, resolved_files, &BTreeMap::new())
    }

    /// Build the rack while rewriting aliased body-frame recovery vectors into
    /// the routed owner frame.
    pub(crate) fn build_with_frame_aliases(
        document: &ScenarioDocument,
        resolved_files: &BTreeMap<String, ResolvedFile>,
        frame_aliases: &BTreeMap<BodyId, RecoveryFrameAlias>,
    ) -> Result<Self, RunnerError> {
        let mut devices: BTreeMap<RecoveryId, Box<dyn RecoveryModel>> = BTreeMap::new();
        let mut finite_mass_opening_loads: BTreeMap<
            RecoveryId,
            RecoveryStageModels<PflanzOpeningLoadModel>,
        > = BTreeMap::new();
        let mut apparent_mass_models: BTreeMap<
            RecoveryId,
            RecoveryStageModels<CanopyApparentMassModel>,
        > = BTreeMap::new();
        let mut sequence_diagnostics: BTreeMap<RecoveryId, RecoverySequenceDiagnostics> =
            BTreeMap::new();
        let mut risers: BTreeMap<RecoveryId, RecoveryRiserSet> = BTreeMap::new();
        let mut scenario_ids: BTreeMap<RecoveryId, String> = BTreeMap::new();

        for config in &document.vehicle.assembly.recovery {
            let id = recovery_id_from_config(config);
            let frame_alias = recovery_frame_alias(config, frame_aliases);
            let (device, finite_mass_opening_load, apparent_mass) =
                build_device(id, config, resolved_files)?;
            let recovery_risers = recovery_riser_models(config, frame_alias)?;
            let sequence = recovery_sequence_diagnostics(config, &recovery_risers, frame_alias)?;
            let shared_canopy = recovery_riser_canopy_runtime(config, frame_alias);
            let riser_set = (!recovery_risers.is_empty())
                .then(|| RecoveryRiserSet::new(id, recovery_risers, shared_canopy))
                .transpose()?;
            if devices.insert(id, device).is_some() {
                return Err(RunnerError::Recovery {
                    field: format!("vehicle.assembly.recovery.{id_text}", id_text = config.id),
                    reason: "duplicate recovery id (collision in fnv1a-64 hash)".to_owned(),
                });
            }
            if let Some(finite_mass_opening_load) = finite_mass_opening_load {
                finite_mass_opening_loads.insert(id, finite_mass_opening_load);
            }
            if let Some(apparent_mass) = apparent_mass {
                apparent_mass_models.insert(id, apparent_mass);
            }
            if let Some(sequence) = sequence {
                sequence_diagnostics.insert(id, sequence);
            }
            if let Some(riser_set) = riser_set {
                risers.insert(id, riser_set);
            }
            scenario_ids.insert(id, config.id.clone());
        }

        Ok(Self {
            devices,
            finite_mass_opening_loads,
            apparent_mass_models,
            sequence_diagnostics,
            risers,
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
                let device = self
                    .devices
                    .get_mut(id)
                    .ok_or_else(|| RunnerError::Recovery {
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
                if matches!(
                    typed_command,
                    RecoveryCommand::Deploy
                        | RecoveryCommand::DeployDrogue
                        | RecoveryCommand::DeployMain
                ) && let Some(sequence) = self.sequence_diagnostics.get_mut(id)
                {
                    sequence.arm_mortar_impulse();
                }
            }
        }
        Ok(())
    }

    /// Advance every recovery device's internal state by one kernel
    /// base tick. The shipped instances are instantaneous-deploy and
    /// `step` is a no-op, but the call is part of the rack contract
    /// for future extensions (canopy inflation transients, brake
    /// rate-limiting, etc.).
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] when a device's `step()`
    /// rejects.
    pub fn step(&mut self, dt_s: f64) -> Result<(), RunnerError> {
        let mut phases: Vec<(RecoveryId, RecoveryPhase)> = Vec::with_capacity(self.devices.len());
        for (id, device) in &mut self.devices {
            device.step(dt_s).map_err(|err| RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}",
                    id_value = id.value()
                ),
                reason: err.to_string(),
            })?;
            phases.push((*id, device.phase()));
        }
        for (id, phase) in phases {
            if let Some(sequence) = self.sequence_diagnostics.get_mut(&id) {
                sequence.step(id, phase, dt_s)?;
            }
            if let Some(risers) = self.risers.get_mut(&id) {
                risers.step(id, phase, dt_s)?;
            }
        }
        Ok(())
    }

    /// Produce the per-recovery state-only snapshot map the kernel consumes via
    /// `set_recovery_snapshot`. Keys are `RecoveryId`; values are
    /// the device's current phase, drag area, and drag coefficient.
    #[must_use]
    pub fn snapshot_map(&self) -> BTreeMap<RecoveryId, RecoverySnapshot> {
        let mut out = BTreeMap::new();
        for (id, device) in &self.devices {
            let mut snap = base_recovery_snapshot(device.as_ref());
            self.populate_sequence_diagnostics(*id, device.phase(), &mut snap);
            self.populate_riser_diagnostics(*id, device.phase(), &mut snap);
            out.insert(*id, snap);
        }
        out
    }

    /// Produce a snapshot map with inflation opening-load diagnostics
    /// evaluated at the supplied dynamic pressure.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] if the dynamic pressure or a
    /// profile diagnostic evaluation is invalid.
    pub fn snapshot_map_with_dynamic_pressure(
        &self,
        dynamic_pressure_pa: f64,
    ) -> Result<BTreeMap<RecoveryId, RecoverySnapshot>, RunnerError> {
        if !self.finite_mass_opening_loads.is_empty() {
            return Err(RunnerError::Recovery {
                field: "vehicle.assembly.recovery[*].*.pflanz".to_owned(),
                reason: "finite-mass opening-load telemetry requires density, speed, and system mass context".to_owned(),
            });
        }
        if !dynamic_pressure_pa.is_finite() || dynamic_pressure_pa < 0.0 {
            return Err(RunnerError::Recovery {
                field: "recovery.opening_dynamic_pressure_pa".to_owned(),
                reason: "dynamic pressure must be finite and non-negative".to_owned(),
            });
        }
        self.snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
            dynamic_pressure_pa,
            density_kg_m3: 1.0,
            air_relative_velocity_eci_m_s: Vector3::zeros(),
            body_to_eci: UnitQuaternion::identity(),
            line_stretch_speed_m_s: 1.0,
            system_mass_kg: 1.0,
        })
    }

    /// Produce a snapshot map with infinite-mass and optional Pflanz
    /// finite-mass opening-load diagnostics evaluated from the supplied
    /// base-tick context.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Recovery`] if the context or a profile/table
    /// diagnostic evaluation is invalid.
    pub(crate) fn snapshot_map_with_opening_load_context(
        &self,
        context: RecoveryOpeningLoadContext,
    ) -> Result<BTreeMap<RecoveryId, RecoverySnapshot>, RunnerError> {
        validate_opening_load_context(&context)?;
        let mut out = BTreeMap::new();
        for (id, device) in &self.devices {
            let mut snap = base_recovery_snapshot(device.as_ref());
            if let Some(sample) = device
                .current_inflation_sample(context.dynamic_pressure_pa)
                .map_err(|err| RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery.{id_value}.inflation",
                        id_value = id.value()
                    ),
                    reason: err.to_string(),
                })?
            {
                snap.inflation_progress = sample.progress;
                snap.inflation_complete = sample.complete;
                snap.opening_dynamic_pressure_pa = context.dynamic_pressure_pa;
                snap.opening_load_n = sample.instantaneous_infinite_mass_load_n;
                snap.opening_peak_load_n = sample.infinite_mass_peak_load_n;
                let phase = device.phase();
                if let Some(model) = self
                    .apparent_mass_models
                    .get(id)
                    .and_then(|models| models.active(phase))
                {
                    let apparent = model.evaluate(context, sample)?;
                    snap.apparent_mass_kg = apparent.apparent_mass_kg;
                    snap.apparent_mass_rate_kg_s = apparent.apparent_mass_rate_kg_s;
                }
                if let Some(model) = self
                    .finite_mass_opening_loads
                    .get(id)
                    .and_then(|models| models.active(phase))
                {
                    let finite = model.evaluate(context, sample.infinite_mass_peak_load_n)?;
                    snap.opening_ballistic_parameter_a = finite.ballistic_parameter_a;
                    snap.opening_pflanz_reduction_factor_x1 = finite.reduction_factor_x1;
                    snap.opening_finite_mass_peak_load_n = finite.finite_mass_peak_load_n;
                }
            }
            self.populate_sequence_diagnostics(*id, device.phase(), &mut snap);
            self.populate_riser_diagnostics(*id, device.phase(), &mut snap);
            self.populate_riser_drag_force_path(*id, device.phase(), &context, &mut snap)?;
            out.insert(*id, snap);
        }
        Ok(out)
    }

    fn populate_sequence_diagnostics(
        &self,
        id: RecoveryId,
        phase: RecoveryPhase,
        snap: &mut RecoverySnapshot,
    ) {
        if !phase.is_deployed() {
            return;
        }
        let Some(sequence) = self.sequence_diagnostics.get(&id) else {
            return;
        };
        snap.snatch_delta_v_m_s = sequence.snatch.delta_v_m_s;
        snap.snatch_stiffness_n_m = sequence.snatch.stiffness_n_m;
        snap.snatch_effective_mass_kg = sequence.snatch.effective_mass_kg;
        snap.snatch_pack_drag_load_n = sequence.snatch.pack_drag_load_n;
        snap.snatch_peak_load_n = sequence.snatch.peak_load_n;
        snap.mortar_impulse_body_n_s = sequence.active_mortar_impulse_body_n_s;
        snap.mortar_force_body_n = sequence.active_mortar_force_body_n;
    }

    fn populate_riser_diagnostics(
        &self,
        id: RecoveryId,
        phase: RecoveryPhase,
        snap: &mut RecoverySnapshot,
    ) {
        if !phase.is_deployed() {
            return;
        }
        let Some(aggregate) = self.risers.get(&id).map(|risers| &risers.aggregate) else {
            return;
        };
        snap.riser_tension_sum_n = aggregate.tension_sum_n;
        snap.riser_tension_max_n = aggregate.tension_max_n;
        snap.riser_force_body_n = aggregate.force_body_n;
        snap.riser_moment_body_n_m = aggregate.moment_body_n_m;
        snap.riser_elastic_energy_j = aggregate.elastic_energy_j;
    }

    fn populate_riser_drag_force_path(
        &self,
        id: RecoveryId,
        phase: RecoveryPhase,
        context: &RecoveryOpeningLoadContext,
        snap: &mut RecoverySnapshot,
    ) -> Result<(), RunnerError> {
        if !phase.is_deployed() {
            return Ok(());
        }
        let Some(aggregate) = self.risers.get(&id).map(|risers| &risers.aggregate) else {
            return Ok(());
        };
        let cd_area = snap.c_d * snap.drag_area_m2;
        if !cd_area.is_finite() || cd_area < 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.riser_force_path",
                    id_value = id.value()
                ),
                reason: "riser drag-force coupling requires finite non-negative C_D A".to_owned(),
            });
        }
        if cd_area <= 0.0 {
            return Ok(());
        }
        snap.riser_force_path_active = true;
        let force_body_n = riser_drag_force_body_n(context, cd_area)?;
        if force_body_n == Vector3::zeros() {
            return Ok(());
        }
        let drag_moment_body_n_m = riser_drag_moment_body_n_m(id, aggregate, force_body_n)?;
        snap.riser_force_body_n += force_body_n;
        snap.riser_moment_body_n_m += drag_moment_body_n_m;
        if !(snap.riser_force_body_n.x.is_finite()
            && snap.riser_force_body_n.y.is_finite()
            && snap.riser_force_body_n.z.is_finite()
            && snap.riser_moment_body_n_m.x.is_finite()
            && snap.riser_moment_body_n_m.y.is_finite()
            && snap.riser_moment_body_n_m.z.is_finite())
        {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.riser_force_path",
                    id_value = id.value()
                ),
                reason: "riser drag-force coupling produced non-finite force or moment".to_owned(),
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

type BuiltRecoveryDevice = (
    Box<dyn RecoveryModel>,
    Option<RecoveryStageModels<PflanzOpeningLoadModel>>,
    Option<RecoveryStageModels<CanopyApparentMassModel>>,
);

fn recovery_frame_alias<'a>(
    config: &RecoveryConfig,
    frame_aliases: &'a BTreeMap<BodyId, RecoveryFrameAlias>,
) -> Option<&'a RecoveryFrameAlias> {
    config
        .mounted_to
        .as_deref()
        .map(body_id_from_scenario_text)
        .and_then(|owner| frame_aliases.get(&owner))
}

fn transform_recovery_point(
    point_body_m: Vector3<f64>,
    alias: Option<&RecoveryFrameAlias>,
) -> Vector3<f64> {
    alias.map_or(point_body_m, |alias| {
        alias.translation_owner_from_body_m + alias.rot_owner_from_body * point_body_m
    })
}

fn transform_recovery_vector(
    vector_body: Vector3<f64>,
    alias: Option<&RecoveryFrameAlias>,
) -> Vector3<f64> {
    alias.map_or(vector_body, |alias| alias.rot_owner_from_body * vector_body)
}

fn body_id_from_scenario_text(id: &str) -> BodyId {
    BodyId::from_path(&format!("vehicle.assembly.bodies.{id}"))
}

fn recovery_id_from_config(config: &RecoveryConfig) -> RecoveryId {
    RecoveryId::from_path(&format!("vehicle.assembly.recovery.{id}", id = config.id))
}

fn recovery_sequence_diagnostics(
    config: &RecoveryConfig,
    risers: &[RecoveryRiserRuntime],
    frame_alias: Option<&RecoveryFrameAlias>,
) -> Result<Option<RecoverySequenceDiagnostics>, RunnerError> {
    let Some(sequence) = &config.sequence else {
        return Ok(None);
    };
    if risers.is_empty() {
        return Err(RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery.{id}.sequence", id = config.id),
            reason: "recovery sequence diagnostics require at least one declared recovery riser"
                .to_owned(),
        });
    }
    let stiffness_n_m = risers
        .iter()
        .map(|riser| riser.line.stiffness_n_m())
        .sum::<f64>();
    let snatch = recovery_snatch_diagnostic(config, sequence, stiffness_n_m)?;
    let mortar = recovery_mortar_impulse_diagnostic(config, sequence, frame_alias)?;
    Ok(Some(RecoverySequenceDiagnostics::new(snatch, mortar)))
}

fn recovery_snatch_diagnostic(
    recovery: &RecoveryConfig,
    config: &RecoverySequenceConfig,
    stiffness_n_m: f64,
) -> Result<RecoverySnatchDiagnostic, RunnerError> {
    if !stiffness_n_m.is_finite() || stiffness_n_m <= 0.0 {
        return Err(RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery.{id}.sequence", id = recovery.id),
            reason: "snatch-load diagnostic requires finite positive equivalent riser stiffness"
                .to_owned(),
        });
    }
    if !config.snatch_delta_v_m_s.is_finite() || config.snatch_delta_v_m_s < 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.sequence.snatch_delta_v_m_s",
                id = recovery.id
            ),
            reason: "snatch relative velocity must be finite and non-negative".to_owned(),
        });
    }
    if !config.snatch_effective_mass_kg.is_finite() || config.snatch_effective_mass_kg <= 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.sequence.snatch_effective_mass_kg",
                id = recovery.id
            ),
            reason: "snatch effective mass must be finite and positive".to_owned(),
        });
    }
    if !config.pack_drag_load_n.is_finite() || config.pack_drag_load_n < 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.sequence.pack_drag_load_n",
                id = recovery.id
            ),
            reason: "snatch pack drag load must be finite and non-negative".to_owned(),
        });
    }
    let inertial_scale_n_s_m = (stiffness_n_m * config.snatch_effective_mass_kg).sqrt();
    let peak_load_n = config.snatch_delta_v_m_s * inertial_scale_n_s_m + config.pack_drag_load_n;
    if !peak_load_n.is_finite() {
        return Err(RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery.{id}.sequence", id = recovery.id),
            reason: "snatch-load diagnostic produced a non-finite peak load".to_owned(),
        });
    }
    Ok(RecoverySnatchDiagnostic {
        delta_v_m_s: config.snatch_delta_v_m_s,
        stiffness_n_m,
        effective_mass_kg: config.snatch_effective_mass_kg,
        pack_drag_load_n: config.pack_drag_load_n,
        peak_load_n,
    })
}

fn recovery_mortar_impulse_diagnostic(
    recovery: &RecoveryConfig,
    config: &RecoverySequenceConfig,
    frame_alias: Option<&RecoveryFrameAlias>,
) -> Result<Option<RecoveryMortarImpulseDiagnostic>, RunnerError> {
    let (Some(pack_mass_kg), Some(pack_velocity_body_m_s)) = (
        config.mortar_pack_mass_kg,
        config.mortar_pack_velocity_body_m_s,
    ) else {
        return Ok(None);
    };
    if !pack_mass_kg.is_finite() || pack_mass_kg <= 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.sequence.mortar_pack_mass_kg",
                id = recovery.id
            ),
            reason: "mortar pack mass must be finite and positive".to_owned(),
        });
    }
    let pack_velocity_body_m_s =
        transform_recovery_vector(Vector3::from(pack_velocity_body_m_s), frame_alias);
    if !is_finite_vec3(pack_velocity_body_m_s) {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.sequence.mortar_pack_velocity_body_m_s",
                id = recovery.id
            ),
            reason: "mortar pack velocity must be finite".to_owned(),
        });
    }
    let impulse_body_n_s = -pack_mass_kg * pack_velocity_body_m_s;
    if !is_finite_vec3(impulse_body_n_s) {
        return Err(RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery.{id}.sequence", id = recovery.id),
            reason: "mortar impulse diagnostic produced a non-finite impulse".to_owned(),
        });
    }
    Ok(Some(RecoveryMortarImpulseDiagnostic {
        pack_mass_kg,
        pack_velocity_body_m_s,
        impulse_body_n_s,
    }))
}

fn recovery_riser_models(
    config: &RecoveryConfig,
    frame_alias: Option<&RecoveryFrameAlias>,
) -> Result<Vec<RecoveryRiserRuntime>, RunnerError> {
    config
        .risers
        .iter()
        .enumerate()
        .map(|(index, riser)| recovery_riser_runtime(config, index, riser, frame_alias))
        .collect()
}

fn recovery_riser_runtime(
    recovery: &RecoveryConfig,
    index: usize,
    config: &RecoveryRiserConfig,
    frame_alias: Option<&RecoveryFrameAlias>,
) -> Result<RecoveryRiserRuntime, RunnerError> {
    let line = RiserLineElement::new(
        config.unstretched_length_m,
        config.axial_rigidity_n,
        config.damping_n_s_m,
    )
    .map_err(|err| RunnerError::Recovery {
        field: format!(
            "vehicle.assembly.recovery.{id}.risers[{index}]",
            id = recovery.id,
        ),
        reason: err.to_string(),
    })?;
    Ok(RecoveryRiserRuntime {
        line,
        vehicle_attach_body_m: transform_recovery_point(
            Vector3::from(config.vehicle_attach_body_m),
            frame_alias,
        ),
        initial_canopy_attach_body_m: transform_recovery_point(
            Vector3::from(config.canopy_attach_body_m),
            frame_alias,
        ),
        initial_canopy_velocity_body_m_s: transform_recovery_vector(
            config
                .canopy_velocity_body_m_s
                .map(Vector3::from)
                .unwrap_or_else(Vector3::zeros),
            frame_alias,
        ),
        canopy_mass_kg: config.canopy_mass_kg,
    })
}

fn recovery_riser_canopy_runtime(
    config: &RecoveryConfig,
    frame_alias: Option<&RecoveryFrameAlias>,
) -> Option<RecoverySharedCanopyRuntime> {
    config
        .riser_canopy
        .as_ref()
        .map(|canopy| recovery_shared_canopy_runtime(canopy, frame_alias))
}

fn recovery_shared_canopy_runtime(
    config: &RecoveryRiserCanopyConfig,
    frame_alias: Option<&RecoveryFrameAlias>,
) -> RecoverySharedCanopyRuntime {
    RecoverySharedCanopyRuntime {
        initial_origin_body_m: transform_recovery_point(
            Vector3::from(config.initial_position_body_m),
            frame_alias,
        ),
        initial_velocity_body_m_s: transform_recovery_vector(
            config
                .initial_velocity_body_m_s
                .map(Vector3::from)
                .unwrap_or_else(Vector3::zeros),
            frame_alias,
        ),
        mass_kg: config.mass_kg,
    }
}

fn advance_force_integrated_riser_endpoint(
    id: RecoveryId,
    riser: &RecoveryRiserRuntime,
    canopy: RiserEndpointState,
    event: Option<RiserSlackTautEvent>,
    dt_s: f64,
    canopy_mass_kg: f64,
) -> Result<RiserEndpointState, RunnerError> {
    if !canopy_mass_kg.is_finite() || canopy_mass_kg <= 0.0 {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: "riser force-integrated endpoint mass must be finite and positive".to_owned(),
        });
    }
    let (anchor, canopy, active_dt_s) = if let Some(event) = event {
        let active_dt_s = dt_s - event.time_s;
        if !active_dt_s.is_finite() || active_dt_s < 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.risers",
                    id_value = id.value()
                ),
                reason: "riser slack-taut event produced an invalid integration interval"
                    .to_owned(),
            });
        }
        (event.anchor, event.canopy, active_dt_s)
    } else {
        return canopy
            .advance_by(dt_s)
            .map_err(|err| RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.risers",
                    id_value = id.value()
                ),
                reason: err.to_string(),
            });
    };

    if active_dt_s == 0.0 {
        return Ok(canopy);
    }

    let sample = riser
        .line
        .sample(anchor, canopy)
        .map_err(|err| RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: err.to_string(),
        })?;
    let acceleration_m_s2 = sample.force_on_canopy_n / canopy_mass_kg;
    let velocity_m_s = canopy.velocity_m_s + acceleration_m_s2 * active_dt_s;
    let position_m = canopy.position_m + velocity_m_s * active_dt_s;
    if !(is_finite_vec3(acceleration_m_s2)
        && is_finite_vec3(velocity_m_s)
        && is_finite_vec3(position_m))
    {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: "riser force-integrated endpoint produced a non-finite state".to_owned(),
        });
    }
    Ok(RiserEndpointState::new(position_m, velocity_m_s))
}

fn build_device(
    id: RecoveryId,
    config: &RecoveryConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BuiltRecoveryDevice, RunnerError> {
    let field = format!("vehicle.assembly.recovery.{id_text}", id_text = config.id);
    match config.kind {
        RecoveryKindConfig::ParachuteDrag {
            c_d,
            area_inflated_m2,
        } => {
            let inflation_runtime = config
                .inflation
                .as_ref()
                .map(|inflation| {
                    recovery_inflation_runtime(
                        inflation,
                        config,
                        resolved_files,
                        c_d,
                        area_inflated_m2,
                        "inflation",
                    )
                })
                .transpose()?;
            let finite_mass_opening_load = inflation_runtime
                .as_ref()
                .and_then(|runtime| runtime.finite_mass_opening_load.clone())
                .map(RecoveryStageModels::Single);
            let apparent_mass = inflation_runtime
                .as_ref()
                .and_then(|runtime| runtime.apparent_mass.clone())
                .map(RecoveryStageModels::Single);
            let device = if let Some(inflation) = &inflation_runtime {
                ParachuteDrag::new_with_inflation(id, c_d, area_inflated_m2, inflation.profile)
            } else {
                ParachuteDrag::new(id, c_d, area_inflated_m2)
            };
            device
                .map(|m| {
                    (
                        Box::new(m) as Box<dyn RecoveryModel>,
                        finite_mass_opening_load,
                        apparent_mass,
                    )
                })
                .map_err(|err| RunnerError::Recovery {
                    field,
                    reason: err.to_string(),
                })
        }
        RecoveryKindConfig::DrogueMain {
            drogue_c_d,
            drogue_area_m2,
            main_c_d,
            main_area_m2,
        } => {
            let drogue_runtime = config
                .drogue_inflation
                .as_ref()
                .map(|inflation| {
                    recovery_inflation_runtime(
                        inflation,
                        config,
                        resolved_files,
                        drogue_c_d,
                        drogue_area_m2,
                        "drogue_inflation",
                    )
                })
                .transpose()?;
            let main_runtime = config
                .main_inflation
                .as_ref()
                .map(|inflation| {
                    recovery_inflation_runtime(
                        inflation,
                        config,
                        resolved_files,
                        main_c_d,
                        main_area_m2,
                        "main_inflation",
                    )
                })
                .transpose()?;
            let finite_mass_opening_load = drogue_main_stage_models(
                drogue_runtime
                    .as_ref()
                    .and_then(|runtime| runtime.finite_mass_opening_load.clone()),
                main_runtime
                    .as_ref()
                    .and_then(|runtime| runtime.finite_mass_opening_load.clone()),
            );
            let apparent_mass = drogue_main_stage_models(
                drogue_runtime
                    .as_ref()
                    .and_then(|runtime| runtime.apparent_mass.clone()),
                main_runtime
                    .as_ref()
                    .and_then(|runtime| runtime.apparent_mass.clone()),
            );
            DrogueMainRecovery::new_with_inflation(
                id,
                drogue_c_d,
                drogue_area_m2,
                main_c_d,
                main_area_m2,
                drogue_runtime.as_ref().map(|runtime| runtime.profile),
                main_runtime.as_ref().map(|runtime| runtime.profile),
            )
            .map(|m| {
                (
                    Box::new(m) as Box<dyn RecoveryModel>,
                    finite_mass_opening_load,
                    apparent_mass,
                )
            })
            .map_err(|err| RunnerError::Recovery {
                field,
                reason: err.to_string(),
            })
        }
        RecoveryKindConfig::DragDevice {
            c_d,
            area_deployed_m2,
        } => DragDevice::new(id, c_d, area_deployed_m2)
            .map(|m| (Box::new(m) as Box<dyn RecoveryModel>, None, None))
            .map_err(|err| RunnerError::Recovery {
                field,
                reason: err.to_string(),
            }),
    }
}

fn drogue_main_stage_models<T>(
    drogue: Option<T>,
    main: Option<T>,
) -> Option<RecoveryStageModels<T>> {
    if drogue.is_some() || main.is_some() {
        Some(RecoveryStageModels::DrogueMain { drogue, main })
    } else {
        None
    }
}

fn recovery_inflation_runtime(
    config: &RecoveryInflationConfig,
    recovery_config: &RecoveryConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    full_c_d: f64,
    full_drag_area_m2: f64,
    inflation_table: &str,
) -> Result<RecoveryInflationRuntime, RunnerError> {
    let fill_time_s = recovery_inflation_fill_time_s(config, inflation_table)?;
    let growth = match config.growth {
        RecoveryInflationGrowthKindConfig::Polynomial => {
            CanopyGrowthShape::polynomial(config.exponent_j)
        }
        RecoveryInflationGrowthKindConfig::Exponential => {
            CanopyGrowthShape::exponential(config.alpha)
        }
    }
    .map_err(|err| RunnerError::Recovery {
        field: format!("vehicle.assembly.recovery[*].{inflation_table}"),
        reason: err.to_string(),
    })?;
    let profile = InflationProfile::new(
        fill_time_s,
        config.initial_drag_area_fraction,
        growth,
        config.opening_force_coeff_c_x,
    )
    .map_err(|err| RunnerError::Recovery {
        field: format!("vehicle.assembly.recovery[*].{inflation_table}"),
        reason: err.to_string(),
    })?;
    let finite_mass_opening_load = config
        .pflanz
        .as_ref()
        .map(|pflanz| {
            let table = recovery_pflanz_reduction_table(
                pflanz,
                recovery_config,
                resolved_files,
                inflation_table,
            )?;
            table
                .reduction_factor(f64::INFINITY, config.exponent_j)
                .map_err(|err| RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz.exponent_j",
                        id = recovery_config.id,
                    ),
                    reason: err.to_string(),
                })?;
            Ok::<PflanzOpeningLoadModel, RunnerError>(PflanzOpeningLoadModel {
                full_c_d,
                full_drag_area_m2,
                fill_time_s,
                exponent_j: config.exponent_j,
                table,
            })
        })
        .transpose()?;
    let apparent_mass = config
        .apparent_mass_coeff_k_a
        .map(|apparent_mass_coeff_k_a| {
            if !apparent_mass_coeff_k_a.is_finite() || apparent_mass_coeff_k_a <= 0.0 {
                return Err(RunnerError::Recovery {
                    field: format!(
                        "vehicle.assembly.recovery[*].{inflation_table}.apparent_mass_coeff_k_a"
                    ),
                    reason: "apparent-mass coefficient must be finite and strictly positive"
                        .to_owned(),
                });
            }
            Ok(CanopyApparentMassModel {
                apparent_mass_coeff_k_a,
            })
        })
        .transpose()?;
    Ok(RecoveryInflationRuntime {
        profile,
        finite_mass_opening_load,
        apparent_mass,
    })
}

fn recovery_pflanz_reduction_table(
    config: &RecoveryPflanzReductionConfig,
    recovery_config: &RecoveryConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    inflation_table: &str,
) -> Result<PflanzReductionTable, RunnerError> {
    if let Some(path) = &config.file {
        let key = format!(
            "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz.file",
            id = recovery_config.id,
        );
        let resolved = resolved_files
            .get(&key)
            .ok_or_else(|| RunnerError::Recovery {
                field: key.clone(),
                reason: format!("resolved Pflanz table `{path}` is missing"),
            })?;
        let text = std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::Recovery {
            field: key.clone(),
            reason: format!("Pflanz table file must be UTF-8 TOML: {err}"),
        })?;
        let deck: RecoveryPflanzDeckDocument =
            toml::from_str(text).map_err(|err| RunnerError::Recovery {
                field: key,
                reason: format!("invalid Pflanz table TOML: {err}"),
            })?;
        if deck.format != RECOVERY_PFLANZ_DECK_FORMAT {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz.format",
                    id = recovery_config.id,
                ),
                reason: format!(
                    "unsupported Pflanz table format `{}`, expected `{RECOVERY_PFLANZ_DECK_FORMAT}`",
                    deck.format
                ),
            });
        }
        if deck.provenance.trim().is_empty() {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz.provenance",
                    id = recovery_config.id,
                ),
                reason: "Pflanz table file requires nonempty provenance".to_owned(),
            });
        }
        return PflanzReductionTable::new(
            deck.ballistic_parameter_a,
            deck.exponent_j,
            deck.reduction_factors,
        )
        .map_err(|err| RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz",
                id = recovery_config.id,
            ),
            reason: err.to_string(),
        });
    }

    PflanzReductionTable::new(
        config.ballistic_parameter_a.clone(),
        config.exponent_j.clone(),
        config.reduction_factors.clone(),
    )
    .map_err(|err| RunnerError::Recovery {
        field: format!(
            "vehicle.assembly.recovery.{id}.{inflation_table}.pflanz",
            id = recovery_config.id,
        ),
        reason: err.to_string(),
    })
}

fn recovery_inflation_fill_time_s(
    config: &RecoveryInflationConfig,
    inflation_table: &str,
) -> Result<f64, RunnerError> {
    if let Some(fill_time_s) = config.fill_time_s {
        return Ok(fill_time_s);
    }
    let fill_constant_n = config
        .fill_constant_n
        .ok_or_else(|| RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery[*].{inflation_table}.fill_constant_n"),
            reason: "line-stretch fill-time derivation requires fill_constant_n".to_owned(),
        })?;
    let nominal_diameter_m = config
        .nominal_diameter_m
        .ok_or_else(|| RunnerError::Recovery {
            field: format!("vehicle.assembly.recovery[*].{inflation_table}.nominal_diameter_m"),
            reason: "line-stretch fill-time derivation requires nominal_diameter_m".to_owned(),
        })?;
    let line_stretch_speed_m_s =
        config
            .line_stretch_speed_m_s
            .ok_or_else(|| RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery[*].{inflation_table}.line_stretch_speed_m_s"
                ),
                reason: "line-stretch fill-time derivation requires line_stretch_speed_m_s"
                    .to_owned(),
            })?;
    canopy_fill_time_s(
        fill_constant_n,
        nominal_diameter_m,
        line_stretch_speed_m_s,
        config.fill_exponent_k,
    )
    .map_err(|err| RunnerError::Recovery {
        field: format!("vehicle.assembly.recovery[*].{inflation_table}"),
        reason: err.to_string(),
    })
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

fn base_recovery_snapshot(device: &dyn RecoveryModel) -> RecoverySnapshot {
    let phase = device.phase();
    RecoverySnapshot {
        deployed: phase.is_deployed(),
        phase_index: phase.index(),
        c_d: device.current_c_d(),
        drag_area_m2: device.current_drag_area_m2(),
        ..RecoverySnapshot::default()
    }
}

fn riser_drag_force_body_n(
    context: &RecoveryOpeningLoadContext,
    cd_area_m2: f64,
) -> Result<Vector3<f64>, RunnerError> {
    let v = context.air_relative_velocity_eci_m_s;
    let speed_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if speed_sq <= 0.0 {
        return Ok(Vector3::zeros());
    }
    let speed = speed_sq.sqrt();
    let drag_magnitude_n = context.dynamic_pressure_pa * cd_area_m2;
    let force_eci_n = -drag_magnitude_n * (v / speed);
    let force_body_n = context.body_to_eci.inverse() * force_eci_n;
    if !(force_body_n.x.is_finite() && force_body_n.y.is_finite() && force_body_n.z.is_finite()) {
        return Err(RunnerError::Recovery {
            field: "recovery.riser_force_path".to_owned(),
            reason: "riser drag-force coupling produced a non-finite body force".to_owned(),
        });
    }
    Ok(force_body_n)
}

fn riser_drag_moment_body_n_m(
    id: RecoveryId,
    aggregate: &RecoveryRiserAggregate,
    force_body_n: Vector3<f64>,
) -> Result<Vector3<f64>, RunnerError> {
    let mut moment_body_n_m = Vector3::zeros();
    for share in &aggregate.load_shares {
        if !share.share.is_finite() || share.share < 0.0 {
            return Err(RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.risers",
                    id_value = id.value()
                ),
                reason: "riser load-share weights must be finite and non-negative".to_owned(),
            });
        }
        let line_force_body_n = force_body_n * share.share;
        moment_body_n_m += share.attach_point_body_m.cross(&line_force_body_n);
    }
    if !(moment_body_n_m.x.is_finite()
        && moment_body_n_m.y.is_finite()
        && moment_body_n_m.z.is_finite())
    {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: "riser drag load-share moment produced a non-finite diagnostic".to_owned(),
        });
    }
    Ok(moment_body_n_m)
}

fn evaluate_declared_risers(
    id: RecoveryId,
    risers: &[RecoveryRiserRuntime],
    states: &[RecoveryRiserState],
) -> Result<RecoveryRiserAggregate, RunnerError> {
    if risers.len() != states.len() {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: "riser runtime state count must match declared riser count".to_owned(),
        });
    }
    let mut aggregate = RecoveryRiserAggregate {
        tension_sum_n: 0.0,
        tension_max_n: 0.0,
        force_body_n: Vector3::zeros(),
        moment_body_n_m: Vector3::zeros(),
        elastic_energy_j: 0.0,
        load_shares: Vec::new(),
    };
    let mut line_tensions: Vec<(Vector3<f64>, f64)> = Vec::with_capacity(risers.len());
    for (index, (riser, state)) in risers.iter().zip(states.iter()).enumerate() {
        let anchor = RiserEndpointState::new(riser.vehicle_attach_body_m, Vector3::zeros());
        let canopy =
            RiserEndpointState::new(state.canopy_attach_body_m, state.canopy_velocity_body_m_s);
        let sample = riser
            .line
            .sample(anchor, canopy)
            .map_err(|err| RunnerError::Recovery {
                field: format!(
                    "vehicle.assembly.recovery.{id_value}.risers[{index}]",
                    id_value = id.value()
                ),
                reason: err.to_string(),
            })?;
        aggregate.tension_sum_n += sample.tension_n;
        aggregate.tension_max_n = aggregate.tension_max_n.max(sample.tension_n);
        aggregate.force_body_n += sample.force_on_anchor_n;
        aggregate.moment_body_n_m +=
            riser_attach_moment_body_n_m(riser.vehicle_attach_body_m, sample.force_on_anchor_n);
        aggregate.elastic_energy_j += sample.elastic_potential_energy_j;
        line_tensions.push((riser.vehicle_attach_body_m, sample.tension_n));
    }
    if aggregate.tension_sum_n > 0.0 {
        for (attach_point_body_m, tension_n) in line_tensions {
            aggregate.load_shares.push(RecoveryRiserLoadShare {
                attach_point_body_m,
                share: tension_n / aggregate.tension_sum_n,
            });
        }
    } else if !risers.is_empty() {
        let equal_share = 1.0 / risers.len() as f64;
        for (attach_point_body_m, _) in line_tensions {
            aggregate.load_shares.push(RecoveryRiserLoadShare {
                attach_point_body_m,
                share: equal_share,
            });
        }
    }
    if !(aggregate.tension_sum_n.is_finite()
        && aggregate.tension_max_n.is_finite()
        && aggregate.force_body_n.x.is_finite()
        && aggregate.force_body_n.y.is_finite()
        && aggregate.force_body_n.z.is_finite()
        && aggregate.moment_body_n_m.x.is_finite()
        && aggregate.moment_body_n_m.y.is_finite()
        && aggregate.moment_body_n_m.z.is_finite()
        && aggregate.elastic_energy_j.is_finite()
        && aggregate.load_shares.iter().all(|share| {
            share.share.is_finite()
                && share.share >= 0.0
                && share.attach_point_body_m.x.is_finite()
                && share.attach_point_body_m.y.is_finite()
                && share.attach_point_body_m.z.is_finite()
        }))
    {
        return Err(RunnerError::Recovery {
            field: format!(
                "vehicle.assembly.recovery.{id_value}.risers",
                id_value = id.value()
            ),
            reason: "riser aggregate produced a non-finite diagnostic".to_owned(),
        });
    }
    Ok(aggregate)
}

fn is_finite_vec3(value: Vector3<f64>) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct FiniteMassOpeningLoadSample {
    ballistic_parameter_a: f64,
    reduction_factor_x1: f64,
    finite_mass_peak_load_n: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct CanopyApparentMassSample {
    apparent_mass_kg: f64,
    apparent_mass_rate_kg_s: f64,
}

impl CanopyApparentMassModel {
    fn evaluate(
        &self,
        context: RecoveryOpeningLoadContext,
        sample: openbmp_vehicle::CanopyInflationSample,
    ) -> Result<CanopyApparentMassSample, RunnerError> {
        let apparent_mass_kg = canopy_apparent_mass_kg(
            context.density_kg_m3,
            self.apparent_mass_coeff_k_a,
            sample.effective_drag_area_m2,
        )
        .map_err(|err| RunnerError::Recovery {
            field: "vehicle.assembly.recovery[*].inflation.apparent_mass_coeff_k_a".to_owned(),
            reason: err.to_string(),
        })?;
        let apparent_mass_rate_kg_s = canopy_apparent_mass_rate_kg_s(
            context.density_kg_m3,
            self.apparent_mass_coeff_k_a,
            sample.effective_drag_area_m2,
            sample.effective_drag_area_rate_m2_s,
        )
        .map_err(|err| RunnerError::Recovery {
            field: "vehicle.assembly.recovery[*].inflation.apparent_mass_coeff_k_a".to_owned(),
            reason: err.to_string(),
        })?;
        Ok(CanopyApparentMassSample {
            apparent_mass_kg,
            apparent_mass_rate_kg_s,
        })
    }
}

impl PflanzOpeningLoadModel {
    fn evaluate(
        &self,
        context: RecoveryOpeningLoadContext,
        infinite_mass_peak_load_n: f64,
    ) -> Result<FiniteMassOpeningLoadSample, RunnerError> {
        let ballistic_parameter_a = pflanz_ballistic_parameter_a(
            context.system_mass_kg,
            context.density_kg_m3,
            self.full_c_d,
            self.full_drag_area_m2,
            context.line_stretch_speed_m_s,
            self.fill_time_s,
        )
        .map_err(|err| RunnerError::Recovery {
            field: "vehicle.assembly.recovery[*].inflation.pflanz.ballistic_parameter_a".to_owned(),
            reason: err.to_string(),
        })?;
        let reduction_factor_x1 = self
            .table
            .reduction_factor(ballistic_parameter_a, self.exponent_j)
            .map_err(|err| RunnerError::Recovery {
                field: "vehicle.assembly.recovery[*].inflation.pflanz.reduction_factors".to_owned(),
                reason: err.to_string(),
            })?;
        let finite_mass_peak_load_n =
            pflanz_finite_mass_peak_load_n(infinite_mass_peak_load_n, reduction_factor_x1)
                .map_err(|err| RunnerError::Recovery {
                    field: "vehicle.assembly.recovery[*].inflation.pflanz".to_owned(),
                    reason: err.to_string(),
                })?;
        Ok(FiniteMassOpeningLoadSample {
            ballistic_parameter_a,
            reduction_factor_x1,
            finite_mass_peak_load_n,
        })
    }
}

fn validate_opening_load_context(context: &RecoveryOpeningLoadContext) -> Result<(), RunnerError> {
    if !context.dynamic_pressure_pa.is_finite() || context.dynamic_pressure_pa < 0.0 {
        return Err(RunnerError::Recovery {
            field: "recovery.opening_dynamic_pressure_pa".to_owned(),
            reason: "dynamic pressure must be finite and non-negative".to_owned(),
        });
    }
    if !context.density_kg_m3.is_finite() || context.density_kg_m3 < 0.0 {
        return Err(RunnerError::Recovery {
            field: "recovery.opening_density_kg_m3".to_owned(),
            reason: "density must be finite and non-negative".to_owned(),
        });
    }
    if !(context.air_relative_velocity_eci_m_s.x.is_finite()
        && context.air_relative_velocity_eci_m_s.y.is_finite()
        && context.air_relative_velocity_eci_m_s.z.is_finite())
    {
        return Err(RunnerError::Recovery {
            field: "recovery.air_relative_velocity_eci_m_s".to_owned(),
            reason: "air-relative velocity must be finite".to_owned(),
        });
    }
    let q = context.body_to_eci.into_inner();
    if !(q.coords.x.is_finite()
        && q.coords.y.is_finite()
        && q.coords.z.is_finite()
        && q.coords.w.is_finite())
    {
        return Err(RunnerError::Recovery {
            field: "recovery.body_to_eci".to_owned(),
            reason: "body-to-ECI attitude must be finite".to_owned(),
        });
    }
    if !context.line_stretch_speed_m_s.is_finite() || context.line_stretch_speed_m_s < 0.0 {
        return Err(RunnerError::Recovery {
            field: "recovery.opening_line_stretch_speed_m_s".to_owned(),
            reason: "line-stretch speed must be finite and non-negative".to_owned(),
        });
    }
    if !context.system_mass_kg.is_finite() || context.system_mass_kg <= 0.0 {
        return Err(RunnerError::Recovery {
            field: "recovery.opening_system_mass_kg".to_owned(),
            reason: "system mass must be finite and strictly positive".to_owned(),
        });
    }
    Ok(())
}

pub(crate) fn recovery_opening_load_context_with_mass(
    velocity_eci_m_s: Vector3<f64>,
    environment: &EnvironmentSample,
    system_mass_kg: f64,
) -> Result<RecoveryOpeningLoadContext, RunnerError> {
    recovery_opening_load_context_with_mass_and_attitude(
        velocity_eci_m_s,
        environment,
        system_mass_kg,
        UnitQuaternion::identity(),
    )
}

pub(crate) fn recovery_opening_load_context_with_mass_and_attitude(
    velocity_eci_m_s: Vector3<f64>,
    environment: &EnvironmentSample,
    system_mass_kg: f64,
    body_to_eci: UnitQuaternion<f64>,
) -> Result<RecoveryOpeningLoadContext, RunnerError> {
    let velocity = environment.air_relative_velocity_eci_m_s(velocity_eci_m_s);
    let speed_sq = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
    let dynamic_pressure_pa = 0.5 * environment.atmosphere_density_kg_m3 * speed_sq;
    let context = RecoveryOpeningLoadContext {
        dynamic_pressure_pa,
        density_kg_m3: environment.atmosphere_density_kg_m3,
        air_relative_velocity_eci_m_s: velocity,
        body_to_eci,
        line_stretch_speed_m_s: speed_sq.sqrt(),
        system_mass_kg,
    };
    validate_opening_load_context(&context)?;
    Ok(context)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use openbmp_core::{SimTime, StepIndex};
    use openbmp_scenario::Scenario;
    use openbmp_sim::EventId;

    use super::*;

    const RECOVERY_INFLATION_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/recovery-inflation-rack-test.toml"
    ));

    const RECOVERY_DROGUE_MAIN_INFLATION_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/recovery-drogue-main-inflation-rack-test.toml"
    ));

    #[test]
    fn recovery_rack_ramps_inflating_parachute_drag_area() {
        let scenario = Scenario::from_toml_str(RECOVERY_INFLATION_SCENARIO).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let initial = rack.snapshot_map();
        assert_eq!(
            initial.get(&id).unwrap().drag_area_m2.to_bits(),
            0.0_f64.to_bits()
        );

        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            0.0_f64.to_bits()
        );
        rack.step(0.25).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            2.0_f64.to_bits()
        );
        let loaded = rack.snapshot_map_with_dynamic_pressure(100.0).unwrap();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.inflation_progress.to_bits(), 0.25_f64.to_bits());
        assert_eq!(
            loaded.opening_dynamic_pressure_pa.to_bits(),
            100.0_f64.to_bits()
        );
        assert_eq!(loaded.opening_load_n.to_bits(), 360.0_f64.to_bits());
        assert_eq!(loaded.opening_peak_load_n.to_bits(), 1440.0_f64.to_bits());
        assert!(!loaded.inflation_complete);
        rack.step(1.0).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            8.0_f64.to_bits()
        );
    }

    #[test]
    fn recovery_rack_derives_inflation_fill_time_from_line_stretch_speed() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_constant_n = 2.0, nominal_diameter_m = 0.5, line_stretch_speed_m_s = 1.0, fill_exponent_k = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            2.0_f64.to_bits()
        );
        rack.step(0.75).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            8.0_f64.to_bits()
        );
    }

    #[test]
    fn recovery_rack_publishes_sequence_snatch_diagnostics_from_declared_risers() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nsequence = { snatch_delta_v_m_s = 3.0, snatch_effective_mass_kg = 5.0, pack_drag_load_n = 4.0 }\nrisers = [\n  { id = \"left\", unstretched_length_m = 2.0, axial_rigidity_n = 1000.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [-1.0, 0.0, 0.0], canopy_attach_body_m = [-1.0, 0.0, 0.0] },\n  { id = \"right\", unstretched_length_m = 4.0, axial_rigidity_n = 2000.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 0.0] }\n]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let stowed = rack.snapshot_map();
        assert_eq!(
            stowed.get(&id).unwrap().snatch_peak_load_n.to_bits(),
            0.0_f64.to_bits()
        );

        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.snatch_delta_v_m_s.to_bits(), 3.0_f64.to_bits());
        assert_eq!(loaded.snatch_stiffness_n_m.to_bits(), 1000.0_f64.to_bits());
        assert_eq!(loaded.snatch_effective_mass_kg.to_bits(), 5.0_f64.to_bits());
        assert_eq!(loaded.snatch_pack_drag_load_n.to_bits(), 4.0_f64.to_bits());
        let expected_peak = 3.0 * (1000.0_f64 * 5.0).sqrt() + 4.0;
        assert!((loaded.snatch_peak_load_n - expected_peak).abs() <= 1.0e-12);
    }

    #[test]
    fn recovery_rack_applies_sequence_mortar_impulse_for_one_tick() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nsequence = { snatch_delta_v_m_s = 3.0, snatch_effective_mass_kg = 5.0, mortar_pack_mass_kg = 2.0, mortar_pack_velocity_body_m_s = [0.0, 0.0, 6.0] }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 2.0, axial_rigidity_n = 1000.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 0.0, 0.0], canopy_attach_body_m = [0.0, 0.0, 0.0] }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();
        assert!(loaded.mortar_impulse_body_n_s.x.abs() <= 0.0);
        assert!(loaded.mortar_impulse_body_n_s.y.abs() <= 0.0);
        assert_eq!(
            loaded.mortar_impulse_body_n_s.z.to_bits(),
            (-12.0_f64).to_bits()
        );
        assert!(loaded.mortar_force_body_n.x.abs() <= 0.0);
        assert!(loaded.mortar_force_body_n.y.abs() <= 0.0);
        assert_eq!(
            loaded.mortar_force_body_n.z.to_bits(),
            (-48.0_f64).to_bits()
        );
        assert_eq!(
            (loaded.mortar_force_body_n.z * 0.25).to_bits(),
            loaded.mortar_impulse_body_n_s.z.to_bits()
        );

        rack.step(0.25).unwrap();
        let cleared = rack.snapshot_map();
        let cleared = cleared.get(&id).unwrap();
        assert_eq!(cleared.mortar_impulse_body_n_s, Vector3::<f64>::zeros());
        assert_eq!(cleared.mortar_force_body_n, Vector3::<f64>::zeros());
    }

    #[test]
    fn recovery_rack_applies_frame_alias_to_risers_and_sequence() {
        let toml = RECOVERY_INFLATION_SCENARIO
            .replace(
                "id = \"main_chute\"\n",
                "id = \"main_chute\"\nmounted_to = \"main\"\n",
            )
            .replace(
                "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
                "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nsequence = { snatch_delta_v_m_s = 3.0, snatch_effective_mass_kg = 5.0, mortar_pack_mass_kg = 2.0, mortar_pack_velocity_body_m_s = [1.0, 0.0, 0.0] }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 1.0] }]",
            );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let main = body_id_from_scenario_text("main");
        let aliases = BTreeMap::from([(
            main,
            RecoveryFrameAlias {
                rot_owner_from_body: Matrix3::new(
                    0.0, -1.0, 0.0, //
                    1.0, 0.0, 0.0, //
                    0.0, 0.0, 1.0,
                ),
                translation_owner_from_body_m: Vector3::new(2.0, 3.0, 0.0),
            },
        )]);
        let mut rack =
            RecoveryRack::build_with_frame_aliases(&scenario.document, &BTreeMap::new(), &aliases)
                .unwrap();
        let id = rack.recovery_ids()[0];
        let riser_set = rack.risers.get(&id).expect("riser runtime");
        assert_eq!(
            riser_set.lines[0].vehicle_attach_body_m,
            Vector3::new(2.0, 4.0, 0.0)
        );
        assert_eq!(
            riser_set.states[0].canopy_attach_body_m,
            Vector3::new(2.0, 4.0, 1.0)
        );

        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();

        assert_eq!(loaded.mortar_impulse_body_n_s, Vector3::new(0.0, -2.0, 0.0));
        assert_eq!(loaded.mortar_force_body_n, Vector3::new(0.0, -8.0, 0.0));
        assert!(loaded.riser_force_body_n.z > 0.0);
        assert!(loaded.riser_force_body_n.x.abs() <= 1.0e-12);
        assert!(loaded.riser_force_body_n.y.abs() <= 1.0e-12);
        assert!(
            (loaded.riser_moment_body_n_m.x - 4.0 * loaded.riser_force_body_n.z).abs() <= 1.0e-9
        );
        assert!(
            (loaded.riser_moment_body_n_m.y + 2.0 * loaded.riser_force_body_n.z).abs() <= 1.0e-9
        );
    }

    #[test]
    fn recovery_rack_publishes_pflanz_finite_mass_peak_load() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0, pflanz = { ballistic_parameter_a = [1.0, 3.0], exponent_j = [1.0], reduction_factors = [0.5, 0.75] } }",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.0,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -1.0),
                body_to_eci: UnitQuaternion::identity(),
                line_stretch_speed_m_s: 1.0,
                system_mass_kg: 18.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(
            loaded.opening_ballistic_parameter_a.to_bits(),
            3.0_f64.to_bits()
        );
        assert_eq!(
            loaded.opening_pflanz_reduction_factor_x1.to_bits(),
            0.75_f64.to_bits()
        );
        assert_eq!(loaded.opening_peak_load_n.to_bits(), 1440.0_f64.to_bits());
        assert_eq!(
            loaded.opening_finite_mass_peak_load_n.to_bits(),
            1080.0_f64.to_bits()
        );
    }

    #[test]
    fn recovery_rack_publishes_scalar_apparent_mass() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0, apparent_mass_coeff_k_a = 0.5 }",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.2,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -1.0),
                body_to_eci: UnitQuaternion::identity(),
                line_stretch_speed_m_s: 1.0,
                system_mass_kg: 18.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.drag_area_m2.to_bits(), 2.0_f64.to_bits());
        assert_eq!(
            loaded.apparent_mass_kg.to_bits(),
            openbmp_vehicle::canopy_apparent_mass_kg(1.2, 0.5, 2.0)
                .unwrap()
                .to_bits()
        );
        assert_eq!(
            loaded.apparent_mass_rate_kg_s.to_bits(),
            openbmp_vehicle::canopy_apparent_mass_rate_kg_s(1.2, 0.5, 2.0, 8.0)
                .unwrap()
                .to_bits()
        );
    }

    #[test]
    fn recovery_rack_publishes_declared_riser_diagnostics() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [1.0, 1.0, 0.0] }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];

        let stowed = rack.snapshot_map();
        let stowed = stowed.get(&id).unwrap();
        assert_eq!(stowed.riser_tension_sum_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(stowed.riser_moment_body_n_m.z.to_bits(), 0.0_f64.to_bits());

        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 200.0_f64.to_bits());
        assert_eq!(loaded.riser_tension_max_n.to_bits(), 200.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.x.to_bits(), 200.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_moment_body_n_m.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_moment_body_n_m.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            loaded.riser_moment_body_n_m.z.to_bits(),
            (-200.0_f64).to_bits()
        );
        assert_eq!(loaded.riser_elastic_energy_j.to_bits(), 50.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_advances_declared_riser_canopy_endpoint_each_step() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 1.0, axial_rigidity_n = 100.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 0.0, 0.0], canopy_attach_body_m = [0.0, 0.0, 0.5], canopy_velocity_body_m_s = [0.0, 0.0, 1.0] }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();

        rack.step(0.25).unwrap();
        let slack = rack.snapshot_map();
        let slack = slack.get(&id).unwrap();
        assert_eq!(slack.riser_tension_sum_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(slack.riser_elastic_energy_j.to_bits(), 0.0_f64.to_bits());

        rack.step(0.5).unwrap();
        let taut = rack.snapshot_map();
        let taut = taut.get(&id).unwrap();
        assert_eq!(taut.riser_tension_sum_n.to_bits(), 25.0_f64.to_bits());
        assert_eq!(taut.riser_tension_max_n.to_bits(), 25.0_f64.to_bits());
        assert_eq!(taut.riser_force_body_n.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(taut.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(taut.riser_force_body_n.z.to_bits(), 25.0_f64.to_bits());
        assert_eq!(taut.riser_moment_body_n_m, Vector3::zeros());
        assert_eq!(taut.riser_elastic_energy_j.to_bits(), 3.125_f64.to_bits());
    }

    #[test]
    fn recovery_rack_force_integrates_riser_after_slack_taut_event() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 1.0, axial_rigidity_n = 100.0, damping_n_s_m = 20.0, vehicle_attach_body_m = [0.0, 0.0, 0.0], canopy_attach_body_m = [0.0, 0.0, 0.5], canopy_velocity_body_m_s = [0.0, 0.0, 1.0], canopy_mass_kg = 10.0 }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        assert_eq!(
            scenario.document.vehicle.assembly.recovery[0].risers[0]
                .canopy_mass_kg
                .expect("declared endpoint mass parses")
                .to_bits(),
            10.0_f64.to_bits()
        );
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        assert_eq!(
            rack.risers.get(&id).expect("riser runtime is built").lines[0]
                .canopy_mass_kg
                .expect("endpoint mass stored in runtime")
                .to_bits(),
            10.0_f64.to_bits()
        );
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();

        rack.step(0.75).unwrap();
        let riser_state = &rack.risers.get(&id).expect("riser state exists").states[0];
        assert_eq!(
            riser_state
                .last_slack_taut_event_time_s
                .expect("slack-taut event located")
                .to_bits(),
            0.5_f64.to_bits()
        );
        assert_eq!(
            riser_state.canopy_attach_body_m.z.to_bits(),
            1.125_f64.to_bits()
        );
        assert_eq!(
            riser_state.canopy_velocity_body_m_s.z.to_bits(),
            0.5_f64.to_bits()
        );
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 22.5_f64.to_bits());
        assert_eq!(loaded.riser_tension_max_n.to_bits(), 22.5_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 22.5_f64.to_bits());
        assert_eq!(loaded.riser_moment_body_n_m, Vector3::zeros());
        assert_eq!(
            loaded.riser_elastic_energy_j.to_bits(),
            0.78125_f64.to_bits()
        );
    }

    #[test]
    fn recovery_rack_force_integrates_shared_riser_canopy_from_summed_line_loads() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nriser_canopy = { initial_position_body_m = [0.0, 0.0, 0.5], initial_velocity_body_m_s = [0.0, 0.0, 1.0], mass_kg = 10.0 }\nrisers = [\n  { id = \"left\", unstretched_length_m = 1.0, axial_rigidity_n = 100.0, damping_n_s_m = 10.0, vehicle_attach_body_m = [-1.0, 0.0, 0.0], canopy_attach_body_m = [-1.0, 0.0, 0.0] },\n  { id = \"right\", unstretched_length_m = 1.0, axial_rigidity_n = 100.0, damping_n_s_m = 10.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 0.0] }\n]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        assert_eq!(
            scenario.document.vehicle.assembly.recovery[0]
                .riser_canopy
                .as_ref()
                .expect("shared riser canopy parses")
                .mass_kg
                .to_bits(),
            10.0_f64.to_bits()
        );
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();

        rack.step(0.75).unwrap();
        let riser_set = rack.risers.get(&id).expect("riser state exists");
        let shared_canopy = riser_set
            .shared_canopy
            .as_ref()
            .expect("shared canopy runtime exists");
        assert_eq!(
            shared_canopy.state.origin_body_m.z.to_bits(),
            1.125_f64.to_bits()
        );
        assert_eq!(
            shared_canopy.state.velocity_body_m_s.z.to_bits(),
            0.5_f64.to_bits()
        );
        for state in &riser_set.states {
            assert_eq!(
                state
                    .last_slack_taut_event_time_s
                    .expect("each line locates slack-taut event")
                    .to_bits(),
                0.5_f64.to_bits()
            );
            assert_eq!(state.canopy_attach_body_m.z.to_bits(), 1.125_f64.to_bits());
            assert_eq!(
                state.canopy_velocity_body_m_s.z.to_bits(),
                0.5_f64.to_bits()
            );
        }
        let loaded = rack.snapshot_map();
        let loaded = loaded.get(&id).unwrap();
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 35.0_f64.to_bits());
        assert_eq!(loaded.riser_tension_max_n.to_bits(), 17.5_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 35.0_f64.to_bits());
        assert_eq!(loaded.riser_moment_body_n_m, Vector3::zeros());
        assert_eq!(
            loaded.riser_elastic_energy_j.to_bits(),
            1.5625_f64.to_bits()
        );
    }

    #[test]
    fn recovery_rack_transmits_drag_load_through_declared_risers() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 1.0] }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.0,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -10.0),
                body_to_eci: UnitQuaternion::identity(),
                line_stretch_speed_m_s: 10.0,
                system_mass_kg: 20.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();
        assert!(loaded.riser_force_path_active);
        assert_eq!(loaded.drag_area_m2.to_bits(), 2.0_f64.to_bits());
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 200.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 500.0_f64.to_bits());
        assert_eq!(loaded.riser_moment_body_n_m.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            loaded.riser_moment_body_n_m.y.to_bits(),
            (-500.0_f64).to_bits()
        );
        assert_eq!(loaded.riser_moment_body_n_m.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_distributes_drag_handoff_by_declared_riser_tension() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [\n  { id = \"left\", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 1.0] },\n  { id = \"right\", unstretched_length_m = 0.5, axial_rigidity_n = 600.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 1.0, 0.0], canopy_attach_body_m = [0.0, 1.0, 1.0] }\n]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.0,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -10.0),
                body_to_eci: UnitQuaternion::identity(),
                line_stretch_speed_m_s: 10.0,
                system_mass_kg: 20.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();

        assert!(loaded.riser_force_path_active);
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 800.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 1100.0_f64.to_bits());
        assert_eq!(
            loaded.riser_moment_body_n_m.x.to_bits(),
            825.0_f64.to_bits()
        );
        assert_eq!(
            loaded.riser_moment_body_n_m.y.to_bits(),
            (-275.0_f64).to_bits()
        );
        assert_eq!(loaded.riser_moment_body_n_m.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_equal_splits_drag_handoff_when_declared_risers_are_slack() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [\n  { id = \"left\", unstretched_length_m = 2.0, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [1.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 1.0] },\n  { id = \"right\", unstretched_length_m = 2.0, axial_rigidity_n = 600.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 2.0, 0.0], canopy_attach_body_m = [0.0, 2.0, 1.0] }\n]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.0,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -10.0),
                body_to_eci: UnitQuaternion::identity(),
                line_stretch_speed_m_s: 10.0,
                system_mass_kg: 20.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();

        assert!(loaded.riser_force_path_active);
        assert_eq!(loaded.riser_tension_sum_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_tension_max_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_elastic_energy_j.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(loaded.riser_force_body_n.z.to_bits(), 300.0_f64.to_bits());
        assert_eq!(
            loaded.riser_moment_body_n_m.x.to_bits(),
            300.0_f64.to_bits()
        );
        assert_eq!(
            loaded.riser_moment_body_n_m.y.to_bits(),
            (-150.0_f64).to_bits()
        );
        assert_eq!(loaded.riser_moment_body_n_m.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn recovery_rack_expresses_riser_drag_handoff_in_live_body_frame() {
        let toml = RECOVERY_INFLATION_SCENARIO.replace(
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }",
            "inflation = { fill_time_s = 1.0, opening_force_coeff_c_x = 1.2, growth = \"polynomial\", exponent_j = 1.0 }\nrisers = [{ id = \"main_riser\", unstretched_length_m = 0.5, axial_rigidity_n = 200.0, damping_n_s_m = 0.0, vehicle_attach_body_m = [0.0, 0.0, 0.0], canopy_attach_body_m = [1.0, 0.0, 0.0] }]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy".to_owned(),
            },
        };
        rack.apply_deploys(&[event]).unwrap();
        rack.step(0.25).unwrap();
        let body_to_eci = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            core::f64::consts::FRAC_PI_2,
        );

        let loaded = rack
            .snapshot_map_with_opening_load_context(RecoveryOpeningLoadContext {
                dynamic_pressure_pa: 100.0,
                density_kg_m3: 1.0,
                air_relative_velocity_eci_m_s: Vector3::new(0.0, -10.0, 0.0),
                body_to_eci,
                line_stretch_speed_m_s: 10.0,
                system_mass_kg: 20.0,
            })
            .unwrap();
        let loaded = loaded.get(&id).unwrap();
        assert!(loaded.riser_force_path_active);
        assert!((loaded.riser_force_body_n.x - 500.0).abs() <= 1.0e-12);
        assert!(loaded.riser_force_body_n.y.abs() <= 1.0e-12);
        assert!(loaded.riser_force_body_n.z.abs() <= 1.0e-12);
    }

    #[test]
    fn recovery_rack_uses_active_stage_inflation_models_for_drogue_main() {
        let scenario = Scenario::from_toml_str(RECOVERY_DROGUE_MAIN_INFLATION_SCENARIO).unwrap();
        let mut rack = RecoveryRack::build(&scenario.document, &BTreeMap::new()).unwrap();
        let id = rack.recovery_ids()[0];
        let drogue_event = FiredEvent {
            binding_id: EventId::new(1),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy_drogue".to_owned(),
            },
        };
        rack.apply_deploys(&[drogue_event]).unwrap();
        rack.step(0.25).unwrap();

        let context = RecoveryOpeningLoadContext {
            dynamic_pressure_pa: 100.0,
            density_kg_m3: 1.0,
            air_relative_velocity_eci_m_s: Vector3::new(0.0, 0.0, -10.0),
            body_to_eci: UnitQuaternion::identity(),
            line_stretch_speed_m_s: 10.0,
            system_mass_kg: 20.0,
        };
        let drogue = rack
            .snapshot_map_with_opening_load_context(context)
            .unwrap();
        let drogue = drogue.get(&id).unwrap();
        assert_eq!(drogue.phase_index, RecoveryPhase::Drogue.index());
        assert_eq!(drogue.drag_area_m2.to_bits(), 0.5_f64.to_bits());
        assert_eq!(drogue.inflation_progress.to_bits(), 0.25_f64.to_bits());
        assert!((drogue.opening_load_n - 55.0).abs() < 1e-12);
        assert!((drogue.opening_peak_load_n - 220.0).abs() < 1e-12);
        assert_eq!(
            drogue.opening_pflanz_reduction_factor_x1.to_bits(),
            0.5_f64.to_bits()
        );
        assert!((drogue.opening_finite_mass_peak_load_n - 110.0).abs() < 1e-12);
        assert_eq!(
            drogue.apparent_mass_kg.to_bits(),
            openbmp_vehicle::canopy_apparent_mass_kg(1.0, 0.5, 0.5)
                .unwrap()
                .to_bits()
        );

        let main_event = FiredEvent {
            binding_id: EventId::new(2),
            step: StepIndex::ZERO,
            time: SimTime::ZERO,
            action: ScenarioScriptAction::DeployRecovery {
                id,
                command: "deploy_main".to_owned(),
            },
        };
        rack.apply_deploys(&[main_event]).unwrap();
        assert_eq!(
            rack.snapshot_map().get(&id).unwrap().drag_area_m2.to_bits(),
            0.0_f64.to_bits()
        );
        rack.step(0.5).unwrap();
        let main = rack
            .snapshot_map_with_opening_load_context(context)
            .unwrap();
        let main = main.get(&id).unwrap();
        assert_eq!(main.phase_index, RecoveryPhase::Main.index());
        assert_eq!(main.drag_area_m2.to_bits(), 1.5_f64.to_bits());
        assert_eq!(main.inflation_progress.to_bits(), 0.25_f64.to_bits());
        assert!((main.opening_load_n - 270.0).abs() < 1e-12);
        assert!((main.opening_peak_load_n - 1080.0).abs() < 1e-12);
        assert_eq!(
            main.opening_pflanz_reduction_factor_x1.to_bits(),
            0.8_f64.to_bits()
        );
        assert!((main.opening_finite_mass_peak_load_n - 864.0).abs() < 1e-12);
        assert_eq!(
            main.apparent_mass_kg.to_bits(),
            openbmp_vehicle::canopy_apparent_mass_kg(1.0, 0.7, 1.5)
                .unwrap()
                .to_bits()
        );
    }
}
