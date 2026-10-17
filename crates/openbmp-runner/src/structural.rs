//! Structural modal rack — the runner-side stateful home for the launch
//! vehicle's lateral bending modes, mirroring the wind / tank
//! racks. It owns the modal state, advances it once per base tick with a
//! one-step-lag body-acceleration driver (the same bit-stable discipline the
//! slosh tank rack documents), and exposes the rate-gyro and accelerometer
//! pickups the FC bridge adds to sensed truth.
//!
//! When `[vehicle.bending]` / `[vehicle.modes]` are absent the rack is
//! [`StructuralRack::Inactive`] and every consumer short-circuits — the run is
//! byte-identical to a perfectly rigid vehicle.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use nalgebra::Vector3;
use openbmp_acoustics::response::ForcePsdSpectrum;
use openbmp_core::{ChannelId, Duration};
use openbmp_modelio::{ModalAsset, ModalModeAsset};
use openbmp_scenario::document::{
    BendingConfig, LoadsEventForceConfig, LoadsEventsConfig, LoadsRecoveryConfig,
    LoadsRecoveryOutputIdConfig, VehicleModeConfig, VehicleTransientConfig,
    VehicleTransientSchemeConfig,
};
use openbmp_scenario::{
    AcousticsResponseSourceConfig, ResolvedFile, ScenarioDocument, VehicleModesAssetConfig,
    VehicleModesAssetFormatConfig,
};
use openbmp_structdyn::events::{EventForceProfile, EventForcing, modal_force_envelope};
use openbmp_structdyn::transient::{
    GeneralizedAlphaIntegrator, GeneralizedAlphaParameters, ModalTransientState,
    ModalTransientSystem, NewmarkIntegrator, NewmarkParameters,
};
use openbmp_structdyn::{
    DenseMatrix, LineLoadRecovery, ModalRecoveryState, OutputId, RecoveryRow, RecoveryStation,
    SectionLoad,
};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use openbmp_uq::{
    CorrelatedErrorBudget, CredibilityLevel, UncertaintySource, structural_bending_margin_source,
};
use openbmp_vehicle::structural::{ModalBlock, ModalState, ModeParams};

use crate::error::RunnerError;

const VEHICLE_MODES_ASSET_FILE_KEY: &str = "vehicle.modes.asset.file";
const DEFAULT_MODAL_ASSET_FREQUENCY_TOLERANCE_FRACTION: f64 = 0.01;
const DEFAULT_MODAL_ASSET_MAC_MIN: f64 = 0.9;

/// Out-of-band structural consumption of an acoustic modal-force PSD handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct StructuralAcousticLoadsReport {
    /// Acoustic response station feeding the structural modal response.
    pub station_id: String,
    /// Acoustic source used by the response transfer.
    pub source: AcousticsResponseSourceConfig,
    /// Vibroacoustic response class that generated the force PSD.
    pub response_class_id: String,
    /// Structural bending natural frequency in Hz.
    pub mode_frequency_hz: f64,
    /// Structural bending damping ratio.
    pub mode_damping_ratio: f64,
    /// Structural bending modal mass in kg.
    pub modal_mass_kg: f64,
    /// Input acoustic modal-force mean-square value, `N^2`.
    pub total_input_mean_square_force_n2: f64,
    /// SDOF modal displacement mean-square response, `m^2`.
    pub total_modal_displacement_mean_square_m2: f64,
    /// SDOF modal acceleration mean-square response, `(m/s^2)^2`.
    pub total_modal_acceleration_mean_square_m2_s4: f64,
    /// Center frequency of the largest modal-acceleration contribution.
    pub peak_acceleration_band_center_hz: f64,
    /// Total equivalent engine-station reaction-moment mean-square, `N^2 m^2`.
    pub total_equivalent_engine_reaction_moment_mean_square_n2_m2: f64,
    /// RMS equivalent engine-station reaction moment, `N m`.
    pub equivalent_engine_reaction_moment_rms_n_m: f64,
    /// Deterministic per-band structural response table CSV bytes.
    pub band_response_csv: Vec<u8>,
    /// Deterministic first-increment station line-load recovery CSV bytes.
    pub line_load_recovery_csv: Vec<u8>,
}

/// Out-of-band deterministic modal force envelopes for declared CLA events.
#[derive(Clone, Debug, PartialEq)]
pub struct StructuralEventLoadsReport {
    /// Sample times used to build every event envelope.
    pub sample_times_s: Vec<f64>,
    /// One max-absolute modal force envelope per declared event.
    pub envelopes: Vec<StructuralEventModalEnvelope>,
    /// Deterministic event/modal envelope table CSV bytes.
    pub envelope_csv: Vec<u8>,
}

/// One declared event's max-absolute modal force envelope.
#[derive(Clone, Debug, PartialEq)]
pub struct StructuralEventModalEnvelope {
    /// Event id.
    pub event_id: String,
    /// Optional closed CLA flight-event class.
    pub flight_event_kind: Option<String>,
    /// Stable event force profile kind.
    pub kind: String,
    /// Max absolute modal generalized force per structural modal coordinate.
    pub max_abs_modal_force_n: Vec<f64>,
}

/// Stateful modal container advanced once per base tick.
#[derive(Clone, Debug)]
pub enum StructuralRack {
    /// No structural mode block declared — perfectly rigid (default).
    Inactive,
    /// Active declaration-ordered structural modes.
    Active(Box<ActiveModes>),
}

/// State for active structural modes plus their one-step-lag driver.
#[derive(Clone, Debug)]
pub struct ActiveModes {
    modes: ModalBlock,
    transient: StructuralTransientIntegrator,
    previous_modal_force: Vec<f64>,
    previous_modal_stiffness_scale_1: Vec<f64>,
    reaction_enabled: bool,
    dt_s: f64,
    /// Body-frame lateral specific force from the previous step; the mode is
    /// advanced against this (one-step lag, matching the slosh rack).
    driver_accel_body: Vector3<f64>,
    /// Gyro pickup computed after the most recent `step`, read by the bridge
    /// at the top of the next tick (contemporaneous with the body rate the
    /// FC also reads from `current_state`).
    pickup_rad_s: Vector3<f64>,
    accel_pickup_m_s2: Vector3<f64>,
    reaction_moment_body_n_m: Vector3<f64>,
    upstream_uq_sources: Vec<UncertaintySource>,
    line_load_recovery: Option<LineLoadRecovery>,
    line_loads: Vec<SectionLoad>,
    event_forcings: Vec<StructuralEventRuntimeForce>,
}

#[derive(Clone, Debug, PartialEq)]
struct StructuralEventRuntimeForce {
    forcing: EventForcing,
    mode_shapes: DenseMatrix,
    modal_stiffness_scale_1: Vec<f64>,
}

#[derive(Clone, Debug)]
enum StructuralTransientIntegrator {
    LegacySymplecticEuler,
    Newmark(NewmarkIntegrator),
    GeneralizedAlpha(GeneralizedAlphaIntegrator),
}

impl StructuralRack {
    /// Build from the scenario. Returns [`StructuralRack::Inactive`] when no
    /// structural mode block is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the bending parameters are invalid.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        Self::build_with_resolved_files(document, &BTreeMap::new())
    }

    /// Build from the scenario plus its pre-verified referenced files. Returns
    /// [`StructuralRack::Inactive`] when no structural mode block is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when bending parameters are invalid or an
    /// external modal asset is missing / malformed.
    pub fn build_with_resolved_files(
        document: &ScenarioDocument,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Self, RunnerError> {
        let (modes, upstream_uq_sources) = if let Some(cfg) = document.vehicle.modes.as_ref() {
            let modes = if let Some(asset) = &cfg.asset {
                match asset.format {
                    VehicleModesAssetFormatConfig::CsvV1 => {
                        mode_params_from_modal_asset(asset, resolved_files)?
                    }
                }
            } else {
                cfg.lateral
                    .iter()
                    .map(mode_params_from_config)
                    .collect::<Result<Vec<_>, _>>()?
            };
            (
                ModalBlock::new(modes).map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("[vehicle.modes] invalid: {err}"),
                })?,
                Vec::new(),
            )
        } else if let Some(cfg) = document.vehicle.bending.as_ref() {
            let upstream_uq_sources = bending_uq_sources(cfg)?;
            let omega_b = core::f64::consts::TAU * cfg.frequency_hz;
            (
                ModalBlock::from_legacy_bending(
                    omega_b,
                    cfg.damping_ratio,
                    cfg.modal_mass_kg,
                    cfg.slope_at_engine,
                    cfg.slope_at_gyro,
                )
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("[vehicle.bending] invalid: {err}"),
                })?,
                upstream_uq_sources,
            )
        } else {
            return Ok(Self::Inactive);
        };
        let transient = transient_integrator_from_config(
            document.vehicle.transient.as_ref(),
            &modes,
            document.time.dt_s,
        )?;
        let line_load_recovery = document
            .loads
            .as_ref()
            .and_then(|loads| loads.recovery.as_ref())
            .map(|recovery| line_load_recovery_from_config(recovery, modes.len() * 2))
            .transpose()?;
        let line_loads = evaluate_line_loads(&modes, line_load_recovery.as_ref())?;
        let event_forcings = document
            .loads
            .as_ref()
            .and_then(|loads| loads.events.as_ref())
            .map(|events| event_runtime_forces_from_config(events, modes.len() * 2))
            .transpose()?
            .unwrap_or_default();
        let coordinate_count = modes.len() * 2;
        let previous_modal_force = event_modal_force(&event_forcings, 0.0, coordinate_count)?;
        let previous_modal_stiffness_scale_1 =
            event_modal_stiffness_scale(&event_forcings, 0.0, coordinate_count)?;
        Ok(Self::Active(Box::new(ActiveModes {
            modes,
            transient,
            previous_modal_force,
            previous_modal_stiffness_scale_1,
            reaction_enabled: document
                .vehicle
                .reaction
                .as_ref()
                .is_some_and(|reaction| reaction.enabled),
            dt_s: document.time.dt_s,
            driver_accel_body: Vector3::zeros(),
            pickup_rad_s: Vector3::zeros(),
            accel_pickup_m_s2: Vector3::zeros(),
            reaction_moment_body_n_m: Vector3::zeros(),
            upstream_uq_sources,
            line_load_recovery,
            line_loads,
            event_forcings,
        })))
    }

    /// Whether the rack carries no active mode (every consumer short-circuits).
    #[must_use]
    pub fn is_inactive(&self) -> bool {
        matches!(self, Self::Inactive)
    }

    /// Reset modal state to rest (called at scenario start for replay
    /// determinism, mirroring the wind rack).
    pub fn reset(&mut self) {
        if let Self::Active(a) = self {
            a.modes.reset();
            a.driver_accel_body = Vector3::zeros();
            a.pickup_rad_s = Vector3::zeros();
            a.accel_pickup_m_s2 = Vector3::zeros();
            a.reaction_moment_body_n_m = Vector3::zeros();
            a.previous_modal_force = event_modal_force(&a.event_forcings, 0.0, a.modes.len() * 2)
                .unwrap_or_else(|_| vec![0.0; a.modes.len() * 2]);
            a.previous_modal_stiffness_scale_1 =
                event_modal_stiffness_scale(&a.event_forcings, 0.0, a.modes.len() * 2)
                    .unwrap_or_else(|_| vec![1.0; a.modes.len() * 2]);
            for load in &mut a.line_loads {
                load.value = 0.0;
            }
        }
    }

    /// Advance the modes one base tick against the prior-step driver, then
    /// refresh the sensor pickups and the opt-in reaction moment.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] on a non-finite modal step.
    pub fn step(&mut self) -> Result<(), RunnerError> {
        self.step_at(0.0)
    }

    /// Advance the modes one base tick while evaluating declared event forces
    /// at `time_s`.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] on a non-finite modal step or event projection.
    pub fn step_at(&mut self, time_s: f64) -> Result<(), RunnerError> {
        if let Self::Active(a) = self {
            match &a.transient {
                StructuralTransientIntegrator::LegacySymplecticEuler => {
                    let event_force =
                        event_modal_force(&a.event_forcings, time_s, a.modes.len() * 2)?;
                    let stiffness_scale =
                        event_modal_stiffness_scale(&a.event_forcings, time_s, a.modes.len() * 2)?;
                    a.modes
                        .step_with_modal_force_and_stiffness_scale(
                            a.driver_accel_body,
                            &event_force,
                            &stiffness_scale,
                            Duration::from_seconds(a.dt_s),
                        )
                        .map_err(|err| RunnerError::UnsupportedScenario {
                            what: format!("structural modal step failed: {err}"),
                        })?;
                    a.previous_modal_force = modal_force_from_driver(&a.modes, a.driver_accel_body);
                    add_event_modal_force(&mut a.previous_modal_force, &a.event_forcings, time_s)?;
                    a.previous_modal_stiffness_scale_1 = stiffness_scale;
                }
                StructuralTransientIntegrator::Newmark(_)
                | StructuralTransientIntegrator::GeneralizedAlpha(_) => {
                    step_native_transient(a, time_s)?;
                }
            }
            a.pickup_rad_s = a.modes.gyro_pickup_rad_s();
            a.accel_pickup_m_s2 = a.modes.accel_pickup_m_s2();
            a.reaction_moment_body_n_m = if a.reaction_enabled {
                a.modes.reaction_moment_body_n_m()
            } else {
                Vector3::zeros()
            };
            a.line_loads = evaluate_line_loads(&a.modes, a.line_load_recovery.as_ref())?;
        }
        Ok(())
    }

    /// Store the post-step body-frame lateral specific force for the next
    /// tick's advance (one-step lag).
    pub fn update_drivers(&mut self, accel_body_m_s2: Vector3<f64>) {
        if let Self::Active(a) = self {
            a.driver_accel_body = accel_body_m_s2;
        }
    }

    /// Rate-gyro pickup from the bending slope rate, added to the sensed body
    /// rate by the FC bridge. Zero when inactive.
    #[must_use]
    pub fn gyro_pickup_rad_s(&self) -> Vector3<f64> {
        match self {
            Self::Inactive => Vector3::zeros(),
            Self::Active(a) => a.pickup_rad_s,
        }
    }

    /// Accelerometer pickup from structural modal acceleration, added to the
    /// sensed body-frame specific force by the FC bridge. Zero when inactive.
    #[must_use]
    pub fn accel_pickup_m_s2(&self) -> Vector3<f64> {
        match self {
            Self::Inactive => Vector3::zeros(),
            Self::Active(a) => a.accel_pickup_m_s2,
        }
    }

    /// Reaction moment the bending mode exerts on the rigid body (body
    /// frame). Zero when inactive or when `[vehicle.reaction] enabled = true`
    /// is not declared.
    #[must_use]
    pub fn reaction_moment_body_n_m(&self) -> Vector3<f64> {
        match self {
            Self::Inactive => Vector3::zeros(),
            Self::Active(a) => a.reaction_moment_body_n_m,
        }
    }

    /// Source-tagged upstream UQ sources declared by the bending-mode deck.
    #[must_use]
    pub fn upstream_uq_sources(&self) -> &[UncertaintySource] {
        match self {
            Self::Inactive => &[],
            Self::Active(a) => &a.upstream_uq_sources,
        }
    }

    /// Gathered upstream UQ budget for the structural rack.
    #[must_use]
    pub fn upstream_uq_budget(&self) -> CorrelatedErrorBudget {
        CorrelatedErrorBudget {
            sources: self.upstream_uq_sources().to_vec(),
            correlation: None,
        }
    }

    /// Latest declaration-ordered section loads. Empty when recovery is absent.
    #[must_use]
    pub fn line_loads(&self) -> &[SectionLoad] {
        match self {
            Self::Inactive => &[],
            Self::Active(a) => &a.line_loads,
        }
    }

    /// Declaration-ordered structural mode parameters. Empty when inactive.
    #[must_use]
    pub fn mode_params(&self) -> &[ModeParams] {
        match self {
            Self::Inactive => &[],
            Self::Active(a) => a.modes.modes(),
        }
    }

    /// Number of structural modal coordinates in locked runner order:
    /// `[mode0.q_x, mode0.q_y, mode1.q_x, mode1.q_y, ...]`.
    #[must_use]
    pub fn modal_coordinate_count(&self) -> usize {
        match self {
            Self::Inactive => 0,
            Self::Active(a) => a.modes.len() * 2,
        }
    }
}

/// Telemetry channels for structural line-load recovery outputs.
#[derive(Clone, Debug)]
pub struct StructuralLineLoadTelemetryChannels {
    channels: Vec<StructuralLineLoadTelemetryChannel>,
}

#[derive(Clone, Debug)]
struct StructuralLineLoadTelemetryChannel {
    station_id: String,
    output_id: OutputId,
    channel: TelemetryChannel<f64>,
}

impl StructuralLineLoadTelemetryChannels {
    /// Build telemetry channels when `[loads.recovery]` is declared.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] if telemetry metadata is invalid.
    pub fn maybe_new<F>(
        document: &ScenarioDocument,
        alloc: &mut F,
    ) -> Result<Option<Self>, RunnerError>
    where
        F: FnMut() -> ChannelId,
    {
        let Some(recovery) = document
            .loads
            .as_ref()
            .and_then(|loads| loads.recovery.as_ref())
        else {
            return Ok(None);
        };
        let mut channels = Vec::new();
        for station in &recovery.stations {
            for output in &station.outputs {
                let output_id = output_id_from_config(output.id);
                channels.push(StructuralLineLoadTelemetryChannel {
                    station_id: station.id.clone(),
                    output_id,
                    channel: TelemetryChannel::<f64>::new(
                        alloc(),
                        format!("structural.load.{}.{}", station.id, output_id.as_str()),
                        output_id.unit(),
                        Some("Body"),
                    )?,
                });
            }
        }
        Ok(Some(Self { channels }))
    }

    /// Push channel metadata in declaration order.
    pub fn push_metadata(&self, out: &mut Vec<ChannelMetadata>) {
        for entry in &self.channels {
            out.push(entry.channel.metadata().clone());
        }
    }

    /// Insert the current line-load snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] if a declared channel has no matching recovered
    /// load.
    pub fn insert_samples(
        &self,
        row: &mut TelemetryRow,
        loads: &[SectionLoad],
    ) -> Result<(), RunnerError> {
        for entry in &self.channels {
            let load = loads
                .iter()
                .find(|load| {
                    load.station_id == entry.station_id && load.output_id == entry.output_id
                })
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "structural line-load telemetry missing station `{}` output `{}`",
                        entry.station_id,
                        entry.output_id.as_str()
                    ),
                })?;
            row.insert(&entry.channel, load.value)?;
        }
        Ok(())
    }
}

fn bending_uq_sources(cfg: &BendingConfig) -> Result<Vec<UncertaintySource>, RunnerError> {
    cfg.uq
        .as_ref()
        .map(|uq| {
            structural_bending_margin_source(
                uq.deck_id.clone(),
                "frequency_hz",
                uq.frequency_lower_hz,
                cfg.frequency_hz,
                uq.frequency_upper_hz,
                CredibilityLevel::from_value(uq.credibility_level).ok_or_else(|| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "[vehicle.bending.uq] credibility_level {} is outside 0..=4",
                            uq.credibility_level
                        ),
                    }
                })?,
                uq.evidence.clone(),
            )
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("[vehicle.bending.uq] invalid: {err}"),
            })
        })
        .transpose()
        .map(|source| source.into_iter().collect())
}

fn mode_params_from_config(cfg: &VehicleModeConfig) -> Result<ModeParams, RunnerError> {
    ModeParams::new(
        core::f64::consts::TAU * cfg.frequency_hz,
        cfg.damping_ratio,
        cfg.modal_mass_kg,
        cfg.slope_at_engine,
        cfg.slope_at_gyro,
        cfg.shape_at_accelerometer,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[vehicle.modes.lateral.{}] invalid: {err}", cfg.id),
    })
}

fn mode_params_from_modal_asset(
    cfg: &VehicleModesAssetConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Vec<ModeParams>, RunnerError> {
    let resolved = resolved_files
        .get(VEHICLE_MODES_ASSET_FILE_KEY)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "internal invariant: resolved file `{VEHICLE_MODES_ASSET_FILE_KEY}` missing after pin verification"
            ),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!(
                "[vehicle.modes.asset] file `{path}` is not UTF-8 CSV: {error}",
                path = resolved.path.display()
            ),
        })?;
    let asset =
        ModalAsset::from_csv_str(text).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.modes.asset] invalid modal asset: {error}"),
        })?;
    let frequency_tolerance = cfg
        .frequency_tolerance_fraction
        .unwrap_or(DEFAULT_MODAL_ASSET_FREQUENCY_TOLERANCE_FRACTION);
    let mac_min = cfg.mac_min.unwrap_or(DEFAULT_MODAL_ASSET_MAC_MIN);
    asset
        .validate_references(frequency_tolerance, mac_min)
        .map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.modes.asset] reference validation failed: {error}"),
        })?;
    asset
        .modes()
        .iter()
        .map(mode_params_from_asset_mode)
        .collect()
}

fn mode_params_from_asset_mode(mode: &ModalModeAsset) -> Result<ModeParams, RunnerError> {
    ModeParams::new(
        core::f64::consts::TAU * mode.frequency_hz,
        mode.damping_ratio,
        mode.modal_mass_kg,
        mode.slope_at_engine,
        mode.slope_at_gyro,
        mode.shape_at_accelerometer,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[vehicle.modes.asset.{}] invalid: {err}", mode.id),
    })
}

fn transient_integrator_from_config(
    cfg: Option<&VehicleTransientConfig>,
    modes: &ModalBlock,
    dt_s: f64,
) -> Result<StructuralTransientIntegrator, RunnerError> {
    let Some(cfg) = cfg else {
        return Ok(StructuralTransientIntegrator::LegacySymplecticEuler);
    };
    match cfg.scheme {
        VehicleTransientSchemeConfig::LegacySymplecticEuler => {
            Ok(StructuralTransientIntegrator::LegacySymplecticEuler)
        }
        VehicleTransientSchemeConfig::NewmarkBeta => {
            let system = modal_transient_system(modes)?;
            let beta = cfg.beta.ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "[vehicle.transient] beta missing after scenario validation".to_owned(),
            })?;
            let gamma = cfg.gamma.ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "[vehicle.transient] gamma missing after scenario validation".to_owned(),
            })?;
            NewmarkIntegrator::new(system, dt_s, NewmarkParameters { beta, gamma })
                .map(StructuralTransientIntegrator::Newmark)
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("[vehicle.transient] Newmark integrator invalid: {err}"),
                })
        }
        VehicleTransientSchemeConfig::GeneralizedAlpha => {
            let system = modal_transient_system(modes)?;
            let rho_inf = cfg
                .rho_inf
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[vehicle.transient] rho_inf missing after scenario validation"
                        .to_owned(),
                })?;
            let parameters =
                GeneralizedAlphaParameters::from_spectral_radius(rho_inf).map_err(|err| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "[vehicle.transient] generalized-alpha parameters invalid: {err}"
                        ),
                    }
                })?;
            GeneralizedAlphaIntegrator::new(system, dt_s, parameters)
                .map(StructuralTransientIntegrator::GeneralizedAlpha)
                .map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!(
                        "[vehicle.transient] generalized-alpha integrator invalid: {err}"
                    ),
                })
        }
    }
}

fn modal_transient_system(modes: &ModalBlock) -> Result<ModalTransientSystem, RunnerError> {
    modal_transient_system_with_stiffness_scale(modes, &[])
}

fn modal_transient_system_with_stiffness_scale(
    modes: &ModalBlock,
    modal_stiffness_scale_1: &[f64],
) -> Result<ModalTransientSystem, RunnerError> {
    let coordinate_count = modes.len() * 2;
    if !modal_stiffness_scale_1.is_empty() && modal_stiffness_scale_1.len() != coordinate_count {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "[vehicle.transient] modal_stiffness_scale_1 length ({}) must match structural modal coordinate count ({coordinate_count})",
                modal_stiffness_scale_1.len()
            ),
        });
    }
    if !modal_stiffness_scale_1
        .iter()
        .all(|scale| scale.is_finite() && *scale > 0.0)
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "[vehicle.transient] modal_stiffness_scale_1 values must be finite and positive"
                .to_owned(),
        });
    }
    let mut mass = vec![0.0; coordinate_count * coordinate_count];
    let mut damping = vec![0.0; coordinate_count * coordinate_count];
    let mut stiffness = vec![0.0; coordinate_count * coordinate_count];
    for (mode_index, mode) in modes.modes().iter().enumerate() {
        for axis_offset in 0..2 {
            let coordinate = (mode_index * 2) + axis_offset;
            let matrix_index = coordinate * coordinate_count + coordinate;
            let stiffness_scale = modal_stiffness_scale_1
                .get(coordinate)
                .copied()
                .unwrap_or(1.0);
            mass[matrix_index] = mode.modal_mass_kg;
            damping[matrix_index] =
                2.0 * mode.zeta * mode.omega_rad_s * mode.modal_mass_kg * stiffness_scale.sqrt();
            stiffness[matrix_index] =
                mode.omega_rad_s * mode.omega_rad_s * mode.modal_mass_kg * stiffness_scale;
        }
    }
    let mass =
        DenseMatrix::from_row_major(coordinate_count, coordinate_count, mass).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("[vehicle.transient] mass matrix invalid: {err}"),
            }
        })?;
    let damping = DenseMatrix::from_row_major(coordinate_count, coordinate_count, damping)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.transient] damping matrix invalid: {err}"),
        })?;
    let stiffness = DenseMatrix::from_row_major(coordinate_count, coordinate_count, stiffness)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.transient] stiffness matrix invalid: {err}"),
        })?;
    ModalTransientSystem::new(mass, damping, stiffness).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("[vehicle.transient] modal system invalid: {err}"),
        }
    })
}

fn step_native_transient(active: &mut ActiveModes, time_s: f64) -> Result<(), RunnerError> {
    let state = modal_transient_state_from_modes(&active.modes)?;
    let mut force_next = modal_force_from_driver(&active.modes, active.driver_accel_body);
    add_event_modal_force(&mut force_next, &active.event_forcings, time_s)?;
    if active.previous_modal_force.len() != force_next.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[vehicle.transient] previous force dimension mismatch".to_owned(),
        });
    }
    let stiffness_scale_next =
        event_modal_stiffness_scale(&active.event_forcings, time_s, active.modes.len() * 2)?;
    if active.previous_modal_stiffness_scale_1.len() != stiffness_scale_next.len() {
        return Err(RunnerError::UnsupportedScenario {
            what: "[vehicle.transient] previous stiffness-scale dimension mismatch".to_owned(),
        });
    }
    let next_state = match &active.transient {
        StructuralTransientIntegrator::Newmark(integrator) => {
            let next_system =
                modal_transient_system_with_stiffness_scale(&active.modes, &stiffness_scale_next)?;
            integrator.step_with_system(&next_system, &state, &force_next)
        }
        StructuralTransientIntegrator::GeneralizedAlpha(integrator) => {
            let current_system = modal_transient_system_with_stiffness_scale(
                &active.modes,
                &active.previous_modal_stiffness_scale_1,
            )?;
            let next_system =
                modal_transient_system_with_stiffness_scale(&active.modes, &stiffness_scale_next)?;
            integrator.step_with_systems(
                &current_system,
                &next_system,
                &state,
                &active.previous_modal_force,
                &force_next,
            )
        }
        StructuralTransientIntegrator::LegacySymplecticEuler => unreachable!(
            "step_native_transient is only called for native structural transient integrators"
        ),
    }
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[vehicle.transient] modal step failed: {err}"),
    })?;
    apply_modal_transient_state(&mut active.modes, &next_state)?;
    active.previous_modal_force = force_next;
    active.previous_modal_stiffness_scale_1 = stiffness_scale_next;
    Ok(())
}

fn event_modal_force(
    event_forcings: &[StructuralEventRuntimeForce],
    time_s: f64,
    coordinate_count: usize,
) -> Result<Vec<f64>, RunnerError> {
    let mut force = vec![0.0; coordinate_count];
    add_event_modal_force(&mut force, event_forcings, time_s)?;
    Ok(force)
}

fn add_event_modal_force(
    force: &mut [f64],
    event_forcings: &[StructuralEventRuntimeForce],
    time_s: f64,
) -> Result<(), RunnerError> {
    for event in event_forcings {
        let event_force = event
            .forcing
            .modal_force_at(time_s, &event.mode_shapes)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "[loads.events.forces.{}] live modal force evaluation failed: {err}",
                    event.forcing.id()
                ),
            })?;
        if event_force.len() != force.len() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "[loads.events.forces.{}] live modal force length ({}) must match structural modal coordinate count ({})",
                    event.forcing.id(),
                    event_force.len(),
                    force.len()
                ),
            });
        }
        for (target, value) in force.iter_mut().zip(event_force) {
            *target += value;
        }
    }
    Ok(())
}

fn event_modal_stiffness_scale(
    event_forcings: &[StructuralEventRuntimeForce],
    time_s: f64,
    coordinate_count: usize,
) -> Result<Vec<f64>, RunnerError> {
    let mut scale = vec![1.0; coordinate_count];
    for event in event_forcings {
        if event.modal_stiffness_scale_1.is_empty() {
            continue;
        }
        let multiplier = event
            .forcing
            .profile()
            .multiplier_at(time_s)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "[loads.events.forces.{}] stiffness profile evaluation failed: {err}",
                    event.forcing.id()
                ),
            })?;
        for (target, event_scale) in scale.iter_mut().zip(&event.modal_stiffness_scale_1) {
            let blended = 1.0 + (multiplier * (*event_scale - 1.0));
            if !blended.is_finite() || blended <= 0.0 {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "[loads.events.forces.{}] modal stiffness scale evaluated non-positive",
                        event.forcing.id()
                    ),
                });
            }
            *target *= blended;
        }
    }
    Ok(scale)
}

fn modal_force_from_driver(modes: &ModalBlock, accel_body_m_s2: Vector3<f64>) -> Vec<f64> {
    let mut force = Vec::with_capacity(modes.len() * 2);
    for mode in modes.modes() {
        force.push(mode.slope_at_engine * accel_body_m_s2.x);
        force.push(mode.slope_at_engine * accel_body_m_s2.y);
    }
    force
}

fn modal_transient_state_from_modes(
    modes: &ModalBlock,
) -> Result<ModalTransientState, RunnerError> {
    let mut q = Vec::with_capacity(modes.len() * 2);
    let mut q_dot = Vec::with_capacity(modes.len() * 2);
    let mut q_ddot = Vec::with_capacity(modes.len() * 2);
    for state in modes.modal_states() {
        q.push(state.q_x);
        q.push(state.q_y);
        q_dot.push(state.q_dot_x);
        q_dot.push(state.q_dot_y);
        q_ddot.push(state.q_ddot_x);
        q_ddot.push(state.q_ddot_y);
    }
    ModalTransientState::new(q, q_dot, q_ddot, modes.len() * 2).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("[vehicle.transient] modal state invalid: {err}"),
        }
    })
}

fn apply_modal_transient_state(
    modes: &mut ModalBlock,
    state: &ModalTransientState,
) -> Result<(), RunnerError> {
    if state.q.len() != modes.len() * 2
        || state.q_dot.len() != modes.len() * 2
        || state.q_ddot.len() != modes.len() * 2
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "[vehicle.transient] modal state dimension mismatch".to_owned(),
        });
    }
    let mut modal_states = Vec::with_capacity(modes.len());
    for mode_index in 0..modes.len() {
        let offset = mode_index * 2;
        modal_states.push(ModalState {
            q_x: state.q[offset],
            q_dot_x: state.q_dot[offset],
            q_ddot_x: state.q_ddot[offset],
            q_y: state.q[offset + 1],
            q_dot_y: state.q_dot[offset + 1],
            q_ddot_y: state.q_ddot[offset + 1],
        });
    }
    modes
        .set_modal_states(&modal_states)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[vehicle.transient] modal state application failed: {err}"),
        })
}

fn line_load_recovery_from_config(
    cfg: &LoadsRecoveryConfig,
    coordinate_count: usize,
) -> Result<LineLoadRecovery, RunnerError> {
    let stations = cfg
        .stations
        .iter()
        .map(|station| {
            let rows = station
                .outputs
                .iter()
                .map(|output| {
                    let mode_acceleration_coefficients =
                        if output.mode_acceleration_coefficients.is_empty() {
                            vec![0.0; coordinate_count]
                        } else {
                            output.mode_acceleration_coefficients.clone()
                        };
                    RecoveryRow::new(
                        output_id_from_config(output.id),
                        output.displacement_coefficients.clone(),
                        output.acceleration_coefficients.clone(),
                        mode_acceleration_coefficients,
                        coordinate_count,
                    )
                    .map_err(|err| RunnerError::UnsupportedScenario {
                        what: format!(
                            "[loads.recovery.stations.{}.{}] invalid: {err}",
                            station.id,
                            output.id.as_str()
                        ),
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            RecoveryStation::new(station.id.clone(), station.axial_m, rows).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!("[loads.recovery.stations.{}] invalid: {err}", station.id),
                }
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    LineLoadRecovery::new(coordinate_count, cfg.mode_acceleration, stations).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("[loads.recovery] invalid: {err}"),
        }
    })
}

fn evaluate_line_loads(
    modes: &ModalBlock,
    recovery: Option<&LineLoadRecovery>,
) -> Result<Vec<SectionLoad>, RunnerError> {
    let Some(recovery) = recovery else {
        return Ok(Vec::new());
    };
    let state = modal_recovery_state(modes)?;
    recovery
        .evaluate(&state)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[loads.recovery] evaluation failed: {err}"),
        })
}

fn modal_recovery_state(modes: &ModalBlock) -> Result<ModalRecoveryState, RunnerError> {
    let mut q = Vec::with_capacity(modes.modal_states().len() * 2);
    let mut q_ddot = Vec::with_capacity(modes.modal_states().len() * 2);
    for state in modes.modal_states() {
        q.push(state.q_x);
        q.push(state.q_y);
        q_ddot.push(state.q_ddot_x);
        q_ddot.push(state.q_ddot_y);
    }
    ModalRecoveryState::new(q, q_ddot).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[loads.recovery] modal state invalid: {err}"),
    })
}

fn output_id_from_config(id: LoadsRecoveryOutputIdConfig) -> OutputId {
    match id {
        LoadsRecoveryOutputIdConfig::AxialForce => OutputId::AxialForce,
        LoadsRecoveryOutputIdConfig::ShearX => OutputId::ShearX,
        LoadsRecoveryOutputIdConfig::ShearY => OutputId::ShearY,
        LoadsRecoveryOutputIdConfig::BendingX => OutputId::BendingX,
        LoadsRecoveryOutputIdConfig::BendingY => OutputId::BendingY,
        LoadsRecoveryOutputIdConfig::Torsion => OutputId::Torsion,
    }
}

/// Evaluate declared structural event-force envelopes.
///
/// The returned report is deterministic evidence for the same declared event
/// rows the structural rack injects into the live modal state.
///
/// # Errors
///
/// Returns [`RunnerError`] when the event declaration does not conform to the
/// active structural modal coordinate count or cannot be evaluated by
/// `openbmp-structdyn`.
pub fn evaluate_event_loads(
    document: &ScenarioDocument,
    rack: &StructuralRack,
) -> Result<Option<StructuralEventLoadsReport>, RunnerError> {
    let Some(events) = document
        .loads
        .as_ref()
        .and_then(|loads| loads.events.as_ref())
    else {
        return Ok(None);
    };
    let coordinate_count = rack.modal_coordinate_count();
    if coordinate_count == 0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "[loads.events] requires an active structural modal rack".to_owned(),
        });
    }
    let envelopes = events
        .forces
        .iter()
        .map(|event| event_envelope_from_config(event, events, coordinate_count))
        .collect::<Result<Vec<_>, _>>()?;
    let envelope_csv = structural_event_envelope_csv(&envelopes);
    Ok(Some(StructuralEventLoadsReport {
        sample_times_s: events.sample_times_s.clone(),
        envelopes,
        envelope_csv: envelope_csv.into_bytes(),
    }))
}

fn event_envelope_from_config(
    event: &LoadsEventForceConfig,
    events: &LoadsEventsConfig,
    coordinate_count: usize,
) -> Result<StructuralEventModalEnvelope, RunnerError> {
    let runtime = event_runtime_force_from_config(event, coordinate_count)?;
    let envelope = modal_force_envelope(
        &runtime.forcing,
        &events.sample_times_s,
        &runtime.mode_shapes,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!(
            "[loads.events.forces.{}] envelope evaluation failed: {err}",
            event.id
        ),
    })?;
    Ok(StructuralEventModalEnvelope {
        event_id: event.id.clone(),
        flight_event_kind: event
            .flight_event_kind
            .map(|flight_event_kind| flight_event_kind.as_str().to_owned()),
        kind: event.kind.as_str().to_owned(),
        max_abs_modal_force_n: envelope.max_abs_modal_force().to_vec(),
    })
}

fn event_runtime_forces_from_config(
    events: &LoadsEventsConfig,
    coordinate_count: usize,
) -> Result<Vec<StructuralEventRuntimeForce>, RunnerError> {
    events
        .forces
        .iter()
        .map(|event| event_runtime_force_from_config(event, coordinate_count))
        .collect()
}

fn event_runtime_force_from_config(
    event: &LoadsEventForceConfig,
    coordinate_count: usize,
) -> Result<StructuralEventRuntimeForce, RunnerError> {
    let rows = event.physical_force_n.len();
    let cols = event
        .mode_shape_rows_1
        .first()
        .map_or(0, std::vec::Vec::len);
    if cols != coordinate_count {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "[loads.events.forces.{}] mode_shape_rows_1 columns ({cols}) must match structural modal coordinate count ({coordinate_count})",
                event.id
            ),
        });
    }
    if !event.modal_stiffness_scale_1.is_empty()
        && event.modal_stiffness_scale_1.len() != coordinate_count
    {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "[loads.events.forces.{}] modal_stiffness_scale_1 length ({}) must match structural modal coordinate count ({coordinate_count})",
                event.id,
                event.modal_stiffness_scale_1.len()
            ),
        });
    }
    let mode_shape_data = event
        .mode_shape_rows_1
        .iter()
        .flat_map(|row| row.iter().copied())
        .collect::<Vec<_>>();
    let mode_shapes = DenseMatrix::from_row_major(rows, coordinate_count, mode_shape_data)
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!(
                "[loads.events.forces.{}] mode_shape_rows_1 invalid: {err}",
                event.id
            ),
        })?;
    let profile = event_profile_from_config(event)?;
    let forcing = EventForcing::new(event.id.clone(), profile, event.physical_force_n.clone())
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("[loads.events.forces.{}] invalid: {err}", event.id),
        })?;
    Ok(StructuralEventRuntimeForce {
        forcing,
        mode_shapes,
        modal_stiffness_scale_1: event.modal_stiffness_scale_1.clone(),
    })
}

fn event_profile_from_config(
    event: &LoadsEventForceConfig,
) -> Result<EventForceProfile, RunnerError> {
    EventForceProfile::from_kind_with_gust(
        event.kind.as_str(),
        event.start_s,
        event.duration_s,
        event.gust_length_scale_s,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("[loads.events.forces.{}] profile invalid: {err}", event.id),
    })
}

fn structural_event_envelope_csv(envelopes: &[StructuralEventModalEnvelope]) -> String {
    let mut csv = String::from(
        "event_id,flight_event_kind,kind,modal_coordinate_index,max_abs_modal_force_n\n",
    );
    for envelope in envelopes {
        for (index, value) in envelope.max_abs_modal_force_n.iter().enumerate() {
            push_csv_field(&mut csv, &envelope.event_id);
            csv.push(',');
            push_csv_field(
                &mut csv,
                envelope.flight_event_kind.as_deref().unwrap_or_default(),
            );
            csv.push(',');
            push_csv_field(&mut csv, &envelope.kind);
            let _ = writeln!(csv, ",{index},{value:.17e}");
        }
    }
    csv
}

fn push_csv_field(out: &mut String, value: &str) {
    if value
        .chars()
        .any(|ch| matches!(ch, ',' | '"' | '\n' | '\r'))
    {
        out.push('"');
        for ch in value.chars() {
            if ch == '"' {
                out.push('"');
            }
            out.push(ch);
        }
        out.push('"');
    } else {
        out.push_str(value);
    }
}

/// Consume the acoustics response modal-force PSD through the first bending
/// mode declared in `[vehicle.bending]`.
///
/// Returns `Ok(None)` unless both `[vehicle.bending]` and
/// `[acoustics.response]` are present. The calculation is out-of-band evidence:
/// it does not modify the time-domain structural state or force authority.
///
/// # Errors
///
/// Returns [`RunnerError`] when the structural random-response denominator is
/// singular or any derived response value is non-finite.
pub fn evaluate_acoustic_loads_handoff(
    document: &ScenarioDocument,
    acoustics: Option<&crate::acoustics::AcousticsRunReport>,
) -> Result<Option<StructuralAcousticLoadsReport>, RunnerError> {
    let Some(bending) = document.vehicle.bending.as_ref() else {
        return Ok(None);
    };
    let Some(response) = acoustics.and_then(|report| report.response.as_ref()) else {
        return Ok(None);
    };
    evaluate_bending_acoustic_response(bending, response).map(Some)
}

fn evaluate_bending_acoustic_response(
    bending: &BendingConfig,
    response: &crate::acoustics::AcousticsResponseRunReport,
) -> Result<StructuralAcousticLoadsReport, RunnerError> {
    let modal_response = structural_response_from_force_psd(
        &response.modal_force_psd,
        bending.frequency_hz,
        bending.damping_ratio,
        bending.modal_mass_kg,
        bending.slope_at_engine,
    )?;
    Ok(StructuralAcousticLoadsReport {
        station_id: response.station_id.clone(),
        source: response.source,
        response_class_id: response.class_id.clone(),
        mode_frequency_hz: bending.frequency_hz,
        mode_damping_ratio: bending.damping_ratio,
        modal_mass_kg: bending.modal_mass_kg,
        total_input_mean_square_force_n2: response.total_mean_square_force_n2,
        total_modal_displacement_mean_square_m2: modal_response.total_displacement_m2,
        total_modal_acceleration_mean_square_m2_s4: modal_response.total_acceleration_m2_s4,
        peak_acceleration_band_center_hz: modal_response.peak_acceleration_band_center_hz,
        total_equivalent_engine_reaction_moment_mean_square_n2_m2: modal_response
            .total_engine_reaction_moment_n2_m2,
        equivalent_engine_reaction_moment_rms_n_m: modal_response.engine_reaction_moment_rms_n_m,
        band_response_csv: modal_response.csv.into_bytes(),
        line_load_recovery_csv: modal_response.line_load_csv.into_bytes(),
    })
}

#[derive(Debug)]
struct ModalForceResponseTable {
    total_displacement_m2: f64,
    total_acceleration_m2_s4: f64,
    peak_acceleration_band_center_hz: f64,
    total_engine_reaction_moment_n2_m2: f64,
    engine_reaction_moment_rms_n_m: f64,
    csv: String,
    line_load_csv: String,
}

fn structural_response_from_force_psd(
    force_psd: &ForcePsdSpectrum,
    frequency_hz: f64,
    damping_ratio: f64,
    modal_mass_kg: f64,
    slope_at_engine: f64,
) -> Result<ModalForceResponseTable, RunnerError> {
    let omega_n = core::f64::consts::TAU * frequency_hz;
    let reaction_gain_n_m_per_modal_accel = -modal_mass_kg * slope_at_engine;
    let mut total_displacement_m2 = 0.0;
    let mut total_acceleration_m2_s4 = 0.0;
    let mut total_engine_reaction_moment_n2_m2 = 0.0;
    let mut peak_acceleration_band_center_hz = 0.0;
    let mut peak_acceleration_band_power = f64::NEG_INFINITY;
    let mut csv = String::from(
        "center_hz,lower_hz,upper_hz,modal_force_psd_n2_per_hz,band_force_mean_square_n2,modal_displacement_psd_m2_per_hz,band_modal_displacement_mean_square_m2,modal_acceleration_psd_m2_s4_per_hz,band_modal_acceleration_mean_square_m2_s4\n",
    );
    let mut line_load_csv = String::from(
        "station_id,recovery_kind,center_hz,lower_hz,upper_hz,mode_slope,modal_acceleration_psd_m2_s4_per_hz,band_modal_acceleration_mean_square_m2_s4,equivalent_reaction_moment_psd_n2_m2_per_hz,band_equivalent_reaction_moment_mean_square_n2_m2,equivalent_reaction_moment_rms_n_m\n",
    );
    for (band, force) in force_psd.bands().iter().zip(force_psd.values()) {
        let omega = core::f64::consts::TAU * band.center_hz;
        let stiffness_term = modal_mass_kg * (omega_n.mul_add(omega_n, -(omega * omega)));
        let damping_term = 2.0 * damping_ratio * modal_mass_kg * omega_n * omega;
        let denominator = stiffness_term.mul_add(stiffness_term, damping_term * damping_term);
        if !denominator.is_finite() || denominator <= 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "structural acoustic loads handoff is singular at {} Hz; \
                     increase vehicle.bending.damping_ratio or move the mode off the acoustic band center",
                    band.center_hz
                ),
            });
        }
        let force_psd_n2_per_hz = force.n2_per_hz();
        let displacement_psd_m2_per_hz = force_psd_n2_per_hz / denominator;
        let acceleration_psd_m2_s4_per_hz =
            displacement_psd_m2_per_hz * omega * omega * omega * omega;
        let bandwidth_hz = band.bandwidth_hz();
        let band_force_n2 = force_psd_n2_per_hz * bandwidth_hz;
        let band_displacement_m2 = displacement_psd_m2_per_hz * bandwidth_hz;
        let band_acceleration_m2_s4 = acceleration_psd_m2_s4_per_hz * bandwidth_hz;
        let reaction_moment_psd_n2_m2_per_hz = reaction_gain_n_m_per_modal_accel
            * reaction_gain_n_m_per_modal_accel
            * acceleration_psd_m2_s4_per_hz;
        let band_reaction_moment_n2_m2 = reaction_moment_psd_n2_m2_per_hz * bandwidth_hz;
        let band_reaction_moment_rms_n_m = band_reaction_moment_n2_m2.sqrt();
        require_finite_structural_handoff(
            "structural_acoustics.band_modal_displacement_mean_square_m2",
            band_displacement_m2,
        )?;
        require_finite_structural_handoff(
            "structural_acoustics.band_modal_acceleration_mean_square_m2_s4",
            band_acceleration_m2_s4,
        )?;
        require_finite_structural_handoff(
            "structural_acoustics.band_equivalent_reaction_moment_mean_square_n2_m2",
            band_reaction_moment_n2_m2,
        )?;
        total_displacement_m2 += band_displacement_m2;
        total_acceleration_m2_s4 += band_acceleration_m2_s4;
        total_engine_reaction_moment_n2_m2 += band_reaction_moment_n2_m2;
        if band_acceleration_m2_s4 > peak_acceleration_band_power {
            peak_acceleration_band_power = band_acceleration_m2_s4;
            peak_acceleration_band_center_hz = band.center_hz;
        }
        let _ = writeln!(
            csv,
            "{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}",
            band.center_hz,
            band.lower_hz,
            band.upper_hz,
            force_psd_n2_per_hz,
            band_force_n2,
            displacement_psd_m2_per_hz,
            band_displacement_m2,
            acceleration_psd_m2_s4_per_hz,
            band_acceleration_m2_s4
        );
        let _ = writeln!(
            line_load_csv,
            "engine,first_bending_reaction,{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}",
            band.center_hz,
            band.lower_hz,
            band.upper_hz,
            slope_at_engine,
            acceleration_psd_m2_s4_per_hz,
            band_acceleration_m2_s4,
            reaction_moment_psd_n2_m2_per_hz,
            band_reaction_moment_n2_m2,
            band_reaction_moment_rms_n_m
        );
    }
    require_finite_structural_handoff(
        "structural_acoustics.total_modal_displacement_mean_square_m2",
        total_displacement_m2,
    )?;
    require_finite_structural_handoff(
        "structural_acoustics.total_modal_acceleration_mean_square_m2_s4",
        total_acceleration_m2_s4,
    )?;
    require_finite_structural_handoff(
        "structural_acoustics.total_equivalent_engine_reaction_moment_mean_square_n2_m2",
        total_engine_reaction_moment_n2_m2,
    )?;
    let engine_reaction_moment_rms_n_m = total_engine_reaction_moment_n2_m2.sqrt();
    require_finite_structural_handoff(
        "structural_acoustics.equivalent_engine_reaction_moment_rms_n_m",
        engine_reaction_moment_rms_n_m,
    )?;
    Ok(ModalForceResponseTable {
        total_displacement_m2,
        total_acceleration_m2_s4,
        peak_acceleration_band_center_hz,
        total_engine_reaction_moment_n2_m2,
        engine_reaction_moment_rms_n_m,
        csv,
        line_load_csv,
    })
}

fn require_finite_structural_handoff(field: &'static str, value: f64) -> Result<(), RunnerError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(RunnerError::UnsupportedScenario {
            what: format!("{field} must be finite"),
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use openbmp_scenario::ResolvedFile;

    const BASE_SCENARIO: &str = include_str!("../tests/fixtures/structural-rack-base.toml");
    const MODAL_CSV: &str = include_str!("../../../data/modal/synthetic-sol103-modal-asset-v1.csv");
    const MODAL_CSV_SHA256: &str =
        "6d3f3c621a9340d509cca35f111826ad06cf45bc2ded92027a98b4af3c5f3171";

    fn insert_before_assembly(insert: &str) -> String {
        BASE_SCENARIO.replace(
            "[vehicle.assembly]\n",
            &format!("{insert}\n[vehicle.assembly]\n"),
        )
    }

    #[test]
    fn explicit_vehicle_modes_drive_gyro_accel_and_reaction_when_enabled() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[vehicle.reaction]
enabled = true

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
shape_at_accelerometer = 0.25

[[vehicle.modes.lateral]]
id = "second"
frequency_hz = 2.0
damping_ratio = 0.03
modal_mass_kg = 700.0
slope_at_engine = -0.25
slope_at_gyro = -0.2
shape_at_accelerometer = -0.1
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        assert!(!rack.is_inactive());
        assert_eq!(rack.gyro_pickup_rad_s(), Vector3::zeros());
        assert_eq!(rack.accel_pickup_m_s2(), Vector3::zeros());
        assert_eq!(rack.reaction_moment_body_n_m(), Vector3::zeros());

        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();

        assert!(rack.gyro_pickup_rad_s().x.abs() > 0.0);
        assert!(rack.gyro_pickup_rad_s().y.abs() > 0.0);
        assert!(rack.accel_pickup_m_s2().x.abs() > 0.0);
        assert!(rack.accel_pickup_m_s2().y.abs() > 0.0);
        assert!(
            rack.reaction_moment_body_n_m()
                .iter()
                .all(|v| v.is_finite())
        );

        rack.reset();
        assert_eq!(rack.gyro_pickup_rad_s(), Vector3::zeros());
        assert_eq!(rack.accel_pickup_m_s2(), Vector3::zeros());
        assert_eq!(rack.reaction_moment_body_n_m(), Vector3::zeros());
    }

    #[test]
    fn reaction_moment_is_disabled_by_default() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
shape_at_accelerometer = 0.25
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();
        assert!(rack.gyro_pickup_rad_s().iter().any(|v| v.abs() > 0.0));
        assert!(rack.accel_pickup_m_s2().iter().any(|v| v.abs() > 0.0));
        assert_eq!(rack.reaction_moment_body_n_m(), Vector3::zeros());
    }

    #[test]
    fn vehicle_transient_newmark_integrator_drives_structural_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
shape_at_accelerometer = 0.25

[vehicle.transient]
scheme = "newmark_beta"
beta = 0.25
gamma = 0.5

[vehicle.reaction]
enabled = true
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();

        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();

        assert!(rack.gyro_pickup_rad_s().iter().any(|v| v.abs() > 0.0));
        assert!(rack.accel_pickup_m_s2().iter().any(|v| v.abs() > 0.0));
        assert!(
            rack.reaction_moment_body_n_m()
                .iter()
                .any(|v| v.abs() > 0.0)
        );
    }

    #[test]
    fn vehicle_transient_generalized_alpha_integrator_resets_force_history() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
shape_at_accelerometer = 0.25

[vehicle.transient]
scheme = "generalized_alpha"
rho_inf = 0.5
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();

        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();
        assert!(rack.gyro_pickup_rad_s().iter().any(|v| v.abs() > 0.0));
        assert!(rack.accel_pickup_m_s2().iter().any(|v| v.abs() > 0.0));

        rack.reset();
        rack.update_drivers(Vector3::zeros());
        rack.step().unwrap();
        assert_eq!(rack.gyro_pickup_rad_s(), Vector3::zeros());
        assert_eq!(rack.accel_pickup_m_s2(), Vector3::zeros());
        assert_eq!(rack.reaction_moment_body_n_m(), Vector3::zeros());
    }

    #[test]
    fn modal_asset_drives_structural_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[vehicle.modes.asset]
format = "csv_v1"
file = "data/modal/synthetic-sol103-modal-asset-v1.csv"
file_sha256 = "6d3f3c621a9340d509cca35f111826ad06cf45bc2ded92027a98b4af3c5f3171"
frequency_tolerance_fraction = 0.01
mac_min = 0.9

[vehicle.reaction]
enabled = true
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let resolved = ResolvedFile::from_bytes(
            "data/modal/synthetic-sol103-modal-asset-v1.csv",
            MODAL_CSV.as_bytes().to_vec(),
        );
        assert_eq!(resolved.sha256_hex, MODAL_CSV_SHA256);
        let mut resolved_files = BTreeMap::new();
        resolved_files.insert(VEHICLE_MODES_ASSET_FILE_KEY.to_owned(), resolved);

        let mut rack =
            StructuralRack::build_with_resolved_files(&scenario.document, &resolved_files).unwrap();
        assert!(!rack.is_inactive());
        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();
        assert!(rack.gyro_pickup_rad_s().iter().any(|v| v.abs() > 0.0));
        assert!(rack.accel_pickup_m_s2().iter().any(|v| v.abs() > 0.0));
        assert!(
            rack.reaction_moment_body_n_m()
                .iter()
                .all(|v| v.is_finite())
        );
        assert!(
            rack.reaction_moment_body_n_m()
                .iter()
                .any(|v| v.abs() > 0.0)
        );
    }

    #[test]
    fn loads_recovery_emits_mode_acceleration_corrected_section_loads() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
shape_at_accelerometer = 0.25

[loads.recovery]
mode_acceleration = true

[[loads.recovery.stations]]
id = "interstage"
axial_m = 12.0

[[loads.recovery.stations.outputs]]
id = "shear_x"
displacement_coefficients = [0.0, 0.0]
acceleration_coefficients = [10.0, 5.0]
mode_acceleration_coefficients = [1.5, -0.5]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        assert_eq!(rack.line_loads().len(), 1);
        assert!(rack.line_loads()[0].value.abs() < f64::EPSILON);

        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();

        let loads = rack.line_loads();
        assert_eq!(loads[0].station_id, "interstage");
        assert_eq!(loads[0].output_id, openbmp_structdyn::OutputId::ShearX);
        assert!(
            (loads[0].value - 0.037).abs() < 1.0e-12,
            "expected corrected line load, got {}",
            loads[0].value
        );
    }

    #[test]
    fn loads_recovery_dimension_mismatch_fails_closed() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.recovery]

[[loads.recovery.stations]]
id = "bad"
axial_m = 1.0

[[loads.recovery.stations.outputs]]
id = "shear_x"
displacement_coefficients = [0.0]
acceleration_coefficients = [1.0]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let err = StructuralRack::build(&scenario.document).unwrap_err();
        assert!(
            err.to_string().contains("[loads.recovery")
                && err.to_string().contains("dimension mismatch"),
            "expected dimension mismatch, got {err}"
        );
    }

    #[test]
    fn loads_events_emit_run_outcome_modal_force_envelopes() {
        let baseline = openbmp_scenario::Scenario::from_toml_str(BASE_SCENARIO).unwrap();
        assert!(crate::run(&baseline).unwrap().structural_events.is_none());

        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.events]
sample_times_s = [0.0, 0.5, 1.0]

[[loads.events.forces]]
id = "liftoff_release"
flight_event_kind = "liftoff_release"
kind = "ramp_release"
start_s = 0.0
duration_s = 1.0
physical_force_n = [8.0, -4.0]
mode_shape_rows_1 = [[1.0, -1.0], [0.25, 2.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.structural_events.unwrap();

        assert_eq!(report.sample_times_s.len(), 3);
        assert_eq!(report.envelopes.len(), 1);
        assert_eq!(report.envelopes[0].event_id, "liftoff_release");
        assert_eq!(
            report.envelopes[0].flight_event_kind.as_deref(),
            Some("liftoff_release")
        );
        assert_eq!(report.envelopes[0].kind, "ramp_release");
        assert!((report.envelopes[0].max_abs_modal_force_n[0] - 7.0).abs() < 1.0e-14);
        assert!((report.envelopes[0].max_abs_modal_force_n[1] - 16.0).abs() < 1.0e-14);
        let csv = std::str::from_utf8(&report.envelope_csv).unwrap();
        assert!(csv.starts_with(
            "event_id,flight_event_kind,kind,modal_coordinate_index,max_abs_modal_force_n\n"
        ));
        assert!(csv.contains("liftoff_release,liftoff_release,ramp_release,0,"));
        assert!(csv.contains("liftoff_release,liftoff_release,ramp_release,1,"));
    }

    #[test]
    fn loads_events_emit_dryden_gust_modal_force_envelope() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.events]
sample_times_s = [0.0, 1.0, 2.0]

[[loads.events.forces]]
id = "max_q_dryden"
kind = "dryden_gust"
start_s = 0.0
duration_s = 2.0
gust_length_scale_s = 1.0
physical_force_n = [10.0]
mode_shape_rows_1 = [[2.0, -1.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.structural_events.unwrap();

        assert_eq!(report.envelopes.len(), 1);
        assert_eq!(report.envelopes[0].event_id, "max_q_dryden");
        assert_eq!(report.envelopes[0].kind, "dryden_gust");
        assert!((report.envelopes[0].max_abs_modal_force_n[0] - 20.0).abs() < 1.0e-14);
        assert!((report.envelopes[0].max_abs_modal_force_n[1] - 10.0).abs() < 1.0e-14);
        let csv = std::str::from_utf8(&report.envelope_csv).unwrap();
        assert!(csv.contains("max_q_dryden,,dryden_gust,0,"));
        assert!(csv.contains("max_q_dryden,,dryden_gust,1,"));
    }

    #[test]
    fn loads_events_emit_half_sine_impulse_modal_force_envelope() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.events]
sample_times_s = [0.0, 0.25, 0.5]

[[loads.events.forces]]
id = "pyroshock_half_sine"
kind = "half_sine_impulse"
start_s = 0.0
duration_s = 0.5
physical_force_n = [3.0]
mode_shape_rows_1 = [[2.0, -1.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.structural_events.unwrap();

        assert_eq!(report.envelopes.len(), 1);
        assert_eq!(report.envelopes[0].event_id, "pyroshock_half_sine");
        assert_eq!(report.envelopes[0].kind, "half_sine_impulse");
        assert!(
            (report.envelopes[0].max_abs_modal_force_n[0] - 6.0 * std::f64::consts::PI).abs()
                < 1.0e-14
        );
        assert!(
            (report.envelopes[0].max_abs_modal_force_n[1] - 3.0 * std::f64::consts::PI).abs()
                < 1.0e-14
        );
        let csv = std::str::from_utf8(&report.envelope_csv).unwrap();
        assert!(csv.contains("pyroshock_half_sine,,half_sine_impulse,0,"));
        assert!(csv.contains("pyroshock_half_sine,,half_sine_impulse,1,"));
    }

    #[test]
    fn loads_events_emit_von_karman_gust_modal_force_envelope() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.events]
sample_times_s = [0.0, 1.0, 2.0]

[[loads.events.forces]]
id = "max_q_von_karman"
kind = "von_karman_gust"
start_s = 0.0
duration_s = 2.0
gust_length_scale_s = 1.0
physical_force_n = [10.0]
mode_shape_rows_1 = [[2.0, -1.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let outcome = crate::run(&scenario).unwrap();
        let report = outcome.structural_events.unwrap();

        assert_eq!(report.envelopes.len(), 1);
        assert_eq!(report.envelopes[0].event_id, "max_q_von_karman");
        assert_eq!(report.envelopes[0].kind, "von_karman_gust");
        assert!((report.envelopes[0].max_abs_modal_force_n[0] - 20.0).abs() < 1.0e-14);
        assert!((report.envelopes[0].max_abs_modal_force_n[1] - 10.0).abs() < 1.0e-14);
        let csv = std::str::from_utf8(&report.envelope_csv).unwrap();
        assert!(csv.contains("max_q_von_karman,,von_karman_gust,0,"));
        assert!(csv.contains("max_q_von_karman,,von_karman_gust,1,"));
    }

    #[test]
    fn loads_events_inject_live_modal_force_into_structural_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.0
modal_mass_kg = 10.0
slope_at_engine = 0.0
slope_at_gyro = 1.0

[loads.events]
sample_times_s = [0.0, 0.1]

[[loads.events.forces]]
id = "step_load"
kind = "step"
start_s = 0.0
physical_force_n = [20.0]
mode_shape_rows_1 = [[1.0, 0.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();

        rack.step_at(0.0).unwrap();

        let StructuralRack::Active(active) = &rack else {
            panic!("expected active structural rack");
        };
        let state = active.modes.modal_states()[0];
        assert!((state.q_ddot_x - 2.0).abs() < 1.0e-14);
        assert!(state.q_ddot_y.abs() < 1.0e-14);
    }

    #[test]
    fn loads_events_apply_live_modal_stiffness_scale_to_legacy_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.0
modal_mass_kg = 10.0
slope_at_engine = 0.0
slope_at_gyro = 1.0

[loads.events]
sample_times_s = [0.0, 0.1]

[[loads.events.forces]]
id = "sep_stiffness"
kind = "step"
start_s = 0.0
modal_stiffness_scale_1 = [0.25, 4.0]
physical_force_n = [0.0]
mode_shape_rows_1 = [[1.0, 0.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        let StructuralRack::Active(active) = &mut rack else {
            panic!("expected active structural rack");
        };
        active
            .modes
            .set_modal_states(&[ModalState {
                q_x: 1.0,
                q_y: -1.0,
                ..ModalState::default()
            }])
            .unwrap();

        rack.step_at(0.0).unwrap();

        let StructuralRack::Active(active) = &rack else {
            panic!("expected active structural rack");
        };
        let state = active.modes.modal_states()[0];
        let omega_sq = std::f64::consts::TAU * std::f64::consts::TAU;
        assert!((state.q_ddot_x + (0.25 * omega_sq)).abs() < 1.0e-12);
        assert!((state.q_ddot_y - (4.0 * omega_sq)).abs() < 1.0e-12);
    }

    #[test]
    fn loads_events_apply_live_modal_stiffness_scale_to_newmark_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.0
modal_mass_kg = 10.0
slope_at_engine = 0.0
slope_at_gyro = 1.0

[vehicle.transient]
scheme = "newmark_beta"
beta = 0.25
gamma = 0.5

[loads.events]
sample_times_s = [0.0, 0.1]

[[loads.events.forces]]
id = "sep_stiffness"
kind = "step"
start_s = 0.0
modal_stiffness_scale_1 = [0.25, 4.0]
physical_force_n = [0.0]
mode_shape_rows_1 = [[1.0, 0.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        let StructuralRack::Active(active) = &mut rack else {
            panic!("expected active structural rack");
        };
        active
            .modes
            .set_modal_states(&[ModalState {
                q_x: 1.0,
                q_y: -1.0,
                ..ModalState::default()
            }])
            .unwrap();

        rack.step_at(0.0).unwrap();

        let StructuralRack::Active(active) = &rack else {
            panic!("expected active structural rack");
        };
        let state = active.modes.modal_states()[0];
        let dt_s = scenario.document.time.dt_s;
        let a0 = 1.0 / (0.25 * dt_s * dt_s);
        let modal_mass_kg = 10.0;
        let base_k = std::f64::consts::TAU * std::f64::consts::TAU * modal_mass_kg;
        let expected_x = -a0 * (base_k * 0.25) / ((modal_mass_kg * a0) + (base_k * 0.25));
        let expected_y = a0 * (base_k * 4.0) / ((modal_mass_kg * a0) + (base_k * 4.0));
        assert!(
            (state.q_ddot_x - expected_x).abs() < 1.0e-10,
            "q_ddot_x expected {expected_x}, got {}",
            state.q_ddot_x
        );
        assert!(
            (state.q_ddot_y - expected_y).abs() < 1.0e-10,
            "q_ddot_y expected {expected_y}, got {}",
            state.q_ddot_y
        );
    }

    #[test]
    fn loads_events_apply_live_modal_stiffness_scale_to_generalized_alpha_rack() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.0
modal_mass_kg = 10.0
slope_at_engine = 0.0
slope_at_gyro = 1.0

[vehicle.transient]
scheme = "generalized_alpha"
rho_inf = 0.5

[loads.events]
sample_times_s = [0.0, 0.1]

[[loads.events.forces]]
id = "sep_stiffness"
kind = "step"
start_s = 0.0
modal_stiffness_scale_1 = [0.25, 4.0]
physical_force_n = [0.0]
mode_shape_rows_1 = [[1.0, 0.0]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        let StructuralRack::Active(active) = &mut rack else {
            panic!("expected active structural rack");
        };
        active
            .modes
            .set_modal_states(&[ModalState {
                q_x: 1.0,
                q_y: -1.0,
                ..ModalState::default()
            }])
            .unwrap();

        rack.step_at(0.0).unwrap();

        let StructuralRack::Active(active) = &rack else {
            panic!("expected active structural rack");
        };
        let state = active.modes.modal_states()[0];
        assert!(state.q_ddot_x < 0.0);
        assert!(state.q_ddot_y > 0.0);
        assert!(
            state.q_ddot_y.abs() > state.q_ddot_x.abs() * 10.0,
            "expected 4x stiffness scale to dominate 0.25x scale: {state:?}"
        );
    }

    #[test]
    fn loads_events_dimension_mismatch_fails_closed() {
        let toml = insert_before_assembly(
            r#"
[vehicle.modes]

[[vehicle.modes.lateral]]
id = "first"
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1

[loads.events]
sample_times_s = [0.0, 0.5]

[[loads.events.forces]]
id = "bad_event"
kind = "step"
start_s = 0.0
physical_force_n = [8.0, -4.0]
mode_shape_rows_1 = [[1.0], [0.25]]
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let err = crate::run(&scenario).unwrap_err();
        assert!(
            err.to_string().contains("[loads.events.forces.bad_event]")
                && err.to_string().contains("mode_shape_rows_1 columns"),
            "expected event shape dimension mismatch, got {err}"
        );
    }

    #[test]
    fn legacy_bending_keeps_accelerometer_pickup_zero() {
        let toml = insert_before_assembly(
            r#"
[vehicle.bending]
frequency_hz = 1.0
damping_ratio = 0.02
modal_mass_kg = 500.0
slope_at_engine = 0.5
slope_at_gyro = 0.1
"#,
        );
        let scenario = openbmp_scenario::Scenario::from_toml_str(&toml).unwrap();
        let mut rack = StructuralRack::build(&scenario.document).unwrap();
        rack.update_drivers(Vector3::new(4.0, -2.0, 0.0));
        rack.step().unwrap();
        assert!(rack.gyro_pickup_rad_s().iter().any(|v| v.abs() > 0.0));
        assert_eq!(rack.accel_pickup_m_s2(), Vector3::zeros());
        assert_eq!(rack.reaction_moment_body_n_m(), Vector3::zeros());
    }
}
