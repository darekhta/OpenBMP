//! Runner-side communications pass-table artifact assembly.
//!
//! This module keeps WP-20.1 comm geometry outside `openbmp-fc`: the runner
//! samples the simulated primary-body state, converts it through the selected
//! frame profile, and asks `openbmp-comm` to extract deterministic passes.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_comm::{
    AntennaGainDeck, AntennaGainSample, BodyMaskDeck, BodyMaskTriangle, CommError,
    ElevationLossDeck, ElevationLossSample, FerCurveDeck, FerCurveSample, GeodeticPosition,
    GroundSite, LinkBudgetInput, LinkChannelModel, LinkErrorAction, LinkPacketContext,
    LinkPacketDirection, LinkPacketDisposition, LinkPacketEffect, LinkState, MaskDeck, MaskDeckBin,
    VisibilitySample, WGS84_A_M, evaluate_link_budget, evaluate_link_loss_rate_gate,
    link_stream_id_from_id, pass_intervals, write_pass_intervals_csv,
};
use openbmp_core::{ChannelId, Eci, Position3, SimTime, StepIndex, Velocity3};
use openbmp_physics::FrameContext;
use openbmp_scenario::{
    CommBridgeLinkSelectionConfig, CommBridgePassPlanConfig, CommGroundSiteConfig, CommLinkConfig,
    CommPacketErrorActionConfig, CommPacketLossRateGateConfig, CommRelayConfig, ResolvedFile,
    ScenarioDocument,
};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use serde::Deserialize;

use crate::error::RunnerError;

/// Communications evidence emitted by a run with `[comm]`.
#[derive(Clone, Debug, PartialEq)]
pub struct CommRunReport {
    /// Deterministic pass-table CSV bytes.
    pub pass_table_csv: Vec<u8>,
    /// Antenna/body-mask decks consumed by the runner.
    pub antenna_decks: Vec<CommAntennaDeckReport>,
    /// Link-budget decks consumed by the runner.
    pub links: Vec<CommLinkReport>,
    /// Sampled per-link pass reports with margin and bridge packet evidence.
    pub link_passes: Vec<CommLinkPassReport>,
    /// FC bridge link/gap selection intervals observed during the run.
    pub bridge_selections: Vec<CommBridgeSelectionReport>,
    /// FC bridge handover events observed between selection intervals.
    pub bridge_handovers: Vec<CommBridgeHandoverReport>,
}

/// Runner evidence for one declared `[[comm.antennas]]` deck set.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommAntennaDeckReport {
    /// Stable scenario antenna id.
    pub antenna_id: String,
    /// SHA-256 digest of the resolved antenna gain deck file, when present.
    pub gain_deck_file_sha256_hex: Option<String>,
    /// Number of validated gain samples in the parsed deck.
    pub gain_sample_count: usize,
    /// SHA-256 digest of the resolved body-mask deck file, when present.
    pub body_mask_deck_file_sha256_hex: Option<String>,
    /// Number of validated body-mask direction samples in the parsed deck.
    pub body_mask_sample_count: usize,
    /// Deterministic digest over the body-mask mesh hash and precomputed samples.
    pub body_mask_derived_sha256_hex: Option<String>,
}

/// Runner evidence for one declared `[[comm.links]]` budget/FER path.
#[derive(Clone, Debug, PartialEq)]
pub struct CommLinkReport {
    /// Stable scenario link id.
    pub link_id: String,
    /// Referenced ground-site id.
    pub site_id: String,
    /// Referenced antenna id.
    pub antenna_id: String,
    /// SHA-256 digest of the resolved FER curve deck file.
    pub fer_curve_deck_file_sha256_hex: String,
    /// Coded-performance curve id.
    pub fer_curve_code_id: String,
    /// Number of validated FER curve samples.
    pub fer_curve_sample_count: usize,
    /// SHA-256 digest of the resolved atmospheric loss deck file, when present.
    pub atmospheric_loss_deck_file_sha256_hex: Option<String>,
    /// Number of validated atmospheric loss samples.
    pub atmospheric_loss_sample_count: usize,
    /// SHA-256 digest of the resolved rain loss deck file, when present.
    pub rain_loss_deck_file_sha256_hex: Option<String>,
    /// Number of validated rain loss samples.
    pub rain_loss_sample_count: usize,
    /// Declared packet processing delay for link-channel effects, seconds.
    pub packet_processing_delay_s: f64,
    /// Packet action for errored FER draws.
    pub packet_error_action: String,
    /// Non-zero bit-flip mask when `packet_error_action == "bit_flip"`.
    pub packet_error_bit_flip_mask: Option<u8>,
    /// Optional packet loss-rate gate evidence for this link.
    pub packet_loss_rate_gate: Option<CommPacketLossRateGateReport>,
    /// Optional downlink data-loss timeout latch evidence for this link.
    pub data_loss_timeout: Option<CommDataLossTimeoutReport>,
    /// Optional declared GEO relay used to compose this link's two-hop budget.
    pub relay: Option<CommRelayReport>,
}

/// Runner evidence for one declared GEO relay used by a comm link.
#[derive(Clone, Debug, PartialEq)]
pub struct CommRelayReport {
    /// Stable relay id.
    pub relay_id: String,
    /// Relay sub-satellite longitude, degrees.
    pub longitude_deg: f64,
    /// Relay height above WGS84 ellipsoid, metres.
    pub altitude_m: f64,
    /// Relay-to-ground minimum elevation, degrees.
    pub min_elevation_deg: f64,
    /// SHA-256 digest of the resolved relay downlink FER curve deck file.
    pub fer_curve_deck_file_sha256_hex: String,
    /// Relay downlink coded-performance curve id.
    pub fer_curve_code_id: String,
    /// Number of relay downlink FER curve samples.
    pub fer_curve_sample_count: usize,
    /// SHA-256 digest of the resolved relay atmospheric loss deck file, when present.
    pub atmospheric_loss_deck_file_sha256_hex: Option<String>,
    /// Number of relay atmospheric loss samples.
    pub atmospheric_loss_sample_count: usize,
    /// SHA-256 digest of the resolved relay rain loss deck file, when present.
    pub rain_loss_deck_file_sha256_hex: Option<String>,
    /// Number of relay rain loss samples.
    pub rain_loss_sample_count: usize,
}

/// Runner evidence for one sampled visible, non-blackout comm-link pass.
#[derive(Clone, Debug, PartialEq)]
pub struct CommLinkPassReport {
    /// Stable scenario link id.
    pub link_id: String,
    /// Zero-based pass index for this link.
    pub pass_index: u64,
    /// First sampled step in this pass.
    pub start_step: u64,
    /// Last sampled step in this pass.
    pub end_step: u64,
    /// First sampled time in this pass, seconds.
    pub start_time_s: f64,
    /// Last sampled time in this pass, seconds.
    pub end_time_s: f64,
    /// Sampled pass duration, seconds.
    pub duration_s: f64,
    /// Number of usable link samples in this pass.
    pub sample_count: u64,
    /// Minimum sampled link margin, dB.
    pub min_margin_db: f64,
    /// Maximum sampled link margin, dB.
    pub max_margin_db: f64,
    /// Arithmetic mean sampled link margin, dB.
    pub mean_margin_db: f64,
    /// Sampled margin profile for the usable link samples in this pass.
    pub margin_profile: Vec<CommLinkPassMarginSampleReport>,
    /// Bridge packet-effect samples attributed to this link pass.
    pub bridge_sample_count: u64,
    /// Downlink sensor frames dropped while the bridge used this link in this pass.
    pub sensor_drop_count: u64,
    /// Uplink command frames dropped while the bridge used this link in this pass.
    pub command_drop_count: u64,
    /// Downlink sensor frames delivered corrupted while the bridge used this link in this pass.
    pub sensor_bit_flip_count: u64,
    /// Uplink command frames delivered corrupted while the bridge used this link in this pass.
    pub command_bit_flip_count: u64,
}

/// One sampled point in a comm-link pass margin profile.
#[derive(Clone, Debug, PartialEq)]
pub struct CommLinkPassMarginSampleReport {
    /// Sample step.
    pub step: u64,
    /// Sample time, seconds.
    pub time_s: f64,
    /// Sampled link margin, dB.
    pub margin_db: f64,
}

/// Runner evidence for one contiguous FC bridge link-selection interval.
#[derive(Clone, Debug, PartialEq)]
pub struct CommBridgeSelectionReport {
    /// Link selected for bridge packet effects, or `None` for a scheduled gap.
    pub link_id: Option<String>,
    /// Fallback link stream used to produce deterministic no-link drops in a gap.
    pub fallback_link_id: Option<String>,
    /// First bridge step in this interval.
    pub start_step: u64,
    /// Last bridge step observed in this interval.
    pub end_step: u64,
    /// Time of the latest comm observation at interval start, seconds.
    pub start_time_s: f64,
    /// Time of the latest comm observation at interval end, seconds.
    pub end_time_s: f64,
    /// Number of bridge packet-effect samples in this interval.
    pub sample_count: u64,
    /// Downlink sensor frames dropped in this interval.
    pub sensor_drop_count: u64,
    /// Uplink command frames dropped in this interval.
    pub command_drop_count: u64,
    /// Downlink sensor frames delivered corrupted in this interval.
    pub sensor_bit_flip_count: u64,
    /// Uplink command frames delivered corrupted in this interval.
    pub command_bit_flip_count: u64,
}

/// Runner evidence for one FC bridge link/gap handover event.
#[derive(Clone, Debug, PartialEq)]
pub struct CommBridgeHandoverReport {
    /// Bridge step where the new selection took effect.
    pub step: u64,
    /// Time of the latest comm observation when the new selection took effect.
    pub time_s: f64,
    /// Link selected before the handover, or `None` for a scheduled gap.
    pub from_link_id: Option<String>,
    /// Fallback link used before the handover when the previous state was a gap.
    pub from_fallback_link_id: Option<String>,
    /// Link selected after the handover, or `None` for a scheduled gap.
    pub to_link_id: Option<String>,
    /// Fallback link used after the handover when the new state is a gap.
    pub to_fallback_link_id: Option<String>,
}

impl CommBridgeHandoverReport {
    fn new(
        step: StepIndex,
        time_s: f64,
        from: &CommBridgeSelectionKey,
        to: &CommBridgeSelectionKey,
    ) -> Self {
        let (from_link_id, from_fallback_link_id) = from.report_ids();
        let (to_link_id, to_fallback_link_id) = to.report_ids();
        Self {
            step: step.value(),
            time_s,
            from_link_id,
            from_fallback_link_id,
            to_link_id,
            to_fallback_link_id,
        }
    }
}

/// Runner evidence for one declared comm packet loss-rate gate.
#[derive(Clone, Debug, PartialEq)]
pub struct CommPacketLossRateGateReport {
    /// Number of packets evaluated per direction.
    pub packet_count: u64,
    /// Two-sided significance level used for exact binomial intervals.
    pub alpha: f64,
    /// Step whose computed [`LinkState`] fed the gate.
    pub sample_step: u64,
    /// Downlink gate evidence.
    pub downlink: CommPacketLossRateDirectionReport,
    /// Uplink gate evidence.
    pub uplink: CommPacketLossRateDirectionReport,
}

/// Direction-specific packet loss-rate gate evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct CommPacketLossRateDirectionReport {
    /// Packet direction label.
    pub direction: String,
    /// Packets that were dropped or delivered corrupted.
    pub errored_packet_count: u64,
    /// Expected packet error rate from link state FER or no-link state.
    pub expected_error_rate: f64,
    /// Observed dropped-or-corrupted packet rate.
    pub observed_error_rate: f64,
    /// Lowest accepted errored-packet count.
    pub lower_accepted_errors: u64,
    /// Highest accepted errored-packet count.
    pub upper_accepted_errors: u64,
    /// Whether the observed count is inside the exact interval.
    pub passed: bool,
}

/// Runner evidence for one declared downlink data-loss timeout.
#[derive(Clone, Debug, PartialEq)]
pub struct CommDataLossTimeoutReport {
    /// Declared timeout in seconds.
    pub timeout_s: f64,
    /// Whether the timeout latched.
    pub triggered: bool,
    /// First lost downlink sample step, if any.
    pub first_loss_step: Option<u64>,
    /// First lost downlink sample time, seconds.
    pub first_loss_time_s: Option<f64>,
    /// First step where continuous loss exceeded `timeout_s`.
    pub first_trigger_step: Option<u64>,
    /// First time where continuous loss exceeded `timeout_s`.
    pub first_trigger_time_s: Option<f64>,
    /// Maximum continuous loss gap observed, seconds.
    pub max_loss_gap_s: f64,
    /// Count of delivered downlink samples.
    pub delivered_sample_count: u64,
    /// Count of lost downlink samples.
    pub lost_sample_count: u64,
}

/// Link-channel packet effects to apply at the FC bridge seam.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CommBridgePacketEffects {
    /// Link id whose packet model was evaluated.
    pub(crate) link_id: String,
    /// Effect for simulator-to-controller sensor frames.
    pub(crate) sensor: LinkPacketEffect,
    /// Effect for controller-to-simulator command frames.
    pub(crate) command: LinkPacketEffect,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommBridgeSelection {
    Link(usize),
    ScheduledGap { fallback_link_index: usize },
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum CommBridgeSelectionKey {
    Link(String),
    ScheduledGap { fallback_link_id: String },
}

impl CommBridgeSelectionKey {
    fn report_ids(&self) -> (Option<String>, Option<String>) {
        match self {
            Self::Link(link_id) => (Some(link_id.clone()), None),
            Self::ScheduledGap { fallback_link_id } => (None, Some(fallback_link_id.clone())),
        }
    }
}

/// Telemetry channels emitted for each declared communications site.
#[derive(Debug)]
pub(crate) struct CommTelemetryChannels {
    sites: Vec<CommSiteTelemetryChannels>,
    links: Vec<CommLinkTelemetryChannels>,
}

#[derive(Debug)]
struct CommSiteTelemetryChannels {
    site_id: String,
    slant_range: TelemetryChannel<f64>,
    range_rate: TelemetryChannel<f64>,
    elevation: TelemetryChannel<f64>,
    azimuth: TelemetryChannel<f64>,
    mask_elevation: TelemetryChannel<f64>,
    visible: TelemetryChannel<bool>,
}

#[derive(Debug)]
struct CommLinkTelemetryChannels {
    link_id: String,
    antenna_gain: TelemetryChannel<f64>,
    body_blocked: TelemetryChannel<bool>,
    free_space_loss: TelemetryChannel<f64>,
    c_n0: TelemetryChannel<f64>,
    eb_n0: TelemetryChannel<f64>,
    margin: TelemetryChannel<f64>,
    fer: TelemetryChannel<f64>,
    visible: TelemetryChannel<bool>,
    blackout: TelemetryChannel<bool>,
}

impl CommTelemetryChannels {
    /// Construct comm telemetry channels when `[comm]` is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when a channel cannot be built.
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        alloc: &mut impl FnMut() -> ChannelId,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(comm) = &document.comm else {
            return Ok(None);
        };
        let mut sites = Vec::with_capacity(comm.sites.len());
        for site in &comm.sites {
            let suffix = crate::telemetry_suffix(&site.id);
            let prefix = format!("comm.{suffix}");
            sites.push(CommSiteTelemetryChannels {
                site_id: site.id.clone(),
                slant_range: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.slant_range_m"),
                    "m",
                    None::<&str>,
                )?,
                range_rate: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.range_rate_m_s"),
                    "m/s",
                    None::<&str>,
                )?,
                elevation: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.elevation_rad"),
                    "rad",
                    None::<&str>,
                )?,
                azimuth: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.azimuth_rad"),
                    "rad",
                    None::<&str>,
                )?,
                mask_elevation: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.mask_elevation_rad"),
                    "rad",
                    None::<&str>,
                )?,
                visible: TelemetryChannel::<bool>::new(
                    alloc(),
                    format!("{prefix}.visible"),
                    "bool",
                    None::<&str>,
                )?,
            });
        }
        let mut links = Vec::with_capacity(comm.links.len());
        for link in &comm.links {
            let suffix = crate::telemetry_suffix(&link.id);
            let prefix = format!("comm.link.{suffix}");
            links.push(CommLinkTelemetryChannels {
                link_id: link.id.clone(),
                antenna_gain: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.antenna_gain_dbi"),
                    "dBi",
                    None::<&str>,
                )?,
                body_blocked: TelemetryChannel::<bool>::new(
                    alloc(),
                    format!("{prefix}.body_blocked"),
                    "bool",
                    None::<&str>,
                )?,
                free_space_loss: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.free_space_loss_db"),
                    "dB",
                    None::<&str>,
                )?,
                c_n0: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.c_n0_dbhz"),
                    "dB-Hz",
                    None::<&str>,
                )?,
                eb_n0: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.eb_n0_db"),
                    "dB",
                    None::<&str>,
                )?,
                margin: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.margin_db"),
                    "dB",
                    None::<&str>,
                )?,
                fer: TelemetryChannel::<f64>::new(
                    alloc(),
                    format!("{prefix}.fer"),
                    "1",
                    None::<&str>,
                )?,
                visible: TelemetryChannel::<bool>::new(
                    alloc(),
                    format!("{prefix}.visible"),
                    "bool",
                    None::<&str>,
                )?,
                blackout: TelemetryChannel::<bool>::new(
                    alloc(),
                    format!("{prefix}.blackout"),
                    "bool",
                    None::<&str>,
                )?,
            });
        }
        Ok(Some(Self { sites, links }))
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        for site in &self.sites {
            channels.push(site.slant_range.metadata().clone());
            channels.push(site.range_rate.metadata().clone());
            channels.push(site.elevation.metadata().clone());
            channels.push(site.azimuth.metadata().clone());
            channels.push(site.mask_elevation.metadata().clone());
            channels.push(site.visible.metadata().clone());
        }
        for link in &self.links {
            channels.push(link.antenna_gain.metadata().clone());
            channels.push(link.body_blocked.metadata().clone());
            channels.push(link.free_space_loss.metadata().clone());
            channels.push(link.c_n0.metadata().clone());
            channels.push(link.eb_n0.metadata().clone());
            channels.push(link.margin.metadata().clone());
            channels.push(link.fer.metadata().clone());
            channels.push(link.visible.metadata().clone());
            channels.push(link.blackout.metadata().clone());
        }
    }

    /// Insert one row of comm site observations.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when row insertion fails, or
    /// [`RunnerError::UnsupportedScenario`] if samples do not match channels.
    pub(crate) fn insert_samples(
        &self,
        row: &mut TelemetryRow,
        observation: &CommObservation,
    ) -> Result<(), RunnerError> {
        let samples = &observation.visibility_samples;
        if samples.len() != self.sites.len() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm telemetry expected {} site samples, got {}",
                    self.sites.len(),
                    samples.len()
                ),
            });
        }
        for (site, sample) in self.sites.iter().zip(samples) {
            if site.site_id != sample.site_id {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm telemetry sample site mismatch: expected {}, got {}",
                        site.site_id, sample.site_id
                    ),
                });
            }
            row.insert(&site.slant_range, sample.slant_range_m)?;
            row.insert(&site.range_rate, sample.range_rate_m_s)?;
            row.insert(&site.elevation, sample.elevation_rad)?;
            row.insert(&site.azimuth, sample.azimuth_rad)?;
            row.insert(&site.mask_elevation, sample.mask_elevation_rad)?;
            row.insert(&site.visible, sample.visible)?;
        }
        if observation.link_states.len() != self.links.len() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm telemetry expected {} link samples, got {}",
                    self.links.len(),
                    observation.link_states.len()
                ),
            });
        }
        for (link, sample) in self.links.iter().zip(&observation.link_states) {
            if link.link_id != sample.link_id {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm telemetry link sample mismatch: expected {}, got {}",
                        link.link_id, sample.link_id
                    ),
                });
            }
            row.insert(&link.antenna_gain, sample.antenna_gain_dbi)?;
            row.insert(&link.body_blocked, sample.body_blocked)?;
            row.insert(&link.free_space_loss, sample.state.free_space_loss_db)?;
            row.insert(&link.c_n0, sample.state.c_n0_dbhz)?;
            row.insert(&link.eb_n0, sample.state.eb_n0_db)?;
            row.insert(&link.margin, sample.state.margin_db)?;
            row.insert(&link.fer, sample.state.fer)?;
            row.insert(&link.visible, sample.state.visible)?;
            row.insert(&link.blackout, sample.state.blackout)?;
        }
        Ok(())
    }
}

/// One runner-side communications observation for a telemetry row.
#[derive(Clone, Debug)]
pub(crate) struct CommObservation {
    visibility_samples: Vec<VisibilitySample>,
    link_states: Vec<CommLinkStateSample>,
}

#[derive(Clone, Debug)]
struct CommLinkStateSample {
    link_id: String,
    slant_range_m: f64,
    antenna_gain_dbi: f64,
    body_blocked: bool,
    state: LinkState,
}

#[derive(Clone, Debug, PartialEq)]
struct CommLinkStateHistorySample {
    step: u64,
    time_s: f64,
    margin_db: f64,
    visible: bool,
    blackout: bool,
}

impl CommLinkStateHistorySample {
    fn usable(&self) -> bool {
        self.visible && !self.blackout
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CommBridgePacketReportSample {
    link_id: Option<String>,
    step: u64,
    sensor_drop_count: u64,
    command_drop_count: u64,
    sensor_bit_flip_count: u64,
    command_bit_flip_count: u64,
}

impl CommBridgePacketReportSample {
    fn new(
        step: StepIndex,
        _time_s: f64,
        key: &CommBridgeSelectionKey,
        sensor: &LinkPacketEffect,
        command: &LinkPacketEffect,
    ) -> Self {
        let (link_id, _) = key.report_ids();
        let (sensor_drop_count, sensor_bit_flip_count) =
            packet_disposition_counts(sensor.disposition);
        let (command_drop_count, command_bit_flip_count) =
            packet_disposition_counts(command.disposition);
        Self {
            link_id,
            step: step.value(),
            sensor_drop_count,
            command_drop_count,
            sensor_bit_flip_count,
            command_bit_flip_count,
        }
    }
}

/// Accumulates site visibility samples during a run.
#[derive(Clone, Debug)]
pub(crate) struct CommRunAccumulator {
    sites: Vec<GroundSite>,
    samples: Vec<Vec<VisibilitySample>>,
    antennas: BTreeMap<String, CommRuntimeAntenna>,
    antenna_decks: Vec<CommAntennaDeckReport>,
    links: Vec<CommRuntimeLink>,
    link_reports: Vec<CommLinkReport>,
    link_sample_history: Vec<Vec<CommLinkStateHistorySample>>,
    latest_observation: Option<CommObservation>,
    latest_observation_step: Option<StepIndex>,
    latest_observation_time_s: Option<f64>,
    data_loss_timeouts: Vec<Option<CommDataLossTimeoutState>>,
    bridge_link_selection: CommBridgeLinkSelectionConfig,
    bridge_link_hysteresis_db: f64,
    bridge_pass_plan: Vec<CommBridgePassPlan>,
    active_bridge_link_id: Option<String>,
    bridge_selection_reports: Vec<CommBridgeSelectionReport>,
    bridge_handover_reports: Vec<CommBridgeHandoverReport>,
    bridge_packet_samples: Vec<CommBridgePacketReportSample>,
    active_bridge_selection_report: Option<CommBridgeSelectionAccumulator>,
    scenario_seed: u64,
}

impl CommRunAccumulator {
    /// Construct the accumulator when `[comm]` is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] if a validated scenario
    /// still cannot be translated to runtime comm geometry.
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(comm) = &document.comm else {
            return Ok(None);
        };
        let sites = comm
            .sites
            .iter()
            .map(site_from_config)
            .collect::<Result<Vec<_>, _>>()?;
        let (antennas, antenna_decks) = load_antenna_decks(document, resolved_files)?;
        let relays = load_relays(document, resolved_files)?;
        let (links, link_reports) = load_links(document, resolved_files, &relays)?;
        let data_loss_timeouts: Vec<Option<CommDataLossTimeoutState>> = links
            .iter()
            .map(|link| link.data_loss_timeout_s.map(CommDataLossTimeoutState::new))
            .collect();
        Ok(Some(Self {
            samples: vec![Vec::new(); sites.len()],
            sites,
            antennas,
            antenna_decks,
            links,
            link_reports,
            link_sample_history: vec![Vec::new(); data_loss_timeouts.len()],
            latest_observation: None,
            latest_observation_step: None,
            latest_observation_time_s: None,
            data_loss_timeouts,
            bridge_link_selection: comm.bridge_link_selection,
            bridge_link_hysteresis_db: comm.bridge_link_hysteresis_db,
            bridge_pass_plan: comm
                .bridge_pass_plan
                .iter()
                .map(CommBridgePassPlan::from_config)
                .collect(),
            active_bridge_link_id: None,
            bridge_selection_reports: Vec::new(),
            bridge_handover_reports: Vec::new(),
            bridge_packet_samples: Vec::new(),
            active_bridge_selection_report: None,
            scenario_seed: document.time.seed,
        }))
    }

    /// Observe one primary-body state sample.
    pub(crate) fn observe_eci(
        &mut self,
        frame: &FrameContext,
        step: StepIndex,
        time: SimTime,
        position_eci: Position3<Eci>,
        velocity_eci: Velocity3<Eci>,
        body_to_eci: UnitQuaternion<f64>,
    ) -> Result<CommObservation, RunnerError> {
        let position_ecef = frame.eci_to_ecef_position(time, position_eci);
        let velocity_ecef = frame.eci_to_ecef_velocity(time, velocity_eci, position_eci);
        let mut visibility_samples = Vec::with_capacity(self.sites.len());
        for (site, samples) in self.sites.iter().zip(&mut self.samples) {
            let sample = site.observe_ecef(
                time.as_seconds(),
                position_ecef.vector,
                velocity_ecef.vector,
            );
            samples.push(sample.clone());
            visibility_samples.push(sample);
        }
        let mut link_states = Vec::with_capacity(self.links.len());
        for (link_index, link) in self.links.iter().enumerate() {
            let (site_index, visibility) = visibility_samples
                .iter()
                .enumerate()
                .find(|(_, sample)| sample.site_id == link.site_id)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm link `{}` references site `{}` with no visibility sample",
                        link.id, link.site_id
                    ),
                })?;
            let antenna = self.antennas.get(&link.antenna_id).ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm link `{}` references antenna `{}` with no runtime deck",
                        link.id, link.antenna_id
                    ),
                }
            })?;
            let link_sample = link
                .observe_state(CommLinkObservationContext {
                    frame,
                    time,
                    vehicle_ecef_m: position_ecef.vector,
                    body_to_eci,
                    site: &self.sites[site_index],
                    direct_visibility: visibility,
                    antenna,
                })
                .map_err(comm_error)?;
            let state = link_sample.state;
            if self.data_loss_timeouts[link_index].is_some() {
                let effect = link
                    .packet_effect(
                        state,
                        link_sample.slant_range_m,
                        self.scenario_seed,
                        step,
                        0,
                        LinkPacketDirection::Downlink,
                    )
                    .map_err(comm_error)?;
                if let Some(timeout) = &mut self.data_loss_timeouts[link_index] {
                    timeout.observe(
                        step,
                        time.as_seconds(),
                        matches!(effect.disposition, LinkPacketDisposition::Deliver),
                    );
                }
            }
            self.link_sample_history[link_index].push(CommLinkStateHistorySample {
                step: step.value(),
                time_s: time.as_seconds(),
                margin_db: state.margin_db,
                visible: state.visible,
                blackout: state.blackout,
            });
            link_states.push(link_sample);
        }
        let observation = CommObservation {
            visibility_samples,
            link_states,
        };
        self.latest_observation = Some(observation.clone());
        self.latest_observation_step = Some(step);
        self.latest_observation_time_s = Some(time.as_seconds());
        Ok(observation)
    }

    /// Evaluate packet effects for the FC bridge sensor/command exchange.
    ///
    /// The default binding preserves declaration-order behavior. Opt-in
    /// best-margin selection uses the current `LinkState` samples and stable
    /// declaration-order tie breaks, with optional margin hysteresis once a
    /// link has been selected.
    pub(crate) fn bridge_packet_effects(
        &mut self,
        scenario_seed: u64,
        step: StepIndex,
    ) -> Result<Option<CommBridgePacketEffects>, RunnerError> {
        let Some(observation) = self.latest_observation.clone() else {
            return Ok(None);
        };
        let Some(selection) = self.select_bridge_link(&observation) else {
            return Ok(None);
        };
        let (link_index, state, slant_range_m, selection_key) = match selection {
            CommBridgeSelection::Link(index) => {
                let link = &self.links[index];
                let Some(sample) = self.link_sample(&observation, link) else {
                    return Ok(None);
                };
                (
                    index,
                    sample.state,
                    sample.slant_range_m,
                    CommBridgeSelectionKey::Link(link.id.clone()),
                )
            }
            CommBridgeSelection::ScheduledGap {
                fallback_link_index,
            } => {
                let slant_range_m = self
                    .links
                    .get(fallback_link_index)
                    .and_then(|link| self.link_sample(&observation, link))
                    .map_or(1.0, |sample| sample.slant_range_m);
                (
                    fallback_link_index,
                    scheduled_gap_link_state(),
                    slant_range_m,
                    CommBridgeSelectionKey::ScheduledGap {
                        fallback_link_id: self.links[fallback_link_index].id.clone(),
                    },
                )
            }
        };
        let link = &self.links[link_index];
        let sensor = link
            .packet_effect(
                state,
                slant_range_m,
                scenario_seed,
                step,
                0,
                LinkPacketDirection::Downlink,
            )
            .map_err(comm_error)?;
        let command = link
            .packet_effect(
                state,
                slant_range_m,
                scenario_seed,
                step,
                0,
                LinkPacketDirection::Uplink,
            )
            .map_err(comm_error)?;
        let link_id = link.id.clone();
        self.record_bridge_selection(step, selection_key, &sensor, &command);
        Ok(Some(CommBridgePacketEffects {
            link_id,
            sensor,
            command,
        }))
    }

    #[cfg(test)]
    fn bridge_link_index(&mut self, observation: &CommObservation) -> Option<usize> {
        match self.select_bridge_link(observation)? {
            CommBridgeSelection::Link(index)
            | CommBridgeSelection::ScheduledGap {
                fallback_link_index: index,
            } => Some(index),
        }
    }

    fn select_bridge_link(&mut self, observation: &CommObservation) -> Option<CommBridgeSelection> {
        match self.bridge_link_selection {
            CommBridgeLinkSelectionConfig::FirstDeclared => self
                .first_declared_link_index(observation)
                .map(CommBridgeSelection::Link),
            CommBridgeLinkSelectionConfig::BestMargin => self
                .best_margin_bridge_link_index(observation)
                .map(CommBridgeSelection::Link),
            CommBridgeLinkSelectionConfig::DeclaredPlan => {
                self.declared_pass_plan_bridge_selection(observation)
            }
        }
    }

    fn first_declared_link_index(&self, observation: &CommObservation) -> Option<usize> {
        self.links
            .first()
            .and_then(|link| self.link_sample(observation, link).map(|_| 0))
    }

    fn best_margin_bridge_link_index(&mut self, observation: &CommObservation) -> Option<usize> {
        let Some(candidate_index) = self.best_margin_link_index(observation) else {
            let fallback_index = self.first_declared_link_index(observation)?;
            self.active_bridge_link_id = Some(self.links[fallback_index].id.clone());
            return Some(fallback_index);
        };
        let mut selected_index = candidate_index;
        let retained_active_index = if self.bridge_link_hysteresis_db > 0.0 {
            self.active_bridge_link_id
                .as_deref()
                .and_then(|active_id| self.link_index_by_id(active_id))
                .and_then(|active_index| {
                    if active_index == candidate_index {
                        return None;
                    }
                    let active_sample =
                        self.usable_link_sample_by_index(observation, active_index)?;
                    let candidate_sample =
                        self.usable_link_sample_by_index(observation, candidate_index)?;
                    (candidate_sample.state.margin_db
                        <= active_sample.state.margin_db + self.bridge_link_hysteresis_db)
                        .then_some(active_index)
                })
        } else {
            None
        };
        if let Some(active_index) = retained_active_index {
            selected_index = active_index;
        }
        self.active_bridge_link_id = Some(self.links[selected_index].id.clone());
        Some(selected_index)
    }

    fn declared_pass_plan_bridge_selection(
        &mut self,
        observation: &CommObservation,
    ) -> Option<CommBridgeSelection> {
        let fallback_link_index = self.first_declared_link_index(observation)?;
        let Some(time_s) = self.latest_observation_time_s else {
            self.active_bridge_link_id = None;
            return Some(CommBridgeSelection::ScheduledGap {
                fallback_link_index,
            });
        };
        let Some(plan) = self
            .bridge_pass_plan
            .iter()
            .find(|plan| plan.contains(time_s))
        else {
            self.active_bridge_link_id = None;
            return Some(CommBridgeSelection::ScheduledGap {
                fallback_link_index,
            });
        };
        let Some(link_index) = self.link_index_by_id(&plan.link_id) else {
            self.active_bridge_link_id = None;
            return Some(CommBridgeSelection::ScheduledGap {
                fallback_link_index,
            });
        };
        let link = &self.links[link_index];
        self.link_sample(observation, link)?;
        self.active_bridge_link_id = Some(link.id.clone());
        Some(CommBridgeSelection::Link(link_index))
    }

    fn best_margin_link_index(&self, observation: &CommObservation) -> Option<usize> {
        let mut best: Option<(usize, f64)> = None;
        for (index, link) in self.links.iter().enumerate() {
            let Some(sample) = self.link_sample(observation, link) else {
                continue;
            };
            if !sample.state.visible || sample.state.blackout {
                continue;
            }
            match best {
                Some((_, best_margin_db)) if sample.state.margin_db <= best_margin_db => {}
                _ => best = Some((index, sample.state.margin_db)),
            }
        }
        best.map(|(index, _)| index)
    }

    fn usable_link_sample_by_index<'a>(
        &'a self,
        observation: &'a CommObservation,
        index: usize,
    ) -> Option<&'a CommLinkStateSample> {
        let link = self.links.get(index)?;
        let sample = self.link_sample(observation, link)?;
        (sample.state.visible && !sample.state.blackout).then_some(sample)
    }

    fn link_index_by_id(&self, id: &str) -> Option<usize> {
        self.links.iter().position(|link| link.id == id)
    }

    fn link_sample<'a>(
        &'a self,
        observation: &'a CommObservation,
        link: &CommRuntimeLink,
    ) -> Option<&'a CommLinkStateSample> {
        observation
            .link_states
            .iter()
            .find(|sample| sample.link_id == link.id)
    }

    /// Assemble the final report.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::UnsupportedScenario`] when visibility samples
    /// cannot be converted into pass intervals or deterministic CSV bytes.
    pub(crate) fn finish(mut self) -> Result<CommRunReport, RunnerError> {
        let mut passes = Vec::new();
        for samples in &self.samples {
            passes.extend(pass_intervals(samples).map_err(comm_error)?);
        }
        let mut pass_table_csv = Vec::new();
        write_pass_intervals_csv(&passes, &mut pass_table_csv).map_err(|err| {
            RunnerError::UnsupportedScenario {
                what: format!("comm pass-table CSV serialization failed: {err}"),
            }
        })?;
        self.finish_bridge_selection_report();
        let link_passes = self.link_pass_reports();
        self.finish_data_loss_timeout_reports();
        self.evaluate_loss_rate_gates()?;
        Ok(CommRunReport {
            pass_table_csv,
            antenna_decks: self.antenna_decks,
            links: self.link_reports,
            link_passes,
            bridge_selections: self.bridge_selection_reports,
            bridge_handovers: self.bridge_handover_reports,
        })
    }

    fn record_bridge_selection(
        &mut self,
        step: StepIndex,
        key: CommBridgeSelectionKey,
        sensor: &LinkPacketEffect,
        command: &LinkPacketEffect,
    ) {
        let time_s = self.latest_observation_time_s.unwrap_or(0.0);
        self.bridge_packet_samples
            .push(CommBridgePacketReportSample::new(
                step, time_s, &key, sensor, command,
            ));
        if self
            .active_bridge_selection_report
            .as_ref()
            .is_some_and(|report| report.key == key)
        {
            if let Some(report) = &mut self.active_bridge_selection_report {
                report.observe(step, time_s, sensor, command);
            }
            return;
        }
        if let Some(previous) = self.active_bridge_selection_report.as_ref() {
            self.bridge_handover_reports
                .push(CommBridgeHandoverReport::new(
                    step,
                    time_s,
                    &previous.key,
                    &key,
                ));
        }
        self.finish_bridge_selection_report();
        let mut report = CommBridgeSelectionAccumulator::new(key, step, time_s);
        report.observe(step, time_s, sensor, command);
        self.active_bridge_selection_report = Some(report);
    }

    fn finish_bridge_selection_report(&mut self) {
        if let Some(report) = self.active_bridge_selection_report.take() {
            self.bridge_selection_reports.push(report.into_report());
        }
    }

    fn link_pass_reports(&self) -> Vec<CommLinkPassReport> {
        let mut reports = Vec::new();
        for (link_index, samples) in self.link_sample_history.iter().enumerate() {
            let Some(link) = self.links.get(link_index) else {
                continue;
            };
            let mut current: Option<CommLinkPassAccumulator> = None;
            let mut next_pass_index = 0_u64;
            for sample in samples {
                if sample.usable() {
                    if let Some(pass) = &mut current {
                        pass.observe(sample);
                    } else {
                        current = Some(CommLinkPassAccumulator::new(
                            link.id.clone(),
                            next_pass_index,
                            sample,
                        ));
                    }
                    continue;
                }
                if let Some(pass) = current.take() {
                    reports.push(pass.into_report(&self.bridge_packet_samples));
                    next_pass_index += 1;
                }
            }
            if let Some(pass) = current.take() {
                reports.push(pass.into_report(&self.bridge_packet_samples));
            }
        }
        reports
    }

    fn finish_data_loss_timeout_reports(&mut self) {
        for (report, timeout) in self
            .link_reports
            .iter_mut()
            .zip(self.data_loss_timeouts.iter())
        {
            report.data_loss_timeout = timeout.as_ref().map(CommDataLossTimeoutState::report);
        }
    }

    fn evaluate_loss_rate_gates(&mut self) -> Result<(), RunnerError> {
        if !self.links.iter().any(|link| link.loss_rate_gate.is_some()) {
            return Ok(());
        }
        let observation =
            self.latest_observation
                .as_ref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "comm packet loss-rate gate requires at least one link observation"
                        .to_owned(),
                })?;
        let step =
            self.latest_observation_step
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "comm packet loss-rate gate missing observation step".to_owned(),
                })?;
        for (index, link) in self.links.iter().enumerate() {
            let Some(gate) = link.loss_rate_gate else {
                continue;
            };
            let sample = observation
                .link_states
                .iter()
                .find(|sample| sample.link_id == link.id)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm packet loss-rate gate for link `{}` has no link-state sample",
                        link.id
                    ),
                })?;
            let report = link
                .packet_loss_rate_gate_report(
                    sample.state,
                    sample.slant_range_m,
                    self.scenario_seed,
                    step,
                    gate,
                )
                .map_err(comm_error)?;
            if !report.passed() {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm link `{}` packet loss-rate gate failed at step {}: downlink {}/{} in [{}, {}], uplink {}/{} in [{}, {}]",
                        link.id,
                        report.sample_step,
                        report.downlink.errored_packet_count,
                        report.packet_count,
                        report.downlink.lower_accepted_errors,
                        report.downlink.upper_accepted_errors,
                        report.uplink.errored_packet_count,
                        report.packet_count,
                        report.uplink.lower_accepted_errors,
                        report.uplink.upper_accepted_errors,
                    ),
                });
            }
            self.link_reports[index].packet_loss_rate_gate = Some(report);
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct CommRuntimeAntenna {
    gain_deck: Option<AntennaGainDeck>,
    body_mask_deck: Option<BodyMaskDeck>,
}

impl CommRuntimeAntenna {
    fn gain_dbi(&self, direction_body: Vector3<f64>) -> Result<f64, CommError> {
        self.gain_deck
            .as_ref()
            .map_or(Ok(0.0), |deck| deck.gain_dbi_for_direction(direction_body))
    }

    fn is_blocked(&self, direction_body: Vector3<f64>) -> Result<bool, CommError> {
        self.body_mask_deck
            .as_ref()
            .map_or(Ok(false), |deck| deck.is_blocked_nearest(direction_body))
    }
}

#[derive(Clone, Debug)]
struct CommRuntimeLink {
    id: String,
    site_id: String,
    antenna_id: String,
    relay: Option<CommRuntimeRelay>,
    budget_template: LinkBudgetInput,
    fer_curve: FerCurveDeck,
    channel_model: LinkChannelModel,
    link_stream_id: u64,
    loss_rate_gate: Option<CommPacketLossRateGateConfig>,
    data_loss_timeout_s: Option<f64>,
    atmospheric_loss_deck: Option<ElevationLossDeck>,
    rain_loss_deck: Option<ElevationLossDeck>,
}

struct CommLinkObservationContext<'a> {
    frame: &'a FrameContext,
    time: SimTime,
    vehicle_ecef_m: Vector3<f64>,
    body_to_eci: UnitQuaternion<f64>,
    site: &'a GroundSite,
    direct_visibility: &'a VisibilitySample,
    antenna: &'a CommRuntimeAntenna,
}

#[derive(Clone, Debug)]
struct CommRuntimeRelay {
    id: String,
    longitude_deg: f64,
    altitude_m: f64,
    min_elevation_deg: f64,
    geodetic: GeodeticPosition,
    budget_template: LinkBudgetInput,
    fer_curve: FerCurveDeck,
    fer_curve_deck_file_sha256_hex: String,
    atmospheric_loss_deck: Option<ElevationLossDeck>,
    atmospheric_loss_deck_file_sha256_hex: Option<String>,
    rain_loss_deck: Option<ElevationLossDeck>,
    rain_loss_deck_file_sha256_hex: Option<String>,
}

impl CommRuntimeLink {
    fn observe_state(
        &self,
        context: CommLinkObservationContext<'_>,
    ) -> Result<CommLinkStateSample, CommError> {
        let target_ecef_m = self
            .relay
            .as_ref()
            .map_or_else(|| context.site.ecef_m(), CommRuntimeRelay::ecef_m);
        let direction_body = vehicle_to_site_body_direction(
            context.frame,
            context.time,
            context.vehicle_ecef_m,
            target_ecef_m,
            context.body_to_eci,
        )?;
        let antenna_gain_dbi = context.antenna.gain_dbi(direction_body)?;
        let body_blocked = context.antenna.is_blocked(direction_body)?;
        if let Some(relay) = &self.relay {
            let vehicle_to_relay_range_m = (target_ecef_m - context.vehicle_ecef_m).norm();
            let vehicle_to_relay_visible =
                !body_blocked && !earth_occludes_segment(context.vehicle_ecef_m, target_ecef_m);
            let vehicle_to_relay = evaluate_link_budget(
                self.input_for_relay_uplink(
                    vehicle_to_relay_range_m,
                    antenna_gain_dbi,
                    vehicle_to_relay_visible,
                )?,
                &self.fer_curve,
            )?;
            let relay_visibility =
                relay.visibility_from_site(context.time.as_seconds(), context.site);
            let relay_to_ground = evaluate_link_budget(
                relay.input_for_visibility(&relay_visibility)?,
                &relay.fer_curve,
            )?;
            let state = compose_two_hop_link_state(vehicle_to_relay, relay_to_ground);
            return Ok(CommLinkStateSample {
                link_id: self.id.clone(),
                slant_range_m: vehicle_to_relay_range_m + relay_visibility.slant_range_m,
                antenna_gain_dbi,
                body_blocked,
                state,
            });
        }
        let input =
            self.input_for_visibility(context.direct_visibility, antenna_gain_dbi, body_blocked)?;
        let state = evaluate_link_budget(input, &self.fer_curve)?;
        Ok(CommLinkStateSample {
            link_id: self.id.clone(),
            slant_range_m: context.direct_visibility.slant_range_m,
            antenna_gain_dbi,
            body_blocked,
            state,
        })
    }

    fn input_for_visibility(
        &self,
        sample: &VisibilitySample,
        antenna_gain_dbi: f64,
        body_blocked: bool,
    ) -> Result<LinkBudgetInput, CommError> {
        let atmospheric_loss_db = self.budget_template.atmospheric_loss_db
            + self
                .atmospheric_loss_deck
                .as_ref()
                .map(|deck| deck.loss_db_at_elevation(sample.elevation_rad))
                .transpose()?
                .unwrap_or(0.0);
        let rain_loss_db = self.budget_template.rain_loss_db
            + self
                .rain_loss_deck
                .as_ref()
                .map(|deck| deck.loss_db_at_elevation(sample.elevation_rad))
                .transpose()?
                .unwrap_or(0.0);
        Ok(LinkBudgetInput {
            eirp_dbw: self.budget_template.eirp_dbw + antenna_gain_dbi,
            slant_range_m: sample.slant_range_m,
            atmospheric_loss_db,
            rain_loss_db,
            visible: sample.visible && !body_blocked,
            blackout: false,
            ..self.budget_template
        })
    }

    fn input_for_relay_uplink(
        &self,
        slant_range_m: f64,
        antenna_gain_dbi: f64,
        visible: bool,
    ) -> Result<LinkBudgetInput, CommError> {
        require_positive_comm_runner("comm.relay.vehicle_to_relay.slant_range_m", slant_range_m)?;
        Ok(LinkBudgetInput {
            eirp_dbw: self.budget_template.eirp_dbw + antenna_gain_dbi,
            slant_range_m,
            visible,
            blackout: false,
            ..self.budget_template
        })
    }

    fn packet_effect(
        &self,
        state: LinkState,
        slant_range_m: f64,
        scenario_seed: u64,
        step: StepIndex,
        packet_index: u64,
        direction: LinkPacketDirection,
    ) -> Result<LinkPacketEffect, CommError> {
        self.channel_model.evaluate_packet(
            state,
            LinkPacketContext {
                scenario_seed,
                step: step.value(),
                link_stream_id: self.link_stream_id,
                packet_index,
                direction,
                slant_range_m,
            },
        )
    }

    fn packet_loss_rate_gate_report(
        &self,
        state: LinkState,
        slant_range_m: f64,
        scenario_seed: u64,
        step: StepIndex,
        gate: CommPacketLossRateGateConfig,
    ) -> Result<CommPacketLossRateGateReport, CommError> {
        let downlink = self.loss_rate_direction_report(
            state,
            slant_range_m,
            scenario_seed,
            step,
            gate,
            LinkPacketDirection::Downlink,
        )?;
        let uplink = self.loss_rate_direction_report(
            state,
            slant_range_m,
            scenario_seed,
            step,
            gate,
            LinkPacketDirection::Uplink,
        )?;
        Ok(CommPacketLossRateGateReport {
            packet_count: gate.packet_count,
            alpha: gate.alpha,
            sample_step: step.value(),
            downlink,
            uplink,
        })
    }

    fn loss_rate_direction_report(
        &self,
        state: LinkState,
        slant_range_m: f64,
        scenario_seed: u64,
        step: StepIndex,
        gate: CommPacketLossRateGateConfig,
        direction: LinkPacketDirection,
    ) -> Result<CommPacketLossRateDirectionReport, CommError> {
        let result = evaluate_link_loss_rate_gate(
            self.channel_model,
            state,
            LinkPacketContext {
                scenario_seed,
                step: step.value(),
                link_stream_id: self.link_stream_id,
                packet_index: 0,
                direction,
                slant_range_m,
            },
            gate.packet_count,
            gate.alpha,
        )?;
        Ok(CommPacketLossRateDirectionReport {
            direction: link_packet_direction_label(direction).to_owned(),
            errored_packet_count: result.errored_packet_count,
            expected_error_rate: result.expected_error_rate,
            observed_error_rate: result.observed_error_rate,
            lower_accepted_errors: result.lower_accepted_errors,
            upper_accepted_errors: result.upper_accepted_errors,
            passed: result.passed(),
        })
    }
}

impl CommRuntimeRelay {
    fn ecef_m(&self) -> Vector3<f64> {
        self.geodetic.to_ecef_m()
    }

    fn visibility_from_site(&self, time_s: f64, site: &GroundSite) -> VisibilitySample {
        let mut sample = site.observe_ecef(time_s, self.ecef_m(), Vector3::zeros());
        let relay_min_elevation_rad = self.min_elevation_deg.to_radians();
        sample.mask_elevation_rad = sample.mask_elevation_rad.max(relay_min_elevation_rad);
        sample.visible = sample.elevation_rad >= sample.mask_elevation_rad;
        sample
    }

    fn input_for_visibility(
        &self,
        sample: &VisibilitySample,
    ) -> Result<LinkBudgetInput, CommError> {
        let atmospheric_loss_db = self.budget_template.atmospheric_loss_db
            + self
                .atmospheric_loss_deck
                .as_ref()
                .map(|deck| deck.loss_db_at_elevation(sample.elevation_rad))
                .transpose()?
                .unwrap_or(0.0);
        let rain_loss_db = self.budget_template.rain_loss_db
            + self
                .rain_loss_deck
                .as_ref()
                .map(|deck| deck.loss_db_at_elevation(sample.elevation_rad))
                .transpose()?
                .unwrap_or(0.0);
        Ok(LinkBudgetInput {
            slant_range_m: sample.slant_range_m,
            atmospheric_loss_db,
            rain_loss_db,
            visible: sample.visible,
            blackout: false,
            ..self.budget_template
        })
    }
}

impl CommPacketLossRateGateReport {
    fn passed(&self) -> bool {
        self.downlink.passed && self.uplink.passed
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CommBridgePassPlan {
    link_id: String,
    start_s: f64,
    end_s: f64,
}

impl CommBridgePassPlan {
    fn from_config(config: &CommBridgePassPlanConfig) -> Self {
        Self {
            link_id: config.link_id.clone(),
            start_s: config.start_s,
            end_s: config.end_s,
        }
    }

    fn contains(&self, time_s: f64) -> bool {
        time_s >= self.start_s && time_s < self.end_s
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CommLinkPassAccumulator {
    link_id: String,
    pass_index: u64,
    start_step: u64,
    end_step: u64,
    start_time_s: f64,
    end_time_s: f64,
    sample_count: u64,
    min_margin_db: f64,
    max_margin_db: f64,
    margin_sum_db: f64,
    margin_profile: Vec<CommLinkPassMarginSampleReport>,
}

impl CommLinkPassAccumulator {
    fn new(link_id: String, pass_index: u64, sample: &CommLinkStateHistorySample) -> Self {
        Self {
            link_id,
            pass_index,
            start_step: sample.step,
            end_step: sample.step,
            start_time_s: sample.time_s,
            end_time_s: sample.time_s,
            sample_count: 1,
            min_margin_db: sample.margin_db,
            max_margin_db: sample.margin_db,
            margin_sum_db: sample.margin_db,
            margin_profile: vec![CommLinkPassMarginSampleReport {
                step: sample.step,
                time_s: sample.time_s,
                margin_db: sample.margin_db,
            }],
        }
    }

    fn observe(&mut self, sample: &CommLinkStateHistorySample) {
        self.end_step = sample.step;
        self.end_time_s = sample.time_s;
        self.sample_count += 1;
        self.min_margin_db = self.min_margin_db.min(sample.margin_db);
        self.max_margin_db = self.max_margin_db.max(sample.margin_db);
        self.margin_sum_db += sample.margin_db;
        self.margin_profile.push(CommLinkPassMarginSampleReport {
            step: sample.step,
            time_s: sample.time_s,
            margin_db: sample.margin_db,
        });
    }

    fn into_report(self, bridge_samples: &[CommBridgePacketReportSample]) -> CommLinkPassReport {
        let mut report = CommLinkPassReport {
            link_id: self.link_id.clone(),
            pass_index: self.pass_index,
            start_step: self.start_step,
            end_step: self.end_step,
            start_time_s: self.start_time_s,
            end_time_s: self.end_time_s,
            duration_s: (self.end_time_s - self.start_time_s).max(0.0),
            sample_count: self.sample_count,
            min_margin_db: self.min_margin_db,
            max_margin_db: self.max_margin_db,
            mean_margin_db: self.margin_sum_db / self.sample_count as f64,
            margin_profile: self.margin_profile,
            bridge_sample_count: 0,
            sensor_drop_count: 0,
            command_drop_count: 0,
            sensor_bit_flip_count: 0,
            command_bit_flip_count: 0,
        };
        for sample in bridge_samples {
            if sample.link_id.as_deref() != Some(report.link_id.as_str()) {
                continue;
            }
            if sample.step < report.start_step || sample.step > report.end_step {
                continue;
            }
            report.bridge_sample_count += 1;
            report.sensor_drop_count += sample.sensor_drop_count;
            report.command_drop_count += sample.command_drop_count;
            report.sensor_bit_flip_count += sample.sensor_bit_flip_count;
            report.command_bit_flip_count += sample.command_bit_flip_count;
        }
        report
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CommBridgeSelectionAccumulator {
    key: CommBridgeSelectionKey,
    report: CommBridgeSelectionReport,
}

impl CommBridgeSelectionAccumulator {
    fn new(key: CommBridgeSelectionKey, step: StepIndex, time_s: f64) -> Self {
        let (link_id, fallback_link_id) = match &key {
            CommBridgeSelectionKey::Link(link_id) => (Some(link_id.clone()), None),
            CommBridgeSelectionKey::ScheduledGap { fallback_link_id } => {
                (None, Some(fallback_link_id.clone()))
            }
        };
        Self {
            key,
            report: CommBridgeSelectionReport {
                link_id,
                fallback_link_id,
                start_step: step.value(),
                end_step: step.value(),
                start_time_s: time_s,
                end_time_s: time_s,
                sample_count: 0,
                sensor_drop_count: 0,
                command_drop_count: 0,
                sensor_bit_flip_count: 0,
                command_bit_flip_count: 0,
            },
        }
    }

    fn observe(
        &mut self,
        step: StepIndex,
        time_s: f64,
        sensor: &LinkPacketEffect,
        command: &LinkPacketEffect,
    ) {
        self.report.end_step = step.value();
        self.report.end_time_s = time_s;
        self.report.sample_count += 1;
        accumulate_packet_disposition(
            sensor.disposition,
            &mut self.report.sensor_drop_count,
            &mut self.report.sensor_bit_flip_count,
        );
        accumulate_packet_disposition(
            command.disposition,
            &mut self.report.command_drop_count,
            &mut self.report.command_bit_flip_count,
        );
    }

    fn into_report(self) -> CommBridgeSelectionReport {
        self.report
    }
}

fn accumulate_packet_disposition(
    disposition: LinkPacketDisposition,
    drop_count: &mut u64,
    bit_flip_count: &mut u64,
) {
    let (drops, bit_flips) = packet_disposition_counts(disposition);
    *drop_count += drops;
    *bit_flip_count += bit_flips;
}

fn packet_disposition_counts(disposition: LinkPacketDisposition) -> (u64, u64) {
    match disposition {
        LinkPacketDisposition::Deliver => (0, 0),
        LinkPacketDisposition::Drop => (1, 0),
        LinkPacketDisposition::BitFlip { .. } => (0, 1),
    }
}

#[derive(Clone, Debug)]
struct CommDataLossTimeoutState {
    report: CommDataLossTimeoutReport,
    current_loss_start_step: Option<u64>,
    current_loss_start_time_s: Option<f64>,
}

impl CommDataLossTimeoutState {
    fn new(timeout_s: f64) -> Self {
        Self {
            report: CommDataLossTimeoutReport {
                timeout_s,
                triggered: false,
                first_loss_step: None,
                first_loss_time_s: None,
                first_trigger_step: None,
                first_trigger_time_s: None,
                max_loss_gap_s: 0.0,
                delivered_sample_count: 0,
                lost_sample_count: 0,
            },
            current_loss_start_step: None,
            current_loss_start_time_s: None,
        }
    }

    fn observe(&mut self, step: StepIndex, time_s: f64, delivered: bool) {
        if delivered {
            self.report.delivered_sample_count += 1;
            self.current_loss_start_step = None;
            self.current_loss_start_time_s = None;
            return;
        }
        self.report.lost_sample_count += 1;
        if self.current_loss_start_time_s.is_none() {
            self.current_loss_start_step = Some(step.value());
            self.current_loss_start_time_s = Some(time_s);
        }
        if self.report.first_loss_time_s.is_none() {
            self.report.first_loss_step = Some(step.value());
            self.report.first_loss_time_s = Some(time_s);
        }
        let Some(loss_start_time_s) = self.current_loss_start_time_s else {
            return;
        };
        let loss_gap_s = (time_s - loss_start_time_s).max(0.0);
        self.report.max_loss_gap_s = self.report.max_loss_gap_s.max(loss_gap_s);
        if !self.report.triggered && loss_gap_s > self.report.timeout_s {
            self.report.triggered = true;
            self.report.first_trigger_step = Some(step.value());
            self.report.first_trigger_time_s = Some(time_s);
        }
    }

    fn report(&self) -> CommDataLossTimeoutReport {
        self.report.clone()
    }
}

fn vehicle_to_site_body_direction(
    frame: &FrameContext,
    time: SimTime,
    vehicle_ecef_m: Vector3<f64>,
    site_ecef_m: Vector3<f64>,
    body_to_eci: UnitQuaternion<f64>,
) -> Result<Vector3<f64>, CommError> {
    let direction_ecef = site_ecef_m - vehicle_ecef_m;
    if !direction_ecef.iter().all(|value| value.is_finite()) {
        return Err(CommError::NonFinite {
            field: "comm.link.direction_ecef",
        });
    }
    let direction_eci = frame.ecef_to_eci_vector(time, direction_ecef);
    let direction_body = body_to_eci.inverse() * direction_eci;
    if !direction_body.iter().all(|value| value.is_finite()) {
        return Err(CommError::NonFinite {
            field: "comm.link.direction_body",
        });
    }
    let norm = direction_body.norm();
    if norm <= f64::EPSILON {
        return Err(CommError::Degenerate {
            field: "comm.link.direction_body",
        });
    }
    Ok(direction_body / norm)
}

fn earth_occludes_segment(start_ecef_m: Vector3<f64>, end_ecef_m: Vector3<f64>) -> bool {
    let segment = end_ecef_m - start_ecef_m;
    let segment_norm_squared = segment.norm_squared();
    if segment_norm_squared <= f64::EPSILON {
        return true;
    }
    let closest_t = (-start_ecef_m.dot(&segment) / segment_norm_squared).clamp(0.0, 1.0);
    if closest_t <= 1.0e-9 || closest_t >= 1.0 - 1.0e-9 {
        return false;
    }
    (start_ecef_m + closest_t * segment).norm() < WGS84_A_M
}

fn compose_two_hop_link_state(
    vehicle_to_relay: LinkState,
    relay_to_ground: LinkState,
) -> LinkState {
    let visible = vehicle_to_relay.visible && relay_to_ground.visible;
    let blackout = vehicle_to_relay.blackout || relay_to_ground.blackout;
    let fer = if visible && !blackout {
        1.0 - (1.0 - vehicle_to_relay.fer) * (1.0 - relay_to_ground.fer)
    } else {
        1.0
    };
    LinkState {
        free_space_loss_db: vehicle_to_relay.free_space_loss_db
            + relay_to_ground.free_space_loss_db,
        c_n0_dbhz: vehicle_to_relay.c_n0_dbhz.min(relay_to_ground.c_n0_dbhz),
        eb_n0_db: vehicle_to_relay.eb_n0_db.min(relay_to_ground.eb_n0_db),
        margin_db: vehicle_to_relay.margin_db.min(relay_to_ground.margin_db),
        fer: fer.clamp(0.0, 1.0),
        visible,
        blackout,
    }
}

fn require_positive_comm_runner(field: &'static str, value: f64) -> Result<(), CommError> {
    if !value.is_finite() {
        return Err(CommError::NonFinite { field });
    }
    if value <= 0.0 {
        return Err(CommError::OutOfRange {
            field,
            value,
            rule: "must be positive",
        });
    }
    Ok(())
}

fn load_relays(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BTreeMap<String, CommRuntimeRelay>, RunnerError> {
    let Some(comm) = &document.comm else {
        return Ok(BTreeMap::new());
    };
    let mut relays = BTreeMap::new();
    for (index, relay) in comm.relays.iter().enumerate() {
        let key = format!("comm.relays[{index}].fer_curve_deck");
        let resolved = resolved_comm_file(&key, resolved_files)?;
        let fer_curve = load_fer_curve_deck(&key, resolved)?;
        let fer_curve_deck_file_sha256_hex = resolved.sha256_hex.clone();
        let atmospheric_loss_deck = if relay.atmospheric_loss_deck.is_some() {
            let key = format!("comm.relays[{index}].atmospheric_loss_deck");
            let resolved = resolved_comm_file(&key, resolved_files)?;
            Some((
                resolved.sha256_hex.clone(),
                load_elevation_loss_deck(&key, resolved, "atmospheric_loss")?,
            ))
        } else {
            None
        };
        let rain_loss_deck = if relay.rain_loss_deck.is_some() {
            let key = format!("comm.relays[{index}].rain_loss_deck");
            let resolved = resolved_comm_file(&key, resolved_files)?;
            Some((
                resolved.sha256_hex.clone(),
                load_elevation_loss_deck(&key, resolved, "rain_loss")?,
            ))
        } else {
            None
        };
        let geodetic =
            GeodeticPosition::new(0.0, relay.longitude_deg.to_radians(), relay.altitude_m)
                .map_err(comm_error)?;
        relays.insert(
            relay.id.clone(),
            CommRuntimeRelay {
                id: relay.id.clone(),
                longitude_deg: relay.longitude_deg,
                altitude_m: relay.altitude_m,
                min_elevation_deg: relay.min_elevation_deg,
                geodetic,
                budget_template: relay_budget_template(relay),
                fer_curve,
                fer_curve_deck_file_sha256_hex,
                atmospheric_loss_deck_file_sha256_hex: atmospheric_loss_deck
                    .as_ref()
                    .map(|(sha256, _)| sha256.clone()),
                atmospheric_loss_deck: atmospheric_loss_deck.map(|(_, deck)| deck),
                rain_loss_deck_file_sha256_hex: rain_loss_deck
                    .as_ref()
                    .map(|(sha256, _)| sha256.clone()),
                rain_loss_deck: rain_loss_deck.map(|(_, deck)| deck),
            },
        );
    }
    Ok(relays)
}

fn load_links(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    relays: &BTreeMap<String, CommRuntimeRelay>,
) -> Result<(Vec<CommRuntimeLink>, Vec<CommLinkReport>), RunnerError> {
    let Some(comm) = &document.comm else {
        return Ok((Vec::new(), Vec::new()));
    };
    let mut links = Vec::with_capacity(comm.links.len());
    let mut reports = Vec::with_capacity(comm.links.len());
    for (index, link) in comm.links.iter().enumerate() {
        let key = format!("comm.links[{index}].fer_curve_deck");
        let resolved = resolved_comm_file(&key, resolved_files)?;
        let fer_curve = load_fer_curve_deck(&key, resolved)?;
        let atmospheric_loss_deck = if link.atmospheric_loss_deck.is_some() {
            let key = format!("comm.links[{index}].atmospheric_loss_deck");
            let resolved = resolved_comm_file(&key, resolved_files)?;
            Some((
                resolved.sha256_hex.clone(),
                load_elevation_loss_deck(&key, resolved, "atmospheric_loss")?,
            ))
        } else {
            None
        };
        let rain_loss_deck = if link.rain_loss_deck.is_some() {
            let key = format!("comm.links[{index}].rain_loss_deck");
            let resolved = resolved_comm_file(&key, resolved_files)?;
            Some((
                resolved.sha256_hex.clone(),
                load_elevation_loss_deck(&key, resolved, "rain_loss")?,
            ))
        } else {
            None
        };
        let channel_model = link_channel_model(link).map_err(comm_error)?;
        let link_stream_id = link_stream_id_from_id(&link.id).map_err(comm_error)?;
        let relay = match link.relay_id.as_deref() {
            Some(relay_id) => Some(relays.get(relay_id).cloned().ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: format!(
                        "comm link `{}` references missing relay `{relay_id}`",
                        link.id
                    ),
                }
            })?),
            None => None,
        };
        reports.push(CommLinkReport {
            link_id: link.id.clone(),
            site_id: link.site_id.clone(),
            antenna_id: link.antenna_id.clone(),
            fer_curve_deck_file_sha256_hex: resolved.sha256_hex.clone(),
            fer_curve_code_id: fer_curve.code_id().to_owned(),
            fer_curve_sample_count: fer_curve.samples().len(),
            atmospheric_loss_deck_file_sha256_hex: atmospheric_loss_deck
                .as_ref()
                .map(|(sha256, _)| sha256.clone()),
            atmospheric_loss_sample_count: atmospheric_loss_deck
                .as_ref()
                .map_or(0, |(_, deck)| deck.samples().len()),
            rain_loss_deck_file_sha256_hex: rain_loss_deck
                .as_ref()
                .map(|(sha256, _)| sha256.clone()),
            rain_loss_sample_count: rain_loss_deck
                .as_ref()
                .map_or(0, |(_, deck)| deck.samples().len()),
            packet_processing_delay_s: channel_model.processing_delay_s(),
            packet_error_action: packet_error_action_label(channel_model.error_action()).to_owned(),
            packet_error_bit_flip_mask: packet_error_bit_flip_mask(channel_model.error_action()),
            packet_loss_rate_gate: None,
            data_loss_timeout: None,
            relay: relay.as_ref().map(|relay| CommRelayReport {
                relay_id: relay.id.clone(),
                longitude_deg: relay.longitude_deg,
                altitude_m: relay.altitude_m,
                min_elevation_deg: relay.min_elevation_deg,
                fer_curve_deck_file_sha256_hex: relay.fer_curve_deck_file_sha256_hex.clone(),
                fer_curve_code_id: relay.fer_curve.code_id().to_owned(),
                fer_curve_sample_count: relay.fer_curve.samples().len(),
                atmospheric_loss_deck_file_sha256_hex: relay
                    .atmospheric_loss_deck_file_sha256_hex
                    .clone(),
                atmospheric_loss_sample_count: relay
                    .atmospheric_loss_deck
                    .as_ref()
                    .map_or(0, |deck| deck.samples().len()),
                rain_loss_deck_file_sha256_hex: relay.rain_loss_deck_file_sha256_hex.clone(),
                rain_loss_sample_count: relay
                    .rain_loss_deck
                    .as_ref()
                    .map_or(0, |deck| deck.samples().len()),
            }),
        });
        links.push(CommRuntimeLink {
            id: link.id.clone(),
            site_id: link.site_id.clone(),
            antenna_id: link.antenna_id.clone(),
            relay,
            budget_template: link_budget_template(link),
            fer_curve,
            channel_model,
            link_stream_id,
            loss_rate_gate: link.packet_loss_rate_gate,
            data_loss_timeout_s: link.data_loss_timeout_s,
            atmospheric_loss_deck: atmospheric_loss_deck.map(|(_, deck)| deck),
            rain_loss_deck: rain_loss_deck.map(|(_, deck)| deck),
        });
    }
    Ok((links, reports))
}

fn link_budget_template(link: &CommLinkConfig) -> LinkBudgetInput {
    LinkBudgetInput {
        eirp_dbw: link.eirp_dbw,
        receiver_g_over_t_db_k: link.receiver_g_over_t_db_k,
        slant_range_m: 1.0,
        frequency_hz: link.frequency_hz,
        bit_rate_bps: link.bit_rate_bps,
        required_eb_n0_db: link.required_eb_n0_db,
        atmospheric_loss_db: link.atmospheric_loss_db,
        rain_loss_db: link.rain_loss_db,
        pointing_loss_db: link.pointing_loss_db,
        polarization_loss_db: link.polarization_loss_db,
        implementation_loss_db: link.implementation_loss_db,
        visible: true,
        blackout: false,
    }
}

fn relay_budget_template(relay: &CommRelayConfig) -> LinkBudgetInput {
    LinkBudgetInput {
        eirp_dbw: relay.eirp_dbw,
        receiver_g_over_t_db_k: relay.receiver_g_over_t_db_k,
        slant_range_m: 1.0,
        frequency_hz: relay.frequency_hz,
        bit_rate_bps: relay.bit_rate_bps,
        required_eb_n0_db: relay.required_eb_n0_db,
        atmospheric_loss_db: relay.atmospheric_loss_db,
        rain_loss_db: relay.rain_loss_db,
        pointing_loss_db: relay.pointing_loss_db,
        polarization_loss_db: relay.polarization_loss_db,
        implementation_loss_db: relay.implementation_loss_db,
        visible: true,
        blackout: false,
    }
}

fn link_channel_model(link: &CommLinkConfig) -> Result<LinkChannelModel, CommError> {
    LinkChannelModel::new(
        link.packet_processing_delay_s,
        match link.packet_error_action {
            CommPacketErrorActionConfig::Drop => LinkErrorAction::Drop,
            CommPacketErrorActionConfig::BitFlip { mask } => LinkErrorAction::BitFlip { mask },
        },
    )
}

fn packet_error_action_label(action: LinkErrorAction) -> &'static str {
    match action {
        LinkErrorAction::Drop => "drop",
        LinkErrorAction::BitFlip { .. } => "bit_flip",
    }
}

fn packet_error_bit_flip_mask(action: LinkErrorAction) -> Option<u8> {
    match action {
        LinkErrorAction::Drop => None,
        LinkErrorAction::BitFlip { mask } => Some(mask),
    }
}

fn link_packet_direction_label(direction: LinkPacketDirection) -> &'static str {
    match direction {
        LinkPacketDirection::Downlink => "downlink",
        LinkPacketDirection::Uplink => "uplink",
    }
}

fn load_antenna_decks(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<
    (
        BTreeMap<String, CommRuntimeAntenna>,
        Vec<CommAntennaDeckReport>,
    ),
    RunnerError,
> {
    let Some(comm) = &document.comm else {
        return Ok((BTreeMap::new(), Vec::new()));
    };
    let mut runtime = BTreeMap::new();
    let mut reports = Vec::with_capacity(comm.antennas.len());
    for (index, antenna) in comm.antennas.iter().enumerate() {
        let (gain_deck_file_sha256_hex, gain_deck) = if antenna.gain_deck.is_some() {
            let key = format!("comm.antennas[{index}].gain_deck");
            let resolved = resolved_comm_file(&key, resolved_files)?;
            let deck = load_gain_deck(&key, resolved)?;
            (Some(resolved.sha256_hex.clone()), Some(deck))
        } else {
            (None, None)
        };
        let (body_mask_deck_file_sha256_hex, body_mask_deck, body_mask_derived_sha256_hex) =
            if antenna.body_mask_deck.is_some() {
                let key = format!("comm.antennas[{index}].body_mask_deck");
                let resolved = resolved_comm_file(&key, resolved_files)?;
                let loaded = load_body_mask_deck(&key, resolved)?;
                (
                    Some(resolved.sha256_hex.clone()),
                    Some(loaded.deck),
                    Some(loaded.derived_sha256_hex),
                )
            } else {
                (None, None, None)
            };
        let gain_sample_count = gain_deck.as_ref().map_or(0, |deck| deck.samples().len());
        let body_mask_sample_count = body_mask_deck
            .as_ref()
            .map_or(0, |deck| deck.samples().len());
        reports.push(CommAntennaDeckReport {
            antenna_id: antenna.id.clone(),
            gain_deck_file_sha256_hex,
            gain_sample_count,
            body_mask_deck_file_sha256_hex,
            body_mask_sample_count,
            body_mask_derived_sha256_hex,
        });
        runtime.insert(
            antenna.id.clone(),
            CommRuntimeAntenna {
                gain_deck,
                body_mask_deck,
            },
        );
    }
    Ok((runtime, reports))
}

fn resolved_comm_file<'a>(
    key: &str,
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
) -> Result<&'a ResolvedFile, RunnerError> {
    resolved_files
        .get(key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm input `{key}` was not resolved"),
        })
}

fn load_gain_deck(key: &str, resolved: &ResolvedFile) -> Result<AntennaGainDeck, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let antenna = file
        .antenna
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` missing [antenna] table"),
        })?;
    let samples = antenna
        .samples
        .into_iter()
        .map(|sample| AntennaGainSample {
            off_boresight_rad: sample.off_boresight_deg.to_radians(),
            gain_dbi: sample.gain_dbi,
        })
        .collect();
    AntennaGainDeck::new(vector3(antenna.boresight_body), samples).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` rejected gain deck: {err}"),
        }
    })
}

fn load_body_mask_deck(
    key: &str,
    resolved: &ResolvedFile,
) -> Result<LoadedBodyMaskDeck, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let body_mask = file
        .body_mask
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` missing [body_mask] table"),
        })?;
    if !is_sha256_hex(&body_mask.mesh_sha256) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "comm antenna deck `{key}` body_mask.mesh_sha256 is not a SHA-256 hex digest"
            ),
        });
    }
    let directions = body_mask
        .directions
        .into_iter()
        .map(|direction| vector3(direction.direction_body))
        .collect::<Vec<_>>();
    let triangles = body_mask
        .triangles
        .into_iter()
        .map(|triangle| BodyMaskTriangle {
            a_m: vector3(triangle.a_m),
            b_m: vector3(triangle.b_m),
            c_m: vector3(triangle.c_m),
        })
        .collect::<Vec<_>>();
    let deck = BodyMaskDeck::precompute_from_triangles(
        vector3(body_mask.antenna_position_body_m),
        &directions,
        &triangles,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm antenna deck `{key}` rejected body-mask deck: {err}"),
    })?;
    let derived_sha256_hex = deck.sha256_hex_with_mesh_hash(&body_mask.mesh_sha256);
    Ok(LoadedBodyMaskDeck {
        deck,
        derived_sha256_hex,
    })
}

fn load_fer_curve_deck(key: &str, resolved: &ResolvedFile) -> Result<FerCurveDeck, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let curve = file
        .fer_curve
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm link deck `{key}` missing [fer_curve] table"),
        })?;
    let samples = curve
        .samples
        .into_iter()
        .map(|sample| FerCurveSample {
            eb_n0_db: sample.eb_n0_db,
            fer: sample.fer,
        })
        .collect();
    FerCurveDeck::new(curve.code_id, samples).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm link deck `{key}` rejected FER curve: {err}"),
    })
}

fn load_elevation_loss_deck(
    key: &str,
    resolved: &ResolvedFile,
    table_name: &'static str,
) -> Result<ElevationLossDeck, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let deck = match table_name {
        "atmospheric_loss" => file.atmospheric_loss,
        "rain_loss" => file.rain_loss,
        _ => None,
    }
    .ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("comm link deck `{key}` missing [{table_name}] table"),
    })?;
    let samples = deck
        .samples
        .into_iter()
        .map(|sample| ElevationLossSample {
            elevation_rad: sample.elevation_deg.to_radians(),
            loss_db: sample.loss_db,
        })
        .collect();
    ElevationLossDeck::new(deck.deck_id, samples).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm link deck `{key}` rejected elevation-loss deck: {err}"),
    })
}

fn parse_comm_deck_file(key: &str, resolved: &ResolvedFile) -> Result<CommDeckFile, RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` is not valid UTF-8: {err}"),
        })?;
    toml::from_str(text).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm antenna deck `{key}` TOML parse failed: {err}"),
    })
}

fn vector3(values: [f64; 3]) -> Vector3<f64> {
    Vector3::new(values[0], values[1], values[2])
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.chars().all(|c| c.is_ascii_hexdigit())
}

#[derive(Debug)]
struct LoadedBodyMaskDeck {
    deck: BodyMaskDeck,
    derived_sha256_hex: String,
}

#[derive(Debug, Deserialize)]
struct CommDeckFile {
    antenna: Option<CommDeckAntennaTable>,
    body_mask: Option<CommDeckBodyMaskTable>,
    fer_curve: Option<CommDeckFerCurveTable>,
    atmospheric_loss: Option<CommDeckElevationLossTable>,
    rain_loss: Option<CommDeckElevationLossTable>,
}

#[derive(Debug, Deserialize)]
struct CommDeckAntennaTable {
    boresight_body: [f64; 3],
    samples: Vec<CommDeckGainSample>,
}

#[derive(Debug, Deserialize)]
struct CommDeckGainSample {
    off_boresight_deg: f64,
    gain_dbi: f64,
}

#[derive(Debug, Deserialize)]
struct CommDeckBodyMaskTable {
    mesh_sha256: String,
    antenna_position_body_m: [f64; 3],
    directions: Vec<CommDeckBodyMaskDirection>,
    triangles: Vec<CommDeckBodyMaskTriangle>,
}

#[derive(Debug, Deserialize)]
struct CommDeckBodyMaskDirection {
    direction_body: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct CommDeckBodyMaskTriangle {
    a_m: [f64; 3],
    b_m: [f64; 3],
    c_m: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct CommDeckFerCurveTable {
    code_id: String,
    samples: Vec<CommDeckFerCurveSample>,
}

#[derive(Debug, Deserialize)]
struct CommDeckFerCurveSample {
    eb_n0_db: f64,
    fer: f64,
}

#[derive(Debug, Deserialize)]
struct CommDeckElevationLossTable {
    deck_id: String,
    samples: Vec<CommDeckElevationLossSample>,
}

#[derive(Debug, Deserialize)]
struct CommDeckElevationLossSample {
    elevation_deg: f64,
    loss_db: f64,
}

fn site_from_config(config: &CommGroundSiteConfig) -> Result<GroundSite, RunnerError> {
    let geodetic = GeodeticPosition::new(
        config.latitude_deg.to_radians(),
        config.longitude_deg.to_radians(),
        config.altitude_m,
    )
    .map_err(comm_error)?;
    let terrain_mask = if config.terrain_mask.is_empty() {
        None
    } else {
        Some(
            MaskDeck::new(
                config
                    .terrain_mask
                    .iter()
                    .map(|bin| MaskDeckBin {
                        azimuth_rad: bin.azimuth_deg.to_radians(),
                        min_elevation_rad: bin.min_elevation_deg.to_radians(),
                    })
                    .collect(),
            )
            .map_err(comm_error)?,
        )
    };
    GroundSite::new(
        config.id.clone(),
        geodetic,
        config.min_elevation_deg.to_radians(),
        terrain_mask,
    )
    .map_err(comm_error)
}

fn comm_error(err: CommError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("comm geometry rejected validated scenario: {err}"),
    }
}

fn scheduled_gap_link_state() -> LinkState {
    LinkState {
        free_space_loss_db: 0.0,
        c_n0_dbhz: 0.0,
        eb_n0_db: 0.0,
        margin_db: 0.0,
        fer: 1.0,
        visible: false,
        blackout: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn selection_accumulator(hysteresis_db: f64) -> CommRunAccumulator {
        CommRunAccumulator {
            sites: Vec::new(),
            samples: Vec::new(),
            antennas: BTreeMap::new(),
            antenna_decks: Vec::new(),
            links: vec![test_link("primary"), test_link("backup")],
            link_reports: Vec::new(),
            link_sample_history: vec![Vec::new(), Vec::new()],
            latest_observation: None,
            latest_observation_step: None,
            latest_observation_time_s: None,
            data_loss_timeouts: vec![None, None],
            bridge_link_selection: CommBridgeLinkSelectionConfig::BestMargin,
            bridge_link_hysteresis_db: hysteresis_db,
            bridge_pass_plan: Vec::new(),
            active_bridge_link_id: None,
            bridge_selection_reports: Vec::new(),
            bridge_handover_reports: Vec::new(),
            bridge_packet_samples: Vec::new(),
            active_bridge_selection_report: None,
            scenario_seed: 7,
        }
    }

    fn test_link(id: &str) -> CommRuntimeLink {
        CommRuntimeLink {
            id: id.to_owned(),
            site_id: "site".to_owned(),
            antenna_id: "antenna".to_owned(),
            relay: None,
            budget_template: LinkBudgetInput {
                eirp_dbw: 0.0,
                receiver_g_over_t_db_k: 0.0,
                slant_range_m: 1.0,
                frequency_hz: 2.0e9,
                bit_rate_bps: 1000.0,
                required_eb_n0_db: 0.0,
                atmospheric_loss_db: 0.0,
                rain_loss_db: 0.0,
                pointing_loss_db: 0.0,
                polarization_loss_db: 0.0,
                implementation_loss_db: 0.0,
                visible: true,
                blackout: false,
            },
            fer_curve: FerCurveDeck::new(
                "test",
                vec![FerCurveSample {
                    eb_n0_db: 0.0,
                    fer: 0.01,
                }],
            )
            .expect("test FER curve validates"),
            channel_model: LinkChannelModel::default(),
            link_stream_id: link_stream_id_from_id(id).expect("test link id validates"),
            loss_rate_gate: None,
            data_loss_timeout_s: None,
            atmospheric_loss_deck: None,
            rain_loss_deck: None,
        }
    }

    fn observation(samples: &[(&str, f64, bool, bool)]) -> CommObservation {
        CommObservation {
            visibility_samples: Vec::new(),
            link_states: samples
                .iter()
                .map(
                    |(link_id, margin_db, visible, blackout)| CommLinkStateSample {
                        link_id: (*link_id).to_owned(),
                        slant_range_m: 1000.0,
                        antenna_gain_dbi: 0.0,
                        body_blocked: false,
                        state: LinkState {
                            free_space_loss_db: 0.0,
                            c_n0_dbhz: 0.0,
                            eb_n0_db: *margin_db,
                            margin_db: *margin_db,
                            fer: 0.01,
                            visible: *visible,
                            blackout: *blackout,
                        },
                    },
                )
                .collect(),
        }
    }

    fn link_history_sample(
        step: u64,
        time_s: f64,
        margin_db: f64,
        visible: bool,
        blackout: bool,
    ) -> CommLinkStateHistorySample {
        CommLinkStateHistorySample {
            step,
            time_s,
            margin_db,
            visible,
            blackout,
        }
    }

    fn link_state_for_relay_test(margin_db: f64, fer: f64, visible: bool) -> LinkState {
        LinkState {
            free_space_loss_db: 100.0 - margin_db,
            c_n0_dbhz: 40.0 + margin_db,
            eb_n0_db: 10.0 + margin_db,
            margin_db,
            fer,
            visible,
            blackout: false,
        }
    }

    #[test]
    fn best_margin_selection_hysteresis_keeps_active_link_inside_deadband() {
        let mut accumulator = selection_accumulator(0.75);

        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 10.0, true, false),
                ("backup", 9.9, true, false),
            ])),
            Some(0)
        );
        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 10.0, true, false),
                ("backup", 10.75, true, false),
            ])),
            Some(0)
        );
        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 10.0, true, false),
                ("backup", 10.8, true, false),
            ])),
            Some(1)
        );
    }

    #[test]
    fn best_margin_zero_hysteresis_preserves_greedy_tie_break() {
        let mut accumulator = selection_accumulator(0.0);

        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 10.0, true, false),
                ("backup", 10.1, true, false),
            ])),
            Some(1)
        );
        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 10.0, true, false),
                ("backup", 10.0, true, false),
            ])),
            Some(0)
        );
    }

    #[test]
    fn best_margin_selection_falls_back_to_first_declared_when_no_link_is_usable() {
        let mut accumulator = selection_accumulator(0.75);

        assert_eq!(
            accumulator.bridge_link_index(&observation(&[
                ("primary", 0.0, false, false),
                ("backup", 100.0, true, true),
            ])),
            Some(0)
        );
    }

    #[test]
    fn relay_two_hop_composition_uses_bottleneck_margin_and_combined_fer() {
        let vehicle_to_relay = link_state_for_relay_test(5.0, 0.1, true);
        let relay_to_ground = link_state_for_relay_test(2.0, 0.2, true);

        let composite = compose_two_hop_link_state(vehicle_to_relay, relay_to_ground);

        assert!(composite.visible);
        assert!(!composite.blackout);
        assert_eq!(
            composite.free_space_loss_db.to_bits(),
            (vehicle_to_relay.free_space_loss_db + relay_to_ground.free_space_loss_db).to_bits()
        );
        assert_eq!(
            composite.margin_db.to_bits(),
            relay_to_ground.margin_db.to_bits()
        );
        assert!((composite.fer - 0.28).abs() <= 1.0e-15);

        let no_second_hop = compose_two_hop_link_state(
            vehicle_to_relay,
            link_state_for_relay_test(10.0, 0.01, false),
        );
        assert!(!no_second_hop.visible);
        assert_eq!(no_second_hop.fer.to_bits(), 1.0_f64.to_bits());
    }

    #[test]
    fn link_pass_reports_include_margin_profile_and_bridge_packet_counts() {
        let mut accumulator = selection_accumulator(0.0);
        accumulator.link_sample_history[1] = vec![
            link_history_sample(3, 11.0, 0.0, false, false),
            link_history_sample(4, 12.0, 11.0, true, false),
            link_history_sample(5, 13.0, 9.0, true, false),
            link_history_sample(6, 14.0, 100.0, true, true),
        ];
        accumulator.bridge_packet_samples = vec![
            CommBridgePacketReportSample {
                link_id: Some("backup".to_owned()),
                step: 4,
                sensor_drop_count: 1,
                command_drop_count: 0,
                sensor_bit_flip_count: 0,
                command_bit_flip_count: 1,
            },
            CommBridgePacketReportSample {
                link_id: Some("backup".to_owned()),
                step: 6,
                sensor_drop_count: 1,
                command_drop_count: 1,
                sensor_bit_flip_count: 0,
                command_bit_flip_count: 0,
            },
            CommBridgePacketReportSample {
                link_id: Some("primary".to_owned()),
                step: 5,
                sensor_drop_count: 1,
                command_drop_count: 1,
                sensor_bit_flip_count: 0,
                command_bit_flip_count: 0,
            },
        ];

        let reports = accumulator.link_pass_reports();

        assert_eq!(reports.len(), 1);
        let report = &reports[0];
        assert_eq!(report.link_id, "backup");
        assert_eq!(report.pass_index, 0);
        assert_eq!(report.start_step, 4);
        assert_eq!(report.end_step, 5);
        assert_eq!(report.start_time_s.to_bits(), 12.0_f64.to_bits());
        assert_eq!(report.end_time_s.to_bits(), 13.0_f64.to_bits());
        assert_eq!(report.duration_s.to_bits(), 1.0_f64.to_bits());
        assert_eq!(report.sample_count, 2);
        assert_eq!(report.min_margin_db.to_bits(), 9.0_f64.to_bits());
        assert_eq!(report.max_margin_db.to_bits(), 11.0_f64.to_bits());
        assert_eq!(report.mean_margin_db.to_bits(), 10.0_f64.to_bits());
        assert_eq!(
            report.margin_profile,
            vec![
                CommLinkPassMarginSampleReport {
                    step: 4,
                    time_s: 12.0,
                    margin_db: 11.0,
                },
                CommLinkPassMarginSampleReport {
                    step: 5,
                    time_s: 13.0,
                    margin_db: 9.0,
                },
            ]
        );
        assert_eq!(report.bridge_sample_count, 1);
        assert_eq!(report.sensor_drop_count, 1);
        assert_eq!(report.command_drop_count, 0);
        assert_eq!(report.sensor_bit_flip_count, 0);
        assert_eq!(report.command_bit_flip_count, 1);
    }

    #[test]
    fn declared_pass_plan_selects_active_window_link() {
        let mut accumulator = selection_accumulator(0.0);
        accumulator.bridge_link_selection = CommBridgeLinkSelectionConfig::DeclaredPlan;
        accumulator.bridge_pass_plan = vec![
            CommBridgePassPlan {
                link_id: "primary".to_owned(),
                start_s: 0.0,
                end_s: 10.0,
            },
            CommBridgePassPlan {
                link_id: "backup".to_owned(),
                start_s: 10.0,
                end_s: 20.0,
            },
        ];
        let sample = observation(&[
            ("primary", 10.0, true, false),
            ("backup", 11.0, true, false),
        ]);

        accumulator.latest_observation_time_s = Some(5.0);
        assert_eq!(accumulator.bridge_link_index(&sample), Some(0));
        accumulator.latest_observation_time_s = Some(10.0);
        assert_eq!(accumulator.bridge_link_index(&sample), Some(1));
    }

    #[test]
    fn declared_pass_plan_gap_drops_bridge_packets_fail_closed() {
        let mut accumulator = selection_accumulator(0.0);
        accumulator.bridge_link_selection = CommBridgeLinkSelectionConfig::DeclaredPlan;
        accumulator.bridge_pass_plan = vec![CommBridgePassPlan {
            link_id: "backup".to_owned(),
            start_s: 10.0,
            end_s: 20.0,
        }];
        accumulator.latest_observation = Some(observation(&[
            ("primary", 10.0, true, false),
            ("backup", 11.0, true, false),
        ]));
        accumulator.latest_observation_time_s = Some(5.0);

        let effects = accumulator
            .bridge_packet_effects(7, StepIndex::new(3))
            .expect("declared pass-plan gap evaluates no-link effects")
            .expect("declared pass-plan gap keeps comm effects active");

        assert_eq!(effects.link_id, "primary");
        assert_eq!(effects.sensor.disposition, LinkPacketDisposition::Drop);
        assert_eq!(effects.command.disposition, LinkPacketDisposition::Drop);
        assert_eq!(effects.sensor.error_draw, None);
        assert_eq!(effects.command.error_draw, None);
    }

    #[test]
    fn declared_pass_plan_reports_selection_intervals_and_gap_drops() {
        let mut accumulator = selection_accumulator(0.0);
        accumulator.bridge_link_selection = CommBridgeLinkSelectionConfig::DeclaredPlan;
        accumulator.bridge_pass_plan = vec![CommBridgePassPlan {
            link_id: "backup".to_owned(),
            start_s: 10.0,
            end_s: 20.0,
        }];
        accumulator.latest_observation = Some(observation(&[
            ("primary", 10.0, true, false),
            ("backup", 11.0, true, false),
        ]));

        accumulator.latest_observation_time_s = Some(5.0);
        accumulator
            .bridge_packet_effects(7, StepIndex::new(3))
            .expect("gap evaluates")
            .expect("gap keeps effects active");
        accumulator.latest_observation_time_s = Some(12.0);
        accumulator
            .bridge_packet_effects(7, StepIndex::new(4))
            .expect("window evaluates")
            .expect("window keeps effects active");

        let report = accumulator.finish().expect("comm report finishes");

        assert_eq!(report.bridge_selections.len(), 2);
        assert_eq!(report.bridge_selections[0].link_id, None);
        assert_eq!(
            report.bridge_selections[0].fallback_link_id.as_deref(),
            Some("primary")
        );
        assert_eq!(report.bridge_selections[0].start_step, 3);
        assert_eq!(report.bridge_selections[0].end_step, 3);
        assert_eq!(report.bridge_selections[0].sample_count, 1);
        assert_eq!(report.bridge_selections[0].sensor_drop_count, 1);
        assert_eq!(report.bridge_selections[0].command_drop_count, 1);
        assert_eq!(
            report.bridge_selections[1].link_id.as_deref(),
            Some("backup")
        );
        assert_eq!(report.bridge_selections[1].fallback_link_id, None);
        assert_eq!(report.bridge_selections[1].start_step, 4);
        assert_eq!(report.bridge_selections[1].end_step, 4);
        assert_eq!(report.bridge_selections[1].sample_count, 1);

        assert_eq!(report.bridge_handovers.len(), 1);
        assert_eq!(report.bridge_handovers[0].step, 4);
        assert_eq!(
            report.bridge_handovers[0].time_s.to_bits(),
            12.0_f64.to_bits()
        );
        assert_eq!(report.bridge_handovers[0].from_link_id, None);
        assert_eq!(
            report.bridge_handovers[0].from_fallback_link_id.as_deref(),
            Some("primary")
        );
        assert_eq!(
            report.bridge_handovers[0].to_link_id.as_deref(),
            Some("backup")
        );
        assert_eq!(report.bridge_handovers[0].to_fallback_link_id, None);
    }
}
