//! Runner-side communications pass-table artifact assembly.
//!
//! This module keeps WP-20.1 comm geometry outside `openbmp-fc`: the runner
//! samples the simulated primary-body state, converts it through the selected
//! frame profile, and asks `openbmp-comm` to extract deterministic passes.

use std::{collections::BTreeMap, io::Write};

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_aero::{LocalInclinationPanels, PanelMesh, PanelMeshBvh};
use openbmp_comm::{
    AntennaGainDeck, AntennaGainSample, BodyMaskDeck, BodyMaskSample, BodyMaskTriangle, CommError,
    ElevationLossDeck, ElevationLossSample, FerCurveDeck, FerCurveSample, GeodeticPosition,
    GroundSite, LinkBudgetInput, LinkChannelModel, LinkErrorAction, LinkPacketContext,
    LinkPacketDirection, LinkPacketDisposition, LinkPacketEffect, LinkState, MaskDeck, MaskDeckBin,
    PlasmaBlackoutInput, PlasmaBlackoutModel, TrackingBiasState, TrackingGaussMarkovContext,
    TrackingInstrumentModel, TrackingNoiseContext, TrackingObservableInput, VisibilitySample,
    WGS84_A_M, evaluate_link_budget, evaluate_link_loss_rate_gate, link_stream_id_from_id,
    pass_intervals, tracking_site_stream_id_from_id, write_pass_intervals_csv,
};
use openbmp_core::{ChannelId, Eci, Position3, SimTime, StepIndex, Velocity3};
use openbmp_physics::FrameContext;
use openbmp_scenario::{
    CommBlackoutElectronDensitySourceConfig, CommBridgeLinkSelectionConfig,
    CommBridgePassPlanConfig, CommGroundSiteConfig, CommLinkConfig, CommPacketErrorActionConfig,
    CommPacketLossRateGateConfig, CommRelayConfig, CommTrackingConfig, ResolvedFile,
    ScenarioDocument,
};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use serde::Deserialize;
use sha2::{Digest, Sha256};

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
    /// Sampled per-link plasma-blackout windows.
    pub blackout_windows: Vec<CommBlackoutWindowReport>,
    /// Optional tracking instrument evidence for the emitted observable stream.
    pub tracking: Option<CommTrackingReport>,
    /// Deterministic CSV bytes for the tracking observable measurement stream.
    pub tracking_observables_csv: Vec<u8>,
    /// Optional ground tracking observable stream for reconstruction consumers.
    pub tracking_observables: Vec<CommTrackingObservableReport>,
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
    /// SHA-256 digest of the resolved plasma-blackout electron-density deck file, when present.
    pub blackout_electron_density_deck_file_sha256_hex: Option<String>,
    /// Active plasma-blackout electron-density source, when enabled.
    pub blackout_electron_density_source: Option<String>,
    /// Number of validated plasma-blackout electron-density samples.
    pub blackout_electron_density_sample_count: usize,
    /// Frequency interval above plasma frequency over which blackout attenuation ramps down, Hz.
    pub blackout_ramp_width_hz: f64,
    /// Maximum plasma attenuation at and below plasma frequency, dB.
    pub blackout_max_attenuation_db: f64,
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
    /// Declared relay availability windows; empty means always available.
    pub availability_windows: Vec<CommRelayAvailabilityWindowReport>,
}

/// Runner evidence for one declared relay availability window.
#[derive(Clone, Debug, PartialEq)]
pub struct CommRelayAvailabilityWindowReport {
    /// Inclusive start time, seconds from scenario start.
    pub start_s: f64,
    /// Exclusive end time, seconds from scenario start.
    pub end_s: f64,
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
    /// Interpolated pass start time, seconds, bracketed by the neighboring unusable sample when available.
    pub interpolated_start_time_s: f64,
    /// Interpolated pass end time, seconds, bracketed by the neighboring unusable sample when available.
    pub interpolated_end_time_s: f64,
    /// Interpolated pass duration, seconds.
    pub interpolated_duration_s: f64,
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
    /// Interpolated profile including start/end boundary points plus usable samples.
    pub interpolated_margin_profile: Vec<CommLinkPassInterpolatedMarginSampleReport>,
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

/// One interpolated point in a comm-link pass margin profile.
#[derive(Clone, Debug, PartialEq)]
pub struct CommLinkPassInterpolatedMarginSampleReport {
    /// Interpolated or sampled time, seconds.
    pub time_s: f64,
    /// Interpolated or sampled link margin, dB.
    pub margin_db: f64,
}

/// Runner evidence for one contiguous sampled blackout window on a comm link.
#[derive(Clone, Debug, PartialEq)]
pub struct CommBlackoutWindowReport {
    /// Stable scenario link id.
    pub link_id: String,
    /// Zero-based blackout-window index for this link.
    pub window_index: u64,
    /// First sampled step in this blackout window.
    pub start_step: u64,
    /// Last sampled step in this blackout window.
    pub end_step: u64,
    /// First sampled time in this blackout window, seconds.
    pub start_time_s: f64,
    /// Last sampled time in this blackout window, seconds.
    pub end_time_s: f64,
    /// Sampled blackout duration, seconds.
    pub duration_s: f64,
    /// Number of blackout samples in this window.
    pub sample_count: u64,
    /// Number of blackout samples that were geometrically visible.
    pub visible_sample_count: u64,
}

/// Runner evidence for the declared `[comm.tracking]` instrument model.
#[derive(Clone, Debug, PartialEq)]
pub struct CommTrackingReport {
    /// SHA-256 digest of the resolved tracking instrument deck file, when present.
    pub instrument_deck_file_sha256_hex: Option<String>,
    /// Measurement latency added to sample time, seconds.
    pub latency_s: f64,
    /// Additive range bias, metres.
    pub range_bias_m: f64,
    /// Additive range-rate bias, metres per second.
    pub range_rate_bias_m_s: f64,
    /// Additive azimuth bias, degrees.
    pub azimuth_bias_deg: f64,
    /// Additive elevation bias, degrees.
    pub elevation_bias_deg: f64,
    /// Range quantization step, metres.
    pub range_quantization_m: f64,
    /// Range-rate quantization step, metres per second.
    pub range_rate_quantization_m_s: f64,
    /// Angle quantization step, degrees.
    pub angle_quantization_deg: f64,
    /// Range white-noise standard deviation, metres.
    pub range_noise_stddev_m: f64,
    /// Range-rate white-noise standard deviation, metres per second.
    pub range_rate_noise_stddev_m_s: f64,
    /// Azimuth white-noise standard deviation, degrees.
    pub azimuth_noise_stddev_deg: f64,
    /// Elevation white-noise standard deviation, degrees.
    pub elevation_noise_stddev_deg: f64,
    /// Range Gauss-Markov bias stationary standard deviation, metres.
    pub range_gauss_markov_stddev_m: f64,
    /// Range Gauss-Markov bias correlation time, seconds.
    pub range_gauss_markov_tau_s: f64,
    /// Range-rate Gauss-Markov bias stationary standard deviation, metres per second.
    pub range_rate_gauss_markov_stddev_m_s: f64,
    /// Range-rate Gauss-Markov bias correlation time, seconds.
    pub range_rate_gauss_markov_tau_s: f64,
    /// Azimuth Gauss-Markov bias stationary standard deviation, degrees.
    pub azimuth_gauss_markov_stddev_deg: f64,
    /// Azimuth Gauss-Markov bias correlation time, seconds.
    pub azimuth_gauss_markov_tau_s: f64,
    /// Elevation Gauss-Markov bias stationary standard deviation, degrees.
    pub elevation_gauss_markov_stddev_deg: f64,
    /// Elevation Gauss-Markov bias correlation time, seconds.
    pub elevation_gauss_markov_tau_s: f64,
}

/// One vehicle-bound ground tracking observable emitted by the runner.
#[derive(Clone, Debug, PartialEq)]
pub struct CommTrackingObservableReport {
    /// Sample step.
    pub step: u64,
    /// Ground-site id.
    pub site_id: String,
    /// Truth sample time, seconds.
    pub truth_time_s: f64,
    /// Time tag after declared measurement latency, seconds.
    pub measurement_time_s: f64,
    /// Measured range, metres.
    pub range_m: f64,
    /// Measured range rate, metres per second.
    pub range_rate_m_s: f64,
    /// Measured azimuth, radians.
    pub azimuth_rad: f64,
    /// Measured elevation, radians.
    pub elevation_rad: f64,
    /// Applied range white-noise value, metres.
    pub range_noise_m: f64,
    /// Applied range-rate white-noise value, metres per second.
    pub range_rate_noise_m_s: f64,
    /// Applied azimuth white-noise value, radians.
    pub azimuth_noise_rad: f64,
    /// Applied elevation white-noise value, radians.
    pub elevation_noise_rad: f64,
    /// Applied range Gauss-Markov bias state, metres.
    pub range_gauss_markov_bias_m: f64,
    /// Applied range-rate Gauss-Markov bias state, metres per second.
    pub range_rate_gauss_markov_bias_m_s: f64,
    /// Applied azimuth Gauss-Markov bias state, radians.
    pub azimuth_gauss_markov_bias_rad: f64,
    /// Applied elevation Gauss-Markov bias state, radians.
    pub elevation_gauss_markov_bias_rad: f64,
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
    /// Monitor-side continuity verdict: `hold` until timeout, then `terminate`.
    pub verdict: String,
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

pub(crate) struct CommEciObservationInput<'a> {
    pub(crate) frame: &'a FrameContext,
    pub(crate) step: StepIndex,
    pub(crate) time: SimTime,
    pub(crate) position_eci: Position3<Eci>,
    pub(crate) velocity_eci: Velocity3<Eci>,
    pub(crate) body_to_eci: UnitQuaternion<f64>,
    pub(crate) aerothermal_electron_number_density_m3: Option<f64>,
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

#[derive(Clone, Copy, Debug, Default)]
struct CommTrackingBiasRuntimeState {
    state: TrackingBiasState,
    last_time_s: Option<f64>,
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
    tracking_model: Option<TrackingInstrumentModel>,
    tracking_report: Option<CommTrackingReport>,
    tracking_bias_states: Vec<CommTrackingBiasRuntimeState>,
    tracking_observables: Vec<CommTrackingObservableReport>,
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
        let (tracking_model, tracking_report) = load_tracking_model(document, resolved_files)?;
        let site_count = sites.len();
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
            tracking_model,
            tracking_report,
            tracking_bias_states: vec![CommTrackingBiasRuntimeState::default(); site_count],
            tracking_observables: Vec::new(),
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
        input: CommEciObservationInput<'_>,
    ) -> Result<CommObservation, RunnerError> {
        let CommEciObservationInput {
            frame,
            step,
            time,
            position_eci,
            velocity_eci,
            body_to_eci,
            aerothermal_electron_number_density_m3,
        } = input;
        let position_ecef = frame.eci_to_ecef_position(time, position_eci);
        let velocity_ecef = frame.eci_to_ecef_velocity(time, velocity_eci, position_eci);
        let mut visibility_samples = Vec::with_capacity(self.sites.len());
        for ((site, samples), bias_state) in self
            .sites
            .iter()
            .zip(&mut self.samples)
            .zip(&mut self.tracking_bias_states)
        {
            let sample = site.observe_ecef(
                time.as_seconds(),
                position_ecef.vector,
                velocity_ecef.vector,
            );
            if sample.visible {
                record_tracking_observable(
                    self.tracking_model,
                    &mut self.tracking_observables,
                    bias_state,
                    self.scenario_seed,
                    step,
                    &sample,
                )
                .map_err(comm_error)?;
            }
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
                    aerothermal_electron_number_density_m3,
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
        let blackout_windows = self.blackout_window_reports();
        let mut tracking_observables_csv = Vec::new();
        write_tracking_observables_csv(&self.tracking_observables, &mut tracking_observables_csv)
            .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("comm tracking-observable CSV serialization failed: {err}"),
        })?;
        self.finish_data_loss_timeout_reports();
        self.evaluate_loss_rate_gates()?;
        Ok(CommRunReport {
            pass_table_csv,
            antenna_decks: self.antenna_decks,
            links: self.link_reports,
            link_passes,
            blackout_windows,
            tracking: self.tracking_report,
            tracking_observables_csv,
            tracking_observables: self.tracking_observables,
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
            let mut previous_sample: Option<CommLinkStateHistorySample> = None;
            for sample in samples {
                if sample.usable() {
                    if let Some(pass) = &mut current {
                        pass.observe(sample);
                    } else {
                        current = Some(CommLinkPassAccumulator::new(
                            link.id.clone(),
                            next_pass_index,
                            sample,
                            previous_sample.as_ref(),
                        ));
                    }
                    previous_sample = Some(sample.clone());
                    continue;
                }
                if let Some(mut pass) = current.take() {
                    pass.close_after_sample(sample);
                    reports.push(pass.into_report(&self.bridge_packet_samples));
                    next_pass_index += 1;
                }
                previous_sample = Some(sample.clone());
            }
            if let Some(pass) = current.take() {
                reports.push(pass.into_report(&self.bridge_packet_samples));
            }
        }
        reports
    }

    fn blackout_window_reports(&self) -> Vec<CommBlackoutWindowReport> {
        let mut reports = Vec::new();
        for (link_index, samples) in self.link_sample_history.iter().enumerate() {
            let Some(link) = self.links.get(link_index) else {
                continue;
            };
            let mut current: Option<CommBlackoutWindowAccumulator> = None;
            let mut next_window_index = 0_u64;
            for sample in samples {
                if sample.blackout {
                    if let Some(window) = &mut current {
                        window.observe(sample);
                    } else {
                        current = Some(CommBlackoutWindowAccumulator::new(
                            link.id.clone(),
                            next_window_index,
                            sample,
                        ));
                    }
                    continue;
                }
                if let Some(window) = current.take() {
                    reports.push(window.into_report());
                    next_window_index += 1;
                }
            }
            if let Some(window) = current.take() {
                reports.push(window.into_report());
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
    blackout_source: Option<CommRuntimeBlackoutSource>,
}

struct CommLinkObservationContext<'a> {
    frame: &'a FrameContext,
    time: SimTime,
    vehicle_ecef_m: Vector3<f64>,
    body_to_eci: UnitQuaternion<f64>,
    site: &'a GroundSite,
    direct_visibility: &'a VisibilitySample,
    antenna: &'a CommRuntimeAntenna,
    aerothermal_electron_number_density_m3: Option<f64>,
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
    availability_windows: Vec<CommRelayAvailabilityWindow>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CommRelayAvailabilityWindow {
    start_s: f64,
    end_s: f64,
}

#[derive(Clone, Debug)]
struct CommRuntimeBlackoutProfile {
    model: PlasmaBlackoutModel,
    samples: Vec<CommBlackoutProfileSample>,
}

#[derive(Clone, Debug)]
enum CommRuntimeBlackoutSource {
    Deck(CommRuntimeBlackoutProfile),
    LiveAerothermal(PlasmaBlackoutModel),
}

impl CommRuntimeBlackoutSource {
    const fn label(&self) -> &'static str {
        match self {
            Self::Deck(_) => "deck",
            Self::LiveAerothermal(_) => "live_aerothermal",
        }
    }

    fn sample_count(&self) -> usize {
        match self {
            Self::Deck(profile) => profile.samples.len(),
            Self::LiveAerothermal(_) => 0,
        }
    }

    fn evaluate(
        &self,
        time_s: f64,
        aerothermal_electron_number_density_m3: Option<f64>,
        input: &mut LinkBudgetInput,
    ) -> Result<(), CommError> {
        let (model, electron_number_density_m3) = match self {
            Self::Deck(profile) => (profile.model, profile.electron_number_density_at(time_s)?),
            Self::LiveAerothermal(model) => (
                *model,
                aerothermal_electron_number_density_m3.ok_or(CommError::Empty {
                    field: "comm.blackout.live_aerothermal_electron_number_density_m3",
                })?,
            ),
        };
        let state = model.evaluate(PlasmaBlackoutInput {
            frequency_hz: input.frequency_hz,
            electron_number_density_m3,
        })?;
        input.atmospheric_loss_db += state.attenuation_db;
        input.blackout |= state.blackout;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CommBlackoutProfileSample {
    time_s: f64,
    electron_number_density_m3: f64,
}

impl CommRuntimeBlackoutProfile {
    fn electron_number_density_at(&self, time_s: f64) -> Result<f64, CommError> {
        if !time_s.is_finite() {
            return Err(CommError::NonFinite {
                field: "comm.blackout.time_s",
            });
        }
        let first = self.samples.first().ok_or(CommError::Empty {
            field: "comm.blackout.samples",
        })?;
        if time_s <= first.time_s {
            return Ok(first.electron_number_density_m3);
        }
        let last = self.samples[self.samples.len() - 1];
        if time_s >= last.time_s {
            return Ok(last.electron_number_density_m3);
        }
        for pair in self.samples.windows(2) {
            let a = pair[0];
            let b = pair[1];
            if time_s >= a.time_s && time_s <= b.time_s {
                let fraction = (time_s - a.time_s) / (b.time_s - a.time_s);
                return Ok(a.electron_number_density_m3
                    + fraction * (b.electron_number_density_m3 - a.electron_number_density_m3));
            }
        }
        Ok(last.electron_number_density_m3)
    }
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
                    context.time.as_seconds(),
                    vehicle_to_relay_range_m,
                    antenna_gain_dbi,
                    vehicle_to_relay_visible,
                    context.aerothermal_electron_number_density_m3,
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
        let input = self.input_for_visibility(
            context.direct_visibility,
            antenna_gain_dbi,
            body_blocked,
            context.aerothermal_electron_number_density_m3,
        )?;
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
        aerothermal_electron_number_density_m3: Option<f64>,
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
        let mut input = LinkBudgetInput {
            eirp_dbw: self.budget_template.eirp_dbw + antenna_gain_dbi,
            slant_range_m: sample.slant_range_m,
            atmospheric_loss_db,
            rain_loss_db,
            visible: sample.visible && !body_blocked,
            blackout: false,
            ..self.budget_template
        };
        self.apply_blackout(
            sample.time_s,
            aerothermal_electron_number_density_m3,
            &mut input,
        )?;
        Ok(input)
    }

    fn input_for_relay_uplink(
        &self,
        time_s: f64,
        slant_range_m: f64,
        antenna_gain_dbi: f64,
        visible: bool,
        aerothermal_electron_number_density_m3: Option<f64>,
    ) -> Result<LinkBudgetInput, CommError> {
        require_positive_comm_runner("comm.relay.vehicle_to_relay.slant_range_m", slant_range_m)?;
        let mut input = LinkBudgetInput {
            eirp_dbw: self.budget_template.eirp_dbw + antenna_gain_dbi,
            slant_range_m,
            visible,
            blackout: false,
            ..self.budget_template
        };
        self.apply_blackout(time_s, aerothermal_electron_number_density_m3, &mut input)?;
        Ok(input)
    }

    fn apply_blackout(
        &self,
        time_s: f64,
        aerothermal_electron_number_density_m3: Option<f64>,
        input: &mut LinkBudgetInput,
    ) -> Result<(), CommError> {
        if let Some(source) = &self.blackout_source {
            source.evaluate(time_s, aerothermal_electron_number_density_m3, input)?;
        }
        Ok(())
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
        sample.visible =
            sample.elevation_rad >= sample.mask_elevation_rad && self.is_available(time_s);
        sample
    }

    fn is_available(&self, time_s: f64) -> bool {
        self.availability_windows.is_empty()
            || self
                .availability_windows
                .iter()
                .any(|window| window.contains(time_s))
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

impl CommRelayAvailabilityWindow {
    fn from_config(config: &openbmp_scenario::CommRelayAvailabilityWindowConfig) -> Self {
        Self {
            start_s: config.start_s,
            end_s: config.end_s,
        }
    }

    fn contains(self, time_s: f64) -> bool {
        time_s >= self.start_s && time_s < self.end_s
    }

    fn into_report(self) -> CommRelayAvailabilityWindowReport {
        CommRelayAvailabilityWindowReport {
            start_s: self.start_s,
            end_s: self.end_s,
        }
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
    start_sample: CommLinkStateHistorySample,
    end_sample: CommLinkStateHistorySample,
    previous_sample: Option<CommLinkStateHistorySample>,
    next_sample: Option<CommLinkStateHistorySample>,
    sample_count: u64,
    min_margin_db: f64,
    max_margin_db: f64,
    margin_sum_db: f64,
    margin_profile: Vec<CommLinkPassMarginSampleReport>,
}

impl CommLinkPassAccumulator {
    fn new(
        link_id: String,
        pass_index: u64,
        sample: &CommLinkStateHistorySample,
        previous_sample: Option<&CommLinkStateHistorySample>,
    ) -> Self {
        Self {
            link_id,
            pass_index,
            start_step: sample.step,
            end_step: sample.step,
            start_time_s: sample.time_s,
            end_time_s: sample.time_s,
            start_sample: sample.clone(),
            end_sample: sample.clone(),
            previous_sample: previous_sample.cloned(),
            next_sample: None,
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
        self.end_sample = sample.clone();
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

    fn close_after_sample(&mut self, sample: &CommLinkStateHistorySample) {
        self.next_sample = Some(sample.clone());
    }

    fn into_report(self, bridge_samples: &[CommBridgePacketReportSample]) -> CommLinkPassReport {
        let interpolated_start_time_s = self.interpolated_start_time_s();
        let interpolated_end_time_s = self.interpolated_end_time_s();
        let interpolated_margin_profile =
            self.interpolated_margin_profile(interpolated_start_time_s, interpolated_end_time_s);
        let mut report = CommLinkPassReport {
            link_id: self.link_id.clone(),
            pass_index: self.pass_index,
            start_step: self.start_step,
            end_step: self.end_step,
            start_time_s: self.start_time_s,
            end_time_s: self.end_time_s,
            duration_s: (self.end_time_s - self.start_time_s).max(0.0),
            interpolated_start_time_s,
            interpolated_end_time_s,
            interpolated_duration_s: (interpolated_end_time_s - interpolated_start_time_s).max(0.0),
            sample_count: self.sample_count,
            min_margin_db: self.min_margin_db,
            max_margin_db: self.max_margin_db,
            mean_margin_db: self.margin_sum_db / self.sample_count as f64,
            margin_profile: self.margin_profile,
            interpolated_margin_profile,
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

    fn interpolated_start_time_s(&self) -> f64 {
        self.previous_sample
            .as_ref()
            .map_or(self.start_time_s, |previous| {
                interpolated_transition_time(previous, &self.start_sample)
            })
    }

    fn interpolated_end_time_s(&self) -> f64 {
        self.next_sample.as_ref().map_or(self.end_time_s, |next| {
            interpolated_transition_time(&self.end_sample, next)
        })
    }

    fn interpolated_margin_profile(
        &self,
        interpolated_start_time_s: f64,
        interpolated_end_time_s: f64,
    ) -> Vec<CommLinkPassInterpolatedMarginSampleReport> {
        let mut profile = Vec::with_capacity(self.margin_profile.len() + 2);
        let start_margin_db =
            self.previous_sample
                .as_ref()
                .map_or(self.start_sample.margin_db, |previous| {
                    interpolated_margin_at(previous, &self.start_sample, interpolated_start_time_s)
                });
        push_interpolated_margin_sample(&mut profile, interpolated_start_time_s, start_margin_db);
        for sample in &self.margin_profile {
            push_interpolated_margin_sample(&mut profile, sample.time_s, sample.margin_db);
        }
        let end_margin_db = self
            .next_sample
            .as_ref()
            .map_or(self.end_sample.margin_db, |next| {
                interpolated_margin_at(&self.end_sample, next, interpolated_end_time_s)
            });
        push_interpolated_margin_sample(&mut profile, interpolated_end_time_s, end_margin_db);
        profile
    }
}

fn interpolated_transition_time(
    before: &CommLinkStateHistorySample,
    after: &CommLinkStateHistorySample,
) -> f64 {
    let dt = after.time_s - before.time_s;
    if !dt.is_finite() || dt <= 0.0 {
        return after.time_s;
    }
    let margin_delta = after.margin_db - before.margin_db;
    let strictly_crosses_zero = (before.margin_db < 0.0 && after.margin_db > 0.0)
        || (before.margin_db > 0.0 && after.margin_db < 0.0);
    if strictly_crosses_zero && margin_delta.is_finite() && margin_delta != 0.0 {
        let fraction = -before.margin_db / margin_delta;
        if fraction.is_finite() && (0.0..=1.0).contains(&fraction) {
            return before.time_s + fraction * dt;
        }
    }
    before.time_s + 0.5 * dt
}

fn interpolated_margin_at(
    before: &CommLinkStateHistorySample,
    after: &CommLinkStateHistorySample,
    time_s: f64,
) -> f64 {
    let dt = after.time_s - before.time_s;
    if !dt.is_finite() || dt <= 0.0 {
        return after.margin_db;
    }
    let fraction = ((time_s - before.time_s) / dt).clamp(0.0, 1.0);
    before.margin_db + fraction * (after.margin_db - before.margin_db)
}

fn push_interpolated_margin_sample(
    profile: &mut Vec<CommLinkPassInterpolatedMarginSampleReport>,
    time_s: f64,
    margin_db: f64,
) {
    if profile
        .last()
        .is_some_and(|last| last.time_s.to_bits() == time_s.to_bits())
    {
        return;
    }
    profile.push(CommLinkPassInterpolatedMarginSampleReport { time_s, margin_db });
}

#[derive(Clone, Debug, PartialEq)]
struct CommBlackoutWindowAccumulator {
    link_id: String,
    window_index: u64,
    start_step: u64,
    end_step: u64,
    start_time_s: f64,
    end_time_s: f64,
    sample_count: u64,
    visible_sample_count: u64,
}

impl CommBlackoutWindowAccumulator {
    fn new(link_id: String, window_index: u64, sample: &CommLinkStateHistorySample) -> Self {
        Self {
            link_id,
            window_index,
            start_step: sample.step,
            end_step: sample.step,
            start_time_s: sample.time_s,
            end_time_s: sample.time_s,
            sample_count: 1,
            visible_sample_count: u64::from(sample.visible),
        }
    }

    fn observe(&mut self, sample: &CommLinkStateHistorySample) {
        self.end_step = sample.step;
        self.end_time_s = sample.time_s;
        self.sample_count += 1;
        self.visible_sample_count += u64::from(sample.visible);
    }

    fn into_report(self) -> CommBlackoutWindowReport {
        CommBlackoutWindowReport {
            link_id: self.link_id,
            window_index: self.window_index,
            start_step: self.start_step,
            end_step: self.end_step,
            start_time_s: self.start_time_s,
            end_time_s: self.end_time_s,
            duration_s: (self.end_time_s - self.start_time_s).max(0.0),
            sample_count: self.sample_count,
            visible_sample_count: self.visible_sample_count,
        }
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
    const HOLD_VERDICT: &'static str = "hold";
    const TERMINATE_VERDICT: &'static str = "terminate";

    fn new(timeout_s: f64) -> Self {
        Self {
            report: CommDataLossTimeoutReport {
                timeout_s,
                verdict: Self::HOLD_VERDICT.to_owned(),
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
            self.report.verdict = Self::TERMINATE_VERDICT.to_owned();
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
                availability_windows: relay
                    .availability_windows
                    .iter()
                    .map(CommRelayAvailabilityWindow::from_config)
                    .collect(),
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
        let blackout_model = PlasmaBlackoutModel::new(
            link.blackout_ramp_width_hz,
            link.blackout_max_attenuation_db,
        )
        .map_err(comm_error)?;
        let blackout_source = match link.blackout_electron_density_source {
            CommBlackoutElectronDensitySourceConfig::Deck => {
                if link.blackout_electron_density_deck.is_some() {
                    let key = format!("comm.links[{index}].blackout_electron_density_deck");
                    let resolved = resolved_comm_file(&key, resolved_files)?;
                    Some((
                        Some(resolved.sha256_hex.clone()),
                        CommRuntimeBlackoutSource::Deck(load_blackout_profile_deck(
                            &key,
                            resolved,
                            blackout_model,
                        )?),
                    ))
                } else {
                    None
                }
            }
            CommBlackoutElectronDensitySourceConfig::LiveAerothermal => {
                if document
                    .aerothermal
                    .as_ref()
                    .and_then(|aerothermal| aerothermal.electron_density_proxy.as_ref())
                    .is_none()
                {
                    return Err(RunnerError::UnsupportedScenario {
                        what: format!(
                            "comm link `{}` requires aerothermal.electron_density_proxy",
                            link.id
                        ),
                    });
                }
                Some((
                    None,
                    CommRuntimeBlackoutSource::LiveAerothermal(blackout_model),
                ))
            }
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
            blackout_electron_density_deck_file_sha256_hex: blackout_source
                .as_ref()
                .and_then(|(sha256, _)| sha256.clone()),
            blackout_electron_density_source: blackout_source
                .as_ref()
                .map(|(_, source)| source.label().to_owned()),
            blackout_electron_density_sample_count: blackout_source
                .as_ref()
                .map_or(0, |(_, source)| source.sample_count()),
            blackout_ramp_width_hz: link.blackout_ramp_width_hz,
            blackout_max_attenuation_db: link.blackout_max_attenuation_db,
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
                availability_windows: relay
                    .availability_windows
                    .iter()
                    .copied()
                    .map(CommRelayAvailabilityWindow::into_report)
                    .collect(),
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
            blackout_source: blackout_source.map(|(_, source)| source),
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

#[derive(Clone, Copy, Debug, Default)]
struct CommTrackingInstrumentDeck {
    range_noise_stddev_m: f64,
    range_rate_noise_stddev_m_s: f64,
    azimuth_noise_stddev_deg: f64,
    elevation_noise_stddev_deg: f64,
    range_gauss_markov_stddev_m: f64,
    range_gauss_markov_tau_s: f64,
    range_rate_gauss_markov_stddev_m_s: f64,
    range_rate_gauss_markov_tau_s: f64,
    azimuth_gauss_markov_stddev_deg: f64,
    azimuth_gauss_markov_tau_s: f64,
    elevation_gauss_markov_stddev_deg: f64,
    elevation_gauss_markov_tau_s: f64,
}

fn load_tracking_model(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(Option<TrackingInstrumentModel>, Option<CommTrackingReport>), RunnerError> {
    let Some(config) = document
        .comm
        .as_ref()
        .and_then(|comm| comm.tracking.as_ref())
    else {
        return Ok((None, None));
    };
    let (instrument_deck_file_sha256_hex, instrument) = if config.instrument_deck.is_some() {
        let key = "comm.tracking.instrument_deck";
        let resolved = resolved_comm_file(key, resolved_files)?;
        (
            Some(resolved.sha256_hex.clone()),
            load_tracking_instrument_deck(key, resolved)?,
        )
    } else {
        (None, CommTrackingInstrumentDeck::default())
    };
    let model = tracking_model_from_config(config, instrument).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("comm tracking instrument rejected declared fields: {err}"),
        }
    })?;
    let report = CommTrackingReport {
        instrument_deck_file_sha256_hex,
        latency_s: config.latency_s,
        range_bias_m: config.range_bias_m,
        range_rate_bias_m_s: config.range_rate_bias_m_s,
        azimuth_bias_deg: config.azimuth_bias_deg,
        elevation_bias_deg: config.elevation_bias_deg,
        range_quantization_m: config.range_quantization_m,
        range_rate_quantization_m_s: config.range_rate_quantization_m_s,
        angle_quantization_deg: config.angle_quantization_deg,
        range_noise_stddev_m: model.range_noise_stddev_m(),
        range_rate_noise_stddev_m_s: model.range_rate_noise_stddev_m_s(),
        azimuth_noise_stddev_deg: model.azimuth_noise_stddev_rad().to_degrees(),
        elevation_noise_stddev_deg: model.elevation_noise_stddev_rad().to_degrees(),
        range_gauss_markov_stddev_m: model.range_gauss_markov_stddev_m(),
        range_gauss_markov_tau_s: model.range_gauss_markov_tau_s(),
        range_rate_gauss_markov_stddev_m_s: model.range_rate_gauss_markov_stddev_m_s(),
        range_rate_gauss_markov_tau_s: model.range_rate_gauss_markov_tau_s(),
        azimuth_gauss_markov_stddev_deg: model.azimuth_gauss_markov_stddev_rad().to_degrees(),
        azimuth_gauss_markov_tau_s: model.azimuth_gauss_markov_tau_s(),
        elevation_gauss_markov_stddev_deg: model.elevation_gauss_markov_stddev_rad().to_degrees(),
        elevation_gauss_markov_tau_s: model.elevation_gauss_markov_tau_s(),
    };
    Ok((Some(model), Some(report)))
}

fn tracking_model_from_config(
    config: &CommTrackingConfig,
    instrument: CommTrackingInstrumentDeck,
) -> Result<TrackingInstrumentModel, CommError> {
    TrackingInstrumentModel::new(
        config.latency_s,
        config.range_bias_m,
        config.range_rate_bias_m_s,
        config.azimuth_bias_deg.to_radians(),
        config.elevation_bias_deg.to_radians(),
        config.range_quantization_m,
        config.range_rate_quantization_m_s,
        config.angle_quantization_deg.to_radians(),
    )
    .and_then(|model| {
        model.with_white_noise_stddevs(
            instrument.range_noise_stddev_m,
            instrument.range_rate_noise_stddev_m_s,
            instrument.azimuth_noise_stddev_deg.to_radians(),
            instrument.elevation_noise_stddev_deg.to_radians(),
        )
    })
    .and_then(|model| {
        model.with_gauss_markov_biases(
            instrument.range_gauss_markov_stddev_m,
            instrument.range_gauss_markov_tau_s,
            instrument.range_rate_gauss_markov_stddev_m_s,
            instrument.range_rate_gauss_markov_tau_s,
            instrument.azimuth_gauss_markov_stddev_deg.to_radians(),
            instrument.azimuth_gauss_markov_tau_s,
            instrument.elevation_gauss_markov_stddev_deg.to_radians(),
            instrument.elevation_gauss_markov_tau_s,
        )
    })
}

fn record_tracking_observable(
    model: Option<TrackingInstrumentModel>,
    reports: &mut Vec<CommTrackingObservableReport>,
    bias_state: &mut CommTrackingBiasRuntimeState,
    scenario_seed: u64,
    step: StepIndex,
    sample: &VisibilitySample,
) -> Result<(), CommError> {
    let Some(model) = model else {
        return Ok(());
    };
    let site_stream_id = tracking_site_stream_id_from_id(&sample.site_id)?;
    let context = TrackingNoiseContext {
        scenario_seed,
        step: step.value(),
        site_stream_id,
    };
    let state = if let Some(last_time_s) = bias_state.last_time_s {
        let dt_s = sample.time_s - last_time_s;
        model.advance_gauss_markov_bias(
            bias_state.state,
            TrackingGaussMarkovContext {
                scenario_seed,
                step: step.value(),
                site_stream_id,
                dt_s,
            },
        )?
    } else {
        model.sample_initial_gauss_markov_bias(context)
    };
    bias_state.state = state;
    bias_state.last_time_s = Some(sample.time_s);
    let observable = model.evaluate_with_noise_and_bias(
        TrackingObservableInput::from_visibility_sample(sample),
        context,
        state,
    )?;
    reports.push(CommTrackingObservableReport {
        step: step.value(),
        site_id: observable.site_id,
        truth_time_s: observable.truth_time_s,
        measurement_time_s: observable.measurement_time_s,
        range_m: observable.range_m,
        range_rate_m_s: observable.range_rate_m_s,
        azimuth_rad: observable.azimuth_rad,
        elevation_rad: observable.elevation_rad,
        range_noise_m: observable.range_noise_m,
        range_rate_noise_m_s: observable.range_rate_noise_m_s,
        azimuth_noise_rad: observable.azimuth_noise_rad,
        elevation_noise_rad: observable.elevation_noise_rad,
        range_gauss_markov_bias_m: observable.range_gauss_markov_bias_m,
        range_rate_gauss_markov_bias_m_s: observable.range_rate_gauss_markov_bias_m_s,
        azimuth_gauss_markov_bias_rad: observable.azimuth_gauss_markov_bias_rad,
        elevation_gauss_markov_bias_rad: observable.elevation_gauss_markov_bias_rad,
    });
    Ok(())
}

fn write_tracking_observables_csv<W: Write>(
    reports: &[CommTrackingObservableReport],
    mut writer: W,
) -> std::io::Result<()> {
    writer.write_all(
        b"step,site_id,truth_time_s,measurement_time_s,range_m,range_rate_m_s,azimuth_rad,elevation_rad,range_noise_m,range_rate_noise_m_s,azimuth_noise_rad,elevation_noise_rad,range_gauss_markov_bias_m,range_rate_gauss_markov_bias_m_s,azimuth_gauss_markov_bias_rad,elevation_gauss_markov_bias_rad\n",
    )?;
    for report in reports {
        write_comm_csv_field(&mut writer, &report.step.to_string())?;
        writer.write_all(b",")?;
        write_comm_csv_field(&mut writer, &report.site_id)?;
        writeln!(
            writer,
            ",{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}",
            report.truth_time_s,
            report.measurement_time_s,
            report.range_m,
            report.range_rate_m_s,
            report.azimuth_rad,
            report.elevation_rad,
            report.range_noise_m,
            report.range_rate_noise_m_s,
            report.azimuth_noise_rad,
            report.elevation_noise_rad,
            report.range_gauss_markov_bias_m,
            report.range_rate_gauss_markov_bias_m_s,
            report.azimuth_gauss_markov_bias_rad,
            report.elevation_gauss_markov_bias_rad,
        )?;
    }
    Ok(())
}

fn write_comm_csv_field<W: Write>(writer: &mut W, value: &str) -> std::io::Result<()> {
    let must_quote = value
        .as_bytes()
        .iter()
        .any(|b| matches!(b, b',' | b'"' | b'\n' | b'\r'));
    if must_quote {
        writer.write_all(b"\"")?;
        for byte in value.as_bytes() {
            if *byte == b'"' {
                writer.write_all(b"\"\"")?;
            } else {
                writer.write_all(std::slice::from_ref(byte))?;
            }
        }
        writer.write_all(b"\"")
    } else {
        writer.write_all(value.as_bytes())
    }
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
    let directions = body_mask
        .directions
        .into_iter()
        .map(|direction| vector3(direction.direction_body))
        .collect::<Vec<_>>();
    let antenna_position_body_m = vector3(body_mask.antenna_position_body_m);
    let (deck, mesh_sha256) = if let Some(triangles) = body_mask.triangles {
        let mesh_sha256 =
            body_mask
                .mesh_sha256
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!("comm antenna deck `{key}` body_mask.mesh_sha256 is required"),
                })?;
        if !is_sha256_hex(&mesh_sha256) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm antenna deck `{key}` body_mask.mesh_sha256 is not a SHA-256 hex digest"
                ),
            });
        }
        let triangles = triangles
            .into_iter()
            .map(|triangle| BodyMaskTriangle {
                a_m: vector3(triangle.a_m),
                b_m: vector3(triangle.b_m),
                c_m: vector3(triangle.c_m),
            })
            .collect::<Vec<_>>();
        let deck = BodyMaskDeck::precompute_from_triangles(
            antenna_position_body_m,
            &directions,
            &triangles,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` rejected body-mask deck: {err}"),
        })?;
        (deck, mesh_sha256)
    } else {
        panel_mesh_body_mask_deck(key, resolved, antenna_position_body_m, &directions)?
    };
    let derived_sha256_hex = deck.sha256_hex_with_mesh_hash(&mesh_sha256);
    Ok(LoadedBodyMaskDeck {
        deck,
        derived_sha256_hex,
    })
}

fn panel_mesh_body_mask_deck(
    key: &str,
    resolved: &ResolvedFile,
    antenna_position_body_m: Vector3<f64>,
    directions: &[Vector3<f64>],
) -> Result<(BodyMaskDeck, String), RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` is not valid UTF-8: {err}"),
        })?;
    let panel_mesh_text = panel_mesh_toml_without_body_mask(key, text)?;
    let panels = LocalInclinationPanels::load_from_str(&panel_mesh_text).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!(
                "comm antenna deck `{key}` missing body_mask.triangles and rejected panel mesh source: {err}"
            ),
        }
    })?;
    let bvh = PanelMeshBvh::from_watertight_mesh(panels.geometry).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!(
                "comm antenna deck `{key}` panel mesh body-mask source is not watertight: {err}"
            ),
        }
    })?;
    let mesh_sha256 = panel_mesh_sha256_hex(bvh.mesh());
    let mut samples = Vec::with_capacity(directions.len());
    for direction in directions {
        let blocked = bvh
            .ray_intersects(antenna_position_body_m, *direction)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("comm antenna deck `{key}` rejected panel mesh body-mask ray: {err}"),
            })?;
        samples.push(BodyMaskSample {
            direction_body: *direction,
            blocked,
        });
    }
    let deck = BodyMaskDeck::new(samples).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm antenna deck `{key}` rejected panel mesh body-mask deck: {err}"),
    })?;
    Ok((deck, mesh_sha256))
}

fn panel_mesh_toml_without_body_mask(key: &str, text: &str) -> Result<String, RunnerError> {
    let mut value =
        toml::from_str::<toml::Value>(text).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` TOML parse failed: {err}"),
        })?;
    let table = value
        .as_table_mut()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm antenna deck `{key}` must be a TOML table"),
        })?;
    table.remove("body_mask");
    toml::to_string(&value).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("comm antenna deck `{key}` panel mesh source could not be serialized: {err}"),
    })
}

fn panel_mesh_sha256_hex(mesh: &PanelMesh) -> String {
    let mut hasher = Sha256::new();
    hasher.update(b"openbmp.comm.body_mask.panel_mesh.v1\n");
    for vertex in mesh.vertices() {
        hasher.update(vertex.x.to_bits().to_le_bytes());
        hasher.update(vertex.y.to_bits().to_le_bytes());
        hasher.update(vertex.z.to_bits().to_le_bytes());
    }
    hasher.update(b"triangles\n");
    for [a, b, c] in mesh.triangles() {
        hasher.update(a.to_le_bytes());
        hasher.update(b.to_le_bytes());
        hasher.update(c.to_le_bytes());
    }
    let digest: [u8; 32] = hasher.finalize().into();
    sha256_hex(&digest)
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

fn load_blackout_profile_deck(
    key: &str,
    resolved: &ResolvedFile,
    model: PlasmaBlackoutModel,
) -> Result<CommRuntimeBlackoutProfile, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let deck = file
        .plasma_blackout
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm link deck `{key}` missing [plasma_blackout] table"),
        })?;
    if deck.samples.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("comm link deck `{key}` plasma_blackout.samples must not be empty"),
        });
    }
    let mut samples = Vec::with_capacity(deck.samples.len());
    let mut previous_time_s = None;
    for (index, sample) in deck.samples.into_iter().enumerate() {
        if !sample.time_s.is_finite() || sample.time_s < 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm link deck `{key}` plasma_blackout.samples[{index}].time_s must be finite and non-negative"
                ),
            });
        }
        if !sample.electron_number_density_m3.is_finite() || sample.electron_number_density_m3 < 0.0
        {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm link deck `{key}` plasma_blackout.samples[{index}].electron_number_density_m3 must be finite and non-negative"
                ),
            });
        }
        if let Some(previous) = previous_time_s
            && sample.time_s <= previous
        {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "comm link deck `{key}` plasma_blackout.samples[{index}].time_s must be strictly increasing"
                ),
            });
        }
        previous_time_s = Some(sample.time_s);
        samples.push(CommBlackoutProfileSample {
            time_s: sample.time_s,
            electron_number_density_m3: sample.electron_number_density_m3,
        });
    }
    Ok(CommRuntimeBlackoutProfile { model, samples })
}

fn load_tracking_instrument_deck(
    key: &str,
    resolved: &ResolvedFile,
) -> Result<CommTrackingInstrumentDeck, RunnerError> {
    let file = parse_comm_deck_file(key, resolved)?;
    let deck = file
        .tracking_instrument
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("comm tracking deck `{key}` missing [tracking_instrument] table"),
        })?;
    Ok(CommTrackingInstrumentDeck {
        range_noise_stddev_m: deck.range_noise_stddev_m,
        range_rate_noise_stddev_m_s: deck.range_rate_noise_stddev_m_s,
        azimuth_noise_stddev_deg: deck.azimuth_noise_stddev_deg,
        elevation_noise_stddev_deg: deck.elevation_noise_stddev_deg,
        range_gauss_markov_stddev_m: deck.range_gauss_markov_stddev_m,
        range_gauss_markov_tau_s: deck.range_gauss_markov_tau_s,
        range_rate_gauss_markov_stddev_m_s: deck.range_rate_gauss_markov_stddev_m_s,
        range_rate_gauss_markov_tau_s: deck.range_rate_gauss_markov_tau_s,
        azimuth_gauss_markov_stddev_deg: deck.azimuth_gauss_markov_stddev_deg,
        azimuth_gauss_markov_tau_s: deck.azimuth_gauss_markov_tau_s,
        elevation_gauss_markov_stddev_deg: deck.elevation_gauss_markov_stddev_deg,
        elevation_gauss_markov_tau_s: deck.elevation_gauss_markov_tau_s,
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

fn sha256_hex(digest: &[u8; 32]) -> String {
    let mut out = String::with_capacity(digest.len() * 2);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in *digest {
        out.push(char::from(HEX[usize::from(byte >> 4)]));
        out.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    out
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
    plasma_blackout: Option<CommDeckPlasmaBlackoutTable>,
    tracking_instrument: Option<CommDeckTrackingInstrumentTable>,
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
    mesh_sha256: Option<String>,
    antenna_position_body_m: [f64; 3],
    directions: Vec<CommDeckBodyMaskDirection>,
    triangles: Option<Vec<CommDeckBodyMaskTriangle>>,
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

#[derive(Debug, Deserialize)]
struct CommDeckPlasmaBlackoutTable {
    samples: Vec<CommDeckPlasmaBlackoutSample>,
}

#[derive(Debug, Deserialize)]
struct CommDeckPlasmaBlackoutSample {
    time_s: f64,
    electron_number_density_m3: f64,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct CommDeckTrackingInstrumentTable {
    #[serde(default)]
    range_noise_stddev_m: f64,
    #[serde(default)]
    range_rate_noise_stddev_m_s: f64,
    #[serde(default)]
    azimuth_noise_stddev_deg: f64,
    #[serde(default)]
    elevation_noise_stddev_deg: f64,
    #[serde(default)]
    range_gauss_markov_stddev_m: f64,
    #[serde(default)]
    range_gauss_markov_tau_s: f64,
    #[serde(default)]
    range_rate_gauss_markov_stddev_m_s: f64,
    #[serde(default)]
    range_rate_gauss_markov_tau_s: f64,
    #[serde(default)]
    azimuth_gauss_markov_stddev_deg: f64,
    #[serde(default)]
    azimuth_gauss_markov_tau_s: f64,
    #[serde(default)]
    elevation_gauss_markov_stddev_deg: f64,
    #[serde(default)]
    elevation_gauss_markov_tau_s: f64,
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
    use nalgebra::DVector;
    use openbmp_testkit::reconstruction::{
        BatchObservation, batch_gauss_newton, normalized_error_squared,
    };

    fn selection_accumulator(hysteresis_db: f64) -> CommRunAccumulator {
        CommRunAccumulator {
            sites: Vec::new(),
            samples: Vec::new(),
            antennas: BTreeMap::new(),
            antenna_decks: Vec::new(),
            links: vec![test_link("primary"), test_link("backup")],
            link_reports: Vec::new(),
            link_sample_history: vec![Vec::new(), Vec::new()],
            tracking_model: None,
            tracking_report: None,
            tracking_bias_states: Vec::new(),
            tracking_observables: Vec::new(),
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
            blackout_source: None,
        }
    }

    fn test_relay(availability_windows: Vec<CommRelayAvailabilityWindow>) -> CommRuntimeRelay {
        CommRuntimeRelay {
            id: "tdrss-east".to_owned(),
            longitude_deg: 0.0,
            altitude_m: 35_786_000.0,
            min_elevation_deg: 0.0,
            geodetic: GeodeticPosition::new(0.0, 0.0, 35_786_000.0)
                .expect("test relay geodetic validates"),
            budget_template: LinkBudgetInput {
                eirp_dbw: 36.0,
                receiver_g_over_t_db_k: 24.0,
                slant_range_m: 1.0,
                frequency_hz: 2.2e9,
                bit_rate_bps: 1000.0,
                required_eb_n0_db: 3.0,
                atmospheric_loss_db: 0.0,
                rain_loss_db: 0.0,
                pointing_loss_db: 0.0,
                polarization_loss_db: 0.0,
                implementation_loss_db: 0.0,
                visible: true,
                blackout: false,
            },
            fer_curve: FerCurveDeck::new(
                "relay-test",
                vec![
                    FerCurveSample {
                        eb_n0_db: -100.0,
                        fer: 1.0,
                    },
                    FerCurveSample {
                        eb_n0_db: 100.0,
                        fer: 1.0e-6,
                    },
                ],
            )
            .expect("test relay FER curve validates"),
            fer_curve_deck_file_sha256_hex: "test-sha256".to_owned(),
            atmospheric_loss_deck: None,
            atmospheric_loss_deck_file_sha256_hex: None,
            rain_loss_deck: None,
            rain_loss_deck_file_sha256_hex: None,
            availability_windows,
        }
    }

    #[test]
    fn blackout_profile_deck_loads_interpolates_and_rejects_bad_samples() {
        let resolved = ResolvedFile::from_bytes(
            std::path::PathBuf::from("/blackout.toml"),
            br#"
[plasma_blackout]
[[plasma_blackout.samples]]
time_s = 10.0
electron_number_density_m3 = 1.0
[[plasma_blackout.samples]]
time_s = 20.0
electron_number_density_m3 = 3.0
"#
            .to_vec(),
        );
        let profile = load_blackout_profile_deck(
            "comm.links[0].blackout_electron_density_deck",
            &resolved,
            PlasmaBlackoutModel::default(),
        )
        .expect("blackout profile deck loads");

        assert_eq!(profile.samples.len(), 2);
        assert_eq!(
            profile.electron_number_density_at(5.0).unwrap().to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            profile.electron_number_density_at(15.0).unwrap().to_bits(),
            2.0_f64.to_bits()
        );
        assert_eq!(
            profile.electron_number_density_at(25.0).unwrap().to_bits(),
            3.0_f64.to_bits()
        );

        let bad = ResolvedFile::from_bytes(
            std::path::PathBuf::from("/bad-blackout.toml"),
            br#"
[plasma_blackout]
[[plasma_blackout.samples]]
time_s = 2.0
electron_number_density_m3 = 1.0
[[plasma_blackout.samples]]
time_s = 2.0
electron_number_density_m3 = 2.0
"#
            .to_vec(),
        );
        let err = load_blackout_profile_deck(
            "comm.links[0].blackout_electron_density_deck",
            &bad,
            PlasmaBlackoutModel::default(),
        )
        .unwrap_err();
        assert!(
            err.to_string().contains("strictly increasing"),
            "expected strict ordering rejection, got {err}"
        );
    }

    #[test]
    fn blackout_profile_applies_attenuation_and_blackout_to_vehicle_hop() {
        let mut link = test_link("blackout");
        let density_for_1_95_ghz = (1.95e9 / 8.98_f64).powi(2);
        link.blackout_source = Some(CommRuntimeBlackoutSource::Deck(
            CommRuntimeBlackoutProfile {
                model: PlasmaBlackoutModel::new(1.0e8, 40.0).unwrap(),
                samples: vec![CommBlackoutProfileSample {
                    time_s: 0.0,
                    electron_number_density_m3: density_for_1_95_ghz,
                }],
            },
        ));
        let sample = VisibilitySample {
            time_s: 12.0,
            site_id: "site".to_owned(),
            slant_range_m: 1000.0,
            range_rate_m_s: 0.0,
            elevation_rad: 0.5,
            azimuth_rad: 0.0,
            mask_elevation_rad: 0.0,
            visible: true,
        };

        let ramp_input = link
            .input_for_visibility(&sample, 0.0, false, None)
            .unwrap();
        assert!(!ramp_input.blackout);
        assert!(
            (ramp_input.atmospheric_loss_db - 20.0).abs() < 1.0e-9,
            "expected half-ramp attenuation, got {}",
            ramp_input.atmospheric_loss_db
        );

        let density_for_2_0_ghz = (2.0e9 / 8.98_f64).powi(2);
        link.blackout_source = Some(CommRuntimeBlackoutSource::Deck(
            CommRuntimeBlackoutProfile {
                model: PlasmaBlackoutModel::new(1.0e8, 40.0).unwrap(),
                samples: vec![CommBlackoutProfileSample {
                    time_s: 0.0,
                    electron_number_density_m3: density_for_2_0_ghz,
                }],
            },
        ));

        let blackout_input = link
            .input_for_visibility(&sample, 0.0, false, None)
            .unwrap();
        assert!(blackout_input.blackout);
        assert!(
            (blackout_input.atmospheric_loss_db - 40.0).abs() < 1.0e-9,
            "expected max blackout attenuation, got {}",
            blackout_input.atmospheric_loss_db
        );
    }

    #[test]
    fn live_aerothermal_blackout_source_uses_current_density() {
        let mut link = test_link("live-blackout");
        link.blackout_source = Some(CommRuntimeBlackoutSource::LiveAerothermal(
            PlasmaBlackoutModel::new(1.0e8, 40.0).unwrap(),
        ));
        let sample = VisibilitySample {
            time_s: 12.0,
            site_id: "site".to_owned(),
            slant_range_m: 1000.0,
            range_rate_m_s: 0.0,
            elevation_rad: 0.5,
            azimuth_rad: 0.0,
            mask_elevation_rad: 0.0,
            visible: true,
        };
        let density_for_2_0_ghz = (2.0e9 / 8.98_f64).powi(2);

        let blackout_input = link
            .input_for_visibility(&sample, 0.0, false, Some(density_for_2_0_ghz))
            .unwrap();
        assert!(blackout_input.blackout);
        assert!(
            (blackout_input.atmospheric_loss_db - 40.0).abs() < 1.0e-9,
            "expected live density max attenuation, got {}",
            blackout_input.atmospheric_loss_db
        );

        let err = link
            .input_for_visibility(&sample, 0.0, false, None)
            .unwrap_err();
        assert_eq!(
            err,
            CommError::Empty {
                field: "comm.blackout.live_aerothermal_electron_number_density_m3"
            }
        );
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
    fn relay_availability_windows_gate_relay_downlink_visibility() {
        let relay = test_relay(vec![CommRelayAvailabilityWindow {
            start_s: 10.0,
            end_s: 20.0,
        }]);
        let site = GroundSite::new(
            "equator".to_owned(),
            GeodeticPosition::new(0.0, 0.0, 0.0).expect("test site geodetic validates"),
            0.0,
            None,
        )
        .expect("test ground site validates");

        let outside = relay.visibility_from_site(5.0, &site);
        assert!(!outside.visible);
        let outside_state = evaluate_link_budget(
            relay
                .input_for_visibility(&outside)
                .expect("outside relay input validates"),
            &relay.fer_curve,
        )
        .expect("outside relay budget evaluates");
        assert!(!outside_state.visible);
        assert_eq!(outside_state.fer.to_bits(), 1.0_f64.to_bits());

        let inside = relay.visibility_from_site(12.0, &site);
        assert!(inside.visible);
        assert!(relay.is_available(19.999));
        assert!(!relay.is_available(20.0));
    }

    #[test]
    fn data_loss_timeout_report_latches_hold_then_terminate_verdict() {
        let mut timeout = CommDataLossTimeoutState::new(0.5);

        timeout.observe(StepIndex::new(0), 0.0, false);
        let report = timeout.report();
        assert_eq!(report.verdict, "hold");
        assert!(!report.triggered);

        timeout.observe(StepIndex::new(1), 0.4, false);
        let report = timeout.report();
        assert_eq!(report.verdict, "hold");
        assert!(!report.triggered);

        timeout.observe(StepIndex::new(2), 0.6, false);
        let report = timeout.report();
        assert_eq!(report.verdict, "terminate");
        assert!(report.triggered);
        assert_eq!(report.first_trigger_step, Some(2));
        assert_eq!(report.first_trigger_time_s, Some(0.6));

        timeout.observe(StepIndex::new(3), 0.7, true);
        let report = timeout.report();
        assert_eq!(report.verdict, "terminate");
        assert!(report.triggered);
        assert_eq!(report.delivered_sample_count, 1);
    }

    #[test]
    fn body_mask_deck_loads_from_panel_mesh_aero_source() {
        let resolved = ResolvedFile::from_bytes(
            std::path::PathBuf::from("/panel-body-mask.toml"),
            br#"
openbmp.panel_mesh_aero = 1
provenance = "unit-test panel mesh body-mask source"
validation = "checked"
method = "modified-newtonian"
cp_max = 2.0

[mesh]
vertices_m = [
  [1.0, -1.0, -1.0],
  [1.0,  1.0, -1.0],
  [1.0,  1.0,  1.0],
  [1.0, -1.0,  1.0],
  [2.0, -1.0, -1.0],
  [2.0,  1.0, -1.0],
  [2.0,  1.0,  1.0],
  [2.0, -1.0,  1.0],
]
triangles = [
  [0, 1, 2], [0, 2, 3],
  [4, 6, 5], [4, 7, 6],
  [0, 3, 7], [0, 7, 4],
  [1, 5, 6], [1, 6, 2],
  [0, 4, 5], [0, 5, 1],
  [3, 2, 6], [3, 6, 7],
]

[body_mask]
antenna_position_body_m = [0.0, 0.0, 0.0]
directions = [
  { direction_body = [1.0, 0.0, 0.0] },
  { direction_body = [1.0, 2.0, 0.0] },
  { direction_body = [0.0, 1.0, 0.0] },
]
"#
            .to_vec(),
        );

        let loaded = load_body_mask_deck("comm.antennas[0].body_mask_deck", &resolved)
            .expect("panel-mesh body mask deck loads");
        let samples = loaded.deck.samples();
        assert_eq!(samples.len(), 3);
        assert!(samples[0].blocked);
        assert!(!samples[1].blocked);
        assert!(!samples[2].blocked);
        assert_eq!(loaded.derived_sha256_hex.len(), 64);

        let repeat = load_body_mask_deck("comm.antennas[0].body_mask_deck", &resolved)
            .expect("panel-mesh body mask deck reloads");
        assert_eq!(loaded.derived_sha256_hex, repeat.derived_sha256_hex);
        assert_eq!(loaded.deck, repeat.deck);
    }

    #[test]
    fn body_mask_panel_mesh_source_rejects_open_mesh() {
        let resolved = ResolvedFile::from_bytes(
            std::path::PathBuf::from("/open-panel-body-mask.toml"),
            br#"
openbmp.panel_mesh_aero = 1
provenance = "unit-test open panel mesh body-mask source"
validation = "checked"
method = "modified-newtonian"
cp_max = 2.0

[mesh]
vertices_m = [
  [1.0, -1.0, -1.0],
  [1.0,  1.0, -1.0],
  [1.0,  1.0,  1.0],
  [1.0, -1.0,  1.0],
]
triangles = [[0, 1, 2], [0, 2, 3]]

[body_mask]
antenna_position_body_m = [0.0, 0.0, 0.0]
directions = [
  { direction_body = [1.0, 0.0, 0.0] },
]
"#
            .to_vec(),
        );

        let err = load_body_mask_deck("comm.antennas[0].body_mask_deck", &resolved)
            .expect_err("open panel mesh should fail closed");
        assert!(
            err.to_string().contains("not watertight"),
            "unexpected error: {err}"
        );
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
        assert_eq!(
            report.interpolated_start_time_s.to_bits(),
            11.5_f64.to_bits()
        );
        assert_eq!(report.interpolated_end_time_s.to_bits(), 13.5_f64.to_bits());
        assert_eq!(report.interpolated_duration_s.to_bits(), 2.0_f64.to_bits());
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
        assert_eq!(
            report.interpolated_margin_profile,
            vec![
                CommLinkPassInterpolatedMarginSampleReport {
                    time_s: 11.5,
                    margin_db: 5.5,
                },
                CommLinkPassInterpolatedMarginSampleReport {
                    time_s: 12.0,
                    margin_db: 11.0,
                },
                CommLinkPassInterpolatedMarginSampleReport {
                    time_s: 13.0,
                    margin_db: 9.0,
                },
                CommLinkPassInterpolatedMarginSampleReport {
                    time_s: 13.5,
                    margin_db: 54.5,
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
    fn blackout_window_reports_group_contiguous_blackout_samples() {
        let mut accumulator = selection_accumulator(0.0);
        accumulator.link_sample_history[1] = vec![
            link_history_sample(3, 11.0, 0.0, false, false),
            link_history_sample(4, 12.0, 11.0, true, true),
            link_history_sample(5, 13.0, 9.0, true, true),
            link_history_sample(6, 14.0, 8.0, false, false),
            link_history_sample(7, 15.0, 7.0, false, true),
        ];

        let reports = accumulator.blackout_window_reports();

        assert_eq!(reports.len(), 2);
        assert_eq!(
            reports[0],
            CommBlackoutWindowReport {
                link_id: "backup".to_owned(),
                window_index: 0,
                start_step: 4,
                end_step: 5,
                start_time_s: 12.0,
                end_time_s: 13.0,
                duration_s: 1.0,
                sample_count: 2,
                visible_sample_count: 2,
            }
        );
        assert_eq!(
            reports[1],
            CommBlackoutWindowReport {
                link_id: "backup".to_owned(),
                window_index: 1,
                start_step: 7,
                end_step: 7,
                start_time_s: 15.0,
                end_time_s: 15.0,
                duration_s: 0.0,
                sample_count: 1,
                visible_sample_count: 0,
            }
        );

        let report = accumulator.finish().expect("comm report finishes");
        assert_eq!(report.blackout_windows, reports);
    }

    #[test]
    fn tracking_observable_report_applies_declared_bias_latency_and_quantization() {
        let config = CommTrackingConfig {
            instrument_deck: None,
            instrument_deck_sha256: None,
            latency_s: 0.5,
            range_bias_m: 2.4,
            range_rate_bias_m_s: -0.26,
            azimuth_bias_deg: 0.0,
            elevation_bias_deg: 0.0,
            range_quantization_m: 1.0,
            range_rate_quantization_m_s: 0.1,
            angle_quantization_deg: 0.0,
        };
        let model = tracking_model_from_config(&config, CommTrackingInstrumentDeck::default())
            .expect("tracking config validates");
        let sample = VisibilitySample {
            time_s: 20.0,
            site_id: "site".to_owned(),
            slant_range_m: 1000.3,
            range_rate_m_s: 12.34,
            elevation_rad: 0.5,
            azimuth_rad: 1.25,
            mask_elevation_rad: 0.0,
            visible: true,
        };
        let mut reports = Vec::new();
        let mut bias_state = CommTrackingBiasRuntimeState::default();

        record_tracking_observable(
            Some(model),
            &mut reports,
            &mut bias_state,
            7,
            StepIndex::new(9),
            &sample,
        )
        .expect("tracking report records");

        assert_eq!(reports.len(), 1);
        let report = &reports[0];
        assert_eq!(report.step, 9);
        assert_eq!(report.site_id, "site");
        assert_eq!(report.truth_time_s.to_bits(), 20.0_f64.to_bits());
        assert_eq!(report.measurement_time_s.to_bits(), 20.5_f64.to_bits());
        assert!((report.range_m - 1003.0).abs() < 1.0e-12);
        assert!((report.range_rate_m_s - 12.1).abs() < 1.0e-12);
        assert_eq!(report.azimuth_rad.to_bits(), 1.25_f64.to_bits());
        assert_eq!(report.elevation_rad.to_bits(), 0.5_f64.to_bits());
        assert_eq!(report.range_noise_m.to_bits(), 0.0_f64.to_bits());
        assert_eq!(report.range_rate_noise_m_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(report.azimuth_noise_rad.to_bits(), 0.0_f64.to_bits());
        assert_eq!(report.elevation_noise_rad.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            report.range_gauss_markov_bias_m.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            report.range_rate_gauss_markov_bias_m_s.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            report.azimuth_gauss_markov_bias_rad.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            report.elevation_gauss_markov_bias_rad.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn tracking_observable_report_uses_deterministic_instrument_noise() {
        let config = CommTrackingConfig {
            instrument_deck: None,
            instrument_deck_sha256: None,
            latency_s: 0.0,
            range_bias_m: 0.0,
            range_rate_bias_m_s: 0.0,
            azimuth_bias_deg: 0.0,
            elevation_bias_deg: 0.0,
            range_quantization_m: 0.0,
            range_rate_quantization_m_s: 0.0,
            angle_quantization_deg: 0.0,
        };
        let instrument = CommTrackingInstrumentDeck {
            range_noise_stddev_m: 1.0,
            range_rate_noise_stddev_m_s: 0.01,
            azimuth_noise_stddev_deg: 0.001,
            elevation_noise_stddev_deg: 0.002,
            ..CommTrackingInstrumentDeck::default()
        };
        let model = tracking_model_from_config(&config, instrument).expect("tracking validates");
        let sample = VisibilitySample {
            time_s: 20.0,
            site_id: "site".to_owned(),
            slant_range_m: 1000.0,
            range_rate_m_s: 12.0,
            elevation_rad: 0.5,
            azimuth_rad: 1.25,
            mask_elevation_rad: 0.0,
            visible: true,
        };
        let mut first = Vec::new();
        let mut second = Vec::new();
        let mut changed_seed = Vec::new();
        let mut first_bias = CommTrackingBiasRuntimeState::default();
        let mut second_bias = CommTrackingBiasRuntimeState::default();
        let mut changed_seed_bias = CommTrackingBiasRuntimeState::default();

        record_tracking_observable(
            Some(model),
            &mut first,
            &mut first_bias,
            7,
            StepIndex::new(9),
            &sample,
        )
        .expect("first tracking report records");
        record_tracking_observable(
            Some(model),
            &mut second,
            &mut second_bias,
            7,
            StepIndex::new(9),
            &sample,
        )
        .expect("second tracking report repeats");
        record_tracking_observable(
            Some(model),
            &mut changed_seed,
            &mut changed_seed_bias,
            8,
            StepIndex::new(9),
            &sample,
        )
        .expect("changed tracking report records");

        assert_eq!(first, second);
        assert_eq!(first.len(), 1);
        let report = &first[0];
        assert_ne!(report.range_noise_m.to_bits(), 0.0_f64.to_bits());
        assert_ne!(
            report.range_noise_m.to_bits(),
            changed_seed[0].range_noise_m.to_bits()
        );
        assert!((report.range_m - (1000.0 + report.range_noise_m)).abs() < 1.0e-12);
        assert!((report.range_rate_m_s - (12.0 + report.range_rate_noise_m_s)).abs() < 1.0e-12);
    }

    #[test]
    fn tracking_observable_report_advances_gauss_markov_bias_state() {
        let config = CommTrackingConfig {
            instrument_deck: None,
            instrument_deck_sha256: None,
            latency_s: 0.0,
            range_bias_m: 0.0,
            range_rate_bias_m_s: 0.0,
            azimuth_bias_deg: 0.0,
            elevation_bias_deg: 0.0,
            range_quantization_m: 0.0,
            range_rate_quantization_m_s: 0.0,
            angle_quantization_deg: 0.0,
        };
        let instrument = CommTrackingInstrumentDeck {
            range_gauss_markov_stddev_m: 2.0,
            range_gauss_markov_tau_s: 10.0,
            range_rate_gauss_markov_stddev_m_s: 0.2,
            range_rate_gauss_markov_tau_s: 20.0,
            azimuth_gauss_markov_stddev_deg: 0.001,
            azimuth_gauss_markov_tau_s: 30.0,
            elevation_gauss_markov_stddev_deg: 0.002,
            elevation_gauss_markov_tau_s: 40.0,
            ..CommTrackingInstrumentDeck::default()
        };
        let model = tracking_model_from_config(&config, instrument).expect("tracking validates");
        let first_sample = VisibilitySample {
            time_s: 20.0,
            site_id: "site".to_owned(),
            slant_range_m: 1000.0,
            range_rate_m_s: 12.0,
            elevation_rad: 0.5,
            azimuth_rad: 1.25,
            mask_elevation_rad: 0.0,
            visible: true,
        };
        let second_sample = VisibilitySample {
            time_s: 21.0,
            ..first_sample.clone()
        };
        let mut reports = Vec::new();
        let mut repeated_reports = Vec::new();
        let mut bias_state = CommTrackingBiasRuntimeState::default();
        let mut repeated_bias_state = CommTrackingBiasRuntimeState::default();

        record_tracking_observable(
            Some(model),
            &mut reports,
            &mut bias_state,
            7,
            StepIndex::new(9),
            &first_sample,
        )
        .expect("first tracking report records");
        record_tracking_observable(
            Some(model),
            &mut reports,
            &mut bias_state,
            7,
            StepIndex::new(10),
            &second_sample,
        )
        .expect("second tracking report records");
        record_tracking_observable(
            Some(model),
            &mut repeated_reports,
            &mut repeated_bias_state,
            7,
            StepIndex::new(9),
            &first_sample,
        )
        .expect("repeated first tracking report records");
        record_tracking_observable(
            Some(model),
            &mut repeated_reports,
            &mut repeated_bias_state,
            7,
            StepIndex::new(10),
            &second_sample,
        )
        .expect("repeated second tracking report records");

        assert_eq!(reports, repeated_reports);
        assert_eq!(reports.len(), 2);
        assert_ne!(
            reports[0].range_gauss_markov_bias_m.to_bits(),
            0.0_f64.to_bits()
        );
        assert_ne!(
            reports[1].range_gauss_markov_bias_m.to_bits(),
            reports[0].range_gauss_markov_bias_m.to_bits()
        );
        assert!(
            (reports[0].range_m
                - (1000.0 + reports[0].range_gauss_markov_bias_m + reports[0].range_noise_m))
                .abs()
                < 1.0e-12
        );
        assert!(
            (reports[1].range_rate_m_s
                - (12.0
                    + reports[1].range_rate_gauss_markov_bias_m_s
                    + reports[1].range_rate_noise_m_s))
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn tracking_observables_csv_is_byte_stable() {
        let reports = vec![CommTrackingObservableReport {
            step: 7,
            site_id: "site, \"alpha\"".to_owned(),
            truth_time_s: 1.0,
            measurement_time_s: 1.25,
            range_m: 1000.0,
            range_rate_m_s: -2.0,
            azimuth_rad: 0.5,
            elevation_rad: 0.25,
            range_noise_m: 0.125,
            range_rate_noise_m_s: -0.03125,
            azimuth_noise_rad: 0.0,
            elevation_noise_rad: 0.0,
            range_gauss_markov_bias_m: 2.0,
            range_rate_gauss_markov_bias_m_s: 0.25,
            azimuth_gauss_markov_bias_rad: -0.5,
            elevation_gauss_markov_bias_rad: 0.75,
        }];
        let mut first = Vec::new();
        let mut second = Vec::new();

        write_tracking_observables_csv(&reports, &mut first).expect("first CSV writes");
        write_tracking_observables_csv(&reports, &mut second).expect("second CSV writes");

        assert_eq!(first, second);
        let text = std::str::from_utf8(&first).expect("CSV is UTF-8");
        assert!(text.starts_with(
            "step,site_id,truth_time_s,measurement_time_s,range_m,range_rate_m_s,azimuth_rad,elevation_rad"
        ));
        assert!(text.contains("7,\"site, \"\"alpha\"\"\","));
        assert!(text.contains(",1.00000000000000000e3,"));
        assert!(text.ends_with("7.50000000000000000e-1\n"));
    }

    #[test]
    fn tracking_observable_batch_gauss_newton_recovers_biases_with_crlb_covariance() {
        let config = CommTrackingConfig {
            instrument_deck: None,
            instrument_deck_sha256: None,
            latency_s: 0.0,
            range_bias_m: 12.0,
            range_rate_bias_m_s: -0.4,
            azimuth_bias_deg: 0.02,
            elevation_bias_deg: -0.01,
            range_quantization_m: 0.0,
            range_rate_quantization_m_s: 0.0,
            angle_quantization_deg: 0.0,
        };
        let model = tracking_model_from_config(&config, CommTrackingInstrumentDeck::default())
            .expect("tracking config validates");
        let samples = [
            VisibilitySample {
                time_s: 0.0,
                site_id: "site".to_owned(),
                slant_range_m: 1000.0,
                range_rate_m_s: -12.0,
                elevation_rad: 0.3,
                azimuth_rad: 0.2,
                mask_elevation_rad: 0.0,
                visible: true,
            },
            VisibilitySample {
                time_s: 1.0,
                site_id: "site".to_owned(),
                slant_range_m: 980.0,
                range_rate_m_s: -10.0,
                elevation_rad: 0.35,
                azimuth_rad: 0.24,
                mask_elevation_rad: 0.0,
                visible: true,
            },
            VisibilitySample {
                time_s: 2.0,
                site_id: "site".to_owned(),
                slant_range_m: 970.0,
                range_rate_m_s: -8.0,
                elevation_rad: 0.4,
                azimuth_rad: 0.28,
                mask_elevation_rad: 0.0,
                visible: true,
            },
        ];
        let mut reports = Vec::new();
        let mut bias_state = CommTrackingBiasRuntimeState::default();
        for (index, sample) in samples.iter().enumerate() {
            record_tracking_observable(
                Some(model),
                &mut reports,
                &mut bias_state,
                7,
                StepIndex::new(index as u64),
                sample,
            )
            .expect("tracking report records");
        }

        let range_variance_m2 = 4.0;
        let range_rate_variance_m2_s2 = 0.25;
        let azimuth_variance_rad2 = 1.0e-6;
        let elevation_variance_rad2 = 2.0e-6;
        let mut observations = Vec::new();
        for (report, sample) in reports.iter().zip(samples.iter()) {
            observations.push(BatchObservation {
                sensitivity: DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]),
                residual: report.range_m - sample.slant_range_m,
                variance: range_variance_m2,
            });
            observations.push(BatchObservation {
                sensitivity: DVector::from_vec(vec![0.0, 1.0, 0.0, 0.0]),
                residual: report.range_rate_m_s - sample.range_rate_m_s,
                variance: range_rate_variance_m2_s2,
            });
            observations.push(BatchObservation {
                sensitivity: DVector::from_vec(vec![0.0, 0.0, 1.0, 0.0]),
                residual: report.azimuth_rad - sample.azimuth_rad,
                variance: azimuth_variance_rad2,
            });
            observations.push(BatchObservation {
                sensitivity: DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                residual: report.elevation_rad - sample.elevation_rad,
                variance: elevation_variance_rad2,
            });
        }

        let bet = batch_gauss_newton(4, &observations).expect("tracking BET solves");
        let expected = DVector::from_vec(vec![
            config.range_bias_m,
            config.range_rate_bias_m_s,
            config.azimuth_bias_deg.to_radians(),
            config.elevation_bias_deg.to_radians(),
        ]);
        let error = &bet.update - &expected;

        assert!(error.norm() <= 1.0e-12);
        assert!(
            normalized_error_squared(&error, &bet.covariance).expect("NEES evaluates") <= 1.0e-18
        );
        let sample_count = samples.len() as f64;
        let expected_crlb = [
            range_variance_m2 / sample_count,
            range_rate_variance_m2_s2 / sample_count,
            azimuth_variance_rad2 / sample_count,
            elevation_variance_rad2 / sample_count,
        ];
        for (index, expected_variance) in expected_crlb.iter().enumerate() {
            assert!(
                (bet.covariance[(index, index)] - expected_variance).abs()
                    <= expected_variance.abs() * 1.0e-12 + 1.0e-18
            );
            assert!((bet.normal_matrix[(index, index)] - 1.0 / expected_variance).abs() <= 1.0e-9);
        }
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
