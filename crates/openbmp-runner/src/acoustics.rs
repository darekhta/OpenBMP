//! Runner-side acoustics artifact assembly.
//!
//! This module keeps acoustics analysis off the force path. The runner maps
//! opt-in scenario declarations into `openbmp-acoustics` and records
//! out-of-band report bytes; default scenarios do not allocate or emit
//! acoustics artifacts.

use std::{collections::BTreeMap, sync::OnceLock};

use openbmp_acoustics::ascent::{
    AscentAcousticDeck, AscentAcousticInput, AscentAcousticTracePrediction,
    AscentAcousticTraceSample, FairingInternalDeck, FairingInternalInput,
    FairingInternalPrediction,
};
use openbmp_acoustics::dsm::{
    DsmAirAbsorptionDeck, DsmApparentSourceCurve, DsmAxialSpectrumCurve, DsmDirectivityDeck,
    DsmExitCondition, DsmLiftoffStep, DsmLiftoffTrace, DsmLiftoffTracePoint, DsmPlumeDeflection,
    DsmPredictionDeck, DsmPropagation, DsmReceiver, DsmSourceAllocation, DsmSourceBandWeights,
    DsmSpectrumShape, DsmStationSpectrum, Sp8072ApparentSourceCurveDeck, Sp8072AxialPowerCurveDeck,
    Sp8072AxialSpectrumCurveDeck, Sp8072DirectivityCurveDeck, Sp8072SourceManifest,
    Sp8072SpectrumCurveDeck, evaluate_apparent_source_liftoff_trace,
    evaluate_apparent_source_station, evaluate_deflected_apparent_source_station,
    evaluate_deflected_free_field_station, evaluate_deflected_source_band_weighted_station,
    evaluate_free_field_liftoff_trace, evaluate_free_field_station,
    evaluate_source_band_weighted_liftoff_trace, evaluate_source_band_weighted_station,
    sp8072_core_length_m,
};
use openbmp_acoustics::iop::{
    IopDuctCoupling, IopDuctDeck, IopPulseInput, IopStartTransient, IopSuppressionDeck,
    IopUncertainty, evaluate_iop_pulse,
};
use openbmp_acoustics::pad::{PadClusterInteractionDeck, PadSuppressionDeck};
use openbmp_acoustics::response::{
    AccelerationPsd, AccelerationPsdSpectrum, ForcePsd, ForcePsdSpectrum,
    VibroacousticResponseInput, VibroacousticResponsePrediction, VibroacousticTransferDeck,
};
use openbmp_acoustics::stats::{
    P95_50_CONFIDENCE_PROBABILITY, P95_50_COVERAGE_PROBABILITY, SplP95_50Environment,
    evaluate_spl_p95_50_environment, one_sided_normal_tolerance_factor,
};
use openbmp_acoustics::{AcousticBand, Spl, SplSpectrum};
use openbmp_core::{ChannelId, EngineId};
use openbmp_mc::{wilks_one_sided_confidence, wilks_one_sided_n};
use openbmp_scenario::{
    AcousticsAscentClassSelectionConfig, AcousticsAscentConfig, AcousticsDsmConfig,
    AcousticsFairingInternalConfig, AcousticsIopConfig, AcousticsResponseConfig,
    AcousticsResponseSourceConfig, EventTriggerConfig, ResolvedFile, ScenarioActionConfig,
    ScenarioDocument,
};
use openbmp_sim::{FiredEvent, ScenarioScriptAction};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};

use crate::error::RunnerError;

/// Acoustics evidence emitted by a run with `[acoustics]`.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsRunReport {
    /// Free-field DSM report, when `[acoustics.dsm]` is declared.
    pub dsm: Option<AcousticsDsmRunReport>,
    /// Ignition-overpressure report, when `[acoustics.iop]` is declared.
    pub iop: Option<AcousticsIopRunReport>,
    /// Ascent acoustic environment report, when `[acoustics.ascent]` is declared.
    pub ascent: Option<AcousticsAscentRunReport>,
    /// Fairing-internal acoustic report, when `[acoustics.fairing_internal]` is declared.
    pub fairing_internal: Option<AcousticsFairingInternalRunReport>,
    /// Vibroacoustic response report, when `[acoustics.response]` is declared.
    pub response: Option<AcousticsResponseRunReport>,
}

/// Runner evidence for one ascent acoustic environment declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsAscentRunReport {
    /// Stable receiver/station id.
    pub station_id: String,
    /// Selected ascent acoustic class id.
    pub class_id: String,
    /// Dynamic pressure sample, Pa.
    pub dynamic_pressure_pa: f64,
    /// Mach sample.
    pub mach: f64,
    /// Number of samples in the evaluated ascent trace.
    pub sample_count: usize,
    /// Index of the sample with the largest individual OASPL.
    pub peak_sample_index: usize,
    /// Optional id of the peak sample.
    pub peak_sample_id: Option<String>,
    /// Individual OASPL of the peak sample, dB re 20 uPa.
    pub peak_sample_oaspl_db: f64,
    /// SHA-256 of the ascent acoustic deck.
    pub deck_file_sha256_hex: String,
    /// Dataset id declared by the parsed ascent acoustic deck.
    pub deck_dataset_id: String,
    /// Source class declared by the parsed ascent acoustic deck.
    pub deck_source_class: String,
    /// Declared class uncertainty, dB.
    pub uncertainty_db: f64,
    /// Predicted external OASPL, dB re 20 uPa.
    pub oaspl_db: f64,
    /// Deterministic one-third-octave spectrum CSV bytes.
    pub spectrum_csv: Vec<u8>,
    /// Deterministic per-sample trace CSV bytes.
    pub sample_trace_csv: Vec<u8>,
    /// P95/50 OASPL from trace samples when at least two samples are declared.
    pub p95_50_oaspl_db: Option<f64>,
    /// P95/50 normal tolerance factor when computed.
    pub p95_50_tolerance_factor: Option<f64>,
    /// Deterministic P95/50 environment table CSV bytes when computed.
    pub p95_50_environment_csv: Vec<u8>,
    /// One-sided Wilks sample count required for the P95/50 target.
    pub p95_50_wilks_required_sample_count: Option<u64>,
    /// One-sided Wilks confidence achieved by the declared sample count.
    pub p95_50_wilks_achieved_confidence: Option<f64>,
    /// Whether the declared sample count satisfies the P95/50 Wilks floor.
    pub p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Deterministic one-row Wilks sizing CSV bytes when P95/50 is computed.
    pub p95_50_wilks_sizing_csv: Vec<u8>,
}

/// Runner evidence for one fairing-internal acoustic transfer declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsFairingInternalRunReport {
    /// Stable receiver/station id.
    pub station_id: String,
    /// Selected fairing-internal acoustic class id.
    pub class_id: String,
    /// Dimensionless payload fill factor.
    pub fill_factor: f64,
    /// Fill-factor gain applied to the fairing transfer, dB.
    pub fill_factor_gain_db: f64,
    /// SHA-256 of the fairing-internal acoustic deck.
    pub deck_file_sha256_hex: String,
    /// Dataset id declared by the parsed fairing-internal acoustic deck.
    pub deck_dataset_id: String,
    /// Source class declared by the parsed fairing-internal acoustic deck.
    pub deck_source_class: String,
    /// Declared class uncertainty, dB.
    pub uncertainty_db: f64,
    /// Predicted internal OASPL, dB re 20 uPa.
    pub oaspl_db: f64,
    /// Deterministic one-third-octave internal spectrum CSV bytes.
    pub spectrum_csv: Vec<u8>,
    /// P95/50 OASPL from transferred trace samples when at least two samples are declared.
    pub p95_50_oaspl_db: Option<f64>,
    /// P95/50 normal tolerance factor when computed.
    pub p95_50_tolerance_factor: Option<f64>,
    /// Deterministic P95/50 internal environment table CSV bytes when computed.
    pub p95_50_environment_csv: Vec<u8>,
    /// Deterministic per-sample fairing P95/50 input trace CSV bytes when computed.
    pub p95_50_sample_trace_csv: Vec<u8>,
    /// One-sided Wilks sample count required for the P95/50 target.
    pub p95_50_wilks_required_sample_count: Option<u64>,
    /// One-sided Wilks confidence achieved by the declared sample count.
    pub p95_50_wilks_achieved_confidence: Option<f64>,
    /// Whether the declared sample count satisfies the P95/50 Wilks floor.
    pub p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Deterministic one-row Wilks sizing CSV bytes when P95/50 is computed.
    pub p95_50_wilks_sizing_csv: Vec<u8>,
}

/// Runner evidence for one vibroacoustic response transfer declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsResponseRunReport {
    /// Stable receiver/station id.
    pub station_id: String,
    /// Acoustic spectrum source used for the response transfer.
    pub source: AcousticsResponseSourceConfig,
    /// Selected vibroacoustic transfer class id.
    pub class_id: String,
    /// Effective surface mass, kg/m^2.
    pub surface_mass_kg_m2: f64,
    /// Acoustic zone area, m^2.
    pub area_m2: f64,
    /// Dimensionless modal participation over the zone.
    pub mode_shape_factor: f64,
    /// Applied modal-force participation, `area_m2 * mode_shape_factor`.
    pub modal_force_participation_m2: f64,
    /// Applied surface-mass response gain.
    pub surface_mass_gain: f64,
    /// SHA-256 of the vibroacoustic response deck.
    pub deck_file_sha256_hex: String,
    /// Dataset id declared by the parsed vibroacoustic response deck.
    pub deck_dataset_id: String,
    /// Source class declared by the parsed vibroacoustic response deck.
    pub deck_source_class: String,
    /// Declared class uncertainty, dB.
    pub uncertainty_db: f64,
    /// Total integrated mean-square acceleration, `(m/s^2)^2`.
    pub total_mean_square_acceleration_m2_s4: f64,
    /// Total integrated modal-force power, `N^2`.
    pub total_mean_square_force_n2: f64,
    /// Deterministic one-third-octave acceleration PSD CSV bytes.
    pub acceleration_psd_csv: Vec<u8>,
    /// Deterministic one-third-octave modal-force PSD CSV bytes.
    pub modal_force_psd_csv: Vec<u8>,
    /// Typed modal-force PSD spectrum consumed by the structural loads stack.
    pub modal_force_psd: ForcePsdSpectrum,
    /// Deterministic structural loads handoff CSV bytes for modal-force PSD.
    pub loads_handoff_csv: Vec<u8>,
    /// P95/50 normal tolerance factor when computed from source trace samples.
    pub p95_50_tolerance_factor: Option<f64>,
    /// P95/50 total integrated mean-square acceleration, `(m/s^2)^2`.
    pub p95_50_total_mean_square_acceleration_m2_s4: Option<f64>,
    /// P95/50 total integrated modal-force power, `N^2`.
    pub p95_50_total_mean_square_force_n2: Option<f64>,
    /// Deterministic P95/50 acceleration PSD CSV bytes when computed.
    pub p95_50_acceleration_psd_csv: Vec<u8>,
    /// Deterministic P95/50 modal-force PSD CSV bytes when computed.
    pub p95_50_modal_force_psd_csv: Vec<u8>,
    /// Deterministic per-sample response input/evaluation CSV bytes when P95/50 is computed.
    pub p95_50_sample_trace_csv: Vec<u8>,
    /// One-sided Wilks sample count required for the P95/50 target.
    pub p95_50_wilks_required_sample_count: Option<u64>,
    /// One-sided Wilks confidence achieved by the declared sample count.
    pub p95_50_wilks_achieved_confidence: Option<f64>,
    /// Whether the declared sample count satisfies the P95/50 Wilks floor.
    pub p95_50_wilks_sample_count_sufficient: Option<bool>,
    /// Deterministic one-row Wilks sizing CSV bytes when P95/50 is computed.
    pub p95_50_wilks_sizing_csv: Vec<u8>,
}

/// Runner evidence for one ignition-overpressure declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsIopRunReport {
    /// Stable receiver/station id.
    pub station_id: String,
    /// Engine id used to derive the start source, when declared.
    pub source_engine_id: Option<String>,
    /// First declared at-time ignite event id for the source engine, when found.
    pub source_engine_ignition_event_id: Option<String>,
    /// First declared at-time ignite event time for the source engine, s.
    pub source_engine_ignition_time_s: Option<f64>,
    /// First runtime-observed source-engine ignite binding id, when fired.
    pub source_engine_observed_ignition_event_id: Option<u64>,
    /// First runtime-observed source-engine ignite step, when fired.
    pub source_engine_observed_ignition_step: Option<u64>,
    /// First runtime-observed source-engine ignite time, s.
    pub source_engine_observed_ignition_time_s: Option<f64>,
    /// Effective engine max thrust for the referenced source engine, N.
    pub source_engine_max_thrust_n: Option<f64>,
    /// Nominal `mdot*c*` source strength used for thermochemical source derivation, N.
    pub source_engine_mdot_cstar_n: Option<f64>,
    /// Effective `c*` used to reconstruct runtime thermochemical source strength, m/s.
    pub source_engine_effective_cstar_m_s: Option<f64>,
    /// Whether thermochemical performance drove source-engine derivation.
    pub source_engine_thermochemical_performance: bool,
    /// Number of runtime source-engine samples recorded into the IOP history.
    pub source_engine_runtime_sample_count: usize,
    /// Last runtime source-strength sample, N.
    pub source_engine_runtime_last_source_strength_n: Option<f64>,
    /// Last runtime source-history sample time, s.
    pub source_engine_runtime_last_time_s: Option<f64>,
    /// Peak positive finite-difference source rate from runtime samples, N/s.
    pub source_engine_runtime_peak_source_rate_n_per_s: Option<f64>,
    /// Runtime-reconstructed nominal peak overpressure from the observed peak source rate, Pa.
    pub source_engine_runtime_peak_overpressure_pa: Option<f64>,
    /// Runtime-reconstructed lower uncertainty-bound peak overpressure, Pa.
    pub source_engine_runtime_lower_peak_overpressure_pa: Option<f64>,
    /// Runtime-reconstructed upper uncertainty-bound peak overpressure, Pa.
    pub source_engine_runtime_upper_peak_overpressure_pa: Option<f64>,
    /// Runtime-reconstructed pulse-duration envelope, s.
    pub source_engine_runtime_duration_s: Option<f64>,
    /// Deterministic CSV history of runtime source-engine source strength.
    pub source_engine_runtime_history_csv: Vec<u8>,
    /// Source-rate term `d(mdot*c*)/dt`, N/s.
    pub gas_generation_rate_n_per_s: f64,
    /// Start-ramp duration, s.
    pub ramp_duration_s: f64,
    /// Duct/trench residence time, s.
    pub duct_residence_time_s: f64,
    /// Duct/trench volume, m^3.
    pub duct_volume_m3: f64,
    /// Effective duct/trench exit area, m^2.
    pub duct_exit_area_m2: f64,
    /// Effective duct/trench discharge velocity, m/s.
    pub duct_discharge_velocity_m_s: f64,
    /// Non-negative duct/trench geometry gain.
    pub duct_geometry_gain: f64,
    /// SHA-256 of the IOP duct deck, when declared.
    pub duct_deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed IOP duct deck, when present.
    pub duct_deck_dataset_id: Option<String>,
    /// Source class declared by the parsed IOP duct deck, when present.
    pub duct_deck_source_class: Option<String>,
    /// Duct class selected from the IOP duct deck, when present.
    pub duct_class_id: Option<String>,
    /// Receiver distance from the IOP source, m.
    pub receiver_distance_m: f64,
    /// Total pressure-amplitude suppression loss applied, dB.
    pub suppression_loss_db: f64,
    /// SHA-256 of the IOP suppression deck, when declared.
    pub suppression_deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed IOP suppression deck, when present.
    pub suppression_deck_dataset_id: Option<String>,
    /// Source class declared by the parsed IOP suppression deck, when present.
    pub suppression_deck_source_class: Option<String>,
    /// Suppression class selected from the IOP suppression deck, when present.
    pub suppression_class_id: Option<String>,
    /// Pressure-amplitude loss from the selected suppression class, dB.
    pub suppression_class_loss_db: Option<f64>,
    /// Declared uncertainty for the selected suppression class, dB.
    pub suppression_class_uncertainty_db: Option<f64>,
    /// Lower-side peak-pressure uncertainty, dB.
    pub uncertainty_minus_db: f64,
    /// Upper-side peak-pressure uncertainty, dB.
    pub uncertainty_plus_db: f64,
    /// Nominal peak overpressure, Pa.
    pub peak_overpressure_pa: f64,
    /// Lower uncertainty-bound peak overpressure, Pa.
    pub lower_peak_overpressure_pa: f64,
    /// Upper uncertainty-bound peak overpressure, Pa.
    pub upper_peak_overpressure_pa: f64,
    /// Pulse-duration envelope, s.
    pub duration_s: f64,
}

/// Runner evidence for one free-field single-engine DSM declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticsDsmRunReport {
    /// Stable receiver/station id.
    pub station_id: String,
    /// Number of source-line points used after validation.
    pub source_count: usize,
    /// Identical engine count used for incoherent cluster power scaling.
    pub cluster_engine_count: u32,
    /// Acoustic power gain from identical-engine cluster scaling, dB.
    pub cluster_power_gain_db: f64,
    /// Scalar cross-engine pressure correlation coefficient used for clusters.
    pub cluster_correlation_coefficient: f64,
    /// Broadband cluster shielding loss applied after summation, dB.
    pub cluster_shielding_loss_db: f64,
    /// SHA-256 of the pad cluster interaction deck, when declared.
    pub cluster_interaction_deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed cluster interaction deck, when present.
    pub cluster_interaction_deck_dataset_id: Option<String>,
    /// Source class declared by the parsed cluster interaction deck, when present.
    pub cluster_interaction_deck_source_class: Option<String>,
    /// Cluster interaction class selected from the deck, when present.
    pub cluster_interaction_class_id: Option<String>,
    /// Declared cluster interaction uncertainty for the selected class, dB.
    pub cluster_interaction_uncertainty_db: Option<f64>,
    /// Extra pad duct/trench/deflector propagation path length, metres.
    pub pad_path_length_extra_m: f64,
    /// Axial origin of a declared single-bend plume deflection, metres.
    pub pad_deflection_origin_axial_m: Option<f64>,
    /// Declared single-bend plume deflection angle, degrees.
    pub pad_deflection_angle_deg: Option<f64>,
    /// Acoustic power from the declared exit condition, watts.
    pub acoustic_power_w: f64,
    /// Sound-power level from the declared exit condition, dB re 1 pW.
    pub sound_power_level_db: f64,
    /// OASPL at the declared zero-height station, dB re 20 uPa.
    pub station_oaspl_db: f64,
    /// SHA-256 of the DSM deck file, when `deck_file` is declared.
    pub deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed DSM deck, when present.
    pub deck_dataset_id: Option<String>,
    /// Source class declared by the parsed DSM deck, when present.
    pub deck_source_class: Option<String>,
    /// SHA-256 of the SP-8072 source curve deck, when `source_curve_file` is declared.
    pub source_curve_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed source curve deck, when present.
    pub source_curve_dataset_id: Option<String>,
    /// SP-8072 figure id declared by the parsed source curve deck, when present.
    pub source_curve_figure_id: Option<String>,
    /// Core length derived for a source curve deck, metres.
    pub source_curve_core_length_m: Option<f64>,
    /// SHA-256 of the SP-8072 apparent-source curve deck, when declared.
    pub apparent_source_curve_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed apparent-source curve deck, when present.
    pub apparent_source_curve_dataset_id: Option<String>,
    /// SP-8072 figure id declared by the parsed apparent-source curve deck, when present.
    pub apparent_source_curve_figure_id: Option<String>,
    /// SHA-256 of the SP-8072 axial spectrum curve deck, when declared.
    pub axial_spectrum_curve_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed axial spectrum curve deck, when present.
    pub axial_spectrum_curve_dataset_id: Option<String>,
    /// SP-8072 figure id declared by the parsed axial spectrum curve deck, when present.
    pub axial_spectrum_curve_figure_id: Option<String>,
    /// SHA-256 of the SP-8072 spectrum curve deck, when declared.
    pub spectrum_curve_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed spectrum curve deck, when present.
    pub spectrum_curve_dataset_id: Option<String>,
    /// SP-8072 figure id declared by the parsed spectrum curve deck, when present.
    pub spectrum_curve_figure_id: Option<String>,
    /// SHA-256 of the SP-8072 directivity curve deck, when declared.
    pub directivity_curve_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed directivity curve deck, when present.
    pub directivity_curve_dataset_id: Option<String>,
    /// SP-8072 figure id declared by the parsed directivity curve deck, when present.
    pub directivity_curve_figure_id: Option<String>,
    /// SHA-256 of the DSM air-absorption deck, when declared.
    pub absorption_deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed air-absorption deck, when present.
    pub absorption_deck_dataset_id: Option<String>,
    /// Source class declared by the parsed air-absorption deck, when present.
    pub absorption_deck_source_class: Option<String>,
    /// Selected air-absorption atmosphere profile id, when a profile deck is used.
    pub absorption_profile_id: Option<String>,
    /// SHA-256 of the pad suppression deck, when declared.
    pub suppression_deck_file_sha256_hex: Option<String>,
    /// Dataset id declared by the parsed pad suppression deck, when present.
    pub suppression_deck_dataset_id: Option<String>,
    /// Source class declared by the parsed pad suppression deck, when present.
    pub suppression_deck_source_class: Option<String>,
    /// Suppression class selected from the pad suppression deck, when present.
    pub suppression_class_id: Option<String>,
    /// Declared suppression uncertainty for the selected class, dB.
    pub suppression_uncertainty_db: Option<f64>,
    /// Deterministic station spectrum CSV bytes.
    pub station_spectrum_csv: Vec<u8>,
    /// Deterministic OASPL-vs-height trace CSV bytes.
    pub liftoff_oaspl_csv: Vec<u8>,
}

struct DsmSpectrumInput {
    shape: DsmSpectrumShape,
    curve_file_sha256_hex: Option<String>,
    curve_dataset_id: Option<String>,
    curve_figure_id: Option<String>,
}

struct DsmDirectivityInput {
    deck: DsmDirectivityDeck,
    curve_file_sha256_hex: Option<String>,
    curve_dataset_id: Option<String>,
    curve_figure_id: Option<String>,
}

enum DsmSourceInput {
    Distributed(DsmSourceAllocation),
    Apparent(DsmApparentSourceCurve),
}

struct IopStartSource {
    transient: IopStartTransient,
    gas_generation_rate_n_per_s: f64,
    ramp_duration_s: f64,
    source_engine_id: Option<String>,
    source_engine_ignition_event_id: Option<String>,
    source_engine_ignition_time_s: Option<f64>,
    source_engine_max_thrust_n: Option<f64>,
    source_engine_mdot_cstar_n: Option<f64>,
    source_engine_effective_cstar_m_s: Option<f64>,
    source_engine_thermochemical_performance: bool,
}

struct DsmPropagationInput {
    propagation: DsmPropagation,
    absorption_deck_file_sha256_hex: Option<String>,
    absorption_deck_dataset_id: Option<String>,
    absorption_deck_source_class: Option<String>,
    absorption_profile_id: Option<String>,
}

struct DsmSuppressionInput {
    directivity: DsmDirectivityDeck,
    suppression_deck_file_sha256_hex: Option<String>,
    suppression_deck_dataset_id: Option<String>,
    suppression_deck_source_class: Option<String>,
    suppression_class_id: Option<String>,
    suppression_uncertainty_db: Option<f64>,
}

struct DsmClusterEngineInput {
    offset_x_m: f64,
    offset_y_m: f64,
}

#[derive(Clone, Debug)]
struct DsmClusterInteraction {
    correlation_coefficient: f64,
    shielding_loss_db: f64,
    correlation_by_band: Option<Vec<f64>>,
    shielding_loss_by_band_db: Option<Vec<f64>>,
}

#[derive(Clone, Debug)]
struct DsmClusterInteractionInput {
    interaction: DsmClusterInteraction,
    deck_file_sha256_hex: Option<String>,
    deck_dataset_id: Option<String>,
    deck_source_class: Option<String>,
    class_id: Option<String>,
    uncertainty_db: Option<f64>,
}

struct AscentEvaluation {
    report: AcousticsAscentRunReport,
    spectrum: SplSpectrum,
    sample_spectra: Vec<SplSpectrum>,
}

struct FairingInternalEvaluation {
    report: AcousticsFairingInternalRunReport,
    spectrum: SplSpectrum,
    sample_spectra: Vec<SplSpectrum>,
}

struct FairingInternalSamplePrediction {
    sample_index: usize,
    sample_id: Option<String>,
    class_id: String,
    input: FairingInternalInput,
    prediction: FairingInternalPrediction,
}

struct ResponseSourceEvaluation<'a> {
    spectrum: &'a SplSpectrum,
    sample_spectra: &'a [SplSpectrum],
}

struct ResponseP95_50Artifacts {
    tolerance_factor: f64,
    total_mean_square_acceleration_m2_s4: f64,
    total_mean_square_force_n2: f64,
    acceleration_psd_csv: Vec<u8>,
    modal_force_psd_csv: Vec<u8>,
    sample_trace_csv: Vec<u8>,
    wilks_sizing: P95_50WilksSizing,
}

struct ResponseP95_50SamplePrediction {
    sample_index: usize,
    sample_id: Option<String>,
    class_id: String,
    input: VibroacousticResponseInput,
    prediction: VibroacousticResponsePrediction,
}

#[derive(Clone, Copy, Debug)]
struct ResponseP95_50BandStatistic {
    band: AcousticBand,
    mean_psd_db: f64,
    sigma_psd_db: f64,
    p95_50_psd_db: f64,
    p95_50_psd_value: f64,
    min_sample_psd_db: f64,
    max_sample_psd_db: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct P95_50WilksSizing {
    sample_count: u64,
    required_sample_count: u64,
    achieved_confidence: f64,
    sample_count_sufficient: bool,
}

impl P95_50WilksSizing {
    fn to_csv(self) -> Vec<u8> {
        format!(
            "coverage_probability,confidence_probability,sample_count,required_sample_count,achieved_confidence,sample_count_sufficient\n{:.17e},{:.17e},{},{},{:.17e},{}\n",
            P95_50_COVERAGE_PROBABILITY,
            P95_50_CONFIDENCE_PROBABILITY,
            self.sample_count,
            self.required_sample_count,
            self.achieved_confidence,
            self.sample_count_sufficient,
        )
        .into_bytes()
    }
}

#[derive(Clone, Copy)]
struct DsmEvaluationContext<'a> {
    source_input: &'a DsmSourceInput,
    axial_spectrum_curve: Option<&'a DsmAxialSpectrumCurve>,
    shape: &'a DsmSpectrumShape,
    directivity: &'a DsmDirectivityDeck,
    steps: &'a [DsmLiftoffStep],
    plume_deflection: Option<DsmPlumeDeflection>,
}

/// Telemetry channels emitted when `[acoustics.dsm]` is declared.
#[derive(Debug)]
pub(crate) struct AcousticsTelemetryChannels {
    dsm: Option<AcousticsDsmTelemetryChannels>,
    iop: Option<AcousticsIopTelemetryChannels>,
    ascent: Option<AcousticsAscentTelemetryChannels>,
    fairing_internal: Option<AcousticsFairingInternalTelemetryChannels>,
    response: Option<AcousticsResponseTelemetryChannels>,
}

#[derive(Debug)]
struct AcousticsDsmTelemetryChannels {
    acoustic_power: TelemetryChannel<f64>,
    sound_power_level: TelemetryChannel<f64>,
    station_oaspl: TelemetryChannel<f64>,
    source_count: TelemetryChannel<i64>,
}

#[derive(Debug)]
struct AcousticsIopTelemetryChannels {
    peak_overpressure: TelemetryChannel<f64>,
    lower_peak_overpressure: TelemetryChannel<f64>,
    upper_peak_overpressure: TelemetryChannel<f64>,
    duration: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct AcousticsAscentTelemetryChannels {
    oaspl: TelemetryChannel<f64>,
    uncertainty: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct AcousticsFairingInternalTelemetryChannels {
    oaspl: TelemetryChannel<f64>,
    uncertainty: TelemetryChannel<f64>,
    fill_factor_gain: TelemetryChannel<f64>,
}

#[derive(Debug)]
struct AcousticsResponseTelemetryChannels {
    total_mean_square_acceleration: TelemetryChannel<f64>,
    total_mean_square_force: TelemetryChannel<f64>,
    uncertainty: TelemetryChannel<f64>,
}

impl AcousticsTelemetryChannels {
    /// Construct acoustics telemetry channels when `[acoustics]` is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when a channel cannot be built.
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        alloc: &mut impl FnMut() -> ChannelId,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(acoustics) = &document.acoustics else {
            return Ok(None);
        };
        let dsm = acoustics
            .dsm
            .as_ref()
            .map(|_| {
                Ok::<_, RunnerError>(AcousticsDsmTelemetryChannels {
                    acoustic_power: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.dsm.acoustic_power_w",
                        "W",
                        None::<&str>,
                    )?,
                    sound_power_level: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.dsm.sound_power_level_db",
                        "dB",
                        None::<&str>,
                    )?,
                    station_oaspl: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.dsm.station_oaspl_db",
                        "dB",
                        None::<&str>,
                    )?,
                    source_count: TelemetryChannel::<i64>::new(
                        alloc(),
                        "acoustics.dsm.source_count",
                        "1",
                        None::<&str>,
                    )?,
                })
            })
            .transpose()?;
        let iop = acoustics
            .iop
            .as_ref()
            .map(|_| {
                Ok::<_, RunnerError>(AcousticsIopTelemetryChannels {
                    peak_overpressure: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.iop.peak_overpressure_pa",
                        "Pa",
                        None::<&str>,
                    )?,
                    lower_peak_overpressure: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.iop.lower_peak_overpressure_pa",
                        "Pa",
                        None::<&str>,
                    )?,
                    upper_peak_overpressure: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.iop.upper_peak_overpressure_pa",
                        "Pa",
                        None::<&str>,
                    )?,
                    duration: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.iop.duration_s",
                        "s",
                        None::<&str>,
                    )?,
                })
            })
            .transpose()?;
        let ascent = acoustics
            .ascent
            .as_ref()
            .map(|_| {
                Ok::<_, RunnerError>(AcousticsAscentTelemetryChannels {
                    oaspl: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.ascent.oaspl_db",
                        "dB",
                        None::<&str>,
                    )?,
                    uncertainty: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.ascent.uncertainty_db",
                        "dB",
                        None::<&str>,
                    )?,
                })
            })
            .transpose()?;
        let fairing_internal = acoustics
            .fairing_internal
            .as_ref()
            .map(|_| {
                Ok::<_, RunnerError>(AcousticsFairingInternalTelemetryChannels {
                    oaspl: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.fairing_internal.oaspl_db",
                        "dB",
                        None::<&str>,
                    )?,
                    uncertainty: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.fairing_internal.uncertainty_db",
                        "dB",
                        None::<&str>,
                    )?,
                    fill_factor_gain: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.fairing_internal.fill_factor_gain_db",
                        "dB",
                        None::<&str>,
                    )?,
                })
            })
            .transpose()?;
        let response = acoustics
            .response
            .as_ref()
            .map(|_| {
                Ok::<_, RunnerError>(AcousticsResponseTelemetryChannels {
                    total_mean_square_acceleration: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.response.total_mean_square_acceleration_m2_s4",
                        "m2/s4",
                        None::<&str>,
                    )?,
                    total_mean_square_force: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.response.total_mean_square_force_n2",
                        "N2",
                        None::<&str>,
                    )?,
                    uncertainty: TelemetryChannel::<f64>::new(
                        alloc(),
                        "acoustics.response.uncertainty_db",
                        "dB",
                        None::<&str>,
                    )?,
                })
            })
            .transpose()?;
        Ok(Some(Self {
            dsm,
            iop,
            ascent,
            fairing_internal,
            response,
        }))
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        if let Some(dsm) = &self.dsm {
            channels.push(dsm.acoustic_power.metadata().clone());
            channels.push(dsm.sound_power_level.metadata().clone());
            channels.push(dsm.station_oaspl.metadata().clone());
            channels.push(dsm.source_count.metadata().clone());
        }
        if let Some(iop) = &self.iop {
            channels.push(iop.peak_overpressure.metadata().clone());
            channels.push(iop.lower_peak_overpressure.metadata().clone());
            channels.push(iop.upper_peak_overpressure.metadata().clone());
            channels.push(iop.duration.metadata().clone());
        }
        if let Some(ascent) = &self.ascent {
            channels.push(ascent.oaspl.metadata().clone());
            channels.push(ascent.uncertainty.metadata().clone());
        }
        if let Some(fairing_internal) = &self.fairing_internal {
            channels.push(fairing_internal.oaspl.metadata().clone());
            channels.push(fairing_internal.uncertainty.metadata().clone());
            channels.push(fairing_internal.fill_factor_gain.metadata().clone());
        }
        if let Some(response) = &self.response {
            channels.push(response.total_mean_square_acceleration.metadata().clone());
            channels.push(response.total_mean_square_force.metadata().clone());
            channels.push(response.uncertainty.metadata().clone());
        }
    }

    /// Insert acoustics summary values from the precomputed run report.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when a row insertion fails or the
    /// report is missing for declared channels.
    pub(crate) fn insert_samples(
        &self,
        row: &mut TelemetryRow,
        report: Option<&AcousticsRunReport>,
    ) -> Result<(), RunnerError> {
        if let Some(dsm_channels) = &self.dsm {
            let dsm_report = report
                .and_then(|report| report.dsm.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[acoustics.dsm] telemetry requested without an acoustics report"
                        .to_owned(),
                })?;
            row.insert(&dsm_channels.acoustic_power, dsm_report.acoustic_power_w)?;
            row.insert(
                &dsm_channels.sound_power_level,
                dsm_report.sound_power_level_db,
            )?;
            row.insert(&dsm_channels.station_oaspl, dsm_report.station_oaspl_db)?;
            row.insert(&dsm_channels.source_count, dsm_report.source_count as i64)?;
        }
        if let Some(iop_channels) = &self.iop {
            let iop_report = report
                .and_then(|report| report.iop.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[acoustics.iop] telemetry requested without an acoustics report"
                        .to_owned(),
                })?;
            row.insert(
                &iop_channels.peak_overpressure,
                iop_report.peak_overpressure_pa,
            )?;
            row.insert(
                &iop_channels.lower_peak_overpressure,
                iop_report.lower_peak_overpressure_pa,
            )?;
            row.insert(
                &iop_channels.upper_peak_overpressure,
                iop_report.upper_peak_overpressure_pa,
            )?;
            row.insert(&iop_channels.duration, iop_report.duration_s)?;
        }
        if let Some(ascent_channels) = &self.ascent {
            let ascent_report = report
                .and_then(|report| report.ascent.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[acoustics.ascent] telemetry requested without an acoustics report"
                        .to_owned(),
                })?;
            row.insert(&ascent_channels.oaspl, ascent_report.oaspl_db)?;
            row.insert(&ascent_channels.uncertainty, ascent_report.uncertainty_db)?;
        }
        if let Some(fairing_channels) = &self.fairing_internal {
            let fairing_report = report
                .and_then(|report| report.fairing_internal.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[acoustics.fairing_internal] telemetry requested without an acoustics report"
                        .to_owned(),
                })?;
            row.insert(&fairing_channels.oaspl, fairing_report.oaspl_db)?;
            row.insert(&fairing_channels.uncertainty, fairing_report.uncertainty_db)?;
            row.insert(
                &fairing_channels.fill_factor_gain,
                fairing_report.fill_factor_gain_db,
            )?;
        }
        if let Some(response_channels) = &self.response {
            let response_report = report
                .and_then(|report| report.response.as_ref())
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "[acoustics.response] telemetry requested without an acoustics report"
                        .to_owned(),
                })?;
            row.insert(
                &response_channels.total_mean_square_acceleration,
                response_report.total_mean_square_acceleration_m2_s4,
            )?;
            row.insert(
                &response_channels.total_mean_square_force,
                response_report.total_mean_square_force_n2,
            )?;
            row.insert(
                &response_channels.uncertainty,
                response_report.uncertainty_db,
            )?;
        }
        Ok(())
    }
}

/// Evaluate the opt-in acoustics report for a parsed scenario.
///
/// # Errors
///
/// Returns [`RunnerError`] if the already-validated scenario cannot be mapped
/// to the acoustics crate. This should be unreachable for valid scenarios, but
/// remains fail-closed at the runner boundary.
pub(crate) fn evaluate_document(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<AcousticsRunReport>, RunnerError> {
    let Some(acoustics) = &document.acoustics else {
        return Ok(None);
    };
    let dsm = acoustics
        .dsm
        .as_ref()
        .map(|dsm| evaluate_dsm(dsm, resolved_files))
        .transpose()?;
    let iop = acoustics
        .iop
        .as_ref()
        .map(|iop| evaluate_iop(iop, document, resolved_files))
        .transpose()?;
    let ascent_evaluation = acoustics
        .ascent
        .as_ref()
        .map(|ascent| evaluate_ascent(ascent, resolved_files))
        .transpose()?;
    let fairing_internal_evaluation = acoustics
        .fairing_internal
        .as_ref()
        .map(|fairing_internal| {
            let ascent =
                ascent_evaluation
                    .as_ref()
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "[acoustics.fairing_internal] requires [acoustics.ascent]".to_owned(),
                    })?;
            evaluate_fairing_internal(fairing_internal, resolved_files, ascent)
        })
        .transpose()?;
    let response = acoustics
        .response
        .as_ref()
        .map(|response| {
            let source = match response.source {
                AcousticsResponseSourceConfig::Ascent => ascent_evaluation
                    .as_ref()
                    .map(|evaluation| ResponseSourceEvaluation {
                        spectrum: &evaluation.spectrum,
                        sample_spectra: &evaluation.sample_spectra,
                    })
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "[acoustics.response] source = \"ascent\" requires [acoustics.ascent]"
                            .to_owned(),
                    })?,
                AcousticsResponseSourceConfig::FairingInternal => fairing_internal_evaluation
                    .as_ref()
                    .map(|evaluation| ResponseSourceEvaluation {
                        spectrum: &evaluation.spectrum,
                        sample_spectra: &evaluation.sample_spectra,
                    })
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "[acoustics.response] source = \"fairing_internal\" requires [acoustics.fairing_internal]"
                            .to_owned(),
                    })?,
            };
            evaluate_response(response, resolved_files, source)
        })
        .transpose()?;
    let ascent = ascent_evaluation.map(|evaluation| evaluation.report);
    let fairing_internal = fairing_internal_evaluation.map(|evaluation| evaluation.report);
    Ok(Some(AcousticsRunReport {
        dsm,
        iop,
        ascent,
        fairing_internal,
        response,
    }))
}

pub(crate) fn observe_source_engine_ignitions(
    report: &mut Option<AcousticsRunReport>,
    fired: &[FiredEvent<ScenarioScriptAction>],
) {
    let Some(iop) = report.as_mut().and_then(|report| report.iop.as_mut()) else {
        return;
    };
    if iop.source_engine_observed_ignition_time_s.is_some() {
        return;
    }
    let Some(source_engine_id) = &iop.source_engine_id else {
        return;
    };
    let source_engine =
        EngineId::from_path(&format!("vehicle.assembly.engines.{source_engine_id}"));
    for event in fired {
        let ScenarioScriptAction::EngineCommand { id, ignite, .. } = event.action else {
            continue;
        };
        if id == source_engine && ignite {
            iop.source_engine_observed_ignition_event_id = Some(event.binding_id.value());
            iop.source_engine_observed_ignition_step = Some(event.step.value());
            iop.source_engine_observed_ignition_time_s = Some(event.time.as_seconds());
            return;
        }
    }
}

pub(crate) fn observe_source_engine_runtime_sample(
    report: &mut Option<AcousticsRunReport>,
    step: u64,
    time_s: f64,
    snapshots: &BTreeMap<EngineId, openbmp_propulsion::EngineSnapshot>,
) -> Result<(), RunnerError> {
    let Some(iop) = report.as_mut().and_then(|report| report.iop.as_mut()) else {
        return Ok(());
    };
    let Some(source_engine_id) = &iop.source_engine_id else {
        return Ok(());
    };
    let source_engine =
        EngineId::from_path(&format!("vehicle.assembly.engines.{source_engine_id}"));
    let Some(snapshot) = snapshots.get(&source_engine) else {
        return Ok(());
    };
    let thrust_n = snapshot.thrust_body.norm();
    let source_strength_n = iop
        .source_engine_effective_cstar_m_s
        .map_or(thrust_n, |effective_cstar_m_s| {
            snapshot.mass_flow_kg_per_s * effective_cstar_m_s
        });
    if !time_s.is_finite()
        || !source_strength_n.is_finite()
        || !thrust_n.is_finite()
        || !snapshot.mass_flow_kg_per_s.is_finite()
    {
        return Err(RunnerError::UnsupportedScenario {
            what:
                "invalid [acoustics] IOP runtime source history sample: non-finite source snapshot"
                    .to_owned(),
        });
    }
    let source_rate_n_per_s = if let (Some(previous_time_s), Some(previous_source_strength_n)) = (
        iop.source_engine_runtime_last_time_s,
        iop.source_engine_runtime_last_source_strength_n,
    ) {
        let dt_s = time_s - previous_time_s;
        if dt_s > 0.0 {
            (source_strength_n - previous_source_strength_n) / dt_s
        } else {
            0.0
        }
    } else {
        0.0
    };
    if !source_rate_n_per_s.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] IOP runtime source history sample: non-finite source rate"
                .to_owned(),
        });
    }
    if iop.source_engine_runtime_history_csv.is_empty() {
        iop.source_engine_runtime_history_csv.extend_from_slice(
            b"step,time_s,source_strength_n,source_rate_n_per_s,thrust_n,mass_flow_kg_per_s\n",
        );
    }
    iop.source_engine_runtime_history_csv.extend_from_slice(
        format!(
            "{step},{time_s:.17e},{source_strength_n:.17e},{source_rate_n_per_s:.17e},{thrust_n:.17e},{mass_flow:.17e}\n",
            mass_flow = snapshot.mass_flow_kg_per_s,
        )
        .as_bytes(),
    );
    iop.source_engine_runtime_sample_count += 1;
    iop.source_engine_runtime_last_source_strength_n = Some(source_strength_n);
    iop.source_engine_runtime_last_time_s = Some(time_s);
    let positive_source_rate_n_per_s = source_rate_n_per_s.max(0.0);
    let peak_source_rate_n_per_s = iop
        .source_engine_runtime_peak_source_rate_n_per_s
        .map_or(positive_source_rate_n_per_s, |peak| {
            peak.max(positive_source_rate_n_per_s)
        });
    iop.source_engine_runtime_peak_source_rate_n_per_s = Some(peak_source_rate_n_per_s);
    if peak_source_rate_n_per_s > 0.0 {
        update_runtime_iop_envelope(iop, peak_source_rate_n_per_s)?;
    }
    Ok(())
}

fn update_runtime_iop_envelope(
    iop: &mut AcousticsIopRunReport,
    peak_source_rate_n_per_s: f64,
) -> Result<(), RunnerError> {
    let source = IopStartTransient::new(peak_source_rate_n_per_s, iop.ramp_duration_s)
        .map_err(iop_scenario_error)?;
    let duct = IopDuctCoupling::new(
        iop.duct_volume_m3,
        iop.duct_exit_area_m2,
        iop.duct_discharge_velocity_m_s,
        iop.duct_geometry_gain,
    )
    .map_err(iop_scenario_error)?;
    let uncertainty = IopUncertainty::new(iop.uncertainty_minus_db, iop.uncertainty_plus_db)
        .map_err(iop_scenario_error)?;
    let input = IopPulseInput::new(
        source,
        duct,
        iop.receiver_distance_m,
        iop.suppression_loss_db,
        uncertainty,
    )
    .map_err(iop_scenario_error)?;
    let envelope = evaluate_iop_pulse(input);
    iop.source_engine_runtime_peak_overpressure_pa = Some(envelope.peak_overpressure_pa());
    iop.source_engine_runtime_lower_peak_overpressure_pa =
        Some(envelope.lower_peak_overpressure_pa());
    iop.source_engine_runtime_upper_peak_overpressure_pa =
        Some(envelope.upper_peak_overpressure_pa());
    iop.source_engine_runtime_duration_s = Some(envelope.duration_s());
    Ok(())
}

fn evaluate_ascent(
    config: &AcousticsAscentConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<AscentEvaluation, RunnerError> {
    let (sha256, deck) = load_ascent_deck(resolved_files)?;
    let (samples, sample_ids) = ascent_trace_samples_from_config(config, &deck)?;
    let trace = deck
        .evaluate_trace(&samples)
        .map_err(ascent_scenario_error)?;
    let peak = trace.points.get(trace.peak_sample_index).ok_or_else(|| {
        RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] ascent trace: peak sample index out of range".to_owned(),
        }
    })?;
    let peak_class_id = peak.class_id.clone();
    let peak_dynamic_pressure_pa = peak.dynamic_pressure_pa;
    let peak_mach = peak.mach;
    let peak_sample_oaspl_db = peak.oaspl.db_re_20upa();
    let peak_sample_id = sample_ids[trace.peak_sample_index].clone();
    let oaspl_db = trace.envelope_oaspl.db_re_20upa();
    let spectrum_csv = spl_spectrum_csv(&trace.envelope_spectrum);
    let sample_trace_csv = ascent_trace_csv(&trace, &sample_ids);
    let p95_50_environment = ascent_p95_50_environment(&trace)?;
    let p95_50_oaspl_db = p95_50_environment
        .as_ref()
        .map(|environment| environment.oaspl().db_re_20upa());
    let p95_50_tolerance_factor = p95_50_environment
        .as_ref()
        .map(SplP95_50Environment::tolerance_factor);
    let p95_50_environment_csv = p95_50_environment
        .as_ref()
        .map_or_else(Vec::new, SplP95_50Environment::to_csv);
    let sample_spectra = trace
        .points
        .iter()
        .map(|point| point.spectrum.clone())
        .collect::<Vec<_>>();
    let p95_50_wilks_sizing = p95_50_wilks_sizing(sample_spectra.len())?;
    let envelope_spectrum = trace.envelope_spectrum;
    Ok(AscentEvaluation {
        report: AcousticsAscentRunReport {
            station_id: config.station_id.clone(),
            class_id: peak_class_id,
            dynamic_pressure_pa: peak_dynamic_pressure_pa,
            mach: peak_mach,
            sample_count: sample_ids.len(),
            peak_sample_index: trace.peak_sample_index,
            peak_sample_id,
            peak_sample_oaspl_db,
            deck_file_sha256_hex: sha256,
            deck_dataset_id: deck.meta().dataset_id.clone(),
            deck_source_class: deck.meta().source_class.clone(),
            uncertainty_db: trace.uncertainty_db,
            oaspl_db,
            spectrum_csv,
            sample_trace_csv,
            p95_50_oaspl_db,
            p95_50_tolerance_factor,
            p95_50_environment_csv,
            p95_50_wilks_required_sample_count: p95_50_wilks_sizing
                .map(|sizing| sizing.required_sample_count),
            p95_50_wilks_achieved_confidence: p95_50_wilks_sizing
                .map(|sizing| sizing.achieved_confidence),
            p95_50_wilks_sample_count_sufficient: p95_50_wilks_sizing
                .map(|sizing| sizing.sample_count_sufficient),
            p95_50_wilks_sizing_csv: p95_50_wilks_sizing
                .map_or_else(Vec::new, P95_50WilksSizing::to_csv),
        },
        spectrum: envelope_spectrum,
        sample_spectra,
    })
}

fn ascent_p95_50_environment(
    trace: &AscentAcousticTracePrediction,
) -> Result<Option<SplP95_50Environment>, RunnerError> {
    if trace.points.len() < 2 {
        return Ok(None);
    }
    let spectra = trace
        .points
        .iter()
        .map(|point| point.spectrum.clone())
        .collect::<Vec<_>>();
    evaluate_spl_p95_50_environment(&spectra)
        .map(Some)
        .map_err(ascent_scenario_error)
}

fn ascent_trace_samples_from_config(
    config: &AcousticsAscentConfig,
    deck: &AscentAcousticDeck,
) -> Result<(Vec<AscentAcousticTraceSample>, Vec<Option<String>>), RunnerError> {
    if config.samples.is_empty() {
        let dynamic_pressure_pa =
            required_ascent_config_value(config.dynamic_pressure_pa, "dynamic_pressure_pa")?;
        let mach = required_ascent_config_value(config.mach, "mach")?;
        let input =
            AscentAcousticInput::new(dynamic_pressure_pa, mach).map_err(ascent_scenario_error)?;
        let class_id = ascent_class_id_from_config(config, None, deck, input)?;
        let sample =
            AscentAcousticTraceSample::new(class_id, input).map_err(ascent_scenario_error)?;
        return Ok((vec![sample], vec![None]));
    }

    let mut samples = Vec::with_capacity(config.samples.len());
    let mut sample_ids = Vec::with_capacity(config.samples.len());
    for sample in &config.samples {
        let input = AscentAcousticInput::new(sample.dynamic_pressure_pa, sample.mach)
            .map_err(ascent_scenario_error)?;
        let class_id =
            ascent_class_id_from_config(config, sample.class_id.as_deref(), deck, input)?;
        samples
            .push(AscentAcousticTraceSample::new(class_id, input).map_err(ascent_scenario_error)?);
        sample_ids.push(sample.sample_id.clone());
    }
    Ok((samples, sample_ids))
}

fn ascent_class_id_from_config<'a>(
    config: &'a AcousticsAscentConfig,
    sample_class_id: Option<&'a str>,
    deck: &'a AscentAcousticDeck,
    input: AscentAcousticInput,
) -> Result<&'a str, RunnerError> {
    match config.class_selection {
        AcousticsAscentClassSelectionConfig::Explicit => sample_class_id
            .or(config.class_id.as_deref())
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "invalid [acoustics] ascent declaration: missing class_id".to_owned(),
            }),
        AcousticsAscentClassSelectionConfig::MachGate => deck
            .select_class_for_input(input)
            .map(|class| class.class_id.as_str())
            .map_err(ascent_scenario_error),
    }
}

fn required_ascent_config_value(
    value: Option<f64>,
    name: &'static str,
) -> Result<f64, RunnerError> {
    value.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] ascent declaration: missing {name}"),
    })
}

fn evaluate_fairing_internal(
    config: &AcousticsFairingInternalConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    ascent: &AscentEvaluation,
) -> Result<FairingInternalEvaluation, RunnerError> {
    let (sha256, deck) = load_fairing_internal_deck(resolved_files)?;
    let input =
        FairingInternalInput::new(config.fill_factor).map_err(fairing_internal_scenario_error)?;
    let prediction = deck
        .evaluate_class(&config.class_id, &ascent.spectrum, input)
        .map_err(fairing_internal_scenario_error)?;
    let sample_predictions =
        fairing_internal_sample_predictions(&deck, config, input, &ascent.sample_spectra)?;
    let sample_spectra = sample_predictions
        .iter()
        .map(|prediction| prediction.prediction.spectrum.clone())
        .collect::<Vec<_>>();
    let p95_50_environment = fairing_internal_p95_50_environment(&sample_spectra)?;
    let p95_50_oaspl_db = p95_50_environment
        .as_ref()
        .map(|environment| environment.oaspl().db_re_20upa());
    let p95_50_tolerance_factor = p95_50_environment
        .as_ref()
        .map(SplP95_50Environment::tolerance_factor);
    let p95_50_environment_csv = p95_50_environment
        .as_ref()
        .map_or_else(Vec::new, SplP95_50Environment::to_csv);
    let p95_50_wilks_sizing = p95_50_wilks_sizing(sample_spectra.len())?;
    Ok(FairingInternalEvaluation {
        report: AcousticsFairingInternalRunReport {
            station_id: config.station_id.clone(),
            class_id: prediction.class_id,
            fill_factor: config.fill_factor,
            fill_factor_gain_db: prediction.fill_factor_gain_db,
            deck_file_sha256_hex: sha256,
            deck_dataset_id: deck.meta().dataset_id.clone(),
            deck_source_class: deck.meta().source_class.clone(),
            uncertainty_db: prediction.uncertainty_db,
            oaspl_db: prediction.oaspl.db_re_20upa(),
            spectrum_csv: spl_spectrum_csv(&prediction.spectrum),
            p95_50_oaspl_db,
            p95_50_tolerance_factor,
            p95_50_environment_csv,
            p95_50_sample_trace_csv: p95_50_environment.as_ref().map_or_else(Vec::new, |_| {
                fairing_internal_p95_50_sample_trace_csv(&sample_predictions)
            }),
            p95_50_wilks_required_sample_count: p95_50_wilks_sizing
                .map(|sizing| sizing.required_sample_count),
            p95_50_wilks_achieved_confidence: p95_50_wilks_sizing
                .map(|sizing| sizing.achieved_confidence),
            p95_50_wilks_sample_count_sufficient: p95_50_wilks_sizing
                .map(|sizing| sizing.sample_count_sufficient),
            p95_50_wilks_sizing_csv: p95_50_wilks_sizing
                .map_or_else(Vec::new, P95_50WilksSizing::to_csv),
        },
        spectrum: prediction.spectrum,
        sample_spectra,
    })
}

fn fairing_internal_sample_predictions(
    deck: &FairingInternalDeck,
    config: &AcousticsFairingInternalConfig,
    input: FairingInternalInput,
    external_sample_spectra: &[SplSpectrum],
) -> Result<Vec<FairingInternalSamplePrediction>, RunnerError> {
    if external_sample_spectra.len() < 2 {
        if !config.samples.is_empty() {
            return Err(fairing_internal_scenario_error(
                "fairing P95/50 sample inputs require at least two ascent sample spectra",
            ));
        }
        return Ok(Vec::new());
    }
    if !config.samples.is_empty() && config.samples.len() != external_sample_spectra.len() {
        return Err(fairing_internal_scenario_error(format!(
            "fairing P95/50 sample input count {} does not match ascent sample spectrum count {}",
            config.samples.len(),
            external_sample_spectra.len()
        )));
    }
    if config.samples.is_empty() {
        return external_sample_spectra
            .iter()
            .enumerate()
            .map(|(sample_index, spectrum)| {
                let prediction = deck
                    .evaluate_class(&config.class_id, spectrum, input)
                    .map_err(fairing_internal_scenario_error)?;
                Ok(FairingInternalSamplePrediction {
                    sample_index,
                    sample_id: None,
                    class_id: config.class_id.clone(),
                    input,
                    prediction,
                })
            })
            .collect();
    }

    external_sample_spectra
        .iter()
        .zip(config.samples.iter())
        .enumerate()
        .map(|(sample_index, (spectrum, sample))| {
            let input = FairingInternalInput::new(sample.fill_factor)
                .map_err(fairing_internal_scenario_error)?;
            let class_id = sample
                .class_id
                .clone()
                .unwrap_or_else(|| config.class_id.clone());
            let prediction = deck
                .evaluate_class(&class_id, spectrum, input)
                .map_err(fairing_internal_scenario_error)?;
            Ok(FairingInternalSamplePrediction {
                sample_index,
                sample_id: sample.sample_id.clone(),
                class_id,
                input,
                prediction,
            })
        })
        .collect()
}

fn fairing_internal_p95_50_environment(
    sample_spectra: &[SplSpectrum],
) -> Result<Option<SplP95_50Environment>, RunnerError> {
    if sample_spectra.len() < 2 {
        return Ok(None);
    }
    evaluate_spl_p95_50_environment(sample_spectra)
        .map(Some)
        .map_err(fairing_internal_scenario_error)
}

fn evaluate_response(
    config: &AcousticsResponseConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    source: ResponseSourceEvaluation<'_>,
) -> Result<AcousticsResponseRunReport, RunnerError> {
    let (sha256, deck) = load_response_deck(resolved_files)?;
    let input = VibroacousticResponseInput::new(
        config.surface_mass_kg_m2,
        config.area_m2,
        config.mode_shape_factor,
    )
    .map_err(response_scenario_error)?;
    let prediction = deck
        .evaluate_class(&config.class_id, source.spectrum, input)
        .map_err(response_scenario_error)?;
    let p95_50_artifacts = response_p95_50_artifacts(&deck, config, source.sample_spectra)?;
    let total_mean_square_acceleration_m2_s4 = prediction
        .acceleration_psd
        .total_mean_square_acceleration_m2_s4();
    let total_mean_square_force_n2 = prediction.modal_force_psd.total_mean_square_force_n2();
    let acceleration_psd_csv = acceleration_psd_spectrum_csv(&prediction.acceleration_psd);
    let modal_force_psd_csv = force_psd_spectrum_csv(&prediction.modal_force_psd);
    let loads_handoff_csv = prediction
        .structural_loads_handoff()
        .map_err(response_scenario_error)?
        .to_csv();
    Ok(AcousticsResponseRunReport {
        station_id: config.station_id.clone(),
        source: config.source,
        class_id: prediction.class_id,
        surface_mass_kg_m2: config.surface_mass_kg_m2,
        area_m2: config.area_m2,
        mode_shape_factor: config.mode_shape_factor,
        modal_force_participation_m2: prediction.modal_force_participation_m2,
        surface_mass_gain: prediction.surface_mass_gain,
        deck_file_sha256_hex: sha256,
        deck_dataset_id: deck.meta().dataset_id.clone(),
        deck_source_class: deck.meta().source_class.clone(),
        uncertainty_db: prediction.uncertainty_db,
        total_mean_square_acceleration_m2_s4,
        total_mean_square_force_n2,
        acceleration_psd_csv,
        modal_force_psd_csv,
        modal_force_psd: prediction.modal_force_psd,
        loads_handoff_csv,
        p95_50_tolerance_factor: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.tolerance_factor),
        p95_50_total_mean_square_acceleration_m2_s4: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.total_mean_square_acceleration_m2_s4),
        p95_50_total_mean_square_force_n2: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.total_mean_square_force_n2),
        p95_50_acceleration_psd_csv: p95_50_artifacts
            .as_ref()
            .map_or_else(Vec::new, |artifacts| artifacts.acceleration_psd_csv.clone()),
        p95_50_modal_force_psd_csv: p95_50_artifacts
            .as_ref()
            .map_or_else(Vec::new, |artifacts| artifacts.modal_force_psd_csv.clone()),
        p95_50_sample_trace_csv: p95_50_artifacts
            .as_ref()
            .map_or_else(Vec::new, |artifacts| artifacts.sample_trace_csv.clone()),
        p95_50_wilks_required_sample_count: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.wilks_sizing.required_sample_count),
        p95_50_wilks_achieved_confidence: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.wilks_sizing.achieved_confidence),
        p95_50_wilks_sample_count_sufficient: p95_50_artifacts
            .as_ref()
            .map(|artifacts| artifacts.wilks_sizing.sample_count_sufficient),
        p95_50_wilks_sizing_csv: p95_50_artifacts
            .map_or_else(Vec::new, |artifacts| artifacts.wilks_sizing.to_csv()),
    })
}

fn response_p95_50_artifacts(
    deck: &VibroacousticTransferDeck,
    config: &AcousticsResponseConfig,
    source_sample_spectra: &[SplSpectrum],
) -> Result<Option<ResponseP95_50Artifacts>, RunnerError> {
    if source_sample_spectra.len() < 2 {
        if !config.samples.is_empty() {
            return Err(response_scenario_error(
                "response P95/50 sample inputs require at least two source sample spectra",
            ));
        }
        return Ok(None);
    }
    let wilks_sizing = p95_50_wilks_sizing(source_sample_spectra.len())?.ok_or_else(|| {
        response_scenario_error("response P95/50 Wilks sizing requires at least two samples")
    })?;
    if !config.samples.is_empty() && config.samples.len() != source_sample_spectra.len() {
        return Err(response_scenario_error(format!(
            "response P95/50 sample input count {} does not match source sample spectrum count {}",
            config.samples.len(),
            source_sample_spectra.len()
        )));
    }
    let predictions = response_p95_50_sample_predictions(deck, config, source_sample_spectra)?;
    let bands = predictions
        .first()
        .ok_or_else(|| response_scenario_error("response P95/50 requires at least two samples"))?
        .prediction
        .acceleration_psd
        .bands()
        .to_vec();
    let acceleration_values = predictions
        .iter()
        .map(|prediction| {
            prediction
                .prediction
                .acceleration_psd
                .values()
                .iter()
                .map(|value| value.m2_s4_per_hz())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let modal_force_values = predictions
        .iter()
        .map(|prediction| {
            prediction
                .prediction
                .modal_force_psd
                .values()
                .iter()
                .map(|value| value.n2_per_hz())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let tolerance_factor = one_sided_normal_tolerance_factor(
        P95_50_COVERAGE_PROBABILITY,
        P95_50_CONFIDENCE_PROBABILITY,
        source_sample_spectra.len(),
    )
    .map_err(response_scenario_error)?;
    let acceleration_statistics = response_p95_50_band_statistics(
        &bands,
        &acceleration_values,
        tolerance_factor,
        "acceleration_psd_m2_s4_per_hz",
    )?;
    let modal_force_statistics = response_p95_50_band_statistics(
        &bands,
        &modal_force_values,
        tolerance_factor,
        "modal_force_psd_n2_per_hz",
    )?;
    let acceleration_psd = AccelerationPsdSpectrum::new(
        bands.clone(),
        acceleration_statistics
            .iter()
            .map(|statistic| AccelerationPsd::new(statistic.p95_50_psd_value))
            .collect::<Result<Vec<_>, _>>()
            .map_err(response_scenario_error)?,
    )
    .map_err(response_scenario_error)?;
    let modal_force_psd = ForcePsdSpectrum::new(
        bands,
        modal_force_statistics
            .iter()
            .map(|statistic| ForcePsd::new(statistic.p95_50_psd_value))
            .collect::<Result<Vec<_>, _>>()
            .map_err(response_scenario_error)?,
    )
    .map_err(response_scenario_error)?;
    Ok(Some(ResponseP95_50Artifacts {
        tolerance_factor,
        total_mean_square_acceleration_m2_s4: acceleration_psd
            .total_mean_square_acceleration_m2_s4(),
        total_mean_square_force_n2: modal_force_psd.total_mean_square_force_n2(),
        acceleration_psd_csv: response_p95_50_psd_csv(
            &acceleration_statistics,
            source_sample_spectra.len(),
            "p95_50_acceleration_psd_m2_s4_per_hz",
            tolerance_factor,
        ),
        modal_force_psd_csv: response_p95_50_psd_csv(
            &modal_force_statistics,
            source_sample_spectra.len(),
            "p95_50_modal_force_psd_n2_per_hz",
            tolerance_factor,
        ),
        sample_trace_csv: response_p95_50_sample_trace_csv(&predictions),
        wilks_sizing,
    }))
}

fn response_p95_50_sample_predictions(
    deck: &VibroacousticTransferDeck,
    config: &AcousticsResponseConfig,
    source_sample_spectra: &[SplSpectrum],
) -> Result<Vec<ResponseP95_50SamplePrediction>, RunnerError> {
    if config.samples.is_empty() {
        let input = VibroacousticResponseInput::new(
            config.surface_mass_kg_m2,
            config.area_m2,
            config.mode_shape_factor,
        )
        .map_err(response_scenario_error)?;
        return source_sample_spectra
            .iter()
            .enumerate()
            .map(|(sample_index, spectrum)| {
                let prediction = deck
                    .evaluate_class(&config.class_id, spectrum, input)
                    .map_err(response_scenario_error)?;
                Ok(ResponseP95_50SamplePrediction {
                    sample_index,
                    sample_id: None,
                    class_id: config.class_id.clone(),
                    input,
                    prediction,
                })
            })
            .collect();
    }

    source_sample_spectra
        .iter()
        .zip(config.samples.iter())
        .enumerate()
        .map(|(sample_index, (spectrum, sample))| {
            let input = VibroacousticResponseInput::new(
                sample.surface_mass_kg_m2,
                sample.area_m2,
                sample.mode_shape_factor,
            )
            .map_err(response_scenario_error)?;
            let class_id = sample
                .class_id
                .clone()
                .unwrap_or_else(|| config.class_id.clone());
            let prediction = deck
                .evaluate_class(&class_id, spectrum, input)
                .map_err(response_scenario_error)?;
            Ok(ResponseP95_50SamplePrediction {
                sample_index,
                sample_id: sample.sample_id.clone(),
                class_id,
                input,
                prediction,
            })
        })
        .collect()
}

fn p95_50_wilks_sizing(sample_count: usize) -> Result<Option<P95_50WilksSizing>, RunnerError> {
    if sample_count < 2 {
        return Ok(None);
    }
    let sample_count = u64::try_from(sample_count)
        .map_err(|_| response_scenario_error("P95/50 Wilks sample count does not fit in u64"))?;
    let required_sample_count =
        wilks_one_sided_n(P95_50_COVERAGE_PROBABILITY, P95_50_CONFIDENCE_PROBABILITY)
            .map_err(response_scenario_error)?;
    let achieved_confidence = wilks_one_sided_confidence(P95_50_COVERAGE_PROBABILITY, sample_count)
        .map_err(response_scenario_error)?;
    Ok(Some(P95_50WilksSizing {
        sample_count,
        required_sample_count,
        achieved_confidence,
        sample_count_sufficient: sample_count >= required_sample_count,
    }))
}

fn response_p95_50_band_statistics(
    bands: &[AcousticBand],
    sample_values: &[Vec<f64>],
    tolerance_factor: f64,
    field: &'static str,
) -> Result<Vec<ResponseP95_50BandStatistic>, RunnerError> {
    for sample in sample_values {
        if sample.len() != bands.len() {
            return Err(response_scenario_error(format!(
                "{field} sample band count {} does not match {}",
                sample.len(),
                bands.len()
            )));
        }
    }
    let mut statistics = Vec::with_capacity(bands.len());
    for (band_index, band) in bands.iter().enumerate() {
        let db_values = sample_values
            .iter()
            .map(|sample| response_psd_db(sample[band_index], field))
            .collect::<Result<Vec<_>, _>>()?;
        statistics.push(response_p95_50_band_statistic(
            *band,
            &db_values,
            tolerance_factor,
            field,
        )?);
    }
    Ok(statistics)
}

fn response_p95_50_band_statistic(
    band: AcousticBand,
    values_db: &[f64],
    tolerance_factor: f64,
    field: &'static str,
) -> Result<ResponseP95_50BandStatistic, RunnerError> {
    debug_assert!(values_db.len() >= 2);
    let mut sum = 0.0;
    let mut min_sample_psd_db = f64::INFINITY;
    let mut max_sample_psd_db = f64::NEG_INFINITY;
    for value in values_db {
        response_require_finite(field, *value)?;
        sum += *value;
        min_sample_psd_db = min_sample_psd_db.min(*value);
        max_sample_psd_db = max_sample_psd_db.max(*value);
    }
    let mean_psd_db = sum / values_db.len() as f64;
    response_require_finite("mean_psd_db", mean_psd_db)?;
    let mut variance_sum = 0.0;
    for value in values_db {
        let delta = *value - mean_psd_db;
        variance_sum += delta * delta;
    }
    let sigma_psd_db = (variance_sum / (values_db.len() - 1) as f64).sqrt();
    response_require_finite("sigma_psd_db", sigma_psd_db)?;
    let p95_50_psd_db = mean_psd_db + tolerance_factor * sigma_psd_db;
    response_require_finite("p95_50_psd_db", p95_50_psd_db)?;
    let p95_50_psd_value = 10.0_f64.powf(p95_50_psd_db / 10.0);
    response_require_positive("p95_50_psd_value", p95_50_psd_value)?;
    Ok(ResponseP95_50BandStatistic {
        band,
        mean_psd_db,
        sigma_psd_db,
        p95_50_psd_db,
        p95_50_psd_value,
        min_sample_psd_db,
        max_sample_psd_db,
    })
}

fn response_psd_db(value: f64, field: &'static str) -> Result<f64, RunnerError> {
    response_require_positive(field, value)?;
    let value_db = 10.0 * value.log10();
    response_require_finite(field, value_db)?;
    Ok(value_db)
}

fn response_require_finite(field: &'static str, value: f64) -> Result<(), RunnerError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(response_scenario_error(format!("{field} must be finite")))
    }
}

fn response_require_positive(field: &'static str, value: f64) -> Result<(), RunnerError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(response_scenario_error(format!("{field} must be positive")))
    }
}

fn evaluate_iop(
    config: &AcousticsIopConfig,
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<AcousticsIopRunReport, RunnerError> {
    let source = iop_start_source(config, document, resolved_files)?;
    let (
        duct,
        duct_deck_file_sha256_hex,
        duct_deck_dataset_id,
        duct_deck_source_class,
        duct_class_id,
    ) = if config.duct_deck_file.is_some() {
        let class_id =
            config
                .duct_class
                .as_deref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] IOP declaration: duct_deck_file requires duct_class"
                        .to_owned(),
                })?;
        let (sha256, deck) = load_iop_duct_deck(resolved_files)?;
        let class = deck.class(class_id).map_err(iop_scenario_error)?;
        (
            class.duct,
            Some(sha256),
            Some(deck.meta().dataset_id.clone()),
            Some(deck.meta().source_class.clone()),
            Some(class.class_id.clone()),
        )
    } else {
        (
            IopDuctCoupling::new(
                config
                    .duct_volume_m3
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "invalid [acoustics] IOP declaration: missing duct_volume_m3"
                            .to_owned(),
                    })?,
                config
                    .duct_exit_area_m2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "invalid [acoustics] IOP declaration: missing duct_exit_area_m2"
                            .to_owned(),
                    })?,
                config
                    .duct_discharge_velocity_m_s
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what:
                            "invalid [acoustics] IOP declaration: missing duct_discharge_velocity_m_s"
                                .to_owned(),
                    })?,
                config
                    .duct_geometry_gain
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "invalid [acoustics] IOP declaration: missing duct_geometry_gain"
                            .to_owned(),
                    })?,
            )
            .map_err(iop_scenario_error)?,
            None,
            None,
            None,
            None,
        )
    };
    let (
        suppression_class_loss_db,
        suppression_deck_file_sha256_hex,
        suppression_deck_dataset_id,
        suppression_deck_source_class,
        suppression_class_id,
        suppression_class_uncertainty_db,
    ) = if config.suppression_deck_file.is_some() {
        let class_id = config.suppression_class.as_deref().ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what:
                    "invalid [acoustics] IOP declaration: suppression_deck_file requires suppression_class"
                        .to_owned(),
            }
        })?;
        let (sha256, deck) = load_iop_suppression_deck(resolved_files)?;
        let class = deck.class(class_id).map_err(iop_scenario_error)?;
        (
            Some(class.suppression_loss_db),
            Some(sha256),
            Some(deck.meta().dataset_id.clone()),
            Some(deck.meta().source_class.clone()),
            Some(class.class_id.clone()),
            Some(class.uncertainty_db),
        )
    } else {
        (None, None, None, None, None, None)
    };
    let suppression_loss_db = config.suppression_loss_db + suppression_class_loss_db.unwrap_or(0.0);
    let uncertainty = IopUncertainty::new(config.uncertainty_minus_db, config.uncertainty_plus_db)
        .map_err(iop_scenario_error)?;
    let input = IopPulseInput::new(
        source.transient,
        duct,
        config.receiver_distance_m,
        suppression_loss_db,
        uncertainty,
    )
    .map_err(iop_scenario_error)?;
    let envelope = evaluate_iop_pulse(input);
    Ok(AcousticsIopRunReport {
        station_id: config.station_id.clone(),
        source_engine_id: source.source_engine_id,
        source_engine_ignition_event_id: source.source_engine_ignition_event_id,
        source_engine_ignition_time_s: source.source_engine_ignition_time_s,
        source_engine_observed_ignition_event_id: None,
        source_engine_observed_ignition_step: None,
        source_engine_observed_ignition_time_s: None,
        source_engine_max_thrust_n: source.source_engine_max_thrust_n,
        source_engine_mdot_cstar_n: source.source_engine_mdot_cstar_n,
        source_engine_effective_cstar_m_s: source.source_engine_effective_cstar_m_s,
        source_engine_thermochemical_performance: source.source_engine_thermochemical_performance,
        source_engine_runtime_sample_count: 0,
        source_engine_runtime_last_source_strength_n: None,
        source_engine_runtime_last_time_s: None,
        source_engine_runtime_peak_source_rate_n_per_s: None,
        source_engine_runtime_peak_overpressure_pa: None,
        source_engine_runtime_lower_peak_overpressure_pa: None,
        source_engine_runtime_upper_peak_overpressure_pa: None,
        source_engine_runtime_duration_s: None,
        source_engine_runtime_history_csv: Vec::new(),
        gas_generation_rate_n_per_s: source.gas_generation_rate_n_per_s,
        ramp_duration_s: source.ramp_duration_s,
        duct_residence_time_s: duct.residence_time_s(),
        duct_volume_m3: duct.volume_m3(),
        duct_exit_area_m2: duct.exit_area_m2(),
        duct_discharge_velocity_m_s: duct.discharge_velocity_m_s(),
        duct_geometry_gain: duct.geometry_gain(),
        duct_deck_file_sha256_hex,
        duct_deck_dataset_id,
        duct_deck_source_class,
        duct_class_id,
        receiver_distance_m: config.receiver_distance_m,
        suppression_loss_db,
        suppression_deck_file_sha256_hex,
        suppression_deck_dataset_id,
        suppression_deck_source_class,
        suppression_class_id,
        suppression_class_loss_db,
        suppression_class_uncertainty_db,
        uncertainty_minus_db: config.uncertainty_minus_db,
        uncertainty_plus_db: config.uncertainty_plus_db,
        peak_overpressure_pa: envelope.peak_overpressure_pa(),
        lower_peak_overpressure_pa: envelope.lower_peak_overpressure_pa(),
        upper_peak_overpressure_pa: envelope.upper_peak_overpressure_pa(),
        duration_s: envelope.duration_s(),
    })
}

fn iop_start_source(
    config: &AcousticsIopConfig,
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<IopStartSource, RunnerError> {
    if let Some(source_engine_id) = &config.source_engine_id {
        let (engine_index, engine) = document
            .vehicle
            .assembly
            .engines
            .iter()
            .enumerate()
            .find(|(_, engine)| engine.id == *source_engine_id)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "invalid [acoustics] IOP declaration: source_engine_id `{source_engine_id}` does not match a declared engine"
                ),
            })?;
        let ramp_duration_s = engine.limits.ignition_transient_s;
        if ramp_duration_s <= 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "invalid [acoustics] IOP declaration: source engine `{source_engine_id}` has zero ignition_transient_s"
                ),
            });
        }
        let source = iop_source_engine_performance(engine_index, engine, document, resolved_files)?;
        let ignition_command = first_source_engine_ignition_command(document, source_engine_id);
        let gas_generation_rate_n_per_s = source.source_strength_n / ramp_duration_s;
        let transient = IopStartTransient::new(gas_generation_rate_n_per_s, ramp_duration_s)
            .map_err(iop_scenario_error)?;
        Ok(IopStartSource {
            transient,
            gas_generation_rate_n_per_s,
            ramp_duration_s,
            source_engine_id: Some(source_engine_id.clone()),
            source_engine_ignition_event_id: ignition_command
                .as_ref()
                .map(|command| command.event_id.clone()),
            source_engine_ignition_time_s: ignition_command.as_ref().map(|command| command.time_s),
            source_engine_max_thrust_n: Some(source.max_thrust_n),
            source_engine_mdot_cstar_n: source.mass_flow_c_star_n,
            source_engine_effective_cstar_m_s: source.effective_cstar_m_s,
            source_engine_thermochemical_performance: source.thermochemical_performance,
        })
    } else {
        let gas_generation_rate_n_per_s =
            config
                .gas_generation_rate_n_per_s
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what:
                        "invalid [acoustics] IOP declaration: missing gas_generation_rate_n_per_s"
                            .to_owned(),
                })?;
        let ramp_duration_s =
            config
                .ramp_duration_s
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] IOP declaration: missing ramp_duration_s".to_owned(),
                })?;
        let transient = IopStartTransient::new(gas_generation_rate_n_per_s, ramp_duration_s)
            .map_err(iop_scenario_error)?;
        Ok(IopStartSource {
            transient,
            gas_generation_rate_n_per_s,
            ramp_duration_s,
            source_engine_id: None,
            source_engine_ignition_event_id: None,
            source_engine_ignition_time_s: None,
            source_engine_max_thrust_n: None,
            source_engine_mdot_cstar_n: None,
            source_engine_effective_cstar_m_s: None,
            source_engine_thermochemical_performance: false,
        })
    }
}

struct IopSourceEngineIgnitionCommand {
    event_id: String,
    time_s: f64,
}

fn first_source_engine_ignition_command(
    document: &ScenarioDocument,
    source_engine_id: &str,
) -> Option<IopSourceEngineIgnitionCommand> {
    let mut selected: Option<IopSourceEngineIgnitionCommand> = None;
    for event in &document.scenario_script.events {
        let ScenarioActionConfig::EngineCommand { id, command } = &event.action else {
            continue;
        };
        if id != source_engine_id || !command.ignite {
            continue;
        }
        let EventTriggerConfig::AtTime { time_s } = &event.trigger else {
            continue;
        };
        let time_s = *time_s;
        let should_replace =
            selected
                .as_ref()
                .is_none_or(|current| match time_s.total_cmp(&current.time_s) {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Equal => event.id.as_str() < current.event_id.as_str(),
                    std::cmp::Ordering::Greater => false,
                });
        if should_replace {
            selected = Some(IopSourceEngineIgnitionCommand {
                event_id: event.id.clone(),
                time_s,
            });
        }
    }
    selected
}

struct IopSourceEnginePerformance {
    max_thrust_n: f64,
    source_strength_n: f64,
    mass_flow_c_star_n: Option<f64>,
    effective_cstar_m_s: Option<f64>,
    thermochemical_performance: bool,
}

fn iop_source_engine_performance(
    engine_index: usize,
    engine: &openbmp_scenario::EngineConfig,
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<IopSourceEnginePerformance, RunnerError> {
    if let Some(performance) = &engine.thermochemical_performance {
        let thermochem = document
            .propulsion
            .as_ref()
            .and_then(|propulsion| propulsion.thermochem.as_ref())
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "invalid [acoustics] IOP declaration: source engine thermochemical performance requires [propulsion.thermochem]".to_owned(),
            })?;
        let performance = crate::engines::resolve_liquid_engine_performance(
            engine_index,
            performance,
            thermochem,
            resolved_files,
        )?;
        Ok(IopSourceEnginePerformance {
            max_thrust_n: performance.performance.max_thrust_n,
            source_strength_n: performance.mass_flow_c_star_n,
            mass_flow_c_star_n: Some(performance.mass_flow_c_star_n),
            effective_cstar_m_s: Some(
                performance.mass_flow_c_star_n / performance.performance.mass_flow_kg_per_s,
            ),
            thermochemical_performance: true,
        })
    } else {
        Ok(IopSourceEnginePerformance {
            max_thrust_n: engine.limits.max_thrust_n,
            source_strength_n: engine.limits.max_thrust_n,
            mass_flow_c_star_n: None,
            effective_cstar_m_s: None,
            thermochemical_performance: false,
        })
    }
}

fn evaluate_dsm(
    config: &AcousticsDsmConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<AcousticsDsmRunReport, RunnerError> {
    let cluster_engines = cluster_engines_from_config(config);
    let cluster_engine_count = if cluster_engines.is_empty() {
        config.cluster_engine_count.unwrap_or(1)
    } else {
        cluster_engines.len() as u32
    };
    let cluster_power_gain_db = 10.0 * f64::from(cluster_engine_count).log10();
    let per_engine_exit = DsmExitCondition {
        mass_flow_kg_s: config.mass_flow_kg_s,
        exit_velocity_m_s: config.exit_velocity_m_s,
        exit_diameter_m: config.exit_diameter_m,
        acoustic_efficiency: config.acoustic_efficiency,
    };
    let report_exit = DsmExitCondition {
        mass_flow_kg_s: config.mass_flow_kg_s * f64::from(cluster_engine_count),
        ..per_engine_exit
    };
    let DsmSpectrumInput {
        shape,
        curve_file_sha256_hex: spectrum_curve_file_sha256_hex,
        curve_dataset_id: spectrum_curve_dataset_id,
        curve_figure_id: spectrum_curve_figure_id,
    } = spectrum_from_config(config, resolved_files)?;
    let DsmClusterInteractionInput {
        interaction: cluster_interaction,
        deck_file_sha256_hex: cluster_interaction_deck_file_sha256_hex,
        deck_dataset_id: cluster_interaction_deck_dataset_id,
        deck_source_class: cluster_interaction_deck_source_class,
        class_id: cluster_interaction_class_id,
        uncertainty_db: cluster_interaction_uncertainty_db,
    } = cluster_interaction_from_config(config, &shape, resolved_files)?;
    let deck = config
        .deck_file
        .as_ref()
        .map(|_| load_dsm_deck(resolved_files))
        .transpose()?;
    let (
        source_input,
        directivity,
        deck_file_sha256_hex,
        deck_dataset_id,
        deck_source_class,
        source_curve_file_sha256_hex,
        source_curve_dataset_id,
        source_curve_figure_id,
        source_curve_core_length_m,
        apparent_source_curve_file_sha256_hex,
        apparent_source_curve_dataset_id,
        apparent_source_curve_figure_id,
        directivity_curve_file_sha256_hex,
        directivity_curve_dataset_id,
        directivity_curve_figure_id,
    ) = if let Some((sha256, deck)) = deck {
        (
            DsmSourceInput::Distributed(deck.source_allocation().clone()),
            deck.directivity().clone(),
            Some(sha256),
            Some(deck.meta().dataset_id.clone()),
            Some(deck.meta().source_class.clone()),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
    } else if config.source_curve_file.is_some() {
        let (sha256, source_curve) = load_dsm_source_curve(resolved_files)?;
        let exit_mach = config
            .exit_mach
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "invalid [acoustics] DSM declaration: source_curve_file requires exit_mach"
                    .to_owned(),
            })?;
        let core_length_m = sp8072_core_length_m(config.exit_diameter_m, exit_mach)
            .map_err(acoustics_scenario_error)?;
        let allocation = source_curve
            .curve()
            .to_source_allocation(core_length_m)
            .map_err(acoustics_scenario_error)?;
        let directivity_input = directivity_from_config(config, &shape, resolved_files)?;
        (
            DsmSourceInput::Distributed(allocation),
            directivity_input.deck,
            None,
            None,
            None,
            Some(sha256),
            Some(source_curve.meta().dataset_id.clone()),
            Some(source_curve.meta().source_figure_id.clone()),
            Some(core_length_m),
            None,
            None,
            None,
            directivity_input.curve_file_sha256_hex,
            directivity_input.curve_dataset_id,
            directivity_input.curve_figure_id,
        )
    } else if config.apparent_source_curve_file.is_some() {
        let (sha256, apparent_source_curve) = load_dsm_apparent_source_curve(resolved_files)?;
        let directivity_input = directivity_from_config(config, &shape, resolved_files)?;
        (
            DsmSourceInput::Apparent(apparent_source_curve.curve().clone()),
            directivity_input.deck,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            Some(sha256),
            Some(apparent_source_curve.meta().dataset_id.clone()),
            Some(apparent_source_curve.meta().source_figure_id.clone()),
            directivity_input.curve_file_sha256_hex,
            directivity_input.curve_dataset_id,
            directivity_input.curve_figure_id,
        )
    } else {
        let source_pairs = config
            .source_line
            .iter()
            .map(|source| (source.axial_m, source.weight))
            .collect::<Vec<_>>();
        let allocation = DsmSourceAllocation::normalized_from_weights(&source_pairs)
            .map_err(acoustics_scenario_error)?;
        let directivity_input = directivity_from_config(config, &shape, resolved_files)?;
        (
            DsmSourceInput::Distributed(allocation),
            directivity_input.deck,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            directivity_input.curve_file_sha256_hex,
            directivity_input.curve_dataset_id,
            directivity_input.curve_figure_id,
        )
    };
    let (
        axial_spectrum_curve_file_sha256_hex,
        axial_spectrum_curve_dataset_id,
        axial_spectrum_curve_figure_id,
        axial_spectrum_curve,
    ) = if config.axial_spectrum_curve_file.is_some() {
        let (sha256, axial_spectrum_curve) = load_dsm_axial_spectrum_curve(resolved_files)?;
        (
            Some(sha256),
            Some(axial_spectrum_curve.meta().dataset_id.clone()),
            Some(axial_spectrum_curve.meta().source_figure_id.clone()),
            Some(axial_spectrum_curve.curve().clone()),
        )
    } else {
        (None, None, None, None)
    };
    let DsmSuppressionInput {
        directivity,
        suppression_deck_file_sha256_hex,
        suppression_deck_dataset_id,
        suppression_deck_source_class,
        suppression_class_id,
        suppression_uncertainty_db,
    } = suppression_from_config(config, directivity, resolved_files)?;
    let receiver = DsmReceiver::new(
        config.station_id.clone(),
        config.receiver_axial_m,
        config.receiver_radial_m,
    )
    .map_err(acoustics_scenario_error)?;
    let DsmPropagationInput {
        propagation,
        absorption_deck_file_sha256_hex,
        absorption_deck_dataset_id,
        absorption_deck_source_class,
        absorption_profile_id,
    } = propagation_from_config(config, &shape, resolved_files)?;
    let plume_deflection = plume_deflection_from_config(config)?;
    let steps = config
        .liftoff_steps
        .iter()
        .map(|step| DsmLiftoffStep::new(step.time_s, step.vehicle_height_m))
        .collect::<Result<Vec<_>, _>>()
        .map_err(acoustics_scenario_error)?;
    let context = DsmEvaluationContext {
        source_input: &source_input,
        axial_spectrum_curve: axial_spectrum_curve.as_ref(),
        shape: &shape,
        directivity: &directivity,
        steps: &steps,
        plume_deflection,
    };
    let (station, trace) = if cluster_engines.is_empty() {
        let (station, trace) =
            evaluate_source_input_station_and_trace(report_exit, &receiver, propagation, context)?;
        apply_identical_cluster_interaction(
            station,
            trace,
            cluster_engine_count,
            &cluster_interaction,
        )?
    } else {
        evaluate_geometric_cluster_station_and_trace(
            per_engine_exit,
            &cluster_engines,
            config.receiver_azimuth_deg.unwrap_or(0.0),
            &receiver,
            propagation,
            context,
            &cluster_interaction,
        )?
    };
    let station_oaspl_db = station
        .spectrum
        .oaspl()
        .map_err(acoustics_scenario_error)?
        .db_re_20upa();
    Ok(AcousticsDsmRunReport {
        station_id: station.station_id.clone(),
        source_count: station.source_count,
        cluster_engine_count,
        cluster_power_gain_db,
        cluster_correlation_coefficient: cluster_interaction.correlation_coefficient,
        cluster_shielding_loss_db: cluster_interaction.shielding_loss_db,
        cluster_interaction_deck_file_sha256_hex,
        cluster_interaction_deck_dataset_id,
        cluster_interaction_deck_source_class,
        cluster_interaction_class_id,
        cluster_interaction_uncertainty_db,
        pad_path_length_extra_m: config.pad_path_length_extra_m.unwrap_or(0.0),
        pad_deflection_origin_axial_m: config.pad_deflection_origin_axial_m,
        pad_deflection_angle_deg: config.pad_deflection_angle_deg,
        acoustic_power_w: report_exit
            .acoustic_power_w()
            .map_err(acoustics_scenario_error)?,
        sound_power_level_db: report_exit
            .sound_power_level_db()
            .map_err(acoustics_scenario_error)?,
        station_oaspl_db,
        deck_file_sha256_hex,
        deck_dataset_id,
        deck_source_class,
        source_curve_file_sha256_hex,
        source_curve_dataset_id,
        source_curve_figure_id,
        source_curve_core_length_m,
        apparent_source_curve_file_sha256_hex,
        apparent_source_curve_dataset_id,
        apparent_source_curve_figure_id,
        axial_spectrum_curve_file_sha256_hex,
        axial_spectrum_curve_dataset_id,
        axial_spectrum_curve_figure_id,
        spectrum_curve_file_sha256_hex,
        spectrum_curve_dataset_id,
        spectrum_curve_figure_id,
        directivity_curve_file_sha256_hex,
        directivity_curve_dataset_id,
        directivity_curve_figure_id,
        absorption_deck_file_sha256_hex,
        absorption_deck_dataset_id,
        absorption_deck_source_class,
        absorption_profile_id,
        suppression_deck_file_sha256_hex,
        suppression_deck_dataset_id,
        suppression_deck_source_class,
        suppression_class_id,
        suppression_uncertainty_db,
        station_spectrum_csv: station.to_csv().into_bytes(),
        liftoff_oaspl_csv: trace
            .to_oaspl_csv()
            .map_err(acoustics_scenario_error)?
            .into_bytes(),
    })
}

fn cluster_engines_from_config(config: &AcousticsDsmConfig) -> Vec<DsmClusterEngineInput> {
    config
        .cluster_engines
        .iter()
        .map(|engine| DsmClusterEngineInput {
            offset_x_m: engine.offset_x_m,
            offset_y_m: engine.offset_y_m,
        })
        .collect()
}

fn cluster_interaction_from_config(
    config: &AcousticsDsmConfig,
    shape: &DsmSpectrumShape,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<DsmClusterInteractionInput, RunnerError> {
    if config.cluster_interaction_deck_file.is_some() {
        let class_id =
            config
                .cluster_interaction_class
                .as_deref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] DSM declaration: cluster_interaction_deck_file requires cluster_interaction_class".to_owned(),
                })?;
        let (sha256, deck) = load_dsm_cluster_interaction_deck(resolved_files)?;
        if !same_acoustic_bands(deck.bands(), shape.bands()) {
            return Err(RunnerError::UnsupportedScenario {
                what: "invalid [acoustics] DSM cluster interaction deck band grid mismatch"
                    .to_owned(),
            });
        }
        let class = deck.class(class_id).map_err(acoustics_scenario_error)?;
        return Ok(DsmClusterInteractionInput {
            interaction: DsmClusterInteraction {
                correlation_coefficient: 0.0,
                shielding_loss_db: 0.0,
                correlation_by_band: Some(class.correlation_coefficient.clone()),
                shielding_loss_by_band_db: Some(class.shielding_loss_db.clone()),
            },
            deck_file_sha256_hex: Some(sha256),
            deck_dataset_id: Some(deck.meta().dataset_id.clone()),
            deck_source_class: Some(deck.meta().source_class.clone()),
            class_id: Some(class.class_id.clone()),
            uncertainty_db: Some(class.uncertainty_db),
        });
    }
    let correlation_coefficient = config.cluster_correlation_coefficient.unwrap_or(0.0);
    if !correlation_coefficient.is_finite() || !(0.0..=1.0).contains(&correlation_coefficient) {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: cluster_correlation_coefficient must be finite and in [0, 1]".to_owned(),
        });
    }
    let shielding_loss_db = config.cluster_shielding_loss_db.unwrap_or(0.0);
    if !shielding_loss_db.is_finite() || shielding_loss_db < 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: cluster_shielding_loss_db must be finite and non-negative".to_owned(),
        });
    }
    Ok(DsmClusterInteractionInput {
        interaction: DsmClusterInteraction {
            correlation_coefficient,
            shielding_loss_db,
            correlation_by_band: None,
            shielding_loss_by_band_db: None,
        },
        deck_file_sha256_hex: None,
        deck_dataset_id: None,
        deck_source_class: None,
        class_id: None,
        uncertainty_db: None,
    })
}

fn plume_deflection_from_config(
    config: &AcousticsDsmConfig,
) -> Result<Option<DsmPlumeDeflection>, RunnerError> {
    match (
        config.pad_deflection_origin_axial_m,
        config.pad_deflection_angle_deg,
    ) {
        (Some(origin_axial_m), Some(angle_deg)) => DsmPlumeDeflection::new(origin_axial_m, angle_deg)
            .map(Some)
            .map_err(acoustics_scenario_error),
        (None, None) => Ok(None),
        _ => Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: pad deflection origin and angle must be declared together".to_owned(),
        }),
    }
}

fn evaluate_source_input_station_and_trace(
    exit: DsmExitCondition,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    context: DsmEvaluationContext<'_>,
) -> Result<(DsmStationSpectrum, DsmLiftoffTrace), RunnerError> {
    match context.source_input {
        DsmSourceInput::Distributed(allocation) => {
            if let Some(axial_spectrum_curve) = context.axial_spectrum_curve {
                let source_band_weights = axial_spectrum_curve
                    .to_source_band_weights(allocation, context.shape, exit.exit_velocity_m_s)
                    .map_err(acoustics_scenario_error)?;
                let station = if let Some(deflection) = context.plume_deflection {
                    evaluate_deflected_source_band_weighted_station(
                        exit,
                        allocation,
                        &source_band_weights,
                        context.directivity,
                        receiver,
                        propagation.clone(),
                        deflection,
                    )
                } else {
                    evaluate_source_band_weighted_station(
                        exit,
                        allocation,
                        &source_band_weights,
                        context.directivity,
                        receiver,
                        propagation.clone(),
                    )
                }
                .map_err(acoustics_scenario_error)?;
                let trace = if let Some(deflection) = context.plume_deflection {
                    evaluate_deflected_source_band_weighted_trace(
                        exit,
                        allocation,
                        &source_band_weights,
                        receiver,
                        propagation,
                        context,
                        deflection,
                    )?
                } else {
                    evaluate_source_band_weighted_liftoff_trace(
                        exit,
                        allocation,
                        &source_band_weights,
                        context.directivity,
                        receiver,
                        propagation,
                        context.steps,
                    )
                    .map_err(acoustics_scenario_error)?
                };
                Ok((station, trace))
            } else {
                let station = if let Some(deflection) = context.plume_deflection {
                    evaluate_deflected_free_field_station(
                        exit,
                        allocation,
                        context.shape,
                        context.directivity,
                        receiver,
                        propagation.clone(),
                        deflection,
                    )
                } else {
                    evaluate_free_field_station(
                        exit,
                        allocation,
                        context.shape,
                        context.directivity,
                        receiver,
                        propagation.clone(),
                    )
                }
                .map_err(acoustics_scenario_error)?;
                let trace = if let Some(deflection) = context.plume_deflection {
                    evaluate_deflected_free_field_trace(
                        exit,
                        allocation,
                        receiver,
                        propagation,
                        context,
                        deflection,
                    )?
                } else {
                    evaluate_free_field_liftoff_trace(
                        exit,
                        allocation,
                        context.shape,
                        context.directivity,
                        receiver,
                        propagation,
                        context.steps,
                    )
                    .map_err(acoustics_scenario_error)?
                };
                Ok((station, trace))
            }
        }
        DsmSourceInput::Apparent(apparent_sources) => {
            if context.axial_spectrum_curve.is_some() {
                return Err(RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] DSM declaration: axial_spectrum_curve_file cannot pair with apparent_source_curve_file".to_owned(),
                });
            }
            let station = if let Some(deflection) = context.plume_deflection {
                evaluate_deflected_apparent_source_station(
                    exit,
                    context.shape,
                    apparent_sources,
                    context.directivity,
                    receiver,
                    propagation.clone(),
                    deflection,
                )
            } else {
                evaluate_apparent_source_station(
                    exit,
                    context.shape,
                    apparent_sources,
                    context.directivity,
                    receiver,
                    propagation.clone(),
                )
            }
            .map_err(acoustics_scenario_error)?;
            let trace = if let Some(deflection) = context.plume_deflection {
                evaluate_deflected_apparent_source_trace(
                    exit,
                    apparent_sources,
                    receiver,
                    propagation,
                    context,
                    deflection,
                )?
            } else {
                evaluate_apparent_source_liftoff_trace(
                    exit,
                    context.shape,
                    apparent_sources,
                    context.directivity,
                    receiver,
                    propagation,
                    context.steps,
                )
                .map_err(acoustics_scenario_error)?
            };
            Ok((station, trace))
        }
    }
}

fn evaluate_deflected_free_field_trace(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    context: DsmEvaluationContext<'_>,
    deflection: DsmPlumeDeflection,
) -> Result<DsmLiftoffTrace, RunnerError> {
    let mut points = Vec::with_capacity(context.steps.len());
    for step in context.steps {
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )
        .map_err(acoustics_scenario_error)?;
        let station = evaluate_deflected_free_field_station(
            exit,
            allocation,
            context.shape,
            context.directivity,
            &moved_receiver,
            propagation.clone(),
            deflection,
        )
        .map_err(acoustics_scenario_error)?;
        points.push(DsmLiftoffTracePoint {
            step: *step,
            station,
        });
    }
    DsmLiftoffTrace::new(receiver.station_id.clone(), points).map_err(acoustics_scenario_error)
}

fn evaluate_deflected_source_band_weighted_trace(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    source_band_weights: &DsmSourceBandWeights,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    context: DsmEvaluationContext<'_>,
    deflection: DsmPlumeDeflection,
) -> Result<DsmLiftoffTrace, RunnerError> {
    let mut points = Vec::with_capacity(context.steps.len());
    for step in context.steps {
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )
        .map_err(acoustics_scenario_error)?;
        let station = evaluate_deflected_source_band_weighted_station(
            exit,
            allocation,
            source_band_weights,
            context.directivity,
            &moved_receiver,
            propagation.clone(),
            deflection,
        )
        .map_err(acoustics_scenario_error)?;
        points.push(DsmLiftoffTracePoint {
            step: *step,
            station,
        });
    }
    DsmLiftoffTrace::new(receiver.station_id.clone(), points).map_err(acoustics_scenario_error)
}

fn evaluate_deflected_apparent_source_trace(
    exit: DsmExitCondition,
    apparent_sources: &DsmApparentSourceCurve,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    context: DsmEvaluationContext<'_>,
    deflection: DsmPlumeDeflection,
) -> Result<DsmLiftoffTrace, RunnerError> {
    let mut points = Vec::with_capacity(context.steps.len());
    for step in context.steps {
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )
        .map_err(acoustics_scenario_error)?;
        let station = evaluate_deflected_apparent_source_station(
            exit,
            context.shape,
            apparent_sources,
            context.directivity,
            &moved_receiver,
            propagation.clone(),
            deflection,
        )
        .map_err(acoustics_scenario_error)?;
        points.push(DsmLiftoffTracePoint {
            step: *step,
            station,
        });
    }
    DsmLiftoffTrace::new(receiver.station_id.clone(), points).map_err(acoustics_scenario_error)
}

fn apply_identical_cluster_interaction(
    station: DsmStationSpectrum,
    trace: DsmLiftoffTrace,
    cluster_engine_count: u32,
    interaction: &DsmClusterInteraction,
) -> Result<(DsmStationSpectrum, DsmLiftoffTrace), RunnerError> {
    if interaction.is_default_for_bands(station.spectrum.levels().len()) {
        return Ok((station, trace));
    }
    let transfer_db = interaction.identical_cluster_transfer_db_by_band(
        cluster_engine_count,
        station.spectrum.levels().len(),
    )?;
    let station = DsmStationSpectrum::new(
        station.station_id,
        station
            .spectrum
            .apply_transfer_db(&transfer_db)
            .map_err(acoustics_scenario_error)?,
        station.source_count,
    )
    .map_err(acoustics_scenario_error)?;
    let points = trace
        .points()
        .iter()
        .map(|point| {
            Ok(DsmLiftoffTracePoint {
                step: point.step,
                station: DsmStationSpectrum::new(
                    point.station.station_id.clone(),
                    point
                        .station
                        .spectrum
                        .apply_transfer_db(&transfer_db)
                        .map_err(acoustics_scenario_error)?,
                    point.station.source_count,
                )
                .map_err(acoustics_scenario_error)?,
            })
        })
        .collect::<Result<Vec<_>, RunnerError>>()?;
    let trace =
        DsmLiftoffTrace::new(trace.station_id.clone(), points).map_err(acoustics_scenario_error)?;
    Ok((station, trace))
}

impl DsmClusterInteraction {
    fn is_default_for_bands(&self, band_count: usize) -> bool {
        self.correlation_by_band.is_none()
            && self.shielding_loss_by_band_db.is_none()
            && self.correlation_coefficient == 0.0
            && self.shielding_loss_db == 0.0
            && band_count > 0
    }

    fn correlation_for_band(&self, band_index: usize) -> Result<f64, RunnerError> {
        if let Some(values) = &self.correlation_by_band {
            values
                .get(band_index)
                .copied()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] DSM cluster interaction correlation band count mismatch".to_owned(),
                })
        } else {
            Ok(self.correlation_coefficient)
        }
    }

    fn shielding_loss_for_band_db(&self, band_index: usize) -> Result<f64, RunnerError> {
        if let Some(values) = &self.shielding_loss_by_band_db {
            values
                .get(band_index)
                .copied()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what:
                        "invalid [acoustics] DSM cluster interaction shielding band count mismatch"
                            .to_owned(),
                })
        } else {
            Ok(self.shielding_loss_db)
        }
    }

    fn identical_cluster_transfer_db_by_band(
        &self,
        cluster_engine_count: u32,
        band_count: usize,
    ) -> Result<Vec<f64>, RunnerError> {
        let mut transfer_db = Vec::with_capacity(band_count);
        for band_index in 0..band_count {
            let coherent_factor = 1.0
                + self.correlation_for_band(band_index)? * (f64::from(cluster_engine_count) - 1.0);
            transfer_db.push(
                10.0 * coherent_factor.log10() - self.shielding_loss_for_band_db(band_index)?,
            );
        }
        Ok(transfer_db)
    }
}

fn combine_cluster_spectra(
    spectra: &[SplSpectrum],
    interaction: &DsmClusterInteraction,
) -> Result<SplSpectrum, RunnerError> {
    let first = spectra
        .first()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: cluster spectra must not be empty"
                .to_owned(),
        })?;
    if interaction.is_default_for_bands(first.levels().len()) {
        let mut combined = first.clone();
        for spectrum in &spectra[1..] {
            combined = combined
                .add_levels(spectrum)
                .map_err(acoustics_scenario_error)?;
        }
        return Ok(combined);
    }
    let bands = first.bands().to_vec();
    let mut levels = Vec::with_capacity(first.levels().len());
    for band_index in 0..first.levels().len() {
        let mut incoherent_pa2 = 0.0;
        let mut pressure_amplitude_sum = 0.0;
        for spectrum in spectra {
            if !same_acoustic_bands(&bands, spectrum.bands()) {
                return Err(RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] DSM cluster spectrum band grid mismatch".to_owned(),
                });
            }
            let band_pressure_pa2 = spectrum.levels()[band_index].mean_square_pressure_pa2();
            incoherent_pa2 += band_pressure_pa2;
            pressure_amplitude_sum += band_pressure_pa2.sqrt();
        }
        let coherent_pa2 = pressure_amplitude_sum * pressure_amplitude_sum;
        let interacted_pa2 = incoherent_pa2
            + interaction.correlation_for_band(band_index)? * (coherent_pa2 - incoherent_pa2);
        let shielded_pa2 = interacted_pa2
            * 10.0_f64.powf(-interaction.shielding_loss_for_band_db(band_index)? / 10.0);
        levels.push(
            Spl::from_mean_square_pressure_pa2(shielded_pa2).map_err(acoustics_scenario_error)?,
        );
    }
    SplSpectrum::new(bands, levels).map_err(acoustics_scenario_error)
}

fn evaluate_geometric_cluster_station_and_trace(
    per_engine_exit: DsmExitCondition,
    cluster_engines: &[DsmClusterEngineInput],
    receiver_azimuth_deg: f64,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    context: DsmEvaluationContext<'_>,
    interaction: &DsmClusterInteraction,
) -> Result<(DsmStationSpectrum, DsmLiftoffTrace), RunnerError> {
    let mut station_spectra = Vec::with_capacity(cluster_engines.len());
    let mut engine_traces = Vec::with_capacity(cluster_engines.len());
    let mut source_count = 0usize;

    for engine in cluster_engines {
        let engine_receiver = receiver_for_cluster_engine(receiver, receiver_azimuth_deg, engine)?;
        let (station, trace) = evaluate_source_input_station_and_trace(
            per_engine_exit,
            &engine_receiver,
            propagation.clone(),
            context,
        )?;
        source_count += station.source_count;
        station_spectra.push(station.spectrum);
        engine_traces.push(trace);
    }

    if station_spectra.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: cluster_engines must not be empty"
                .to_owned(),
        });
    }
    let station_spectrum = combine_cluster_spectra(&station_spectra, interaction)?;
    let station =
        DsmStationSpectrum::new(receiver.station_id.clone(), station_spectrum, source_count)
            .map_err(acoustics_scenario_error)?;
    let first_trace = engine_traces
        .first()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM declaration: cluster_engines must not be empty"
                .to_owned(),
        })?;
    let mut points = Vec::with_capacity(first_trace.points().len());
    for step_index in 0..first_trace.points().len() {
        let step = first_trace.points()[step_index].step;
        let mut step_spectra = Vec::with_capacity(engine_traces.len());
        for trace in &engine_traces {
            let Some(point) = trace.points().get(step_index) else {
                return Err(RunnerError::UnsupportedScenario {
                    what: "internal invariant: cluster liftoff trace length mismatch".to_owned(),
                });
            };
            step_spectra.push(point.station.spectrum.clone());
        }
        let spectrum = combine_cluster_spectra(&step_spectra, interaction)?;
        points.push(DsmLiftoffTracePoint {
            step,
            station: DsmStationSpectrum::new(receiver.station_id.clone(), spectrum, source_count)
                .map_err(acoustics_scenario_error)?,
        });
    }
    let trace = DsmLiftoffTrace::new(receiver.station_id.clone(), points)
        .map_err(acoustics_scenario_error)?;
    Ok((station, trace))
}

fn receiver_for_cluster_engine(
    receiver: &DsmReceiver,
    receiver_azimuth_deg: f64,
    engine: &DsmClusterEngineInput,
) -> Result<DsmReceiver, RunnerError> {
    let azimuth_rad = receiver_azimuth_deg.to_radians();
    let receiver_x_m = receiver.radial_m * azimuth_rad.cos();
    let receiver_y_m = receiver.radial_m * azimuth_rad.sin();
    let dx_m = receiver_x_m - engine.offset_x_m;
    let dy_m = receiver_y_m - engine.offset_y_m;
    DsmReceiver::new(
        receiver.station_id.clone(),
        receiver.axial_m,
        dx_m.hypot(dy_m),
    )
    .map_err(acoustics_scenario_error)
}

fn load_iop_duct_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, IopDuctDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.iop.duct_deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.iop.duct_deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] IOP duct deck UTF-8: {error}"),
        })?;
    let deck = IopDuctDeck::load_from_str(text).map_err(iop_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] IOP duct deck digest mismatch after parsing".to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_iop_suppression_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, IopSuppressionDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.iop.suppression_deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.iop.suppression_deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] IOP suppression deck UTF-8: {error}"),
        })?;
    let deck = IopSuppressionDeck::load_from_str(text).map_err(iop_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] IOP suppression deck digest mismatch after parsing"
                .to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, DsmPredictionDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM deck UTF-8: {error}"),
        })?;
    let deck = DsmPredictionDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM deck digest mismatch after parsing".to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_ascent_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, AscentAcousticDeck), RunnerError> {
    let resolved =
        resolved_files
            .get("acoustics.ascent.deck_file")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "internal invariant: resolved file `acoustics.ascent.deck_file` missing after pin verification".to_owned(),
            })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] ascent deck UTF-8: {error}"),
        })?;
    let deck = AscentAcousticDeck::load_from_str(text).map_err(ascent_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] ascent deck digest mismatch after parsing".to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_fairing_internal_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, FairingInternalDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.fairing_internal.deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.fairing_internal.deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] fairing-internal deck UTF-8: {error}"),
        })?;
    let deck = FairingInternalDeck::load_from_str(text).map_err(fairing_internal_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] fairing-internal deck digest mismatch after parsing"
                .to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_response_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, VibroacousticTransferDeck), RunnerError> {
    let resolved =
        resolved_files
            .get("acoustics.response.deck_file")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "internal invariant: resolved file `acoustics.response.deck_file` missing after pin verification".to_owned(),
            })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] response deck UTF-8: {error}"),
        })?;
    let deck = VibroacousticTransferDeck::load_from_str(text).map_err(response_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] response deck digest mismatch after parsing".to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_source_curve(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, Sp8072AxialPowerCurveDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.source_curve_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.source_curve_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM source curve UTF-8: {error}"),
        })?;
    let deck = Sp8072AxialPowerCurveDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM source curve digest mismatch after parsing".to_owned(),
        });
    }
    deck.validate_source_manifest(sp8072_source_manifest()?)
        .map_err(acoustics_scenario_error)?;
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_apparent_source_curve(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, Sp8072ApparentSourceCurveDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.apparent_source_curve_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.apparent_source_curve_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM apparent-source curve UTF-8: {error}"),
        })?;
    let deck =
        Sp8072ApparentSourceCurveDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM apparent-source curve digest mismatch after parsing"
                .to_owned(),
        });
    }
    deck.validate_source_manifest(sp8072_source_manifest()?)
        .map_err(acoustics_scenario_error)?;
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_axial_spectrum_curve(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, Sp8072AxialSpectrumCurveDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.axial_spectrum_curve_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.axial_spectrum_curve_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM axial spectrum curve UTF-8: {error}"),
        })?;
    let deck =
        Sp8072AxialSpectrumCurveDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM axial spectrum curve digest mismatch after parsing"
                .to_owned(),
        });
    }
    deck.validate_source_manifest(sp8072_source_manifest()?)
        .map_err(acoustics_scenario_error)?;
    Ok((resolved.sha256_hex.clone(), deck))
}

fn spectrum_from_config(
    config: &AcousticsDsmConfig,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<DsmSpectrumInput, RunnerError> {
    if config.spectrum_curve_file.is_some() {
        let (sha256, curve) = load_dsm_spectrum_curve(resolved_files)?;
        let shape = curve
            .curve()
            .to_spectrum_shape(config.exit_diameter_m, config.exit_velocity_m_s)
            .map_err(acoustics_scenario_error)?;
        Ok(DsmSpectrumInput {
            shape,
            curve_file_sha256_hex: Some(sha256),
            curve_dataset_id: Some(curve.meta().dataset_id.clone()),
            curve_figure_id: Some(curve.meta().source_figure_id.clone()),
        })
    } else {
        Ok(DsmSpectrumInput {
            shape: DsmSpectrumShape::strouhal_gaussian(
                config.exit_diameter_m,
                config.exit_velocity_m_s,
                config.peak_strouhal,
                config.width_decades,
            )
            .map_err(acoustics_scenario_error)?,
            curve_file_sha256_hex: None,
            curve_dataset_id: None,
            curve_figure_id: None,
        })
    }
}

fn load_dsm_spectrum_curve(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, Sp8072SpectrumCurveDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.spectrum_curve_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.spectrum_curve_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM spectrum curve UTF-8: {error}"),
        })?;
    let deck = Sp8072SpectrumCurveDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM spectrum curve digest mismatch after parsing".to_owned(),
        });
    }
    deck.validate_source_manifest(sp8072_source_manifest()?)
        .map_err(acoustics_scenario_error)?;
    Ok((resolved.sha256_hex.clone(), deck))
}

fn directivity_from_config(
    config: &AcousticsDsmConfig,
    shape: &DsmSpectrumShape,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<DsmDirectivityInput, RunnerError> {
    if config.directivity_curve_file.is_some() {
        let (sha256, curve) = load_dsm_directivity_curve(resolved_files)?;
        let deck = curve
            .curve()
            .to_directivity_deck(
                shape.bands().to_vec(),
                config.exit_diameter_m,
                config.exit_velocity_m_s,
            )
            .map_err(acoustics_scenario_error)?;
        Ok(DsmDirectivityInput {
            deck,
            curve_file_sha256_hex: Some(sha256),
            curve_dataset_id: Some(curve.meta().dataset_id.clone()),
            curve_figure_id: Some(curve.meta().source_figure_id.clone()),
        })
    } else {
        Ok(DsmDirectivityInput {
            deck: DsmDirectivityDeck::omnidirectional(shape.bands().to_vec())
                .map_err(acoustics_scenario_error)?,
            curve_file_sha256_hex: None,
            curve_dataset_id: None,
            curve_figure_id: None,
        })
    }
}

fn suppression_from_config(
    config: &AcousticsDsmConfig,
    directivity: DsmDirectivityDeck,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<DsmSuppressionInput, RunnerError> {
    if config.suppression_deck_file.is_some() {
        let class_id =
            config
                .suppression_class
                .as_deref()
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "invalid [acoustics] DSM declaration: suppression_deck_file requires suppression_class".to_owned(),
                })?;
        let (sha256, deck) = load_dsm_suppression_deck(resolved_files)?;
        if !same_acoustic_bands(deck.bands(), directivity.bands()) {
            return Err(RunnerError::UnsupportedScenario {
                what: "invalid [acoustics] DSM suppression deck band grid mismatch".to_owned(),
            });
        }
        let class = deck.class(class_id).map_err(acoustics_scenario_error)?;
        let selected_class_id = class.class_id.clone();
        let uncertainty_db = class.uncertainty_db;
        let transfer_db = deck
            .transfer_db_for_class(class_id)
            .map_err(acoustics_scenario_error)?;
        Ok(DsmSuppressionInput {
            directivity: directivity
                .with_band_transfer_db(&transfer_db)
                .map_err(acoustics_scenario_error)?,
            suppression_deck_file_sha256_hex: Some(sha256),
            suppression_deck_dataset_id: Some(deck.meta().dataset_id.clone()),
            suppression_deck_source_class: Some(deck.meta().source_class.clone()),
            suppression_class_id: Some(selected_class_id),
            suppression_uncertainty_db: Some(uncertainty_db),
        })
    } else {
        Ok(DsmSuppressionInput {
            directivity,
            suppression_deck_file_sha256_hex: None,
            suppression_deck_dataset_id: None,
            suppression_deck_source_class: None,
            suppression_class_id: None,
            suppression_uncertainty_db: None,
        })
    }
}

fn propagation_from_config(
    config: &AcousticsDsmConfig,
    shape: &DsmSpectrumShape,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<DsmPropagationInput, RunnerError> {
    let path_length_extra_m = config.pad_path_length_extra_m.unwrap_or(0.0);
    if config.absorption_deck_file.is_some() {
        let (sha256, deck) = load_dsm_absorption_deck(resolved_files)?;
        let profile = deck
            .absorption_profile(config.absorption_profile_id.as_deref())
            .map_err(acoustics_scenario_error)?;
        let propagation = DsmPropagation {
            ambient_impedance_pa_s_m: config.ambient_impedance_pa_s_m,
            absorption: profile.absorption().clone(),
            path_length_extra_m: 0.0,
        }
        .with_path_length_extra_m(path_length_extra_m)
        .map_err(acoustics_scenario_error)?;
        Ok(DsmPropagationInput {
            propagation,
            absorption_deck_file_sha256_hex: Some(sha256),
            absorption_deck_dataset_id: Some(deck.meta().dataset_id.clone()),
            absorption_deck_source_class: Some(deck.meta().source_class.clone()),
            absorption_profile_id: Some(profile.profile_id.clone()),
        })
    } else {
        Ok(DsmPropagationInput {
            propagation: DsmPropagation::uniform(
                config.ambient_impedance_pa_s_m,
                config.absorption_db_per_m,
                shape.bands().to_vec(),
            )
            .and_then(|propagation| propagation.with_path_length_extra_m(path_length_extra_m))
            .map_err(acoustics_scenario_error)?,
            absorption_deck_file_sha256_hex: None,
            absorption_deck_dataset_id: None,
            absorption_deck_source_class: None,
            absorption_profile_id: None,
        })
    }
}

fn load_dsm_suppression_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, PadSuppressionDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.suppression_deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.suppression_deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM suppression deck UTF-8: {error}"),
        })?;
    let deck = PadSuppressionDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM suppression deck digest mismatch after parsing"
                .to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_cluster_interaction_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, PadClusterInteractionDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.cluster_interaction_deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.cluster_interaction_deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM cluster interaction deck UTF-8: {error}"),
        })?;
    let deck = PadClusterInteractionDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM cluster interaction deck digest mismatch after parsing"
                .to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn load_dsm_absorption_deck(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, DsmAirAbsorptionDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.absorption_deck_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.absorption_deck_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM absorption deck UTF-8: {error}"),
        })?;
    let deck = DsmAirAbsorptionDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM absorption deck digest mismatch after parsing"
                .to_owned(),
        });
    }
    Ok((resolved.sha256_hex.clone(), deck))
}

fn same_acoustic_bands(lhs: &[AcousticBand], rhs: &[AcousticBand]) -> bool {
    lhs.len() == rhs.len()
        && lhs.iter().zip(rhs.iter()).all(|(lhs, rhs)| {
            lhs.center_hz.to_bits() == rhs.center_hz.to_bits()
                && lhs.lower_hz.to_bits() == rhs.lower_hz.to_bits()
                && lhs.upper_hz.to_bits() == rhs.upper_hz.to_bits()
        })
}

fn load_dsm_directivity_curve(
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<(String, Sp8072DirectivityCurveDeck), RunnerError> {
    let resolved = resolved_files
        .get("acoustics.dsm.directivity_curve_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "internal invariant: resolved file `acoustics.dsm.directivity_curve_file` missing after pin verification".to_owned(),
        })?;
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|error| RunnerError::UnsupportedScenario {
            what: format!("invalid [acoustics] DSM directivity curve UTF-8: {error}"),
        })?;
    let deck = Sp8072DirectivityCurveDeck::load_from_str(text).map_err(acoustics_scenario_error)?;
    if deck.sha256_hex() != resolved.sha256_hex {
        return Err(RunnerError::UnsupportedScenario {
            what: "invalid [acoustics] DSM directivity curve digest mismatch after parsing"
                .to_owned(),
        });
    }
    deck.validate_source_manifest(sp8072_source_manifest()?)
        .map_err(acoustics_scenario_error)?;
    Ok((resolved.sha256_hex.clone(), deck))
}

fn sp8072_source_manifest() -> Result<&'static Sp8072SourceManifest, RunnerError> {
    static SP8072_SOURCE_MANIFEST: OnceLock<Result<Sp8072SourceManifest, String>> = OnceLock::new();

    SP8072_SOURCE_MANIFEST
        .get_or_init(|| {
            Sp8072SourceManifest::load_from_str(include_str!(
                "../../../data/acoustics/sp8072-source-v1.toml"
            ))
            .map_err(|error| error.to_string())
        })
        .as_ref()
        .map_err(|error| acoustics_scenario_error(error.as_str()))
}

fn spl_spectrum_csv(spectrum: &SplSpectrum) -> Vec<u8> {
    let mut csv = String::from("center_hz,lower_hz,upper_hz,spl_db\n");
    for (band, level) in spectrum.bands().iter().zip(spectrum.levels().iter()) {
        csv.push_str(&format!(
            "{:.17e},{:.17e},{:.17e},{:.17e}\n",
            band.center_hz,
            band.lower_hz,
            band.upper_hz,
            level.db_re_20upa()
        ));
    }
    csv.into_bytes()
}

fn acceleration_psd_spectrum_csv(spectrum: &AccelerationPsdSpectrum) -> Vec<u8> {
    let mut csv = String::from("center_hz,lower_hz,upper_hz,acceleration_psd_m2_s4_per_hz\n");
    for (band, value) in spectrum.bands().iter().zip(spectrum.values().iter()) {
        csv.push_str(&format!(
            "{:.17e},{:.17e},{:.17e},{:.17e}\n",
            band.center_hz,
            band.lower_hz,
            band.upper_hz,
            value.m2_s4_per_hz()
        ));
    }
    csv.into_bytes()
}

fn force_psd_spectrum_csv(spectrum: &ForcePsdSpectrum) -> Vec<u8> {
    let mut csv = String::from("center_hz,lower_hz,upper_hz,modal_force_psd_n2_per_hz\n");
    for (band, value) in spectrum.bands().iter().zip(spectrum.values().iter()) {
        csv.push_str(&format!(
            "{:.17e},{:.17e},{:.17e},{:.17e}\n",
            band.center_hz,
            band.lower_hz,
            band.upper_hz,
            value.n2_per_hz()
        ));
    }
    csv.into_bytes()
}

fn response_p95_50_psd_csv(
    statistics: &[ResponseP95_50BandStatistic],
    sample_count: usize,
    value_column: &'static str,
    tolerance_factor: f64,
) -> Vec<u8> {
    let mut csv = format!(
        "center_hz,lower_hz,upper_hz,mean_psd_db,sigma_psd_db,p95_50_psd_db,{value_column},min_sample_psd_db,max_sample_psd_db,sample_count,coverage_probability,confidence_probability,tolerance_factor\n",
    );
    for statistic in statistics {
        csv.push_str(&format!(
            "{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{},{:.17e},{:.17e},{:.17e}\n",
            statistic.band.center_hz,
            statistic.band.lower_hz,
            statistic.band.upper_hz,
            statistic.mean_psd_db,
            statistic.sigma_psd_db,
            statistic.p95_50_psd_db,
            statistic.p95_50_psd_value,
            statistic.min_sample_psd_db,
            statistic.max_sample_psd_db,
            sample_count,
            P95_50_COVERAGE_PROBABILITY,
            P95_50_CONFIDENCE_PROBABILITY,
            tolerance_factor,
        ));
    }
    csv.into_bytes()
}

fn response_p95_50_sample_trace_csv(predictions: &[ResponseP95_50SamplePrediction]) -> Vec<u8> {
    let mut csv = String::from(
        "sample_index,sample_id,class_id,surface_mass_kg_m2,area_m2,mode_shape_factor,modal_force_participation_m2,surface_mass_gain,total_mean_square_acceleration_m2_s4,total_mean_square_force_n2\n",
    );
    for sample in predictions {
        let sample_id = sample.sample_id.as_deref().unwrap_or("");
        csv.push_str(&format!(
            "{},{},{},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}\n",
            sample.sample_index,
            csv_field(sample_id),
            csv_field(&sample.class_id),
            sample.input.surface_mass_kg_m2,
            sample.input.area_m2,
            sample.input.mode_shape_factor,
            sample.prediction.modal_force_participation_m2,
            sample.prediction.surface_mass_gain,
            sample
                .prediction
                .acceleration_psd
                .total_mean_square_acceleration_m2_s4(),
            sample
                .prediction
                .modal_force_psd
                .total_mean_square_force_n2(),
        ));
    }
    csv.into_bytes()
}

fn fairing_internal_p95_50_sample_trace_csv(
    predictions: &[FairingInternalSamplePrediction],
) -> Vec<u8> {
    let mut csv = String::from(
        "sample_index,sample_id,class_id,fill_factor,fill_factor_gain_db,oaspl_db,uncertainty_db\n",
    );
    for sample in predictions {
        let sample_id = sample.sample_id.as_deref().unwrap_or("");
        csv.push_str(&format!(
            "{},{},{},{:.17e},{:.17e},{:.17e},{:.17e}\n",
            sample.sample_index,
            csv_field(sample_id),
            csv_field(&sample.class_id),
            sample.input.fill_factor,
            sample.prediction.fill_factor_gain_db,
            sample.prediction.oaspl.db_re_20upa(),
            sample.prediction.uncertainty_db,
        ));
    }
    csv.into_bytes()
}

fn ascent_trace_csv(
    trace: &AscentAcousticTracePrediction,
    sample_ids: &[Option<String>],
) -> Vec<u8> {
    let mut csv = String::from(
        "sample_index,sample_id,class_id,dynamic_pressure_pa,mach,oaspl_db,uncertainty_db\n",
    );
    for (index, point) in trace.points.iter().enumerate() {
        let sample_id = sample_ids
            .get(index)
            .and_then(Option::as_deref)
            .unwrap_or("");
        csv.push_str(&format!(
            "{index},{},{},{:.17e},{:.17e},{:.17e},{:.17e}\n",
            csv_field(sample_id),
            csv_field(&point.class_id),
            point.dynamic_pressure_pa,
            point.mach,
            point.oaspl.db_re_20upa(),
            point.uncertainty_db,
        ));
    }
    csv.into_bytes()
}

fn csv_field(value: &str) -> String {
    if value
        .bytes()
        .any(|byte| matches!(byte, b',' | b'"' | b'\n' | b'\r'))
    {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

fn acoustics_scenario_error(error: impl std::fmt::Display) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] DSM declaration: {error}"),
    }
}

fn ascent_scenario_error(error: impl std::fmt::Display) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] ascent declaration: {error}"),
    }
}

fn fairing_internal_scenario_error(error: impl std::fmt::Display) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] fairing-internal declaration: {error}"),
    }
}

fn response_scenario_error(error: impl std::fmt::Display) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] response declaration: {error}"),
    }
}

fn iop_scenario_error(error: impl std::fmt::Display) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [acoustics] IOP declaration: {error}"),
    }
}
