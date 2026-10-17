//! Validation fixture helpers for aerodynamic buildup methods.
//!
//! These helpers are intentionally offline/parser-gated. They parse
//! provenance-bearing TOML fixtures, evaluate the public buildup API, and
//! return deterministic tolerance reports without changing the runtime aero
//! hot path.

use core::f64::consts::PI;
use std::collections::BTreeMap;

use serde::Deserialize;

use crate::buildup::{
    Afterbody, BodyGeometry, BuildupGrid, DatcomComponentBuildup, DragBuildupModel, FinSet,
    FlowCondition, NoseShape, ProtuberanceSet,
};
use crate::deck::AeroDynamicDerivatives;
use crate::error::AeroError;

const DATCOM_VALIDATION_SCHEMA_VERSION: u32 = 1;
const NACA_1048_CROSSFLOW_SCHEMA_VERSION: u32 = 1;
const NACA_1048_CROSSFLOW_DRAG_RATIO_SCHEMA_VERSION: u32 = 1;
const NACA_1048_INCREMENTAL_FOREDRAG_SCHEMA_VERSION: u32 = 1;
const SEARS_HAACK_WAVE_DRAG_SCHEMA_VERSION: u32 = 1;
const DATCOM_FIN_BODY_CARRYOVER_SCHEMA_VERSION: u32 = 1;
const DATCOM_PROTUBERANCE_DRAG_SCHEMA_VERSION: u32 = 1;
const DEFAULT_ABS_TOLERANCE: f64 = 0.0;
const DEFAULT_REL_TOLERANCE: f64 = 0.0;

/// Parsed DATCOM-style buildup validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatcomBuildupValidationDeck {
    openbmp: DatcomValidationSchemaMarker,
    provenance: DatcomValidationProvenance,
    geometry: DatcomValidationGeometry,
    tolerances: DatcomValidationTolerances,
    reference_reynolds_at_mach1: f64,
    samples: Vec<DatcomValidationSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationSchemaMarker {
    aero_datcom_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
}

/// Validation report emitted from a DATCOM-style buildup fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct DatcomBuildupValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Per-sample validation report.
#[derive(Clone, Debug, PartialEq)]
pub struct DatcomBuildupValidationSampleReport {
    /// Sample id from the fixture.
    pub id: String,
    /// Number of scalar values checked for this sample.
    pub checked_value_count: usize,
    /// Maximum absolute error observed for this sample.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed for this sample.
    pub max_normalized_error: f64,
    /// Whether this sample stayed inside tolerance.
    pub passed: bool,
}

/// Parsed NACA Report 1048 crossflow-equation validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NacaReport1048CrossflowValidationDeck {
    openbmp: NacaReport1048CrossflowSchemaMarker,
    provenance: NacaReport1048CrossflowProvenance,
    geometry: DatcomValidationGeometry,
    tolerances: DatcomValidationTolerances,
    samples: Vec<NacaReport1048CrossflowSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowSchemaMarker {
    naca_1048_crossflow_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
    source_url: String,
}

/// Validation report emitted from a NACA Report 1048 crossflow fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct NacaReport1048CrossflowValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Parsed NACA Report 1048 finite-length crossflow-drag-ratio fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NacaReport1048CrossflowDragRatioValidationDeck {
    openbmp: NacaReport1048CrossflowDragRatioSchemaMarker,
    provenance: NacaReport1048CrossflowDragRatioProvenance,
    tolerances: DatcomValidationTolerances,
    samples: Vec<NacaReport1048CrossflowDragRatioSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowDragRatioSchemaMarker {
    naca_1048_crossflow_drag_ratio_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowDragRatioProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
    source_url: String,
    report_page: u32,
}

/// Validation report emitted from a NACA Report 1048 drag-ratio fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct NacaReport1048CrossflowDragRatioValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Parsed NACA Report 1048 incremental-foredrag validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NacaReport1048IncrementalForedragValidationDeck {
    openbmp: NacaReport1048IncrementalForedragSchemaMarker,
    provenance: NacaReport1048IncrementalForedragProvenance,
    geometry: DatcomValidationGeometry,
    tolerances: DatcomValidationTolerances,
    samples: Vec<NacaReport1048IncrementalForedragSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048IncrementalForedragSchemaMarker {
    naca_1048_incremental_foredrag_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048IncrementalForedragProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
    source_url: String,
}

/// Validation report emitted from a NACA Report 1048 foredrag fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct NacaReport1048IncrementalForedragValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Parsed Sears-Haack/area-rule wave-drag validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SearsHaackWaveDragValidationDeck {
    openbmp: SearsHaackWaveDragSchemaMarker,
    provenance: SearsHaackWaveDragProvenance,
    geometry: DatcomValidationGeometry,
    tolerances: DatcomValidationTolerances,
    samples: Vec<SearsHaackWaveDragSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct SearsHaackWaveDragSchemaMarker {
    sears_haack_wave_drag_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct SearsHaackWaveDragProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
}

/// Validation report emitted from a Sears-Haack wave-drag fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct SearsHaackWaveDragValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Parsed DATCOM fin/body carryover validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatcomFinBodyCarryoverValidationDeck {
    openbmp: DatcomFinBodyCarryoverSchemaMarker,
    provenance: DatcomFinBodyCarryoverProvenance,
    tolerances: DatcomValidationTolerances,
    samples: Vec<DatcomFinBodyCarryoverSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomFinBodyCarryoverSchemaMarker {
    datcom_fin_body_carryover_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomFinBodyCarryoverProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
}

/// Validation report emitted from a DATCOM fin/body carryover fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct DatcomFinBodyCarryoverValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

/// Parsed DATCOM protuberance-drag validation fixture.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatcomProtuberanceDragValidationDeck {
    openbmp: DatcomProtuberanceDragSchemaMarker,
    provenance: DatcomProtuberanceDragProvenance,
    geometry: DatcomValidationGeometry,
    tolerances: DatcomValidationTolerances,
    samples: Vec<DatcomProtuberanceDragSample>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomProtuberanceDragSchemaMarker {
    datcom_protuberance_drag_validation: u32,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomProtuberanceDragProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
    source_title: String,
}

/// Validation report emitted from a DATCOM protuberance-drag fixture.
#[derive(Clone, Debug, PartialEq)]
pub struct DatcomProtuberanceDragValidationReport {
    /// Fixture dataset id.
    pub dataset_id: String,
    /// Number of samples evaluated.
    pub sample_count: usize,
    /// Number of scalar values checked across all samples.
    pub checked_value_count: usize,
    /// Maximum absolute error observed.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Per-sample reports in fixture order.
    pub samples: Vec<DatcomBuildupValidationSampleReport>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationGeometry {
    body_diameter_m: f64,
    body_length_m: f64,
    reference_area_m2: f64,
    reference_length_m: f64,
    #[serde(default)]
    surface_roughness_m: f64,
    #[serde(default)]
    center_of_gravity_from_nose_m: Option<f64>,
    nose: DatcomValidationNose,
    #[serde(default)]
    afterbody: Option<DatcomValidationAfterbody>,
    #[serde(default)]
    fins: Option<DatcomValidationFinSet>,
    #[serde(default)]
    protuberances: Option<DatcomValidationProtuberanceSet>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationNose {
    kind: String,
    #[serde(default)]
    half_angle_deg: Option<f64>,
    #[serde(default)]
    fineness: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationAfterbody {
    exit_diameter_m: f64,
    length_m: f64,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationFinSet {
    count: u32,
    root_chord_m: f64,
    tip_chord_m: f64,
    span_m: f64,
    thickness_ratio: f64,
    sweep_deg: f64,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationProtuberanceSet {
    count: u32,
    frontal_area_m2: f64,
    drag_coefficient: f64,
    interference_factor: f64,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationTolerances {
    max_abs_error: f64,
    #[serde(default)]
    max_relative_error: f64,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomValidationSample {
    id: String,
    mach: f64,
    alpha_deg: f64,
    reynolds_length: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
    #[serde(default)]
    expected: ExpectedCoefficients,
    #[serde(default)]
    expected_derivatives: ExpectedDynamicDerivatives,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowSample {
    id: String,
    mach: f64,
    alpha_deg: f64,
    reynolds_length: f64,
    expected_crossflow_cn: f64,
    expected_cp_m: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048CrossflowDragRatioSample {
    id: String,
    body_label: String,
    fineness_ratio: f64,
    cylinder_section_cd_low_mach: f64,
    finite_length_drag_ratio: f64,
    expected_cd_alpha_90_low_mach: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct NacaReport1048IncrementalForedragSample {
    id: String,
    mach: f64,
    alpha_deg: f64,
    reynolds_length: f64,
    expected_delta_cd_foredrag: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct SearsHaackWaveDragSample {
    id: String,
    mach: f64,
    expected_cd_wave: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomFinBodyCarryoverSample {
    id: String,
    body_diameter_m: f64,
    fin_span_m: f64,
    expected_fin_on_body: f64,
    expected_body_on_fin: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatcomProtuberanceDragSample {
    id: String,
    mach: f64,
    expected_cd_protuberance: f64,
    #[serde(default)]
    max_abs_error: Option<f64>,
    #[serde(default)]
    max_relative_error: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct ExpectedCoefficients {
    cn: Option<f64>,
    cd: Option<f64>,
    cm: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct ExpectedDynamicDerivatives {
    cl_p: Option<f64>,
    cl_q: Option<f64>,
    cl_r: Option<f64>,
    cm_p: Option<f64>,
    cm_q: Option<f64>,
    cm_r: Option<f64>,
    cn_p: Option<f64>,
    cn_q: Option<f64>,
    cn_r: Option<f64>,
}

impl DatcomBuildupValidationDeck {
    /// Parse a DATCOM-style buildup validation fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "DATCOM validation fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against [`DatcomComponentBuildup`].
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation, model evaluation, or deck
    /// derivative lookup fails.
    pub fn evaluate(&self) -> Result<DatcomBuildupValidationReport, AeroError> {
        self.validate()?;
        let geom = self.geometry.body_geometry()?;
        let model = DatcomComponentBuildup::default();
        let derivative_deck = if self
            .samples
            .iter()
            .any(DatcomValidationSample::expects_derivatives)
        {
            Some(model.bake_deck(&geom, &self.derivative_grid())?)
        } else {
            None
        };

        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let flow = FlowCondition {
                mach: sample.mach,
                reynolds_length: sample.reynolds_length,
                alpha_rad: sample.alpha_deg.to_radians(),
            };
            let coeffs = model.coefficients(&geom, &flow)?;
            let derivatives = if sample.expects_derivatives() {
                derivative_deck
                    .as_ref()
                    .ok_or(AeroError::MalformedDeck {
                        reason: "DATCOM validation derivative deck is missing",
                    })?
                    .lookup_dynamic_derivatives(
                        sample.mach,
                        sample.alpha_deg,
                        0.0,
                        &BTreeMap::new(),
                    )?
            } else {
                None
            };

            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                sample.expected.cn,
                coeffs.cn,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            compare_optional(
                sample.expected.cd,
                coeffs.cd,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            compare_optional(
                sample.expected.cm,
                coeffs.cm,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;

            if let Some(derivatives) = derivatives {
                compare_dynamic_derivatives(
                    &sample.expected_derivatives,
                    derivatives,
                    sample.abs_tolerance(self.tolerances.max_abs_error),
                    sample.rel_tolerance(self.tolerances.max_relative_error),
                    &mut sample_report,
                )?;
            }

            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(DatcomBuildupValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.aero_datcom_validation != DATCOM_VALIDATION_SCHEMA_VERSION {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported DATCOM validation schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "DATCOM validation dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "DATCOM validation source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "DATCOM validation label is empty",
        )?;
        if self.provenance.source_class == "fielded-vehicle" {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM validation fixtures must not be fielded-vehicle data",
            });
        }
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if !self.reference_reynolds_at_mach1.is_finite() || self.reference_reynolds_at_mach1 <= 0.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM validation reference Reynolds number must be positive",
            });
        }
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM validation fixture has no samples",
            });
        }
        self.geometry.body_geometry()?;
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }

    fn derivative_grid(&self) -> BuildupGrid {
        let mut mach = self
            .samples
            .iter()
            .map(|sample| sample.mach)
            .collect::<Vec<_>>();
        let mut alpha_deg = self
            .samples
            .iter()
            .map(|sample| sample.alpha_deg)
            .collect::<Vec<_>>();
        sort_dedup_f64(&mut mach);
        sort_dedup_f64(&mut alpha_deg);
        BuildupGrid {
            mach,
            alpha_deg,
            beta_deg: vec![0.0],
            reference_reynolds_at_mach1: self.reference_reynolds_at_mach1,
        }
    }
}

impl NacaReport1048CrossflowValidationDeck {
    /// Parse a NACA Report 1048 crossflow-equation fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "NACA 1048 crossflow fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the DATCOM crossflow increment.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation or model evaluation fails.
    pub fn evaluate(&self) -> Result<NacaReport1048CrossflowValidationReport, AeroError> {
        self.validate()?;
        let geom = self.geometry.body_geometry()?;
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let flow = FlowCondition {
                mach: sample.mach,
                reynolds_length: sample.reynolds_length,
                alpha_rad: sample.alpha_deg.to_radians(),
            };
            let (actual_cn, actual_cp_m) =
                DatcomComponentBuildup::crossflow_normal_force_increment(&geom, &flow)?;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_crossflow_cn),
                actual_cn,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            compare_optional(
                Some(sample.expected_cp_m),
                actual_cp_m,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(NacaReport1048CrossflowValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.naca_1048_crossflow_validation != NACA_1048_CROSSFLOW_SCHEMA_VERSION {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported NACA 1048 crossflow schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "NACA 1048 crossflow dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "NACA 1048 crossflow source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "NACA 1048 crossflow validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "NACA 1048 crossflow source title is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_url,
            "NACA 1048 crossflow source URL is empty",
        )?;
        if self.provenance.source_class != "public-domain-naca" {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 crossflow fixture must be public-domain NACA data",
            });
        }
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "NACA 1048 crossflow fixture has no samples",
            });
        }
        self.geometry.body_geometry()?;
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl NacaReport1048CrossflowDragRatioValidationDeck {
    /// Parse a NACA Report 1048 finite-length crossflow-drag-ratio fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "NACA 1048 crossflow drag-ratio fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the NACA appendix finite-length correction.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation fails.
    pub fn evaluate(&self) -> Result<NacaReport1048CrossflowDragRatioValidationReport, AeroError> {
        self.validate()?;
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let actual_cd = sample.cylinder_section_cd_low_mach * sample.finite_length_drag_ratio;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_cd_alpha_90_low_mach),
                actual_cd,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(NacaReport1048CrossflowDragRatioValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.naca_1048_crossflow_drag_ratio_validation
            != NACA_1048_CROSSFLOW_DRAG_RATIO_SCHEMA_VERSION
        {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported NACA 1048 crossflow drag-ratio schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "NACA 1048 crossflow drag-ratio dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "NACA 1048 crossflow drag-ratio source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "NACA 1048 crossflow drag-ratio validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "NACA 1048 crossflow drag-ratio source title is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_url,
            "NACA 1048 crossflow drag-ratio source URL is empty",
        )?;
        if self.provenance.source_class != "public-domain-naca" {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 crossflow drag-ratio fixture must be public-domain NACA data",
            });
        }
        if self.provenance.report_page == 0 {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 crossflow drag-ratio report page must be positive",
            });
        }
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "NACA 1048 crossflow drag-ratio fixture has no samples",
            });
        }
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl NacaReport1048IncrementalForedragValidationDeck {
    /// Parse a NACA Report 1048 incremental-foredrag fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "NACA 1048 incremental-foredrag fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the DATCOM incremental foredrag path.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation or model evaluation fails.
    pub fn evaluate(&self) -> Result<NacaReport1048IncrementalForedragValidationReport, AeroError> {
        self.validate()?;
        let geom = self.geometry.body_geometry()?;
        let model = DatcomComponentBuildup::default();
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let flow = FlowCondition {
                mach: sample.mach,
                reynolds_length: sample.reynolds_length,
                alpha_rad: sample.alpha_deg.to_radians(),
            };
            let zero_flow = FlowCondition {
                alpha_rad: 0.0,
                ..flow
            };
            let actual_delta_cd =
                model.coefficients(&geom, &flow)?.cd - model.coefficients(&geom, &zero_flow)?.cd;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_delta_cd_foredrag),
                actual_delta_cd,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(NacaReport1048IncrementalForedragValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.naca_1048_incremental_foredrag_validation
            != NACA_1048_INCREMENTAL_FOREDRAG_SCHEMA_VERSION
        {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported NACA 1048 incremental-foredrag schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "NACA 1048 incremental-foredrag dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "NACA 1048 incremental-foredrag source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "NACA 1048 incremental-foredrag validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "NACA 1048 incremental-foredrag source title is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_url,
            "NACA 1048 incremental-foredrag source URL is empty",
        )?;
        if self.provenance.source_class != "public-domain-naca" {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 incremental-foredrag fixture must be public-domain NACA data",
            });
        }
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "NACA 1048 incremental-foredrag fixture has no samples",
            });
        }
        self.geometry.body_geometry()?;
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl SearsHaackWaveDragValidationDeck {
    /// Parse a Sears-Haack/area-rule wave-drag fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "Sears-Haack wave-drag fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the DATCOM area-rule wave-drag term.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation or model evaluation fails.
    pub fn evaluate(&self) -> Result<SearsHaackWaveDragValidationReport, AeroError> {
        self.validate()?;
        let geom = self.geometry.body_geometry()?;
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let actual_cd =
                DatcomComponentBuildup::area_rule_wave_drag_coefficient(&geom, sample.mach)?;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_cd_wave),
                actual_cd,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(SearsHaackWaveDragValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.sears_haack_wave_drag_validation != SEARS_HAACK_WAVE_DRAG_SCHEMA_VERSION {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported Sears-Haack wave-drag schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "Sears-Haack wave-drag dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "Sears-Haack wave-drag source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "Sears-Haack wave-drag validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "Sears-Haack wave-drag source title is empty",
        )?;
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "Sears-Haack wave-drag fixture has no samples",
            });
        }
        self.geometry.body_geometry()?;
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl DatcomFinBodyCarryoverValidationDeck {
    /// Parse a DATCOM fin/body carryover fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "DATCOM fin/body carryover fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the DATCOM carryover-factor seam.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation or model evaluation fails.
    pub fn evaluate(&self) -> Result<DatcomFinBodyCarryoverValidationReport, AeroError> {
        self.validate()?;
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let geom = sample.body_geometry();
            let fins = sample.fin_set();
            let (actual_fin_on_body, actual_body_on_fin) =
                DatcomComponentBuildup::fin_body_carryover_factors(&geom, fins)?;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_fin_on_body),
                actual_fin_on_body,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            compare_optional(
                Some(sample.expected_body_on_fin),
                actual_body_on_fin,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(DatcomFinBodyCarryoverValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.datcom_fin_body_carryover_validation
            != DATCOM_FIN_BODY_CARRYOVER_SCHEMA_VERSION
        {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported DATCOM fin/body carryover schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "DATCOM fin/body carryover dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "DATCOM fin/body carryover source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "DATCOM fin/body carryover validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "DATCOM fin/body carryover source title is empty",
        )?;
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM fin/body carryover fixture has no samples",
            });
        }
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl DatcomProtuberanceDragValidationDeck {
    /// Parse a DATCOM protuberance-drag fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML/schema failures and
    /// [`AeroError::InvalidParameter`] / [`AeroError::NonFinite`] for invalid
    /// fixture values.
    pub fn load_from_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_err| AeroError::MalformedDeck {
            reason: "DATCOM protuberance-drag fixture TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the fixture against the DATCOM protuberance-drag seam.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation or model evaluation fails.
    pub fn evaluate(&self) -> Result<DatcomProtuberanceDragValidationReport, AeroError> {
        self.validate()?;
        let geom = self.geometry.body_geometry()?;
        let mut sample_reports = Vec::with_capacity(self.samples.len());
        let mut checked_value_count = 0_usize;
        let mut max_abs_error = 0.0_f64;
        let mut max_normalized_error = 0.0_f64;
        let mut passed = true;

        for sample in &self.samples {
            let actual_cd =
                DatcomComponentBuildup::protuberance_drag_coefficient(&geom, sample.mach)?;
            let mut sample_report = SampleAccumulator::new(sample.id.clone());
            compare_optional(
                Some(sample.expected_cd_protuberance),
                actual_cd,
                sample.abs_tolerance(self.tolerances.max_abs_error),
                sample.rel_tolerance(self.tolerances.max_relative_error),
                &mut sample_report,
            )?;
            let report = sample_report.finish()?;
            checked_value_count += report.checked_value_count;
            max_abs_error = max_abs_error.max(report.max_abs_error);
            max_normalized_error = max_normalized_error.max(report.max_normalized_error);
            passed &= report.passed;
            sample_reports.push(report);
        }

        Ok(DatcomProtuberanceDragValidationReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count,
            max_abs_error,
            max_normalized_error,
            passed,
            samples: sample_reports,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.datcom_protuberance_drag_validation
            != DATCOM_PROTUBERANCE_DRAG_SCHEMA_VERSION
        {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported DATCOM protuberance-drag schema version",
            });
        }
        validate_nonempty(
            &self.provenance.dataset_id,
            "DATCOM protuberance-drag dataset id is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_class,
            "DATCOM protuberance-drag source class is empty",
        )?;
        validate_nonempty(
            &self.provenance.validation_label,
            "DATCOM protuberance-drag validation label is empty",
        )?;
        validate_nonempty(
            &self.provenance.source_title,
            "DATCOM protuberance-drag source title is empty",
        )?;
        validate_tolerances(
            self.tolerances.max_abs_error,
            self.tolerances.max_relative_error,
        )?;
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM protuberance-drag fixture has no samples",
            });
        }
        let geom = self.geometry.body_geometry()?;
        if geom.protuberances.is_none() {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM protuberance-drag fixture requires protuberances",
            });
        }
        for sample in &self.samples {
            sample.validate(&self.tolerances)?;
        }
        Ok(())
    }
}

impl DatcomValidationGeometry {
    fn body_geometry(&self) -> Result<BodyGeometry, AeroError> {
        Ok(BodyGeometry {
            body_diameter_m: self.body_diameter_m,
            body_length_m: self.body_length_m,
            nose: self.nose.nose_shape()?,
            afterbody: self.afterbody.clone().map(Into::into),
            surface_roughness_m: self.surface_roughness_m,
            fins: self.fins.clone().map(Into::into),
            protuberances: self.protuberances.clone().map(Into::into),
            reference_area_m2: self.reference_area_m2,
            reference_length_m: self.reference_length_m,
            center_of_gravity_from_nose_m: self.center_of_gravity_from_nose_m,
        })
    }
}

impl DatcomValidationNose {
    fn nose_shape(&self) -> Result<NoseShape, AeroError> {
        match self.kind.as_str() {
            "conical" => {
                let half_angle_deg = self.half_angle_deg.ok_or(AeroError::MalformedDeck {
                    reason: "conical DATCOM validation nose requires half_angle_deg",
                })?;
                Ok(NoseShape::Conical {
                    half_angle_rad: deg_to_rad(half_angle_deg)?,
                })
            }
            "ogive" => {
                let fineness = self.fineness.ok_or(AeroError::MalformedDeck {
                    reason: "ogive DATCOM validation nose requires fineness",
                })?;
                Ok(NoseShape::Ogive { fineness })
            }
            "von_karman" => {
                let fineness = self.fineness.ok_or(AeroError::MalformedDeck {
                    reason: "von_karman DATCOM validation nose requires fineness",
                })?;
                Ok(NoseShape::VonKarman { fineness })
            }
            "hemispherical" => Ok(NoseShape::Hemispherical),
            _ => Err(AeroError::MalformedDeck {
                reason: "unsupported DATCOM validation nose kind",
            }),
        }
    }
}

impl From<DatcomValidationAfterbody> for Afterbody {
    fn from(value: DatcomValidationAfterbody) -> Self {
        Self {
            exit_diameter_m: value.exit_diameter_m,
            length_m: value.length_m,
        }
    }
}

impl From<DatcomValidationFinSet> for FinSet {
    fn from(value: DatcomValidationFinSet) -> Self {
        Self {
            count: value.count,
            root_chord_m: value.root_chord_m,
            tip_chord_m: value.tip_chord_m,
            span_m: value.span_m,
            thickness_ratio: value.thickness_ratio,
            sweep_rad: value.sweep_deg * PI / 180.0,
        }
    }
}

impl From<DatcomValidationProtuberanceSet> for ProtuberanceSet {
    fn from(value: DatcomValidationProtuberanceSet) -> Self {
        Self {
            count: value.count,
            frontal_area_m2: value.frontal_area_m2,
            drag_coefficient: value.drag_coefficient,
            interference_factor: value.interference_factor,
        }
    }
}

impl DatcomValidationSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(&self.id, "DATCOM validation sample id is empty")?;
        for value in [self.mach, self.alpha_deg, self.reynolds_length] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "DATCOM validation sample contains NaN or Inf",
                });
            }
        }
        if self.mach < 0.0 || self.reynolds_length <= 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM validation sample is outside the positive flow envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        if self.expected.is_empty() && self.expected_derivatives.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM validation sample has no expected values",
            });
        }
        self.expected.validate()?;
        self.expected_derivatives.validate()?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }

    fn expects_derivatives(&self) -> bool {
        !self.expected_derivatives.is_empty()
    }
}

impl NacaReport1048CrossflowSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(&self.id, "NACA 1048 crossflow sample id is empty")?;
        for value in [
            self.mach,
            self.alpha_deg,
            self.reynolds_length,
            self.expected_crossflow_cn,
            self.expected_cp_m,
        ] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "NACA 1048 crossflow sample contains NaN or Inf",
                });
            }
        }
        if self.mach < 0.0 || self.reynolds_length <= 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 crossflow sample is outside the positive flow envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl NacaReport1048CrossflowDragRatioSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(
            &self.id,
            "NACA 1048 crossflow drag-ratio sample id is empty",
        )?;
        validate_nonempty(
            &self.body_label,
            "NACA 1048 crossflow drag-ratio body label is empty",
        )?;
        for value in [
            self.fineness_ratio,
            self.cylinder_section_cd_low_mach,
            self.finite_length_drag_ratio,
            self.expected_cd_alpha_90_low_mach,
        ] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "NACA 1048 crossflow drag-ratio sample contains NaN or Inf",
                });
            }
        }
        if self.fineness_ratio <= 0.0
            || self.cylinder_section_cd_low_mach <= 0.0
            || self.finite_length_drag_ratio <= 0.0
            || self.finite_length_drag_ratio > 1.0
            || self.expected_cd_alpha_90_low_mach < 0.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 crossflow drag-ratio sample is outside the positive envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl NacaReport1048IncrementalForedragSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(
            &self.id,
            "NACA 1048 incremental-foredrag sample id is empty",
        )?;
        for value in [
            self.mach,
            self.alpha_deg,
            self.reynolds_length,
            self.expected_delta_cd_foredrag,
        ] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "NACA 1048 incremental-foredrag sample contains NaN or Inf",
                });
            }
        }
        if self.mach < 0.0 || self.reynolds_length <= 0.0 || self.expected_delta_cd_foredrag < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "NACA 1048 incremental-foredrag sample is outside the non-negative envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl SearsHaackWaveDragSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(&self.id, "Sears-Haack wave-drag sample id is empty")?;
        for value in [self.mach, self.expected_cd_wave] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "Sears-Haack wave-drag sample contains NaN or Inf",
                });
            }
        }
        if self.mach < 0.0 || self.expected_cd_wave < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "Sears-Haack wave-drag sample is outside the non-negative envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl DatcomFinBodyCarryoverSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(&self.id, "DATCOM fin/body carryover sample id is empty")?;
        for value in [
            self.body_diameter_m,
            self.fin_span_m,
            self.expected_fin_on_body,
            self.expected_body_on_fin,
        ] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "DATCOM fin/body carryover sample contains NaN or Inf",
                });
            }
        }
        if self.body_diameter_m <= 0.0
            || self.fin_span_m <= 0.0
            || self.expected_fin_on_body < 1.0
            || self.expected_body_on_fin < 1.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM fin/body carryover sample is outside the positive envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn body_geometry(&self) -> BodyGeometry {
        BodyGeometry {
            body_diameter_m: self.body_diameter_m,
            body_length_m: 6.0 * self.body_diameter_m,
            nose: NoseShape::Conical {
                half_angle_rad: 5.0_f64.to_radians(),
            },
            afterbody: None,
            surface_roughness_m: 0.0,
            fins: Some(self.fin_set()),
            protuberances: None,
            reference_area_m2: PI * self.body_diameter_m * self.body_diameter_m / 4.0,
            reference_length_m: self.body_diameter_m,
            center_of_gravity_from_nose_m: None,
        }
    }

    fn fin_set(&self) -> FinSet {
        FinSet {
            count: 4,
            root_chord_m: self.body_diameter_m,
            tip_chord_m: 0.5 * self.body_diameter_m,
            span_m: self.fin_span_m,
            thickness_ratio: 0.06,
            sweep_rad: 25.0_f64.to_radians(),
        }
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl DatcomProtuberanceDragSample {
    fn validate(&self, default_tolerances: &DatcomValidationTolerances) -> Result<(), AeroError> {
        validate_nonempty(&self.id, "DATCOM protuberance-drag sample id is empty")?;
        for value in [self.mach, self.expected_cd_protuberance] {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "DATCOM protuberance-drag sample contains NaN or Inf",
                });
            }
        }
        if self.mach < 0.0 || self.expected_cd_protuberance < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM protuberance-drag sample is outside the non-negative envelope",
            });
        }
        validate_tolerances(
            self.abs_tolerance(default_tolerances.max_abs_error),
            self.rel_tolerance(default_tolerances.max_relative_error),
        )?;
        Ok(())
    }

    fn abs_tolerance(&self, default: f64) -> f64 {
        self.max_abs_error.unwrap_or(default)
    }

    fn rel_tolerance(&self, default: f64) -> f64 {
        self.max_relative_error.unwrap_or(default)
    }
}

impl ExpectedCoefficients {
    fn is_empty(&self) -> bool {
        self.cn.is_none() && self.cd.is_none() && self.cm.is_none()
    }

    fn validate(&self) -> Result<(), AeroError> {
        for value in [self.cn, self.cd, self.cm].into_iter().flatten() {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "DATCOM validation expected coefficient is NaN or Inf",
                });
            }
        }
        Ok(())
    }
}

impl ExpectedDynamicDerivatives {
    fn is_empty(&self) -> bool {
        self.cl_p.is_none()
            && self.cl_q.is_none()
            && self.cl_r.is_none()
            && self.cm_p.is_none()
            && self.cm_q.is_none()
            && self.cm_r.is_none()
            && self.cn_p.is_none()
            && self.cn_q.is_none()
            && self.cn_r.is_none()
    }

    fn validate(&self) -> Result<(), AeroError> {
        for value in [
            self.cl_p, self.cl_q, self.cl_r, self.cm_p, self.cm_q, self.cm_r, self.cn_p, self.cn_q,
            self.cn_r,
        ]
        .into_iter()
        .flatten()
        {
            if !value.is_finite() {
                return Err(AeroError::NonFinite {
                    reason: "DATCOM validation expected derivative is NaN or Inf",
                });
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct SampleAccumulator {
    id: String,
    checked_value_count: usize,
    max_abs_error: f64,
    max_normalized_error: f64,
    passed: bool,
}

impl SampleAccumulator {
    fn new(id: String) -> Self {
        Self {
            id,
            checked_value_count: 0,
            max_abs_error: 0.0,
            max_normalized_error: 0.0,
            passed: true,
        }
    }

    fn record(&mut self, expected: f64, actual: f64, abs_tol: f64, rel_tol: f64) {
        let abs_error = (actual - expected).abs();
        let tolerance = abs_tol.max(rel_tol * expected.abs());
        let normalized = if tolerance > 0.0 {
            abs_error / tolerance
        } else if abs_error == 0.0 {
            0.0
        } else {
            f64::INFINITY
        };
        self.checked_value_count += 1;
        self.max_abs_error = self.max_abs_error.max(abs_error);
        self.max_normalized_error = self.max_normalized_error.max(normalized);
        self.passed &= normalized <= 1.0;
    }

    fn finish(self) -> Result<DatcomBuildupValidationSampleReport, AeroError> {
        if self.checked_value_count == 0 {
            return Err(AeroError::MalformedDeck {
                reason: "DATCOM validation sample checked no values",
            });
        }
        Ok(DatcomBuildupValidationSampleReport {
            id: self.id,
            checked_value_count: self.checked_value_count,
            max_abs_error: self.max_abs_error,
            max_normalized_error: self.max_normalized_error,
            passed: self.passed,
        })
    }
}

fn compare_optional(
    expected: Option<f64>,
    actual: f64,
    abs_tol: f64,
    rel_tol: f64,
    sample: &mut SampleAccumulator,
) -> Result<(), AeroError> {
    let Some(expected) = expected else {
        return Ok(());
    };
    if !actual.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "DATCOM validation actual coefficient is NaN or Inf",
        });
    }
    sample.record(expected, actual, abs_tol, rel_tol);
    Ok(())
}

fn compare_dynamic_derivatives(
    expected: &ExpectedDynamicDerivatives,
    actual: AeroDynamicDerivatives,
    abs_tol: f64,
    rel_tol: f64,
    sample: &mut SampleAccumulator,
) -> Result<(), AeroError> {
    compare_optional(expected.cl_p, actual.cl_p, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cl_q, actual.cl_q, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cl_r, actual.cl_r, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cm_p, actual.cm_p, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cm_q, actual.cm_q, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cm_r, actual.cm_r, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cn_p, actual.cn_p, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cn_q, actual.cn_q, abs_tol, rel_tol, sample)?;
    compare_optional(expected.cn_r, actual.cn_r, abs_tol, rel_tol, sample)?;
    Ok(())
}

fn validate_nonempty(value: &str, reason: &'static str) -> Result<(), AeroError> {
    if value.trim().is_empty() {
        return Err(AeroError::MalformedDeck { reason });
    }
    Ok(())
}

fn validate_tolerances(abs_tol: f64, rel_tol: f64) -> Result<(), AeroError> {
    if !(abs_tol.is_finite() && rel_tol.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "DATCOM validation tolerance is NaN or Inf",
        });
    }
    if abs_tol < DEFAULT_ABS_TOLERANCE || rel_tol < DEFAULT_REL_TOLERANCE {
        return Err(AeroError::InvalidParameter {
            reason: "DATCOM validation tolerances must be non-negative",
        });
    }
    if abs_tol == 0.0 && rel_tol == 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "DATCOM validation needs a positive tolerance",
        });
    }
    Ok(())
}

fn deg_to_rad(deg: f64) -> Result<f64, AeroError> {
    if !deg.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "DATCOM validation angle is NaN or Inf",
        });
    }
    Ok(deg * PI / 180.0)
}

fn sort_dedup_f64(values: &mut Vec<f64>) {
    values.sort_by(f64::total_cmp);
    values.dedup_by(|a, b| a.to_bits() == b.to_bits());
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/datcom-buildup-validation-v1.toml"
    ));
    const NACA_CROSSFLOW_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-crossflow-equation-v1.toml"
    ));
    const NACA_BODY_I_CROSSFLOW_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-i-crossflow-v1.toml"
    ));
    const NACA_BODY_II_CROSSFLOW_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-ii-crossflow-v1.toml"
    ));
    const NACA_BODY_I_FORCE_MOMENT_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-i-force-moment-v1.toml"
    ));
    const NACA_BODY_II_FORCE_MOMENT_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-ii-force-moment-v1.toml"
    ));
    const NACA_BODY_I_INCREMENTAL_FOREDRAG_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-i-incremental-foredrag-v1.toml"
    ));
    const NACA_BODY_II_INCREMENTAL_FOREDRAG_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-body-ii-incremental-foredrag-v1.toml"
    ));
    const NACA_CROSSFLOW_DRAG_RATIO_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/naca-report-1048-crossflow-drag-ratio-v1.toml"
    ));
    const SEARS_HAACK_WAVE_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/sears-haack-area-rule-wave-drag-v1.toml"
    ));
    const DATCOM_CARRYOVER_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/datcom-fin-body-carryover-v1.toml"
    ));
    const DATCOM_PROTUBERANCE_FIXTURE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/aero/datcom-protuberance-drag-v1.toml"
    ));

    #[test]
    fn datcom_validation_fixture_matches_buildup_model() {
        let fixture = DatcomBuildupValidationDeck::load_from_str(FIXTURE).unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 3);
        assert_eq!(report.checked_value_count, 36);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn datcom_validation_rejects_empty_expected_values() {
        let fixture = FIXTURE.replace(
            "[samples.expected]\ncn = 1.5210554487349282\ncd = 0.5153568570688235\ncm = -5.211568840465396\n\n[samples.expected_derivatives]\ncl_p = -1.49076166429326\ncl_q = 0.0\ncl_r = 0.0\ncm_p = 0.0\ncm_q = -430.61939222494664\ncm_r = 0.0\ncn_p = 0.0\ncn_q = 0.0\ncn_r = -430.61939222494664\n",
            "# expected values removed for malformed sample\n",
        );
        assert!(matches!(
            DatcomBuildupValidationDeck::load_from_str(&fixture),
            Err(AeroError::MalformedDeck { .. })
        ));
    }

    #[test]
    fn naca_report_1048_force_moment_fixtures_match_equation_15() {
        for input in [
            NACA_BODY_I_FORCE_MOMENT_FIXTURE,
            NACA_BODY_II_FORCE_MOMENT_FIXTURE,
        ] {
            let fixture = DatcomBuildupValidationDeck::load_from_str(input).unwrap();
            let report = fixture.evaluate().unwrap();
            assert!(report.passed, "{report:#?}");
            assert_eq!(report.sample_count, 4);
            assert_eq!(report.checked_value_count, 8);
            assert!(report.max_normalized_error <= 1.0);
        }
    }

    #[test]
    fn naca_report_1048_crossflow_fixture_matches_equation_shape() {
        let fixture =
            NacaReport1048CrossflowValidationDeck::load_from_str(NACA_CROSSFLOW_FIXTURE).unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 3);
        assert_eq!(report.checked_value_count, 6);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn naca_report_1048_body_crossflow_fixtures_match_report_geometry() {
        for input in [
            NACA_BODY_I_CROSSFLOW_FIXTURE,
            NACA_BODY_II_CROSSFLOW_FIXTURE,
        ] {
            let fixture = NacaReport1048CrossflowValidationDeck::load_from_str(input).unwrap();
            let report = fixture.evaluate().unwrap();
            assert!(report.passed, "{report:#?}");
            assert_eq!(report.sample_count, 4);
            assert_eq!(report.checked_value_count, 8);
            assert!(report.max_normalized_error <= 1.0);
        }
    }

    #[test]
    fn naca_report_1048_crossflow_rejects_non_public_source_class() {
        let fixture = NACA_CROSSFLOW_FIXTURE.replace(
            "source_class = \"public-domain-naca\"",
            "source_class = \"synthetic\"",
        );
        assert!(matches!(
            NacaReport1048CrossflowValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn naca_report_1048_crossflow_drag_ratio_fixture_matches_appendix_values() {
        let fixture = NacaReport1048CrossflowDragRatioValidationDeck::load_from_str(
            NACA_CROSSFLOW_DRAG_RATIO_FIXTURE,
        )
        .unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 2);
        assert_eq!(report.checked_value_count, 2);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn naca_report_1048_crossflow_drag_ratio_rejects_non_public_source_class() {
        let fixture = NACA_CROSSFLOW_DRAG_RATIO_FIXTURE.replace(
            "source_class = \"public-domain-naca\"",
            "source_class = \"synthetic\"",
        );
        assert!(matches!(
            NacaReport1048CrossflowDragRatioValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn naca_report_1048_incremental_foredrag_fixtures_match_equation_15() {
        for input in [
            NACA_BODY_I_INCREMENTAL_FOREDRAG_FIXTURE,
            NACA_BODY_II_INCREMENTAL_FOREDRAG_FIXTURE,
        ] {
            let fixture =
                NacaReport1048IncrementalForedragValidationDeck::load_from_str(input).unwrap();
            let report = fixture.evaluate().unwrap();
            assert!(report.passed, "{report:#?}");
            assert_eq!(report.sample_count, 4);
            assert_eq!(report.checked_value_count, 4);
            assert!(report.max_normalized_error <= 1.0);
        }
    }

    #[test]
    fn naca_report_1048_incremental_foredrag_rejects_non_public_source_class() {
        let fixture = NACA_BODY_I_INCREMENTAL_FOREDRAG_FIXTURE.replace(
            "source_class = \"public-domain-naca\"",
            "source_class = \"synthetic\"",
        );
        assert!(matches!(
            NacaReport1048IncrementalForedragValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn sears_haack_wave_drag_fixture_matches_area_rule_branch() {
        let fixture =
            SearsHaackWaveDragValidationDeck::load_from_str(SEARS_HAACK_WAVE_FIXTURE).unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 5);
        assert_eq!(report.checked_value_count, 5);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn sears_haack_wave_drag_rejects_negative_expected_drag() {
        let fixture = SEARS_HAACK_WAVE_FIXTURE.replace(
            "expected_cd_wave = 0.004811252243246881",
            "expected_cd_wave = -0.004811252243246881",
        );
        assert!(matches!(
            SearsHaackWaveDragValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn datcom_fin_body_carryover_fixture_matches_factor_branch() {
        let fixture =
            DatcomFinBodyCarryoverValidationDeck::load_from_str(DATCOM_CARRYOVER_FIXTURE).unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 3);
        assert_eq!(report.checked_value_count, 6);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn datcom_fin_body_carryover_rejects_negative_span() {
        let fixture = DATCOM_CARRYOVER_FIXTURE.replace("fin_span_m = 0.5", "fin_span_m = -0.5");
        assert!(matches!(
            DatcomFinBodyCarryoverValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn datcom_protuberance_drag_fixture_matches_mach_branch() {
        let fixture =
            DatcomProtuberanceDragValidationDeck::load_from_str(DATCOM_PROTUBERANCE_FIXTURE)
                .unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(report.sample_count, 4);
        assert_eq!(report.checked_value_count, 4);
        assert!(report.max_normalized_error <= 1.0);
    }

    #[test]
    fn datcom_protuberance_drag_rejects_negative_expected_drag() {
        let fixture = DATCOM_PROTUBERANCE_FIXTURE.replace(
            "expected_cd_protuberance = 0.047057999999999996",
            "expected_cd_protuberance = -0.047057999999999996",
        );
        assert!(matches!(
            DatcomProtuberanceDragValidationDeck::load_from_str(&fixture),
            Err(AeroError::InvalidParameter { .. })
        ));
    }
}
