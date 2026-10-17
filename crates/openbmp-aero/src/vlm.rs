//! First attached-flow fin lifting-surface substrate for WP-03.5.
//!
//! This module is intentionally small: it provides deterministic finite-wing
//! strip/lifting-line, grid-fin authority, and horseshoe-vortex substrates that
//! expose the coefficient channels a full validated VLM module will own later.
//! It is valid only for attached subsonic flow below the transonic rise and
//! does not claim AVL code-to-code closure yet.

use core::f64::consts::PI;

#[cfg(not(feature = "std"))]
use alloc::{vec, vec::Vec};
#[cfg(feature = "parser")]
use std::string::String;
#[cfg(feature = "std")]
use std::vec::Vec;

use nalgebra::Vector3;
#[cfg(not(feature = "std"))]
use num_traits::Float;
#[cfg(feature = "parser")]
use serde::Deserialize;

use crate::error::AeroError;

const MIN_HINGE_FRACTION: f64 = 0.05;
const MAX_HINGE_FRACTION: f64 = 0.95;
const MAX_CHORDWISE_PANELS: u32 = 32;
const MAX_SPANWISE_PANELS: u32 = 512;
const MAX_GRID_FIN_CELLS_PER_AXIS: u32 = 128;
const MAX_ATTACHED_FLOW_MACH: f64 = 0.8;
const MAX_GRID_FIN_ATTACHED_ANGLE_RAD: f64 = 0.5;
const DEFAULT_OSWALD_EFFICIENCY: f64 = 0.85;
const BIOT_SAVART_EPS: f64 = 1.0e-14;
const SOLVE_PIVOT_EPS: f64 = 1.0e-12;
#[cfg(feature = "parser")]
const VLM_HORSESHOE_DERIVATIVE_COMPARISON_SCHEMA_VERSION: u32 = 1;

/// Trapezoidal fin geometry for the first WP-03.5 lifting-surface substrate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmFinGeometry {
    /// Root chord length (m).
    pub root_chord_m: f64,
    /// Tip chord length (m).
    pub tip_chord_m: f64,
    /// Exposed semispan/height of one fin (m).
    pub span_m: f64,
    /// Leading-edge sweep angle (rad).
    pub sweep_rad: f64,
    /// Body-x coordinate of the root leading edge (m).
    pub root_le_x_m: f64,
    /// Body-x moment reference (m).
    pub moment_reference_x_m: f64,
    /// Reference area used for reported coefficients (m^2).
    pub reference_area_m2: f64,
    /// Reference length used for reported moments (m).
    pub reference_length_m: f64,
    /// Hinge-line chord fraction measured from local leading edge.
    pub hinge_chord_fraction: f64,
}

/// Flow condition for a fin lifting-surface evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmFinFlow {
    /// Freestream Mach number. The first substrate is valid for `M < 0.8`.
    pub mach: f64,
    /// Angle of attack of the fin section (rad).
    pub alpha_rad: f64,
    /// Control-surface deflection behind the hinge line (rad).
    pub control_deflection_rad: f64,
}

/// Coefficients from a deterministic attached-flow fin evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmFinSolution {
    /// Lift coefficient on the caller-supplied reference area.
    pub cl: f64,
    /// Induced-drag coefficient on the caller-supplied reference area.
    pub cd_induced: f64,
    /// Pitching-moment coefficient about the caller-supplied x-reference.
    pub cm: f64,
    /// Hinge-moment coefficient on `q * S_ref * L_ref`.
    pub hinge_moment_coefficient: f64,
    /// Lift-curve slope on the caller-supplied reference area (1/rad).
    pub lift_curve_slope_per_rad: f64,
    /// Strip-quadrature aerodynamic-center body-x location (m).
    pub aerodynamic_center_x_m: f64,
}

/// Geometry for a deterministic first grid-fin authority substrate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmGridFinGeometry {
    /// Projected grid-fin planform area (m^2).
    pub projected_area_m2: f64,
    /// Reference area used for reported coefficients (m^2).
    pub reference_area_m2: f64,
    /// Reference length used for reported moments (m).
    pub reference_length_m: f64,
    /// Body-x coordinate of the grid-fin aerodynamic center (m).
    pub aerodynamic_center_x_m: f64,
    /// Body-x moment reference (m).
    pub moment_reference_x_m: f64,
    /// Positive hinge-to-pressure-center arm used for hinge moment (m).
    pub hinge_moment_arm_m: f64,
    /// Number of cells along the local chord/open-flow direction.
    pub chordwise_cells: u32,
    /// Number of cells along the local span direction.
    pub spanwise_cells: u32,
    /// Open projected-area fraction in `(0, 1)`.
    pub open_area_ratio: f64,
    /// Web thickness divided by cell pitch.
    pub web_thickness_to_cell_pitch: f64,
}

/// Flow condition for a grid-fin authority evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmGridFinFlow {
    /// Freestream Mach number. The first substrate is valid for `M < 0.8`.
    pub mach: f64,
    /// Local grid-fin incidence/angle of attack (rad).
    pub alpha_rad: f64,
    /// Grid-fin control deflection (rad).
    pub control_deflection_rad: f64,
}

/// Coefficients from a deterministic attached-flow grid-fin evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmGridFinSolution {
    /// Normal-force coefficient on the caller-supplied reference area.
    pub normal_force_coefficient: f64,
    /// Axial drag coefficient on the caller-supplied reference area.
    pub axial_drag_coefficient: f64,
    /// Pitching-moment coefficient about the caller-supplied x-reference.
    pub pitching_moment_coefficient: f64,
    /// Hinge-moment coefficient on `q * S_ref * L_ref`.
    pub hinge_moment_coefficient: f64,
    /// Normal-force slope on the caller-supplied reference area (1/rad).
    pub normal_force_slope_per_rad: f64,
    /// Effective authority factor after open-area, web, and cell-count effects.
    pub effective_open_area_ratio: f64,
    /// Total declared grid-fin cell count.
    pub cell_count: u32,
}

/// One horseshoe-vortex panel in the deterministic WP-03.5 lattice substrate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmHorseshoePanel {
    /// Bound-vortex endpoint at the lower/span-start side.
    pub bound_start_m: Vector3<f64>,
    /// Bound-vortex endpoint at the upper/span-end side.
    pub bound_end_m: Vector3<f64>,
    /// Collocation/control point.
    pub control_point_m: Vector3<f64>,
    /// Unit panel normal used for flow-tangency.
    pub normal: Vector3<f64>,
    /// Panel area (m^2).
    pub area_m2: f64,
    /// Local chord at the panel midpoint (m).
    pub chord_m: f64,
    /// Chordwise control-point fraction from the local leading edge.
    pub control_chord_fraction: f64,
    /// Body-x coordinate of the local hinge line (m).
    pub hinge_x_m: f64,
}

/// Deterministic horseshoe-vortex lattice over one trapezoidal fin.
#[derive(Clone, Debug, PartialEq)]
pub struct VlmHorseshoeLattice {
    geometry: VlmFinGeometry,
    chordwise_panels: u32,
    spanwise_panels: u32,
    wake_length_m: f64,
    panels: Vec<VlmHorseshoePanel>,
}

/// Result of solving a deterministic horseshoe-vortex lattice.
#[derive(Clone, Debug, PartialEq)]
pub struct VlmHorseshoeSolution {
    /// Circulation strengths in panel order for unit freestream speed.
    pub circulation: Vec<f64>,
    /// Lift coefficient on the caller-supplied reference area.
    pub cl: f64,
    /// Induced-drag coefficient estimate on the caller-supplied reference area.
    pub cd_induced: f64,
    /// Pitching-moment coefficient about the caller-supplied x-reference.
    pub cm: f64,
    /// Hinge-moment coefficient on `q * S_ref * L_ref`.
    pub hinge_moment_coefficient: f64,
    /// Maximum absolute flow-tangency residual after the solve.
    pub max_tangency_residual: f64,
    /// Number of panels in the solved lattice.
    pub panel_count: usize,
}

/// Deterministic finite-difference derivatives from a horseshoe-vortex lattice.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmHorseshoeDerivatives {
    /// Lift-curve slope `dCL / d alpha` (1/rad).
    pub lift_curve_slope_per_rad: f64,
    /// Pitching-moment slope `dCm / d alpha` (1/rad).
    pub pitching_moment_slope_per_rad: f64,
    /// Hinge-moment slope `dCh / d alpha` (1/rad).
    pub hinge_moment_slope_per_rad: f64,
    /// Control lift effectiveness `dCL / d delta` (1/rad).
    pub control_lift_effectiveness_per_rad: f64,
    /// Control hinge effectiveness `dCh / d delta` (1/rad).
    pub control_hinge_effectiveness_per_rad: f64,
    /// Central-difference angle-of-attack step (rad).
    pub alpha_step_rad: f64,
    /// Central-difference control-deflection step (rad).
    pub control_step_rad: f64,
}

/// Parser-gated VLM horseshoe derivative comparison fixture.
#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VlmHorseshoeDerivativeComparisonDeck {
    openbmp: VlmHorseshoeDerivativeComparisonSchemaMarker,
    provenance: VlmHorseshoeDerivativeComparisonProvenance,
    geometry: VlmHorseshoeDerivativeComparisonGeometry,
    lattice: VlmHorseshoeDerivativeComparisonLattice,
    tolerances: VlmHorseshoeDerivativeComparisonTolerances,
    samples: Vec<VlmHorseshoeDerivativeComparisonSample>,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonSchemaMarker {
    vlm_horseshoe_derivative_comparison: u32,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonProvenance {
    dataset_id: String,
    source_class: String,
    validation_label: String,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonGeometry {
    root_chord_m: f64,
    tip_chord_m: f64,
    span_m: f64,
    sweep_deg: f64,
    root_le_x_m: f64,
    moment_reference_x_m: f64,
    reference_area_m2: f64,
    reference_length_m: f64,
    hinge_chord_fraction: f64,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonLattice {
    chordwise_panels: u32,
    spanwise_panels: u32,
    wake_length_m: f64,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonTolerances {
    abs_tolerance: f64,
    rel_tolerance: f64,
}

#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct VlmHorseshoeDerivativeComparisonSample {
    id: String,
    mach: f64,
    alpha_deg: f64,
    control_deflection_deg: f64,
    alpha_step_deg: f64,
    control_step_deg: f64,
    expected_lift_curve_slope_per_rad: f64,
    expected_pitching_moment_slope_per_rad: f64,
    expected_hinge_moment_slope_per_rad: f64,
    expected_control_lift_effectiveness_per_rad: f64,
    expected_control_hinge_effectiveness_per_rad: f64,
}

/// Report emitted from a VLM horseshoe derivative comparison fixture.
#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq)]
pub struct VlmHorseshoeDerivativeComparisonReport {
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
    pub samples: Vec<VlmHorseshoeDerivativeComparisonSampleReport>,
}

/// Per-sample VLM horseshoe derivative comparison report.
#[cfg(feature = "parser")]
#[derive(Clone, Debug, PartialEq)]
pub struct VlmHorseshoeDerivativeComparisonSampleReport {
    /// Sample id from the fixture.
    pub id: String,
    /// Number of scalar values checked for this sample.
    pub checked_value_count: usize,
    /// Maximum absolute error observed for this sample.
    pub max_abs_error: f64,
    /// Maximum tolerance-normalized error observed for this sample.
    pub max_normalized_error: f64,
    /// Whether every checked value stayed inside tolerance.
    pub passed: bool,
    /// Actual derivative values emitted by the lattice.
    pub actual: VlmHorseshoeDerivatives,
}

struct VlmLinearSystem {
    matrix: Vec<f64>,
    rhs: Vec<f64>,
    dimension: usize,
}

/// Deterministic first lifting-surface provider for one fin.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmFinSurface {
    geometry: VlmFinGeometry,
    spanwise_panels: u32,
}

/// Deterministic first grid-fin authority provider.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VlmGridFinSurface {
    geometry: VlmGridFinGeometry,
}

impl VlmFinSurface {
    /// Construct a fin lifting-surface model with a fixed spanwise strip count.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry is non-finite or outside the
    /// attached-flow model envelope.
    pub fn new(geometry: VlmFinGeometry, spanwise_panels: u32) -> Result<Self, AeroError> {
        validate_geometry(geometry, spanwise_panels)?;
        Ok(Self {
            geometry,
            spanwise_panels,
        })
    }

    /// Return the checked fin geometry.
    #[must_use]
    pub const fn geometry(&self) -> VlmFinGeometry {
        self.geometry
    }

    /// Return the fixed spanwise strip count.
    #[must_use]
    pub const fn spanwise_panels(&self) -> u32 {
        self.spanwise_panels
    }

    /// Evaluate the fin at one attached-flow condition.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] for non-finite flow inputs, negative Mach, or Mach
    /// at/above the first-substrate transonic ceiling.
    pub fn evaluate(&self, flow: &VlmFinFlow) -> Result<VlmFinSolution, AeroError> {
        validate_flow(flow)?;
        let geometry = self.geometry;
        let area_m2 = fin_area_m2(geometry);
        let aspect_ratio = geometry.span_m * geometry.span_m / area_m2;
        let beta = subsonic_compressibility_beta(flow.mach)?;
        let sweep_factor = geometry.sweep_rad.cos().abs().max(0.2);
        let effective_ar = aspect_ratio * sweep_factor;
        let surface_lift_slope = finite_wing_lift_curve_slope_per_rad(effective_ar, beta);
        let area_ratio = area_m2 / geometry.reference_area_m2;
        let lift_curve_slope_per_rad = surface_lift_slope * area_ratio;
        let alpha_effective =
            flow.alpha_rad + control_effectiveness(geometry) * flow.control_deflection_rad;
        let surface_cl = surface_lift_slope * alpha_effective;
        let cl = surface_cl * area_ratio;
        let cd_induced =
            surface_cl * surface_cl / (PI * aspect_ratio * DEFAULT_OSWALD_EFFICIENCY) * area_ratio;
        let aerodynamic_center_x_m = aerodynamic_center_x_m(geometry, self.spanwise_panels);
        let cm = -cl * (aerodynamic_center_x_m - geometry.moment_reference_x_m)
            / geometry.reference_length_m;
        let aft_fraction = 1.0 - geometry.hinge_chord_fraction;
        let mean_chord_m = area_m2 / geometry.span_m;
        let hinge_arm_over_reference =
            aft_fraction * (0.5 * aft_fraction * mean_chord_m / geometry.reference_length_m);
        let hinge_moment_coefficient = -cl * hinge_arm_over_reference;
        if ![
            cl,
            cd_induced,
            cm,
            hinge_moment_coefficient,
            lift_curve_slope_per_rad,
            aerodynamic_center_x_m,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(AeroError::NonFinite {
                reason: "VLM fin solution is NaN or Inf",
            });
        }
        Ok(VlmFinSolution {
            cl,
            cd_induced,
            cm,
            hinge_moment_coefficient,
            lift_curve_slope_per_rad,
            aerodynamic_center_x_m,
        })
    }
}

impl VlmGridFinSurface {
    /// Construct a grid-fin authority model.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry is non-finite or outside the
    /// attached-flow grid-fin substrate envelope.
    pub fn new(geometry: VlmGridFinGeometry) -> Result<Self, AeroError> {
        validate_grid_fin_geometry(geometry)?;
        Ok(Self { geometry })
    }

    /// Return the checked grid-fin geometry.
    #[must_use]
    pub const fn geometry(&self) -> VlmGridFinGeometry {
        self.geometry
    }

    /// Evaluate the grid fin at one attached-flow condition.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] for non-finite flow inputs, negative Mach, Mach
    /// at/above the first-substrate transonic ceiling, or an incidence outside
    /// the attached-flow grid-fin angle envelope.
    pub fn evaluate(&self, flow: &VlmGridFinFlow) -> Result<VlmGridFinSolution, AeroError> {
        validate_grid_fin_flow(flow)?;
        let geometry = self.geometry;
        let effective_angle_rad = flow.alpha_rad + flow.control_deflection_rad;
        if effective_angle_rad.abs() > MAX_GRID_FIN_ATTACHED_ANGLE_RAD {
            return Err(AeroError::OutOfEnvelope {
                reason: "VLM grid fin first substrate is attached-flow only",
            });
        }
        let beta = subsonic_compressibility_beta(flow.mach)?;
        let cell_count = geometry
            .chordwise_cells
            .checked_mul(geometry.spanwise_cells)
            .ok_or(AeroError::InvalidParameter {
                reason: "VLM grid fin cell count overflows",
            })?;
        let area_ratio = geometry.projected_area_m2 / geometry.reference_area_m2;
        let cell_count_f64 = f64::from(cell_count);
        let cell_authority_factor = cell_count_f64 / (cell_count_f64 + 4.0);
        let web_authority_factor = (1.0 - 0.65 * geometry.web_thickness_to_cell_pitch).max(0.25);
        let effective_open_area_ratio =
            geometry.open_area_ratio * cell_authority_factor * web_authority_factor;
        let normal_force_slope_per_rad = 2.0 * PI * area_ratio * effective_open_area_ratio / beta;
        let normal_force_coefficient = normal_force_slope_per_rad * effective_angle_rad;
        let blockage = 1.0 - geometry.open_area_ratio;
        let lattice_aspect =
            (f64::from(geometry.spanwise_cells) / f64::from(geometry.chordwise_cells)).max(0.25);
        let induced_drag = normal_force_coefficient * normal_force_coefficient
            / (PI * (0.5 + lattice_aspect) * DEFAULT_OSWALD_EFFICIENCY);
        let blockage_drag = area_ratio
            * (0.015
                + 0.75 * blockage * blockage
                + 0.35
                    * geometry.web_thickness_to_cell_pitch
                    * geometry.web_thickness_to_cell_pitch
                + 0.08 * effective_angle_rad * effective_angle_rad);
        let axial_drag_coefficient = blockage_drag + induced_drag;
        let pitching_moment_coefficient = -normal_force_coefficient
            * (geometry.aerodynamic_center_x_m - geometry.moment_reference_x_m)
            / geometry.reference_length_m;
        let hinge_moment_coefficient =
            -normal_force_coefficient * geometry.hinge_moment_arm_m / geometry.reference_length_m;
        if ![
            normal_force_coefficient,
            axial_drag_coefficient,
            pitching_moment_coefficient,
            hinge_moment_coefficient,
            normal_force_slope_per_rad,
            effective_open_area_ratio,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(AeroError::NonFinite {
                reason: "VLM grid fin solution is NaN or Inf",
            });
        }
        Ok(VlmGridFinSolution {
            normal_force_coefficient,
            axial_drag_coefficient,
            pitching_moment_coefficient,
            hinge_moment_coefficient,
            normal_force_slope_per_rad,
            effective_open_area_ratio,
            cell_count,
        })
    }
}

#[cfg(feature = "parser")]
impl VlmHorseshoeDerivativeComparisonDeck {
    /// Parse a VLM horseshoe derivative comparison fixture from TOML.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] when the TOML is malformed,
    /// the schema marker is unsupported, required sample rows are absent,
    /// or tolerances are invalid.
    pub fn from_toml_str(input: &str) -> Result<Self, AeroError> {
        let deck: Self = toml::from_str(input).map_err(|_| AeroError::MalformedDeck {
            reason: "VLM horseshoe derivative comparison TOML is malformed",
        })?;
        deck.validate()?;
        Ok(deck)
    }

    /// Evaluate the comparison fixture through the public horseshoe lattice API.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if fixture validation, lattice construction,
    /// derivative evaluation, or tolerance checking fails.
    pub fn evaluate(&self) -> Result<VlmHorseshoeDerivativeComparisonReport, AeroError> {
        self.validate()?;
        let geometry = VlmFinGeometry {
            root_chord_m: self.geometry.root_chord_m,
            tip_chord_m: self.geometry.tip_chord_m,
            span_m: self.geometry.span_m,
            sweep_rad: self.geometry.sweep_deg.to_radians(),
            root_le_x_m: self.geometry.root_le_x_m,
            moment_reference_x_m: self.geometry.moment_reference_x_m,
            reference_area_m2: self.geometry.reference_area_m2,
            reference_length_m: self.geometry.reference_length_m,
            hinge_chord_fraction: self.geometry.hinge_chord_fraction,
        };
        let lattice = VlmHorseshoeLattice::new(
            geometry,
            self.lattice.chordwise_panels,
            self.lattice.spanwise_panels,
            self.lattice.wake_length_m,
        )?;

        let mut report = VlmHorseshoeDerivativeComparisonReport {
            dataset_id: self.provenance.dataset_id.clone(),
            sample_count: self.samples.len(),
            checked_value_count: 0,
            max_abs_error: 0.0,
            max_normalized_error: 0.0,
            passed: true,
            samples: Vec::with_capacity(self.samples.len()),
        };

        for sample in &self.samples {
            let actual = lattice.finite_difference_derivatives(
                &VlmFinFlow {
                    mach: sample.mach,
                    alpha_rad: sample.alpha_deg.to_radians(),
                    control_deflection_rad: sample.control_deflection_deg.to_radians(),
                },
                sample.alpha_step_deg.to_radians(),
                sample.control_step_deg.to_radians(),
            )?;
            let mut sample_report = VlmHorseshoeDerivativeComparisonSampleReport {
                id: sample.id.clone(),
                checked_value_count: 0,
                max_abs_error: 0.0,
                max_normalized_error: 0.0,
                passed: true,
                actual,
            };
            self.check_value(
                actual.lift_curve_slope_per_rad,
                sample.expected_lift_curve_slope_per_rad,
                &mut sample_report,
            )?;
            self.check_value(
                actual.pitching_moment_slope_per_rad,
                sample.expected_pitching_moment_slope_per_rad,
                &mut sample_report,
            )?;
            self.check_value(
                actual.hinge_moment_slope_per_rad,
                sample.expected_hinge_moment_slope_per_rad,
                &mut sample_report,
            )?;
            self.check_value(
                actual.control_lift_effectiveness_per_rad,
                sample.expected_control_lift_effectiveness_per_rad,
                &mut sample_report,
            )?;
            self.check_value(
                actual.control_hinge_effectiveness_per_rad,
                sample.expected_control_hinge_effectiveness_per_rad,
                &mut sample_report,
            )?;
            report.checked_value_count += sample_report.checked_value_count;
            report.max_abs_error = report.max_abs_error.max(sample_report.max_abs_error);
            report.max_normalized_error = report
                .max_normalized_error
                .max(sample_report.max_normalized_error);
            report.passed &= sample_report.passed;
            report.samples.push(sample_report);
        }
        Ok(report)
    }

    fn validate(&self) -> Result<(), AeroError> {
        if self.openbmp.vlm_horseshoe_derivative_comparison
            != VLM_HORSESHOE_DERIVATIVE_COMPARISON_SCHEMA_VERSION
        {
            return Err(AeroError::MalformedDeck {
                reason: "unsupported VLM horseshoe derivative comparison schema version",
            });
        }
        validate_nonempty_str(
            &self.provenance.dataset_id,
            "VLM horseshoe derivative comparison dataset id is empty",
        )?;
        validate_nonempty_str(
            &self.provenance.source_class,
            "VLM horseshoe derivative comparison source class is empty",
        )?;
        validate_nonempty_str(
            &self.provenance.validation_label,
            "VLM horseshoe derivative comparison validation label is empty",
        )?;
        validate_finite_values(
            &[
                self.geometry.root_chord_m,
                self.geometry.tip_chord_m,
                self.geometry.span_m,
                self.geometry.sweep_deg,
                self.geometry.root_le_x_m,
                self.geometry.moment_reference_x_m,
                self.geometry.reference_area_m2,
                self.geometry.reference_length_m,
                self.geometry.hinge_chord_fraction,
                self.lattice.wake_length_m,
                self.tolerances.abs_tolerance,
                self.tolerances.rel_tolerance,
            ],
            "VLM horseshoe derivative comparison contains NaN or Inf",
        )?;
        if self.tolerances.abs_tolerance < 0.0 || self.tolerances.rel_tolerance < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe derivative comparison tolerances must be non-negative",
            });
        }
        if self.tolerances.abs_tolerance == 0.0 && self.tolerances.rel_tolerance == 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe derivative comparison needs a positive tolerance",
            });
        }
        if self.samples.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "VLM horseshoe derivative comparison has no samples",
            });
        }
        for sample in &self.samples {
            sample.validate()?;
        }
        Ok(())
    }

    fn check_value(
        &self,
        actual: f64,
        expected: f64,
        sample_report: &mut VlmHorseshoeDerivativeComparisonSampleReport,
    ) -> Result<(), AeroError> {
        validate_finite_values(
            &[actual, expected],
            "VLM horseshoe derivative comparison value is NaN or Inf",
        )?;
        let abs_error = (actual - expected).abs();
        let tolerance =
            self.tolerances.abs_tolerance + self.tolerances.rel_tolerance * expected.abs();
        if tolerance <= 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe derivative comparison scalar tolerance must be positive",
            });
        }
        let normalized_error = abs_error / tolerance;
        sample_report.checked_value_count += 1;
        sample_report.max_abs_error = sample_report.max_abs_error.max(abs_error);
        sample_report.max_normalized_error =
            sample_report.max_normalized_error.max(normalized_error);
        sample_report.passed &= abs_error <= tolerance;
        Ok(())
    }
}

#[cfg(feature = "parser")]
impl VlmHorseshoeDerivativeComparisonSample {
    fn validate(&self) -> Result<(), AeroError> {
        validate_nonempty_str(
            &self.id,
            "VLM horseshoe derivative comparison sample id is empty",
        )?;
        validate_finite_values(
            &[
                self.mach,
                self.alpha_deg,
                self.control_deflection_deg,
                self.alpha_step_deg,
                self.control_step_deg,
                self.expected_lift_curve_slope_per_rad,
                self.expected_pitching_moment_slope_per_rad,
                self.expected_hinge_moment_slope_per_rad,
                self.expected_control_lift_effectiveness_per_rad,
                self.expected_control_hinge_effectiveness_per_rad,
            ],
            "VLM horseshoe derivative comparison sample contains NaN or Inf",
        )
    }
}

impl VlmHorseshoeLattice {
    /// Build a deterministic horseshoe-vortex lattice over one trapezoidal fin.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] for invalid fin geometry, invalid panel counts, or
    /// an invalid finite wake length.
    pub fn new(
        geometry: VlmFinGeometry,
        chordwise_panels: u32,
        spanwise_panels: u32,
        wake_length_m: f64,
    ) -> Result<Self, AeroError> {
        validate_geometry(geometry, spanwise_panels)?;
        if chordwise_panels == 0 || chordwise_panels > MAX_CHORDWISE_PANELS {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe chordwise panel count is outside envelope",
            });
        }
        if !wake_length_m.is_finite() || wake_length_m <= 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe wake length is outside envelope",
            });
        }
        let panel_count =
            chordwise_panels
                .checked_mul(spanwise_panels)
                .ok_or(AeroError::InvalidParameter {
                    reason: "VLM horseshoe panel count overflows",
                })?;
        let mut panels = Vec::with_capacity(panel_count as usize);
        for chord_idx in 0..chordwise_panels {
            let chord_start = f64::from(chord_idx) / f64::from(chordwise_panels);
            let chord_end = f64::from(chord_idx + 1) / f64::from(chordwise_panels);
            let bound_fraction = chord_start + 0.25 * (chord_end - chord_start);
            let control_fraction = chord_start + 0.75 * (chord_end - chord_start);
            for span_idx in 0..spanwise_panels {
                let span_start = f64::from(span_idx) / f64::from(spanwise_panels);
                let span_end = f64::from(span_idx + 1) / f64::from(spanwise_panels);
                let span_mid = 0.5 * (span_start + span_end);
                let y_start = geometry.span_m * span_start;
                let y_end = geometry.span_m * span_end;
                let y_mid = geometry.span_m * span_mid;
                let chord_start_m = local_chord_m(geometry, span_start);
                let chord_end_m = local_chord_m(geometry, span_end);
                let chord_mid_m = local_chord_m(geometry, span_mid);
                let x_le_start_m = local_le_x_m(geometry, y_start);
                let x_le_end_m = local_le_x_m(geometry, y_end);
                let x_le_mid_m = local_le_x_m(geometry, y_mid);
                let panel_span_m = y_end - y_start;
                let panel_area_m2 = 0.5 * (chord_start_m + chord_end_m) * panel_span_m
                    / f64::from(chordwise_panels);
                panels.push(VlmHorseshoePanel {
                    bound_start_m: Vector3::new(
                        x_le_start_m + bound_fraction * chord_start_m,
                        y_start,
                        0.0,
                    ),
                    bound_end_m: Vector3::new(
                        x_le_end_m + bound_fraction * chord_end_m,
                        y_end,
                        0.0,
                    ),
                    control_point_m: Vector3::new(
                        x_le_mid_m + control_fraction * chord_mid_m,
                        y_mid,
                        0.0,
                    ),
                    normal: Vector3::new(0.0, 0.0, 1.0),
                    area_m2: panel_area_m2,
                    chord_m: chord_mid_m,
                    control_chord_fraction: control_fraction,
                    hinge_x_m: x_le_mid_m + geometry.hinge_chord_fraction * chord_mid_m,
                });
            }
        }
        Ok(Self {
            geometry,
            chordwise_panels,
            spanwise_panels,
            wake_length_m,
            panels,
        })
    }

    /// Return the panels in locked assembly order.
    #[must_use]
    pub fn panels(&self) -> &[VlmHorseshoePanel] {
        &self.panels
    }

    /// Return the chordwise panel count.
    #[must_use]
    pub const fn chordwise_panels(&self) -> u32 {
        self.chordwise_panels
    }

    /// Return the spanwise panel count.
    #[must_use]
    pub const fn spanwise_panels(&self) -> u32 {
        self.spanwise_panels
    }

    /// Solve the horseshoe lattice for one attached-flow condition.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when the flow is invalid or the dense influence
    /// matrix is singular under the deterministic pivot rule.
    pub fn solve(&self, flow: &VlmFinFlow) -> Result<VlmHorseshoeSolution, AeroError> {
        let system = self.assemble_linear_system(flow)?;
        let circulation =
            solve_dense_deterministic(system.matrix.clone(), system.rhs.clone(), system.dimension)?;
        let residual =
            max_linear_residual(&system.matrix, &circulation, &system.rhs, system.dimension);
        let compressibility_scale = 1.0 / subsonic_compressibility_beta(flow.mach)?;
        let solution =
            self.coefficients_from_circulation(&circulation, residual, compressibility_scale)?;
        Ok(solution)
    }

    /// Evaluate deterministic central-difference lift, moment, hinge, and
    /// control-effectiveness derivatives for validation comparisons.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] for invalid flow inputs, invalid positive
    /// finite-difference steps, or any failed perturbed solve.
    pub fn finite_difference_derivatives(
        &self,
        flow: &VlmFinFlow,
        alpha_step_rad: f64,
        control_step_rad: f64,
    ) -> Result<VlmHorseshoeDerivatives, AeroError> {
        validate_flow(flow)?;
        validate_finite_difference_step(alpha_step_rad, "VLM horseshoe alpha derivative step")?;
        validate_finite_difference_step(control_step_rad, "VLM horseshoe control derivative step")?;

        let alpha_plus = self.solve(&VlmFinFlow {
            alpha_rad: flow.alpha_rad + alpha_step_rad,
            ..*flow
        })?;
        let alpha_minus = self.solve(&VlmFinFlow {
            alpha_rad: flow.alpha_rad - alpha_step_rad,
            ..*flow
        })?;
        let control_plus = self.solve(&VlmFinFlow {
            control_deflection_rad: flow.control_deflection_rad + control_step_rad,
            ..*flow
        })?;
        let control_minus = self.solve(&VlmFinFlow {
            control_deflection_rad: flow.control_deflection_rad - control_step_rad,
            ..*flow
        })?;
        let inv_alpha_span = 1.0 / (2.0 * alpha_step_rad);
        let inv_control_span = 1.0 / (2.0 * control_step_rad);
        let derivatives = VlmHorseshoeDerivatives {
            lift_curve_slope_per_rad: (alpha_plus.cl - alpha_minus.cl) * inv_alpha_span,
            pitching_moment_slope_per_rad: (alpha_plus.cm - alpha_minus.cm) * inv_alpha_span,
            hinge_moment_slope_per_rad: (alpha_plus.hinge_moment_coefficient
                - alpha_minus.hinge_moment_coefficient)
                * inv_alpha_span,
            control_lift_effectiveness_per_rad: (control_plus.cl - control_minus.cl)
                * inv_control_span,
            control_hinge_effectiveness_per_rad: (control_plus.hinge_moment_coefficient
                - control_minus.hinge_moment_coefficient)
                * inv_control_span,
            alpha_step_rad,
            control_step_rad,
        };
        if ![
            derivatives.lift_curve_slope_per_rad,
            derivatives.pitching_moment_slope_per_rad,
            derivatives.hinge_moment_slope_per_rad,
            derivatives.control_lift_effectiveness_per_rad,
            derivatives.control_hinge_effectiveness_per_rad,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(AeroError::NonFinite {
                reason: "VLM horseshoe derivatives are NaN or Inf",
            });
        }
        Ok(derivatives)
    }

    fn assemble_linear_system(&self, flow: &VlmFinFlow) -> Result<VlmLinearSystem, AeroError> {
        validate_flow(flow)?;
        let alpha_effective =
            flow.alpha_rad + control_effectiveness(self.geometry) * flow.control_deflection_rad;
        let freestream = Vector3::new(alpha_effective.cos(), 0.0, -alpha_effective.sin());
        let n = self.panels.len();
        let mut matrix = vec![0.0_f64; n * n];
        let mut rhs = vec![0.0_f64; n];
        for (row, target) in self.panels.iter().enumerate() {
            rhs[row] = -freestream.dot(&target.normal);
            for (col, source) in self.panels.iter().enumerate() {
                matrix[row * n + col] = self
                    .horseshoe_unit_velocity(source, target.control_point_m)
                    .dot(&target.normal);
            }
        }
        Ok(VlmLinearSystem {
            matrix,
            rhs,
            dimension: n,
        })
    }

    fn horseshoe_unit_velocity(
        &self,
        panel: &VlmHorseshoePanel,
        point_m: Vector3<f64>,
    ) -> Vector3<f64> {
        let wake = Vector3::new(self.wake_length_m, 0.0, 0.0);
        let wake_start = panel.bound_start_m + wake;
        let wake_end = panel.bound_end_m + wake;
        vortex_segment_velocity(point_m, panel.bound_start_m, panel.bound_end_m)
            + vortex_segment_velocity(point_m, panel.bound_end_m, wake_end)
            + vortex_segment_velocity(point_m, wake_start, panel.bound_start_m)
    }

    fn coefficients_from_circulation(
        &self,
        circulation: &[f64],
        residual: f64,
        compressibility_scale: f64,
    ) -> Result<VlmHorseshoeSolution, AeroError> {
        let mut cl = 0.0_f64;
        let mut cd_induced = 0.0_f64;
        let mut cm = 0.0_f64;
        let mut hinge_moment_coefficient = 0.0_f64;
        for (gamma, panel) in circulation.iter().copied().zip(&self.panels) {
            let scaled_gamma = gamma * compressibility_scale;
            let span_length_m = (panel.bound_end_m - panel.bound_start_m).norm();
            let dcl = -2.0 * scaled_gamma * span_length_m / self.geometry.reference_area_m2;
            cl += dcl;
            cm -= dcl * (panel.control_point_m.x - self.geometry.moment_reference_x_m)
                / self.geometry.reference_length_m;
            if panel.control_chord_fraction >= self.geometry.hinge_chord_fraction {
                hinge_moment_coefficient -= dcl * (panel.control_point_m.x - panel.hinge_x_m)
                    / self.geometry.reference_length_m;
            }
            let downwash = self
                .panels
                .iter()
                .zip(circulation)
                .map(|(source, source_gamma)| {
                    source_gamma
                        * compressibility_scale
                        * self
                            .horseshoe_unit_velocity(source, panel.control_point_m)
                            .z
                })
                .sum::<f64>();
            cd_induced += (dcl * downwash).abs();
        }
        if ![cl, cd_induced, cm, hinge_moment_coefficient, residual]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(AeroError::NonFinite {
                reason: "VLM horseshoe solution is NaN or Inf",
            });
        }
        Ok(VlmHorseshoeSolution {
            circulation: circulation.to_vec(),
            cl,
            cd_induced,
            cm,
            hinge_moment_coefficient,
            max_tangency_residual: residual,
            panel_count: self.panels.len(),
        })
    }
}

fn validate_geometry(geometry: VlmFinGeometry, spanwise_panels: u32) -> Result<(), AeroError> {
    for value in [
        geometry.root_chord_m,
        geometry.tip_chord_m,
        geometry.span_m,
        geometry.sweep_rad,
        geometry.root_le_x_m,
        geometry.moment_reference_x_m,
        geometry.reference_area_m2,
        geometry.reference_length_m,
        geometry.hinge_chord_fraction,
    ] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "VLM fin geometry contains NaN or Inf",
            });
        }
    }
    if geometry.root_chord_m <= 0.0
        || geometry.tip_chord_m < 0.0
        || geometry.span_m <= 0.0
        || geometry.reference_area_m2 <= 0.0
        || geometry.reference_length_m <= 0.0
        || !(MIN_HINGE_FRACTION..=MAX_HINGE_FRACTION).contains(&geometry.hinge_chord_fraction)
        || spanwise_panels == 0
        || spanwise_panels > MAX_SPANWISE_PANELS
    {
        return Err(AeroError::InvalidParameter {
            reason: "VLM fin geometry is outside envelope",
        });
    }
    if fin_area_m2(geometry) <= 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "VLM fin planform area must be positive",
        });
    }
    Ok(())
}

fn validate_flow(flow: &VlmFinFlow) -> Result<(), AeroError> {
    for value in [flow.mach, flow.alpha_rad, flow.control_deflection_rad] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "VLM fin flow contains NaN or Inf",
            });
        }
    }
    if flow.mach < 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "VLM fin Mach must be non-negative",
        });
    }
    if flow.mach >= MAX_ATTACHED_FLOW_MACH {
        return Err(AeroError::OutOfEnvelope {
            reason: "VLM fin first substrate is subsonic attached-flow only",
        });
    }
    Ok(())
}

fn validate_grid_fin_geometry(geometry: VlmGridFinGeometry) -> Result<(), AeroError> {
    for value in [
        geometry.projected_area_m2,
        geometry.reference_area_m2,
        geometry.reference_length_m,
        geometry.aerodynamic_center_x_m,
        geometry.moment_reference_x_m,
        geometry.hinge_moment_arm_m,
        geometry.open_area_ratio,
        geometry.web_thickness_to_cell_pitch,
    ] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "VLM grid fin geometry contains NaN or Inf",
            });
        }
    }
    if geometry.projected_area_m2 <= 0.0
        || geometry.reference_area_m2 <= 0.0
        || geometry.reference_length_m <= 0.0
        || geometry.hinge_moment_arm_m < 0.0
        || !(0.05..=0.95).contains(&geometry.open_area_ratio)
        || !(0.0..=0.45).contains(&geometry.web_thickness_to_cell_pitch)
        || geometry.chordwise_cells == 0
        || geometry.chordwise_cells > MAX_GRID_FIN_CELLS_PER_AXIS
        || geometry.spanwise_cells == 0
        || geometry.spanwise_cells > MAX_GRID_FIN_CELLS_PER_AXIS
    {
        return Err(AeroError::InvalidParameter {
            reason: "VLM grid fin geometry is outside envelope",
        });
    }
    if geometry
        .chordwise_cells
        .checked_mul(geometry.spanwise_cells)
        .is_none()
    {
        return Err(AeroError::InvalidParameter {
            reason: "VLM grid fin cell count overflows",
        });
    }
    Ok(())
}

fn validate_grid_fin_flow(flow: &VlmGridFinFlow) -> Result<(), AeroError> {
    for value in [flow.mach, flow.alpha_rad, flow.control_deflection_rad] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "VLM grid fin flow contains NaN or Inf",
            });
        }
    }
    if flow.mach < 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "VLM grid fin Mach must be non-negative",
        });
    }
    if flow.mach >= MAX_ATTACHED_FLOW_MACH {
        return Err(AeroError::OutOfEnvelope {
            reason: "VLM grid fin first substrate is subsonic attached-flow only",
        });
    }
    Ok(())
}

#[cfg(feature = "parser")]
fn validate_nonempty_str(value: &str, reason: &'static str) -> Result<(), AeroError> {
    if value.trim().is_empty() {
        return Err(AeroError::MalformedDeck { reason });
    }
    Ok(())
}

#[cfg(feature = "parser")]
fn validate_finite_values(values: &[f64], reason: &'static str) -> Result<(), AeroError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(AeroError::NonFinite { reason });
    }
    Ok(())
}

fn validate_finite_difference_step(step_rad: f64, reason: &'static str) -> Result<(), AeroError> {
    if !step_rad.is_finite() {
        return Err(AeroError::NonFinite { reason });
    }
    if !(0.0..=0.1).contains(&step_rad) || step_rad <= 0.0 {
        return Err(AeroError::InvalidParameter { reason });
    }
    Ok(())
}

fn subsonic_compressibility_beta(mach: f64) -> Result<f64, AeroError> {
    if mach >= MAX_ATTACHED_FLOW_MACH {
        return Err(AeroError::OutOfEnvelope {
            reason: "VLM fin first substrate is subsonic attached-flow only",
        });
    }
    Ok((1.0 - mach * mach).sqrt().max(0.25))
}

fn control_effectiveness(geometry: VlmFinGeometry) -> f64 {
    (1.0 - geometry.hinge_chord_fraction).powf(0.75)
}

fn finite_wing_lift_curve_slope_per_rad(effective_aspect_ratio: f64, beta: f64) -> f64 {
    let denominator = beta * effective_aspect_ratio + 2.903 + 0.027 / effective_aspect_ratio;
    2.0 * PI * effective_aspect_ratio / denominator
}

fn fin_area_m2(geometry: VlmFinGeometry) -> f64 {
    0.5 * (geometry.root_chord_m + geometry.tip_chord_m) * geometry.span_m
}

fn local_chord_m(geometry: VlmFinGeometry, span_fraction: f64) -> f64 {
    geometry.root_chord_m + (geometry.tip_chord_m - geometry.root_chord_m) * span_fraction
}

fn local_le_x_m(geometry: VlmFinGeometry, y_m: f64) -> f64 {
    geometry.root_le_x_m + y_m * geometry.sweep_rad.tan()
}

fn aerodynamic_center_x_m(geometry: VlmFinGeometry, spanwise_panels: u32) -> f64 {
    let dy = geometry.span_m / f64::from(spanwise_panels);
    let mut moment = 0.0_f64;
    let mut area = 0.0_f64;
    for panel in 0..spanwise_panels {
        let y_mid = dy * (f64::from(panel) + 0.5);
        let span_fraction = y_mid / geometry.span_m;
        let chord_m =
            geometry.root_chord_m + (geometry.tip_chord_m - geometry.root_chord_m) * span_fraction;
        let x_le_m = geometry.root_le_x_m + y_mid * geometry.sweep_rad.tan();
        let strip_area = chord_m * dy;
        moment += strip_area * (x_le_m + 0.25 * chord_m);
        area += strip_area;
    }
    moment / area
}

fn vortex_segment_velocity(
    point_m: Vector3<f64>,
    start_m: Vector3<f64>,
    end_m: Vector3<f64>,
) -> Vector3<f64> {
    let r1 = point_m - start_m;
    let r2 = point_m - end_m;
    let r0 = end_m - start_m;
    let cross = r1.cross(&r2);
    let cross_norm_squared = cross.norm_squared();
    let r1_norm = r1.norm();
    let r2_norm = r2.norm();
    if cross_norm_squared <= BIOT_SAVART_EPS
        || r1_norm <= BIOT_SAVART_EPS
        || r2_norm <= BIOT_SAVART_EPS
    {
        return Vector3::zeros();
    }
    let scale = r0.dot(&(r1 / r1_norm - r2 / r2_norm)) / (4.0 * PI * cross_norm_squared);
    cross * scale
}

fn solve_dense_deterministic(
    mut matrix: Vec<f64>,
    mut rhs: Vec<f64>,
    n: usize,
) -> Result<Vec<f64>, AeroError> {
    for pivot in 0..n {
        let mut pivot_row = pivot;
        let mut pivot_abs = matrix[pivot * n + pivot].abs();
        for row in (pivot + 1)..n {
            let candidate = matrix[row * n + pivot].abs();
            if candidate > pivot_abs {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }
        if pivot_abs <= SOLVE_PIVOT_EPS {
            return Err(AeroError::InvalidParameter {
                reason: "VLM horseshoe influence matrix is singular",
            });
        }
        if pivot_row != pivot {
            for col in 0..n {
                matrix.swap(pivot * n + col, pivot_row * n + col);
            }
            rhs.swap(pivot, pivot_row);
        }
        let pivot_value = matrix[pivot * n + pivot];
        for row in (pivot + 1)..n {
            let factor = matrix[row * n + pivot] / pivot_value;
            matrix[row * n + pivot] = 0.0;
            for col in (pivot + 1)..n {
                matrix[row * n + col] -= factor * matrix[pivot * n + col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..n {
            sum -= matrix[row * n + col] * x[col];
        }
        x[row] = sum / matrix[row * n + row];
    }
    Ok(x)
}

fn max_linear_residual(matrix: &[f64], x: &[f64], rhs: &[f64], n: usize) -> f64 {
    let mut max_abs = 0.0_f64;
    for row in 0..n {
        let mut ax = 0.0_f64;
        for col in 0..n {
            ax += matrix[row * n + col] * x[col];
        }
        max_abs = max_abs.max((ax - rhs[row]).abs());
    }
    max_abs
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::missing_panics_doc)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmFinTrendFixture {
        case: VlmFinTrendCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmFiniteWingAnalyticFixture {
        case: VlmFiniteWingAnalyticCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmNacaReport921MethodFixture {
        case: VlmNacaReport921MethodCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmFinTrendCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        root_chord_m: f64,
        tip_chord_m: f64,
        span_m: f64,
        sweep_deg: f64,
        root_le_x_m: f64,
        moment_reference_x_m: f64,
        reference_area_m2: f64,
        reference_length_m: f64,
        hinge_chord_fraction: f64,
        spanwise_panels: u32,
        mach: f64,
        alpha_deg: f64,
        positive_deflection_deg: f64,
        negative_deflection_deg: f64,
        expected_lift_curve_slope_per_rad: f64,
        expected_cl_alpha: f64,
        expected_cm_alpha: f64,
        expected_hinge_alpha: f64,
        expected_cl_positive_deflection: f64,
        expected_hinge_positive_deflection: f64,
        expected_cl_negative_deflection: f64,
        expected_hinge_negative_deflection: f64,
        max_relative_error: f64,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmFiniteWingAnalyticCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        #[serde(default)]
        report_id: Option<String>,
        #[serde(default)]
        source_title: Option<String>,
        #[serde(default)]
        source_url: Option<String>,
        #[serde(default)]
        source_figure: Option<String>,
        root_chord_m: f64,
        tip_chord_m: f64,
        span_m: f64,
        sweep_deg: f64,
        root_le_x_m: f64,
        moment_reference_x_m: f64,
        reference_area_m2: f64,
        reference_length_m: f64,
        hinge_chord_fraction: f64,
        spanwise_panels: u32,
        mach: f64,
        alpha_deg: f64,
        expected_aspect_ratio: f64,
        expected_lift_curve_slope_per_rad: f64,
        expected_cl: f64,
        expected_cd_induced: f64,
        expected_aerodynamic_center_x_m: f64,
        max_relative_error: f64,
        max_absolute_error: f64,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct VlmNacaReport921MethodCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        report_id: String,
        root_chord_m: f64,
        tip_chord_m: f64,
        span_m: f64,
        sweep_deg: f64,
        root_le_x_m: f64,
        moment_reference_x_m: f64,
        reference_area_m2: f64,
        reference_length_m: f64,
        hinge_chord_fraction: f64,
        chordwise_panels: u32,
        spanwise_panels: u32,
        wake_length_m: f64,
        expected_panel_count: usize,
        expected_bound_chord_fraction: f64,
        expected_control_chord_fraction: f64,
        expected_normal_z: f64,
        max_absolute_error: f64,
    }

    fn geometry() -> VlmFinGeometry {
        VlmFinGeometry {
            root_chord_m: 0.30,
            tip_chord_m: 0.15,
            span_m: 0.20,
            sweep_rad: 20.0_f64.to_radians(),
            root_le_x_m: 2.1,
            moment_reference_x_m: 1.25,
            reference_area_m2: PI * 0.2 * 0.2 / 4.0,
            reference_length_m: 0.2,
            hinge_chord_fraction: 0.70,
        }
    }

    fn grid_geometry() -> VlmGridFinGeometry {
        VlmGridFinGeometry {
            projected_area_m2: 0.018,
            reference_area_m2: PI * 0.2 * 0.2 / 4.0,
            reference_length_m: 0.2,
            aerodynamic_center_x_m: 2.15,
            moment_reference_x_m: 1.25,
            hinge_moment_arm_m: 0.06,
            chordwise_cells: 5,
            spanwise_cells: 6,
            open_area_ratio: 0.65,
            web_thickness_to_cell_pitch: 0.08,
        }
    }

    fn linear_system_hex_words(system: &VlmLinearSystem) -> Vec<String> {
        system
            .matrix
            .iter()
            .chain(&system.rhs)
            .map(|value| format!("{:016x}", value.to_bits()))
            .collect()
    }

    #[test]
    fn vlm_fin_lift_slope_and_moment_are_deterministic() {
        let fin = VlmFinSurface::new(geometry(), 8).unwrap();
        let flow = VlmFinFlow {
            mach: 0.3,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        let first = fin.evaluate(&flow).unwrap();
        let second = fin.evaluate(&flow).unwrap();
        assert_eq!(first, second);
        assert!(first.cl > 0.0);
        assert!(first.cd_induced > 0.0);
        assert!(first.cm < 0.0);
        assert!(first.hinge_moment_coefficient < 0.0);
    }

    #[test]
    fn vlm_fin_hinge_moment_is_signed_and_monotonic_with_deflection() {
        let fin = VlmFinSurface::new(geometry(), 8).unwrap();
        let neutral = fin
            .evaluate(&VlmFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let positive = fin
            .evaluate(&VlmFinFlow {
                control_deflection_rad: 5.0_f64.to_radians(),
                ..VlmFinFlow {
                    mach: 0.3,
                    alpha_rad: 5.0_f64.to_radians(),
                    control_deflection_rad: 0.0,
                }
            })
            .unwrap();
        let negative = fin
            .evaluate(&VlmFinFlow {
                control_deflection_rad: -5.0_f64.to_radians(),
                ..VlmFinFlow {
                    mach: 0.3,
                    alpha_rad: 5.0_f64.to_radians(),
                    control_deflection_rad: 0.0,
                }
            })
            .unwrap();
        assert!(positive.cl > neutral.cl);
        assert!(negative.cl < neutral.cl);
        assert!(positive.hinge_moment_coefficient < neutral.hinge_moment_coefficient);
        assert!(negative.hinge_moment_coefficient > neutral.hinge_moment_coefficient);
    }

    #[test]
    fn vlm_fin_rejects_invalid_inputs_and_transonic_flow() {
        assert!(matches!(
            VlmFinSurface::new(
                VlmFinGeometry {
                    root_chord_m: -1.0,
                    ..geometry()
                },
                8
            ),
            Err(AeroError::InvalidParameter { .. })
        ));
        let fin = VlmFinSurface::new(geometry(), 8).unwrap();
        assert!(matches!(
            fin.evaluate(&VlmFinFlow {
                mach: 0.8,
                alpha_rad: 0.0,
                control_deflection_rad: 0.0,
            }),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn vlm_grid_fin_authority_is_deterministic_and_tracks_deflection() {
        let grid = VlmGridFinSurface::new(grid_geometry()).unwrap();
        let neutral = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let repeat = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let positive = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 5.0_f64.to_radians(),
            })
            .unwrap();
        let negative = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: -5.0_f64.to_radians(),
            })
            .unwrap();
        assert_eq!(neutral, repeat);
        assert_eq!(neutral.cell_count, 30);
        assert!(neutral.normal_force_coefficient > 0.0);
        assert!(neutral.axial_drag_coefficient > 0.0);
        assert!(neutral.pitching_moment_coefficient < 0.0);
        assert!(neutral.hinge_moment_coefficient < 0.0);
        assert!(positive.normal_force_coefficient > neutral.normal_force_coefficient);
        assert!(negative.normal_force_coefficient < neutral.normal_force_coefficient);
        assert!(positive.hinge_moment_coefficient < neutral.hinge_moment_coefficient);
        assert!(negative.hinge_moment_coefficient > neutral.hinge_moment_coefficient);
    }

    #[test]
    fn vlm_grid_fin_open_area_controls_authority_and_blockage_drag() {
        let open_grid = VlmGridFinSurface::new(grid_geometry()).unwrap();
        let blocked_grid = VlmGridFinSurface::new(VlmGridFinGeometry {
            open_area_ratio: 0.35,
            ..grid_geometry()
        })
        .unwrap();
        let flow = VlmGridFinFlow {
            mach: 0.2,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        let open = open_grid.evaluate(&flow).unwrap();
        let blocked = blocked_grid.evaluate(&flow).unwrap();
        assert!(open.effective_open_area_ratio > blocked.effective_open_area_ratio);
        assert!(open.normal_force_slope_per_rad > blocked.normal_force_slope_per_rad);
        assert!(open.normal_force_coefficient > blocked.normal_force_coefficient);
        assert!(blocked.axial_drag_coefficient > open.axial_drag_coefficient);
    }

    #[test]
    fn vlm_grid_fin_applies_subsonic_compressibility_scale() {
        let grid = VlmGridFinSurface::new(grid_geometry()).unwrap();
        let incompressible = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.0,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let compressed = grid
            .evaluate(&VlmGridFinFlow {
                mach: 0.6,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let scale = 1.0 / (1.0_f64 - 0.6_f64 * 0.6_f64).sqrt();
        assert_relative_eq!(
            compressed.normal_force_coefficient / incompressible.normal_force_coefficient,
            scale,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            compressed.pitching_moment_coefficient / incompressible.pitching_moment_coefficient,
            scale,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            compressed.hinge_moment_coefficient / incompressible.hinge_moment_coefficient,
            scale,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn vlm_grid_fin_rejects_invalid_geometry_and_flow() {
        assert!(matches!(
            VlmGridFinSurface::new(VlmGridFinGeometry {
                open_area_ratio: 1.0,
                ..grid_geometry()
            }),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            VlmGridFinSurface::new(VlmGridFinGeometry {
                chordwise_cells: 0,
                ..grid_geometry()
            }),
            Err(AeroError::InvalidParameter { .. })
        ));
        let grid = VlmGridFinSurface::new(grid_geometry()).unwrap();
        assert!(matches!(
            grid.evaluate(&VlmGridFinFlow {
                mach: 0.8,
                alpha_rad: 0.0,
                control_deflection_rad: 0.0,
            }),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            grid.evaluate(&VlmGridFinFlow {
                mach: 0.2,
                alpha_rad: 0.6,
                control_deflection_rad: 0.0,
            }),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn vlm_horseshoe_lattice_solves_tangency_and_is_deterministic() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let flow = VlmFinFlow {
            mach: 0.3,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        let first = lattice.solve(&flow).unwrap();
        let second = lattice.solve(&flow).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.panel_count, 8);
        assert_eq!(first.circulation.len(), 8);
        assert!(first.max_tangency_residual < 1.0e-10);
        assert!(first.cl > 0.0, "{first:#?}");
        assert!(first.cd_induced >= 0.0);
        assert!(first.cm < 0.0, "{first:#?}");
    }

    #[test]
    fn vlm_horseshoe_lattice_applies_subsonic_compressibility_scale() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let incompressible = lattice
            .solve(&VlmFinFlow {
                mach: 0.0,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let compressed = lattice
            .solve(&VlmFinFlow {
                mach: 0.6,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let scale = 1.0 / (1.0_f64 - 0.6_f64 * 0.6_f64).sqrt();
        assert_relative_eq!(compressed.cl / incompressible.cl, scale, epsilon = 1.0e-12);
        assert_relative_eq!(compressed.cm / incompressible.cm, scale, epsilon = 1.0e-12);
        assert_relative_eq!(
            compressed.hinge_moment_coefficient / incompressible.hinge_moment_coefficient,
            scale,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            compressed.cd_induced / incompressible.cd_induced,
            scale * scale,
            epsilon = 1.0e-12
        );
        assert!(
            (compressed.max_tangency_residual - incompressible.max_tangency_residual).abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn vlm_horseshoe_lattice_hinge_moment_tracks_deflection() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let neutral = lattice
            .solve(&VlmFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let positive = lattice
            .solve(&VlmFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 5.0_f64.to_radians(),
            })
            .unwrap();
        let negative = lattice
            .solve(&VlmFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: -5.0_f64.to_radians(),
            })
            .unwrap();
        assert!(positive.cl > neutral.cl, "{positive:#?} <= {neutral:#?}");
        assert!(negative.cl < neutral.cl, "{negative:#?} >= {neutral:#?}");
        assert!(positive.hinge_moment_coefficient < neutral.hinge_moment_coefficient);
        assert!(negative.hinge_moment_coefficient > neutral.hinge_moment_coefficient);
    }

    #[test]
    fn vlm_horseshoe_derivatives_are_deterministic_and_signed() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let flow = VlmFinFlow {
            mach: 0.3,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        let first = lattice
            .finite_difference_derivatives(&flow, 0.25_f64.to_radians(), 0.25_f64.to_radians())
            .unwrap();
        let second = lattice
            .finite_difference_derivatives(&flow, 0.25_f64.to_radians(), 0.25_f64.to_radians())
            .unwrap();
        assert_eq!(first, second);
        assert!(first.lift_curve_slope_per_rad > 0.0, "{first:#?}");
        assert!(first.pitching_moment_slope_per_rad < 0.0, "{first:#?}");
        assert!(first.hinge_moment_slope_per_rad < 0.0, "{first:#?}");
        assert!(first.control_lift_effectiveness_per_rad > 0.0, "{first:#?}");
        assert!(
            first.control_hinge_effectiveness_per_rad < 0.0,
            "{first:#?}"
        );
    }

    #[test]
    fn vlm_horseshoe_derivatives_match_declared_central_difference() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let flow = VlmFinFlow {
            mach: 0.3,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        let alpha_step = 0.5_f64.to_radians();
        let control_step = 0.25_f64.to_radians();
        let derivatives = lattice
            .finite_difference_derivatives(&flow, alpha_step, control_step)
            .unwrap();
        let alpha_plus = lattice
            .solve(&VlmFinFlow {
                alpha_rad: flow.alpha_rad + alpha_step,
                ..flow
            })
            .unwrap();
        let alpha_minus = lattice
            .solve(&VlmFinFlow {
                alpha_rad: flow.alpha_rad - alpha_step,
                ..flow
            })
            .unwrap();
        let control_plus = lattice
            .solve(&VlmFinFlow {
                control_deflection_rad: flow.control_deflection_rad + control_step,
                ..flow
            })
            .unwrap();
        let control_minus = lattice
            .solve(&VlmFinFlow {
                control_deflection_rad: flow.control_deflection_rad - control_step,
                ..flow
            })
            .unwrap();
        assert_relative_eq!(
            derivatives.lift_curve_slope_per_rad,
            (alpha_plus.cl - alpha_minus.cl) / (2.0 * alpha_step),
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            derivatives.pitching_moment_slope_per_rad,
            (alpha_plus.cm - alpha_minus.cm) / (2.0 * alpha_step),
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            derivatives.control_lift_effectiveness_per_rad,
            (control_plus.cl - control_minus.cl) / (2.0 * control_step),
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            derivatives.control_hinge_effectiveness_per_rad,
            (control_plus.hinge_moment_coefficient - control_minus.hinge_moment_coefficient)
                / (2.0 * control_step),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn vlm_horseshoe_derivatives_reject_invalid_steps_and_flow() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        let flow = VlmFinFlow {
            mach: 0.3,
            alpha_rad: 5.0_f64.to_radians(),
            control_deflection_rad: 0.0,
        };
        assert!(matches!(
            lattice.finite_difference_derivatives(&flow, 0.0, 0.01),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            lattice.finite_difference_derivatives(&flow, 0.01, 0.2),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            lattice.finite_difference_derivatives(&VlmFinFlow { mach: 0.9, ..flow }, 0.01, 0.01,),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn vlm_horseshoe_lattice_rejects_invalid_grid_and_flow() {
        assert!(matches!(
            VlmHorseshoeLattice::new(geometry(), 0, 4, 20.0),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            VlmHorseshoeLattice::new(geometry(), 2, 4, -20.0),
            Err(AeroError::InvalidParameter { .. })
        ));
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 4, 20.0).unwrap();
        assert!(matches!(
            lattice.solve(&VlmFinFlow {
                mach: 0.9,
                alpha_rad: 0.0,
                control_deflection_rad: 0.0,
            }),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn vlm_horseshoe_linear_system_operand_order_is_pinned() {
        let lattice = VlmHorseshoeLattice::new(geometry(), 2, 2, 20.0).unwrap();
        let system = lattice
            .assemble_linear_system(&VlmFinFlow {
                mach: 0.3,
                alpha_rad: 5.0_f64.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        assert_eq!(system.dimension, 4);
        let expected = [
            "c01cd1b99cfe32bc",
            "3ff95461a1389b84",
            "3fea4a9802a8da16",
            "3fdc0281970d8baf",
            "3ffc21217061411e",
            "c01f8b270b197224",
            "3fdcb351f1ad23d9",
            "3ff7a6c5a4758937",
            "c019dee798d5d2e3",
            "40001bf53c7c3a52",
            "c01cc04a52713096",
            "3ffc8f29921e393d",
            "40000fc1cabe72c0",
            "c01a407b03c98a7d",
            "3ff894d37bfa0230",
            "c01f60a8bb3edfcd",
            "3fb64fd6b8c28102",
            "3fb64fd6b8c28102",
            "3fb64fd6b8c28102",
            "3fb64fd6b8c28102",
        ]
        .map(String::from);
        let actual = linear_system_hex_words(&system);
        assert_eq!(actual.as_slice(), expected.as_slice());
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_naca_report_921_method_fixture_pins_panel_convention() {
        let fixture: VlmNacaReport921MethodFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/naca-report-921-vlm-method-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.naca_report_921_vlm_method.v1"
        );
        assert_eq!(case.source_class, "public-domain");
        assert_eq!(case.validation_label, "checked");
        assert_eq!(case.report_id, "NACA-TR-921");
        let geometry = VlmFinGeometry {
            root_chord_m: case.root_chord_m,
            tip_chord_m: case.tip_chord_m,
            span_m: case.span_m,
            sweep_rad: case.sweep_deg.to_radians(),
            root_le_x_m: case.root_le_x_m,
            moment_reference_x_m: case.moment_reference_x_m,
            reference_area_m2: case.reference_area_m2,
            reference_length_m: case.reference_length_m,
            hinge_chord_fraction: case.hinge_chord_fraction,
        };
        let lattice = VlmHorseshoeLattice::new(
            geometry,
            case.chordwise_panels,
            case.spanwise_panels,
            case.wake_length_m,
        )
        .unwrap();
        assert_eq!(lattice.panels().len(), case.expected_panel_count);
        for panel in lattice.panels() {
            let bound_start_fraction =
                (panel.bound_start_m.x - geometry.root_le_x_m) / geometry.root_chord_m;
            let bound_end_fraction =
                (panel.bound_end_m.x - geometry.root_le_x_m) / geometry.root_chord_m;
            let control_fraction =
                (panel.control_point_m.x - geometry.root_le_x_m) / geometry.root_chord_m;
            assert_relative_eq!(
                bound_start_fraction,
                case.expected_bound_chord_fraction,
                epsilon = case.max_absolute_error
            );
            assert_relative_eq!(
                bound_end_fraction,
                case.expected_bound_chord_fraction,
                epsilon = case.max_absolute_error
            );
            assert_relative_eq!(
                control_fraction,
                case.expected_control_chord_fraction,
                epsilon = case.max_absolute_error
            );
            assert_relative_eq!(
                panel.control_chord_fraction,
                case.expected_control_chord_fraction,
                epsilon = case.max_absolute_error
            );
            assert_relative_eq!(panel.normal.x, 0.0, epsilon = case.max_absolute_error);
            assert_relative_eq!(panel.normal.y, 0.0, epsilon = case.max_absolute_error);
            assert_relative_eq!(
                panel.normal.z,
                case.expected_normal_z,
                epsilon = case.max_absolute_error
            );
        }
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_horseshoe_derivative_comparison_fixture_reports_expected_values() {
        let fixture = VlmHorseshoeDerivativeComparisonDeck::from_toml_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/vlm-horseshoe-derivative-comparison-v1.toml"
        )))
        .unwrap();
        let report = fixture.evaluate().unwrap();
        assert!(report.passed, "{report:#?}");
        assert_eq!(
            report.dataset_id,
            "openbmp.aero.vlm_horseshoe_derivative_comparison.v1"
        );
        assert_eq!(report.sample_count, 1);
        assert_eq!(report.checked_value_count, 5);
        assert_eq!(report.samples[0].checked_value_count, 5);
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_horseshoe_derivative_comparison_rejects_empty_samples() {
        let malformed = r#"
[openbmp]
vlm_horseshoe_derivative_comparison = 1

[provenance]
dataset_id = "openbmp.aero.bad"
source_class = "synthetic-openbmp"
validation_label = "checked"

[geometry]
root_chord_m = 0.30
tip_chord_m = 0.15
span_m = 0.20
sweep_deg = 20.0
root_le_x_m = 2.1
moment_reference_x_m = 1.25
reference_area_m2 = 0.031415926535897934
reference_length_m = 0.2
hinge_chord_fraction = 0.70

[lattice]
chordwise_panels = 2
spanwise_panels = 4
wake_length_m = 20.0

[tolerances]
abs_tolerance = 1.0e-12
rel_tolerance = 1.0e-12
"#;
        assert!(matches!(
            VlmHorseshoeDerivativeComparisonDeck::from_toml_str(malformed),
            Err(AeroError::MalformedDeck { .. })
        ));
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_fin_fixture_matches_lift_and_hinge_trends() {
        let fixture: VlmFinTrendFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/vlm-fin-lift-hinge-trend-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(case.dataset_id, "openbmp.aero.vlm_fin_lift_hinge_trend.v1");
        assert_eq!(case.source_class, "synthetic-openbmp");
        assert_eq!(case.validation_label, "checked");
        let geometry = VlmFinGeometry {
            root_chord_m: case.root_chord_m,
            tip_chord_m: case.tip_chord_m,
            span_m: case.span_m,
            sweep_rad: case.sweep_deg.to_radians(),
            root_le_x_m: case.root_le_x_m,
            moment_reference_x_m: case.moment_reference_x_m,
            reference_area_m2: case.reference_area_m2,
            reference_length_m: case.reference_length_m,
            hinge_chord_fraction: case.hinge_chord_fraction,
        };
        let fin = VlmFinSurface::new(geometry, case.spanwise_panels).unwrap();
        let neutral = fin
            .evaluate(&VlmFinFlow {
                mach: case.mach,
                alpha_rad: case.alpha_deg.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        let positive = fin
            .evaluate(&VlmFinFlow {
                mach: case.mach,
                alpha_rad: case.alpha_deg.to_radians(),
                control_deflection_rad: case.positive_deflection_deg.to_radians(),
            })
            .unwrap();
        let negative = fin
            .evaluate(&VlmFinFlow {
                mach: case.mach,
                alpha_rad: case.alpha_deg.to_radians(),
                control_deflection_rad: case.negative_deflection_deg.to_radians(),
            })
            .unwrap();

        assert_relative_eq!(
            neutral.lift_curve_slope_per_rad,
            case.expected_lift_curve_slope_per_rad,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            neutral.cl,
            case.expected_cl_alpha,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            neutral.cm,
            case.expected_cm_alpha,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            neutral.hinge_moment_coefficient,
            case.expected_hinge_alpha,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            positive.cl,
            case.expected_cl_positive_deflection,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            positive.hinge_moment_coefficient,
            case.expected_hinge_positive_deflection,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            negative.cl,
            case.expected_cl_negative_deflection,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            negative.hinge_moment_coefficient,
            case.expected_hinge_negative_deflection,
            max_relative = case.max_relative_error
        );
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_finite_wing_analytic_fixture_matches_lift_slope_and_induced_drag() {
        let fixture: VlmFiniteWingAnalyticFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/vlm-finite-wing-analytic-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(case.dataset_id, "openbmp.aero.vlm_finite_wing_analytic.v1");
        assert_eq!(case.source_class, "analytic-openbmp");
        assert_eq!(case.validation_label, "checked");
        let geometry = VlmFinGeometry {
            root_chord_m: case.root_chord_m,
            tip_chord_m: case.tip_chord_m,
            span_m: case.span_m,
            sweep_rad: case.sweep_deg.to_radians(),
            root_le_x_m: case.root_le_x_m,
            moment_reference_x_m: case.moment_reference_x_m,
            reference_area_m2: case.reference_area_m2,
            reference_length_m: case.reference_length_m,
            hinge_chord_fraction: case.hinge_chord_fraction,
        };
        let area = fin_area_m2(geometry);
        let aspect_ratio = geometry.span_m * geometry.span_m / area;
        assert_relative_eq!(
            aspect_ratio,
            case.expected_aspect_ratio,
            max_relative = case.max_relative_error
        );
        let fin = VlmFinSurface::new(geometry, case.spanwise_panels).unwrap();
        let solution = fin
            .evaluate(&VlmFinFlow {
                mach: case.mach,
                alpha_rad: case.alpha_deg.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();

        assert_relative_eq!(
            solution.lift_curve_slope_per_rad,
            case.expected_lift_curve_slope_per_rad,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.cl,
            case.expected_cl,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.cd_induced,
            case.expected_cd_induced,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.aerodynamic_center_x_m,
            case.expected_aerodynamic_center_x_m,
            epsilon = case.max_absolute_error
        );
    }

    #[cfg(feature = "parser")]
    #[test]
    fn vlm_nasa_sp405_rectangular_wing_fixture_matches_lift_slope_formula() {
        let fixture: VlmFiniteWingAnalyticFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/nasa-sp405-rectangular-wing-lift-slope-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.nasa_sp405_rectangular_wing_lift_slope.v1"
        );
        assert_eq!(case.source_class, "public-domain-nasa");
        assert_eq!(case.validation_label, "research");
        assert_eq!(case.report_id.as_deref(), Some("NASA-SP-405"));
        assert_eq!(case.source_figure.as_deref(), Some("Figure 5"));
        assert!(
            case.source_title
                .as_deref()
                .is_some_and(|title| !title.is_empty())
        );
        assert!(
            case.source_url
                .as_deref()
                .is_some_and(|url| !url.is_empty())
        );

        let geometry = VlmFinGeometry {
            root_chord_m: case.root_chord_m,
            tip_chord_m: case.tip_chord_m,
            span_m: case.span_m,
            sweep_rad: case.sweep_deg.to_radians(),
            root_le_x_m: case.root_le_x_m,
            moment_reference_x_m: case.moment_reference_x_m,
            reference_area_m2: case.reference_area_m2,
            reference_length_m: case.reference_length_m,
            hinge_chord_fraction: case.hinge_chord_fraction,
        };
        let area = fin_area_m2(geometry);
        let aspect_ratio = geometry.span_m * geometry.span_m / area;
        assert_relative_eq!(
            aspect_ratio,
            case.expected_aspect_ratio,
            max_relative = case.max_relative_error
        );
        let expected_sp405_slope =
            2.0 * PI * aspect_ratio / (aspect_ratio + 2.903 + 0.027 / aspect_ratio);
        assert_relative_eq!(
            expected_sp405_slope,
            case.expected_lift_curve_slope_per_rad,
            max_relative = case.max_relative_error
        );

        let fin = VlmFinSurface::new(geometry, case.spanwise_panels).unwrap();
        let solution = fin
            .evaluate(&VlmFinFlow {
                mach: case.mach,
                alpha_rad: case.alpha_deg.to_radians(),
                control_deflection_rad: 0.0,
            })
            .unwrap();
        assert_relative_eq!(
            solution.lift_curve_slope_per_rad,
            case.expected_lift_curve_slope_per_rad,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.cl,
            case.expected_cl,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.cd_induced,
            case.expected_cd_induced,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            solution.aerodynamic_center_x_m,
            case.expected_aerodynamic_center_x_m,
            epsilon = case.max_absolute_error
        );
    }
}
