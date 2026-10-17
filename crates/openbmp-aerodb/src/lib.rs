//! Offline aerodynamic run-matrix orchestration and deck baking.
//!
//! `openbmp-aerodb` is an L6 offline crate. It builds condition-only
//! aerodynamic run matrices, evaluates in-Rust aerodynamic methods over those
//! matrices, and bakes the ordered results into the existing
//! [`openbmp_aero::AeroDeck`] substrate. It intentionally has no flight
//! controller, scenario, runner, vehicle-target, or linked external solver
//! dependency. Optional offline command execution is direct argv/env
//! subprocess orchestration, not a flight-runtime force source.

use nalgebra::Vector3;
use openbmp_aero::{
    AeroCoefficients, AeroContext, AeroDeck, AeroError, AeroForceMomentBody, AeroMethod,
    AeroSixCoefficients, AxialLineLoad, PanelMesh,
};
use openbmp_core::DeterministicRng;
use openbmp_uq::{GridConvergence, UqError};
use rayon::prelude::*;
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Stdio};
use std::string::String;
use std::thread;
use std::time::{Duration, Instant};
use thiserror::Error;

pub mod stagesep;

const RUN_MATRIX_SCHEMA_VERSION_1: u32 = 1;
const SOLVER_CASE_MANIFEST_SCHEMA_VERSION_1: u32 = 1;
const SOLVER_COMMAND_PLAN_SCHEMA_VERSION_1: u32 = 1;
const GCI_SIGMA_SCHEMA_VERSION_1: u32 = 1;
const SIX_GCI_SIGMA_SCHEMA_VERSION_1: u32 = 1;
const GCI_SIGMA_CSV_SCHEMA_V1: &str = "openbmp.aerodb_gci_sigma.csv.v1";
const SIX_GCI_SIGMA_CSV_SCHEMA_V1: &str = "openbmp.aerodb_six_gci_sigma.csv.v1";
const GCI_SIGMA_CSV_HEADER: [&str; 21] = [
    "schema",
    "node_index",
    "cn_fine",
    "cn_medium",
    "cn_coarse",
    "cd_fine",
    "cd_medium",
    "cd_coarse",
    "cm_fine",
    "cm_medium",
    "cm_coarse",
    "cn_wind_tunnel",
    "cn_model_form",
    "cn_extrapolation",
    "cd_wind_tunnel",
    "cd_model_form",
    "cd_extrapolation",
    "cm_wind_tunnel",
    "cm_model_form",
    "cm_extrapolation",
    "provenance",
];
const SIX_GCI_SIGMA_CSV_HEADER: [&str; 39] = [
    "schema",
    "node_index",
    "cx_fine",
    "cx_medium",
    "cx_coarse",
    "cy_fine",
    "cy_medium",
    "cy_coarse",
    "cz_fine",
    "cz_medium",
    "cz_coarse",
    "cl_fine",
    "cl_medium",
    "cl_coarse",
    "cm_fine",
    "cm_medium",
    "cm_coarse",
    "cn_fine",
    "cn_medium",
    "cn_coarse",
    "cx_wind_tunnel",
    "cx_model_form",
    "cx_extrapolation",
    "cy_wind_tunnel",
    "cy_model_form",
    "cy_extrapolation",
    "cz_wind_tunnel",
    "cz_model_form",
    "cz_extrapolation",
    "cl_wind_tunnel",
    "cl_model_form",
    "cl_extrapolation",
    "cm_wind_tunnel",
    "cm_model_form",
    "cm_extrapolation",
    "cn_wind_tunnel",
    "cn_model_form",
    "cn_extrapolation",
    "provenance",
];
const SOLVER_COEFFICIENT_CSV_SCHEMA_V1: &str = "openbmp.aerodb_solver_coefficients.csv.v1";
const SOLVER_COEFFICIENT_CSV_HEADER: [&str; 14] = [
    "schema",
    "node_index",
    "mach",
    "alpha_deg",
    "beta_deg",
    "reynolds_length",
    "knudsen",
    "dynamic_pressure_pa",
    "cn",
    "cd",
    "cm",
    "solver",
    "case_id",
    "provenance",
];
const SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1: &str = "openbmp.aerodb_solver_six_coefficients.csv.v1";
const SOLVER_SIX_COEFFICIENT_CSV_HEADER: [&str; 17] = [
    "schema",
    "node_index",
    "mach",
    "alpha_deg",
    "beta_deg",
    "reynolds_length",
    "knudsen",
    "dynamic_pressure_pa",
    "cx",
    "cy",
    "cz",
    "cl",
    "cm",
    "cn",
    "solver",
    "case_id",
    "provenance",
];
const SURFACE_CP_CSV_SCHEMA_V1: &str = "openbmp.aerodb_surface_cp.csv.v1";
const SURFACE_CP_CSV_HEADER: [&str; 13] = [
    "schema",
    "node_index",
    "panel_index",
    "mach",
    "alpha_deg",
    "beta_deg",
    "reynolds_length",
    "knudsen",
    "dynamic_pressure_pa",
    "cp",
    "solver",
    "case_id",
    "provenance",
];
const SU2_EULER_SOLVER_LABEL: &str = "su2-euler";
const SU2_EULER_PROTECTED_CONFIG_KEYS: &[&str] = &[
    "SOLVER",
    "MATH_PROBLEM",
    "MESH_FILENAME",
    "MACH_NUMBER",
    "AOA",
    "SIDESLIP_ANGLE",
    "REF_AREA",
    "REF_LENGTH",
    "REF_ORIGIN_MOMENT_X",
    "REF_ORIGIN_MOMENT_Y",
    "REF_ORIGIN_MOMENT_Z",
    "MARKER_EULER",
    "MARKER_PLOTTING",
    "MARKER_MONITORING",
    "TABULAR_FORMAT",
    "CONV_FILENAME",
    "SURFACE_FILENAME",
    "OUTPUT_FILES",
];
const MAX_SOLVER_COMMAND_RETRY_ATTEMPTS: u32 = 16;
const MAX_SOLVER_CLUSTER_POLL_COUNT: u32 = 4096;
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Error returned by offline aero database helpers.
#[derive(Debug, Error)]
pub enum AerodbError {
    /// A run-matrix axis or scalar is malformed.
    #[error("invalid aero run-matrix input: {field}")]
    InvalidInput {
        /// Field that failed validation.
        field: &'static str,
    },
    /// A run-matrix TOML document failed strict parsing.
    #[error("invalid aero run-matrix TOML: {reason}")]
    MalformedToml {
        /// Human-readable parse or schema reason.
        reason: &'static str,
    },
    /// A solver-exported CSV document failed strict parsing.
    #[error("invalid aero CSV: {reason}")]
    MalformedCsv {
        /// Human-readable parse or schema reason.
        reason: &'static str,
    },
    /// An OpenFOAM field dictionary failed strict parsing.
    #[error("invalid OpenFOAM field: {reason}")]
    MalformedOpenFoam {
        /// Human-readable parse or schema reason.
        reason: &'static str,
    },
    /// A run-matrix TOML key used forbidden target/aimpoint/range vocabulary.
    #[error("aero run-matrix TOML uses forbidden condition vocabulary at {field}")]
    ForbiddenVocabulary {
        /// Field path that failed the condition-only vocabulary guard.
        field: String,
    },
    /// The requested deck bake cannot represent the matrix in today's
    /// reduced deck shape.
    #[error("unsupported aero deck bake: {reason}")]
    UnsupportedDeckBake {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// The underlying aerodynamic method failed.
    #[error(transparent)]
    Aero(#[from] AeroError),
    /// The underlying UQ helper failed.
    #[error(transparent)]
    Uq(#[from] UqError),
}

/// One condition-only run-matrix node.
///
/// The type is intentionally condition-only: it carries Mach, attitude angles,
/// dynamic pressure, Reynolds number, and Knudsen number, but no target,
/// aimpoint, range, latitude/longitude, or weapon/mission semantics.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RunNode {
    /// Freestream Mach number.
    pub mach: f64,
    /// Angle of attack in degrees.
    pub alpha_deg: f64,
    /// Sideslip angle in degrees.
    pub beta_deg: f64,
    /// Freestream dynamic pressure in pascals.
    pub dynamic_pressure_pa: f64,
    /// Reynolds number based on the method reference length.
    pub reynolds_length: f64,
    /// Knudsen number.
    pub knudsen: f64,
}

impl RunNode {
    /// Construct a validated condition node.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any value is non-finite, when
    /// Mach / dynamic pressure / Reynolds / Knudsen are negative, or when
    /// dynamic pressure is zero.
    pub fn new(
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
        dynamic_pressure_pa: f64,
        reynolds_length: f64,
        knudsen: f64,
    ) -> Result<Self, AerodbError> {
        let node = Self {
            mach,
            alpha_deg,
            beta_deg,
            dynamic_pressure_pa,
            reynolds_length,
            knudsen,
        };
        node.validate()?;
        Ok(node)
    }

    /// Convert this node to the runtime aero-method context.
    #[must_use]
    pub fn context(self) -> AeroContext {
        AeroContext {
            mach: self.mach,
            alpha_deg: self.alpha_deg,
            beta_deg: self.beta_deg,
            dynamic_pressure_pa: self.dynamic_pressure_pa,
            airspeed_m_s: 0.0,
            body_angular_rates_rad_s: Vector3::zeros(),
            reynolds_length: self.reynolds_length,
            knudsen: self.knudsen,
        }
    }

    fn validate(self) -> Result<(), AerodbError> {
        for (field, value) in [
            ("mach", self.mach),
            ("alpha_deg", self.alpha_deg),
            ("beta_deg", self.beta_deg),
            ("dynamic_pressure_pa", self.dynamic_pressure_pa),
            ("reynolds_length", self.reynolds_length),
            ("knudsen", self.knudsen),
        ] {
            if !value.is_finite() {
                return Err(AerodbError::InvalidInput { field });
            }
        }
        if self.mach < 0.0 {
            return Err(AerodbError::InvalidInput { field: "mach" });
        }
        if self.dynamic_pressure_pa <= 0.0 {
            return Err(AerodbError::InvalidInput {
                field: "dynamic_pressure_pa",
            });
        }
        if self.reynolds_length < 0.0 {
            return Err(AerodbError::InvalidInput {
                field: "reynolds_length",
            });
        }
        if self.knudsen < 0.0 {
            return Err(AerodbError::InvalidInput { field: "knudsen" });
        }
        Ok(())
    }
}

/// Axis grids used to generate a condition-only [`RunMatrix`].
#[derive(Clone, Debug, PartialEq)]
pub struct RunMatrixAxes {
    /// Mach grid.
    pub mach: Vec<f64>,
    /// Angle-of-attack grid in degrees.
    pub alpha_deg: Vec<f64>,
    /// Sideslip grid in degrees.
    pub beta_deg: Vec<f64>,
    /// Reynolds-number grid based on the method reference length.
    pub reynolds_length: Vec<f64>,
    /// Knudsen-number grid.
    pub knudsen: Vec<f64>,
    /// Dynamic-pressure grid in pascals.
    pub dynamic_pressure_pa: Vec<f64>,
}

impl RunMatrixAxes {
    /// Validate every axis.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when an axis is empty, non-finite,
    /// not strictly increasing, or outside its allowed envelope.
    pub fn validate(&self) -> Result<(), AerodbError> {
        validate_axis(&self.mach, "mach", AxisEnvelope::NonNegative)?;
        validate_axis(&self.alpha_deg, "alpha_deg", AxisEnvelope::AnyFinite)?;
        validate_axis(&self.beta_deg, "beta_deg", AxisEnvelope::AnyFinite)?;
        validate_axis(
            &self.reynolds_length,
            "reynolds_length",
            AxisEnvelope::NonNegative,
        )?;
        validate_axis(&self.knudsen, "knudsen", AxisEnvelope::NonNegative)?;
        validate_axis(
            &self.dynamic_pressure_pa,
            "dynamic_pressure_pa",
            AxisEnvelope::Positive,
        )?;
        Ok(())
    }
}

/// Deterministic condition-only run matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct RunMatrix {
    axes: RunMatrixAxes,
    nodes: Vec<RunNode>,
    seed: u64,
}

impl RunMatrix {
    /// Construct a deterministic run matrix from axis grids.
    ///
    /// Node order is row-major with Mach outermost and dynamic pressure
    /// innermost. When Reynolds / Knudsen / dynamic-pressure axes are singleton,
    /// this order is bit-identical to [`AeroDeck`] row-major `(mach, alpha,
    /// beta)` table order.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid axes or node-count
    /// overflow.
    pub fn from_axes(axes: RunMatrixAxes, seed: u64) -> Result<Self, AerodbError> {
        axes.validate()?;
        let capacity = matrix_node_count(&axes)?;
        let mut nodes = Vec::with_capacity(capacity);
        for &mach in &axes.mach {
            for &alpha_deg in &axes.alpha_deg {
                for &beta_deg in &axes.beta_deg {
                    for &reynolds_length in &axes.reynolds_length {
                        for &knudsen in &axes.knudsen {
                            for &dynamic_pressure_pa in &axes.dynamic_pressure_pa {
                                nodes.push(RunNode::new(
                                    mach,
                                    alpha_deg,
                                    beta_deg,
                                    dynamic_pressure_pa,
                                    reynolds_length,
                                    knudsen,
                                )?);
                            }
                        }
                    }
                }
            }
        }
        Ok(Self { axes, nodes, seed })
    }

    /// Parse a strict run-matrix TOML document.
    ///
    /// The parser uses `serde(deny_unknown_fields)` on every table. Fields such
    /// as `target`, `aimpoint`, or `range` therefore fail closed because the
    /// matrix is condition-only.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedToml`] for TOML syntax or schema errors,
    /// and [`AerodbError::InvalidInput`] for invalid axis values.
    pub fn load_toml_str(s: &str) -> Result<Self, AerodbError> {
        lint_condition_only_run_matrix_toml(s)?;
        let parsed: RunMatrixToml = toml::from_str(s).map_err(|_| AerodbError::MalformedToml {
            reason: "run-matrix TOML did not parse against schema",
        })?;
        if parsed.openbmp.aerodb_run_matrix != RUN_MATRIX_SCHEMA_VERSION_1 {
            return Err(AerodbError::MalformedToml {
                reason: "openbmp.aerodb_run_matrix schema version must be 1",
            });
        }
        Self::from_axes(parsed.grid.into_axes(), parsed.seed)
    }

    /// Borrow the source axes.
    #[must_use]
    pub const fn axes(&self) -> &RunMatrixAxes {
        &self.axes
    }

    /// Borrow the ordered run nodes.
    #[must_use]
    pub fn nodes(&self) -> &[RunNode] {
        &self.nodes
    }

    /// Deterministic seed associated with this matrix.
    #[must_use]
    pub const fn seed(&self) -> u64 {
        self.seed
    }
}

/// Lint run-matrix TOML for target/aimpoint/range vocabulary.
///
/// This is a condition-only guard that can be run before strict schema parsing
/// or future file/scenario integration. It scans TOML keys recursively and
/// rejects exact key tokens `target`, `aimpoint`, and `range`, including joined
/// forms such as `target_aimpoint_eci_m`.
///
/// # Errors
///
/// Returns [`AerodbError::MalformedToml`] when the document is not TOML and
/// [`AerodbError::ForbiddenVocabulary`] when a forbidden key is present.
pub fn lint_condition_only_run_matrix_toml(s: &str) -> Result<(), AerodbError> {
    let value: toml::Value = toml::from_str(s).map_err(|_| AerodbError::MalformedToml {
        reason: "run-matrix TOML did not parse",
    })?;
    lint_toml_value_for_condition_only_keys(&value, "")
}

/// Builder for reduced aerodynamic coefficient decks.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DeckBuilder {
    /// Reference area used for coefficient normalization.
    pub reference_area_m2: f64,
    /// Reference length used for pitch-moment normalization.
    pub reference_length_m: f64,
}

/// One calibrated six-coefficient validation point for an [`AeroDeck`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SixCoefficientValidationCase<'a> {
    /// Stable validation-case identifier.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for the calibrated reference.
    pub provenance: &'a str,
    /// Query Mach number.
    pub mach: f64,
    /// Query angle of attack in degrees.
    pub alpha_deg: f64,
    /// Query sideslip angle in degrees.
    pub beta_deg: f64,
    /// Expected calibrated body-axis six coefficients.
    pub expected: AeroSixCoefficients,
    /// Per-channel absolute tolerances.
    pub tolerance: AeroSixCoefficients,
}

/// Calibrated six-coefficient validation report.
#[derive(Clone, Debug, PartialEq)]
pub struct SixCoefficientValidationReport {
    /// Validation rows in caller-supplied order.
    pub cases: Vec<SixCoefficientValidationRow>,
}

/// One calibrated six-coefficient validation row.
#[derive(Clone, Debug, PartialEq)]
pub struct SixCoefficientValidationRow {
    /// Stable validation-case identifier.
    pub case_id: String,
    /// Human-readable provenance pointer for the calibrated reference.
    pub provenance: String,
    /// Query Mach number.
    pub mach: f64,
    /// Query angle of attack in degrees.
    pub alpha_deg: f64,
    /// Query sideslip angle in degrees.
    pub beta_deg: f64,
    /// Expected calibrated body-axis six coefficients.
    pub expected: AeroSixCoefficients,
    /// Actual deck lookup body-axis six coefficients.
    pub actual: AeroSixCoefficients,
    /// Absolute per-channel error.
    pub absolute_error: AeroSixCoefficients,
    /// Per-channel absolute tolerances.
    pub tolerance: AeroSixCoefficients,
}

/// One SU2 `history.csv` coefficient source for a run-matrix node.
///
/// SU2 writes customizable history output; this adapter consumes the recorded
/// CSV text for one completed solver run and extracts the final coefficient row.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Su2HistoryCoefficientCsv<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the SU2 history CSV file.
    pub csv_text: &'a str,
}

/// One SU2 surface CSV coefficient source for a run-matrix node.
///
/// The adapter expects recorded SU2-style surface output with finite point
/// coordinates and pressure coefficient columns. Rows are mapped to the supplied
/// [`PanelMesh`] vertices by exact finite coordinates, and triangle Cp is the
/// average of its three vertex values.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Su2SurfaceCpCsv<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the SU2 surface CSV file.
    pub csv_text: &'a str,
}

/// One Cart3D ASCII `.triq` surface Cp source for a run-matrix node.
///
/// Cart3D TRI/Q files carry a surface triangulation plus scalar state values at
/// vertices. This adapter consumes recorded ASCII `.triq` text, validates that
/// the geometry matches the supplied [`PanelMesh`], uses the first per-vertex
/// scalar as Cp, and integrates triangle Cp from the average of its vertices.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cart3dTriqSurfaceCp<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the Cart3D ASCII `.triq` file.
    pub triq_text: &'a str,
}

/// One OpenFOAM sampled-surface Cp source for a run-matrix node.
///
/// OpenFOAM sampled surfaces can be exported as whitespace-delimited raw point
/// rows. This adapter consumes recorded rows with finite `x y z Cp` fields,
/// maps them to [`PanelMesh`] vertices by exact finite coordinates, and
/// integrates triangle Cp from the average of each triangle's vertices.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OpenFoamSurfaceCp<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the OpenFOAM sampled-surface Cp text.
    pub text: &'a str,
}

/// One OpenFOAM boundary scalar-field Cp source for a run-matrix node.
///
/// This adapter consumes a recorded OpenFOAM scalar field dictionary, selects a
/// named boundary patch, and interprets that patch's scalar `value` as one Cp
/// value per [`PanelMesh`] triangle in mesh order.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OpenFoamBoundaryCpField<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Boundary patch name whose scalar values are consumed.
    pub patch_name: &'a str,
    /// Contents of the OpenFOAM scalar field dictionary.
    pub field_text: &'a str,
}

/// Parsed OpenFOAM scalar field dictionary.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamScalarField {
    /// OpenFOAM field class, e.g. `volScalarField` or `surfaceScalarField`.
    pub class_name: String,
    /// OpenFOAM field object name, e.g. `Cp`.
    pub object_name: String,
    /// Seven OpenFOAM dimension exponents from the `dimensions` entry.
    pub dimensions: [f64; 7],
    /// Parsed `internalField` scalar data.
    pub internal_field: OpenFoamScalarFieldValue,
    /// Parsed boundary patches keyed by patch name.
    pub boundary_patches: BTreeMap<String, OpenFoamScalarPatch>,
}

/// OpenFOAM scalar field value.
#[derive(Clone, Debug, PartialEq)]
pub enum OpenFoamScalarFieldValue {
    /// `uniform <scalar>`.
    Uniform(f64),
    /// `nonuniform List<scalar> <n> (...)`.
    Nonuniform(Vec<f64>),
}

/// Parsed OpenFOAM boundary patch scalar data.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamScalarPatch {
    /// Patch `type` entry.
    pub patch_type: String,
    /// Optional patch `value` entry.
    pub value: Option<OpenFoamScalarFieldValue>,
}

/// Parsed OpenFOAM vector field dictionary.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamVectorField {
    /// OpenFOAM field class, e.g. `volVectorField`.
    pub class_name: String,
    /// OpenFOAM field object name, e.g. `U`.
    pub object_name: String,
    /// Seven OpenFOAM dimension exponents from the `dimensions` entry.
    pub dimensions: [f64; 7],
    /// Parsed `internalField` vector data.
    pub internal_field: OpenFoamVectorFieldValue,
    /// Parsed boundary patches keyed by patch name.
    pub boundary_patches: BTreeMap<String, OpenFoamVectorPatch>,
}

/// OpenFOAM vector field value.
#[derive(Clone, Debug, PartialEq)]
pub enum OpenFoamVectorFieldValue {
    /// `uniform (<x> <y> <z>)`.
    Uniform(Vector3<f64>),
    /// `nonuniform List<vector> <n> ((<x> <y> <z>) ...)`.
    Nonuniform(Vec<Vector3<f64>>),
}

/// Parsed OpenFOAM boundary patch vector data.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamVectorPatch {
    /// Patch `type` entry.
    pub patch_type: String,
    /// Optional patch `value` entry.
    pub value: Option<OpenFoamVectorFieldValue>,
}

/// Recorded OpenFOAM case field file declaration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct OpenFoamCaseFieldFile<'a> {
    /// Expected OpenFOAM `FoamFile.object` name.
    pub object_name: &'a str,
    /// Path to the recorded field dictionary relative to the case root.
    pub relative_path: &'a str,
}

/// Parsed set of recorded OpenFOAM case field dictionaries.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamRecordedCaseFields {
    /// Parsed scalar fields keyed by OpenFOAM object name.
    pub scalar_fields: BTreeMap<String, OpenFoamScalarField>,
    /// Parsed vector fields keyed by OpenFOAM object name.
    pub vector_fields: BTreeMap<String, OpenFoamVectorField>,
}

/// Parsed OpenFOAM `constant/polyMesh/boundary` metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamPolyMeshBoundary {
    /// OpenFOAM boundary file class, expected to be `polyBoundaryMesh`.
    pub class_name: String,
    /// OpenFOAM boundary file object, expected to be `boundary`.
    pub object_name: String,
    /// Parsed boundary patches keyed by patch name.
    pub patches: BTreeMap<String, OpenFoamBoundaryPatchTopology>,
}

/// OpenFOAM boundary patch topology metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OpenFoamBoundaryPatchTopology {
    /// Mesh boundary patch `type` entry.
    pub patch_type: String,
    /// Number of boundary faces in the patch.
    pub n_faces: usize,
    /// Start face index in OpenFOAM face order.
    pub start_face: usize,
}

/// Parsed OpenFOAM `constant/polyMesh` connectivity metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamPolyMeshConnectivity {
    /// Point coordinates from `constant/polyMesh/points`.
    pub points: Vec<Vector3<f64>>,
    /// Faces as point-index loops from `constant/polyMesh/faces`.
    pub faces: Vec<Vec<usize>>,
    /// Owner cell labels, one per face.
    pub owner: Vec<usize>,
    /// Neighbour cell labels, one per internal face.
    pub neighbour: Vec<usize>,
}

/// Derived OpenFOAM `constant/polyMesh` face and cell geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamPolyMeshGeometry {
    /// Reconstructed face geometry in OpenFOAM face order.
    pub faces: Vec<OpenFoamPolyMeshFaceGeometry>,
    /// Reconstructed cell volumes in inferred cell-label order.
    pub cell_volumes_m3: Vec<f64>,
}

/// Derived OpenFOAM face geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamPolyMeshFaceGeometry {
    /// Area-weighted face centroid from fan triangulation.
    pub centroid: Vector3<f64>,
    /// Oriented face area vector from face winding.
    pub area_vector: Vector3<f64>,
    /// Scalar face area from fan triangulation.
    pub area_m2: f64,
    /// Unit normal from the oriented face area vector.
    pub unit_normal: Vector3<f64>,
}

/// Parsed optional OpenFOAM `constant/polyMesh` zone metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OpenFoamPolyMeshZones {
    /// Point zones keyed by zone name.
    pub point_zones: BTreeMap<String, OpenFoamPolyMeshZone>,
    /// Face zones keyed by zone name.
    pub face_zones: BTreeMap<String, OpenFoamPolyMeshZone>,
    /// Cell zones keyed by zone name.
    pub cell_zones: BTreeMap<String, OpenFoamPolyMeshZone>,
}

/// OpenFOAM point/face/cell zone membership metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OpenFoamPolyMeshZone {
    /// Mesh zone `type` entry.
    pub zone_type: String,
    /// Zone point, face, or cell labels.
    pub labels: Vec<usize>,
    /// Optional face-zone flip map, present for face zones in many OpenFOAM files.
    pub flip_map: Option<Vec<bool>>,
}

/// Parsed OpenFOAM `constant/turbulenceProperties` simulation type.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OpenFoamTurbulenceSimulationType {
    /// `simulationType laminar`.
    Laminar,
    /// `simulationType RAS`.
    Ras,
    /// `simulationType LES`.
    Les,
}

/// Parsed OpenFOAM `constant/turbulenceProperties` metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenFoamTurbulenceProperties {
    /// OpenFOAM dictionary class, expected to be `dictionary`.
    pub class_name: String,
    /// OpenFOAM dictionary object, expected to be `turbulenceProperties`.
    pub object_name: String,
    /// Parsed top-level `simulationType`.
    pub simulation_type: OpenFoamTurbulenceSimulationType,
    /// Parsed `RASModel` or `LESModel` for turbulent cases.
    pub model_name: Option<String>,
    /// Parsed `turbulence` flag for RAS/LES dictionaries.
    pub turbulence: Option<bool>,
    /// Parsed `printCoeffs` flag for RAS/LES dictionaries.
    pub print_coeffs: Option<bool>,
    /// Parsed scalar entries from model `*Coeffs` dictionaries.
    pub model_coefficients: BTreeMap<String, f64>,
}

/// Expected calibrated OpenFOAM turbulence-model coefficient value.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OpenFoamTurbulenceCoefficientExpectation<'a> {
    /// Coefficient name, such as `betaStar` or `a1`.
    pub name: &'a str,
    /// Expected calibrated scalar value.
    pub expected: f64,
    /// Allowed absolute tolerance.
    pub tolerance: f64,
}

/// One SPARTA surface-force source for a run-matrix node.
///
/// SPARTA `compute surf` can emit per-surface-element force components `fx`,
/// `fy`, and `fz`. This adapter consumes recorded whitespace-delimited rows
/// keyed by surface element id, maps those rows to [`PanelMesh`] triangles, and
/// integrates force/moment loads directly.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpartaSurfaceForces<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the recorded SPARTA surface-force table.
    pub text: &'a str,
}

/// One OpenFOAM `forceCoeffs` source for a run-matrix node.
///
/// OpenFOAM writes whitespace-delimited coefficient tables under
/// `postProcessing/forceCoeffs`. This adapter consumes one recorded table for a
/// completed solver run and extracts the final coefficient row.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OpenFoamForceCoeffs<'a> {
    /// Expected run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier for provenance.
    pub case_id: &'a str,
    /// Human-readable provenance pointer for this solver run.
    pub provenance: &'a str,
    /// Contents of the OpenFOAM force-coefficients table.
    pub text: &'a str,
}

/// Integrated load report for one run-matrix node from normalized surface-Cp
/// input.
///
/// `force_coefficient_body = force_n_body / (q S)` and
/// `moment_coefficient_body = moment_n_m_body / (q S L)`, so the reduced deck
/// coefficients are `CD = -Cx`, `CN = -Cz`, and `CM = Cm_pitch`.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceCpLoadReport {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Integrated body-frame force over all surface-Cp panels (N).
    pub force_n_body: Vector3<f64>,
    /// Integrated body-frame moment about `cg_body_m` (N m).
    pub moment_n_m_body: Vector3<f64>,
    /// Moment reference point in body-frame metres.
    pub cg_body_m: Vector3<f64>,
    /// Body-frame force coefficients `(Cx, Cy, Cz)`.
    pub force_coefficient_body: Vector3<f64>,
    /// Body-frame moment coefficients `(Cl, Cm, Cn)`.
    pub moment_coefficient_body: Vector3<f64>,
    /// Reduced `(CN, CD, CM)` coefficients used by the current [`AeroDeck`].
    pub reduced_coefficients: AeroCoefficients,
    /// Optional integrated axial station loads, one per adjacent station pair.
    pub axial_line_loads: Vec<AxialLineLoad>,
}

/// Recorded solver artifact kind for one run-matrix node.
#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SolverCaseOutputKind {
    /// Strict normalized CN/CD/CM coefficient CSV.
    NormalizedCoefficientsCsv,
    /// Strict normalized body-axis CX/CY/CZ/Cl/Cm/Cn coefficient CSV.
    NormalizedSixCoefficientsCsv,
    /// Strict normalized surface-Cp CSV over a [`PanelMesh`].
    SurfaceCpCsv,
    /// Recorded SU2 `history.csv` coefficient file.
    Su2HistoryCsv,
    /// Recorded SU2 surface CSV with vertex pressure coefficients.
    Su2SurfaceCsv,
    /// Recorded Cart3D ASCII `.triq` surface Cp file.
    Cart3dTriq,
    /// Recorded OpenFOAM sampled-surface Cp raw text.
    OpenFoamSurfaceCp,
    /// Recorded OpenFOAM scalar field dictionary carrying boundary Cp values.
    OpenFoamBoundaryCpField,
    /// Recorded OpenFOAM scalar field dictionary.
    OpenFoamScalarField,
    /// Recorded OpenFOAM vector field dictionary.
    OpenFoamVectorField,
    /// Recorded SPARTA per-surface-element force table.
    SpartaSurfaceForces,
    /// Recorded OpenFOAM `forceCoeffs` table.
    OpenFoamForceCoeffs,
}

/// One recorded solver artifact declaration for a run-matrix node.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCaseArtifact {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Run-matrix node echoed by the manifest.
    pub node: RunNode,
    /// Solver label, e.g. `su2-euler` or `openfoam-rans`.
    pub solver: String,
    /// Stable case identifier.
    pub case_id: String,
    /// Declared output artifact kind.
    pub output_kind: SolverCaseOutputKind,
    /// Relative or repository-local artifact path recorded by the manifest.
    pub output_file: String,
    /// Expected SHA-256 digest for `output_file`, as 64 hex characters.
    pub output_sha256: String,
    /// Human-readable provenance pointer for this recorded solver run.
    pub provenance: String,
}

/// Strict subprocess command plan for recorded external-solver cases.
///
/// This is orchestration metadata only. It records argv/env/working-directory
/// intent plus the expected output artifact pin for each run-matrix node, but
/// it does not execute a process or interpret shell strings.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandPlan {
    cases: Vec<SolverCommandCase>,
}

/// One planned external-solver subprocess case aligned to a run-matrix node.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandCase {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Run-matrix node echoed by the plan.
    pub node: RunNode,
    /// Solver label, e.g. `su2-euler` or `openfoam-rans`.
    pub solver: String,
    /// Stable case identifier.
    pub case_id: String,
    /// Program path or executable name passed directly to `std::process::Command`.
    pub program: String,
    /// Argument vector passed directly to `std::process::Command`; no shell is used.
    pub args: Vec<String>,
    /// Environment key/value overrides for the subprocess.
    pub env: BTreeMap<String, String>,
    /// Working directory for the solver case.
    pub working_dir: String,
    /// Per-case timeout, in seconds.
    pub timeout_s: u64,
    /// Expected output artifact kind.
    pub output_kind: SolverCaseOutputKind,
    /// Relative or repository-local artifact path expected after execution.
    pub output_file: String,
    /// Expected SHA-256 digest for `output_file`, as 64 hex characters.
    pub output_sha256: String,
    /// Human-readable provenance pointer for this planned solver run.
    pub provenance: String,
}

/// Execution report for a completed solver command plan.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandExecutionReport {
    cases: Vec<SolverCommandCaseExecution>,
}

/// Execution evidence for one completed solver command case.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandCaseExecution {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier.
    pub case_id: String,
    /// Process exit code when the platform reported one.
    pub exit_code: Option<i32>,
    /// Captured stdout byte count.
    pub stdout_len: usize,
    /// SHA-256 digest of captured stdout bytes.
    pub stdout_sha256: String,
    /// Captured stderr byte count.
    pub stderr_len: usize,
    /// SHA-256 digest of captured stderr bytes.
    pub stderr_sha256: String,
    /// Output artifact path that was verified.
    pub output_file: String,
    /// SHA-256 digest verified for `output_file`.
    pub output_sha256: String,
}

/// Bounded retry policy for direct solver command execution.
///
/// The policy only affects process execution failures. Static validation
/// failures such as unsafe paths are never retried.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SolverCommandRetryPolicy {
    /// Maximum attempts per case, including the first attempt.
    pub max_attempts: u32,
    /// Retry when the subprocess fails to spawn.
    pub retry_on_spawn_failure: bool,
    /// Retry when the subprocess exceeds its per-case timeout.
    pub retry_on_timeout: bool,
    /// Retry when the subprocess exits with a nonzero status.
    pub retry_on_nonzero_status: bool,
    /// Retry when the output artifact is missing or fails SHA verification.
    pub retry_on_output_verification: bool,
}

/// Execution report for a completed solver command plan with retry evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandRetryExecutionReport {
    cases: Vec<SolverCommandCaseRetryExecution>,
}

/// Retry execution evidence for one completed solver command case.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCommandCaseRetryExecution {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier.
    pub case_id: String,
    /// Attempt records, including failed attempts before the final success.
    pub attempts: Vec<SolverCommandCaseAttempt>,
    /// Successful final execution evidence.
    pub execution: SolverCommandCaseExecution,
}

/// One direct command attempt inside a retrying execution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SolverCommandCaseAttempt {
    /// One-based attempt index.
    pub attempt_index: u32,
    /// Whether this attempt succeeded.
    pub success: bool,
    /// Executor failure field when the attempt failed.
    pub failure_field: Option<&'static str>,
}

/// External batch scheduler targeted by a rendered solver schedule.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SolverClusterScheduler {
    /// Slurm `sbatch` array script.
    Slurm,
}

/// Template for rendering a solver command plan as a cluster scheduler script.
///
/// The template records scheduling intent only. Rendering a script does not
/// submit it to a queue, execute a solver, or collect artifacts.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterScheduleTemplate {
    /// Target scheduler.
    pub scheduler: SolverClusterScheduler,
    /// Scheduler job name.
    pub job_name: String,
    /// Relative scheduler script filename.
    pub script_filename: String,
    /// Relative directory for scheduler stdout/stderr logs.
    pub log_dir: String,
    /// MPI ranks requested for each run-matrix case.
    pub mpi_ranks_per_case: u32,
    /// CPU threads requested per MPI task.
    pub cpus_per_task: u32,
    /// Slurm wall-clock limit for each array task, in seconds.
    pub wall_time_s: u64,
    /// Optional Slurm account.
    pub account: Option<String>,
    /// Optional Slurm partition.
    pub partition: Option<String>,
}

/// Rendered cluster scheduler script for a solver command plan.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterSchedule {
    /// Target scheduler.
    pub scheduler: SolverClusterScheduler,
    /// Number of run-matrix cases covered by the schedule.
    pub case_count: usize,
    /// Relative scheduler script filename.
    pub script_filename: String,
    /// Command vector a caller can use to submit the script.
    pub submit_command: Vec<String>,
    /// Rendered scheduler script text.
    pub script_text: String,
    /// SHA-256 digest for `script_text`.
    pub script_sha256: String,
}

/// Evidence returned after submitting a rendered solver schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterSubmissionReport {
    /// Target scheduler.
    pub scheduler: SolverClusterScheduler,
    /// Scheduler job id parsed from the submit command.
    pub job_id: String,
    /// Relative scheduler script filename that was submitted.
    pub script_filename: String,
    /// SHA-256 digest for the submitted script text.
    pub script_sha256: String,
    /// Captured submit stdout byte count.
    pub stdout_len: usize,
    /// SHA-256 digest of submit stdout bytes.
    pub stdout_sha256: String,
    /// Captured submit stderr byte count.
    pub stderr_len: usize,
    /// SHA-256 digest of submit stderr bytes.
    pub stderr_sha256: String,
}

/// Coarse cluster job state parsed from scheduler output.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SolverClusterJobStatus {
    /// Queued but not running.
    Pending,
    /// Running on allocated resources.
    Running,
    /// Scheduler reports the job is completing.
    Completing,
    /// Completed successfully.
    Completed,
    /// Failed or scheduler-reported unsuccessful terminal state.
    Failed,
    /// Cancelled before successful completion.
    Cancelled,
    /// Timed out under the scheduler wall-time limit.
    Timeout,
    /// Scheduler output did not match a known state token.
    Unknown,
}

/// Evidence returned after querying a scheduler job state.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterJobStateReport {
    /// Target scheduler.
    pub scheduler: SolverClusterScheduler,
    /// Scheduler job id queried.
    pub job_id: String,
    /// Parsed coarse job status.
    pub status: SolverClusterJobStatus,
    /// Raw scheduler state token used for parsing.
    pub raw_state: String,
    /// Captured query stdout byte count.
    pub stdout_len: usize,
    /// SHA-256 digest of query stdout bytes.
    pub stdout_sha256: String,
    /// Captured query stderr byte count.
    pub stderr_len: usize,
    /// SHA-256 digest of query stderr bytes.
    pub stderr_sha256: String,
}

/// Bounded scheduler polling policy.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SolverClusterPollingPolicy {
    /// Maximum scheduler state queries before giving up.
    pub max_polls: u32,
    /// Sleep interval between non-terminal polls.
    pub poll_interval_ms: u64,
    /// Timeout for each scheduler state query command.
    pub query_timeout_s: u64,
}

/// Evidence returned after a scheduler job reaches successful completion.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterCompletionReport {
    /// Target scheduler.
    pub scheduler: SolverClusterScheduler,
    /// Scheduler job id that was polled.
    pub job_id: String,
    /// Final successful scheduler status.
    pub final_status: SolverClusterJobStatus,
    /// Ordered scheduler state samples captured during polling.
    pub polls: Vec<SolverClusterJobStateReport>,
}

/// SHA-verified output artifacts collected after external solver execution.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterArtifactCollectionReport {
    cases: Vec<SolverClusterArtifactEvidence>,
}

/// Verified output artifact evidence for one solver command case.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverClusterArtifactEvidence {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Stable case identifier.
    pub case_id: String,
    /// Expected artifact kind.
    pub output_kind: SolverCaseOutputKind,
    /// Relative output artifact path.
    pub output_file: String,
    /// Output artifact byte count.
    pub output_len: usize,
    /// Verified output artifact SHA-256 digest.
    pub output_sha256: String,
}

/// Site-specific command used to stage remote solver artifacts under a root.
///
/// The command is executed directly from the solver root without a shell. It is
/// expected to copy or materialize the output files already declared by a
/// [`SolverCommandPlan`]; the crate verifies those files by SHA-256 after the
/// command completes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SolverRemoteArtifactTransferPlan {
    /// Command vector executed under the solver root.
    pub command: Vec<String>,
    /// Transfer command timeout, in seconds.
    pub timeout_s: u64,
}

/// Evidence returned after staging and verifying remote solver artifacts.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverRemoteArtifactTransferReport {
    /// Command vector that was executed under the solver root.
    pub command: Vec<String>,
    /// Captured transfer stdout byte count.
    pub stdout_len: usize,
    /// SHA-256 digest of transfer stdout bytes.
    pub stdout_sha256: String,
    /// Captured transfer stderr byte count.
    pub stderr_len: usize,
    /// SHA-256 digest of transfer stderr bytes.
    pub stderr_sha256: String,
    /// SHA-verified artifacts collected after the transfer command completed.
    pub artifacts: SolverClusterArtifactCollectionReport,
}

/// Template for deterministic SU2-Euler case generation over a [`RunMatrix`].
///
/// The template renders input configuration files and command metadata only. It
/// does not launch SU2, infer solver output hashes, or make solver artifacts a
/// runtime force source.
#[derive(Clone, Debug, PartialEq)]
pub struct Su2EulerCaseTemplate {
    /// Program path or executable name intended for direct command execution.
    pub program: String,
    /// Relative SU2 mesh filename written into each generated config.
    pub mesh_filename: String,
    /// SU2 marker used for Euler, plotting, and monitoring output.
    pub surface_marker: String,
    /// Relative parent directory for generated case directories.
    pub case_dir_prefix: String,
    /// Prefix used to create stable case identifiers as `<prefix>-<node_index>`.
    pub case_id_prefix: String,
    /// Config filename inside every case directory.
    pub config_filename: String,
    /// Expected SU2 history output filename inside every case directory.
    pub history_filename: String,
    /// Expected SU2 surface output filename inside every case directory.
    pub surface_filename: String,
    /// Per-case timeout, in seconds, for downstream command-plan construction.
    pub timeout_s: u64,
    /// SU2 reference area in square metres.
    pub reference_area_m2: f64,
    /// SU2 reference length in metres.
    pub reference_length_m: f64,
    /// SU2 moment reference origin in body-frame metres.
    pub moment_reference_m: Vector3<f64>,
    /// Additional SU2 config key/value pairs appended after protected keys.
    pub extra_config: BTreeMap<String, String>,
    /// Extra argv entries after the generated config filename.
    pub extra_args: Vec<String>,
    /// Environment overrides intended for direct command execution.
    pub env: BTreeMap<String, String>,
    /// Human-readable provenance pointer for generated cases.
    pub provenance: String,
}

/// Rendered SU2-Euler cases aligned to a [`RunMatrix`].
#[derive(Clone, Debug, PartialEq)]
pub struct Su2EulerCaseBundle {
    cases: Vec<Su2EulerCase>,
}

/// One rendered SU2-Euler case aligned to a run-matrix node.
#[derive(Clone, Debug, PartialEq)]
pub struct Su2EulerCase {
    /// Run-matrix node index.
    pub node_index: usize,
    /// Run-matrix node echoed by the generated case.
    pub node: RunNode,
    /// Solver label, always `su2-euler` for this generator.
    pub solver: String,
    /// Stable case identifier.
    pub case_id: String,
    /// Program path or executable name intended for direct command execution.
    pub program: String,
    /// Argument vector beginning with `config_filename`; no shell syntax.
    pub args: Vec<String>,
    /// Environment key/value overrides for downstream execution.
    pub env: BTreeMap<String, String>,
    /// Relative working directory for this case.
    pub working_dir: String,
    /// Relative generated config path.
    pub config_file: String,
    /// Generated SU2 config text.
    pub config_text: String,
    /// SHA-256 digest for `config_text`.
    pub config_sha256: String,
    /// Relative expected SU2 history output path.
    pub history_file: String,
    /// Relative expected SU2 surface output path.
    pub surface_file: String,
    /// Per-case timeout, in seconds.
    pub timeout_s: u64,
    /// Human-readable provenance pointer for this generated case.
    pub provenance: String,
}

/// Strict recorded-solver case manifest aligned to a [`RunMatrix`].
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCaseManifest {
    cases: Vec<SolverCaseArtifact>,
}

impl SolverCommandPlan {
    /// Parse and validate a strict solver command-plan TOML document.
    ///
    /// The plan is metadata only: it validates a deterministic argv/env
    /// subprocess plan and the expected output artifact pins. It does not launch
    /// a solver, evaluate shell commands, or make any artifact a runtime force
    /// source.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedToml`] for TOML/schema failures and
    /// [`AerodbError::InvalidInput`] for row count, node order, condition echo,
    /// blank command/provenance/path fields, malformed env entries, zero
    /// timeout, or malformed SHA-256 pins.
    pub fn load_toml_str(matrix: &RunMatrix, s: &str) -> Result<Self, AerodbError> {
        let parsed: SolverCommandPlanToml =
            toml::from_str(s).map_err(|_| AerodbError::MalformedToml {
                reason: "solver command plan TOML did not match schema",
            })?;
        if parsed.openbmp.aerodb_solver_command_plan != SOLVER_COMMAND_PLAN_SCHEMA_VERSION_1 {
            return Err(AerodbError::MalformedToml {
                reason: "unsupported solver command plan schema version",
            });
        }
        let expected = matrix.nodes().len();
        if parsed.cases.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_plan_rows",
            });
        }
        let mut cases = Vec::with_capacity(expected);
        for (record_index, case) in parsed.cases.into_iter().enumerate() {
            let node = matrix.nodes()[record_index];
            cases.push(case.into_case(record_index, node)?);
        }
        Ok(Self { cases })
    }

    /// Planned subprocess cases in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[SolverCommandCase] {
        &self.cases
    }

    /// Project the expected output pins into the existing recorded-artifact
    /// manifest shape.
    #[must_use]
    pub fn expected_artifact_manifest(&self) -> SolverCaseManifest {
        SolverCaseManifest {
            cases: self
                .cases
                .iter()
                .map(|case| SolverCaseArtifact {
                    node_index: case.node_index,
                    node: case.node,
                    solver: case.solver.clone(),
                    case_id: case.case_id.clone(),
                    output_kind: case.output_kind,
                    output_file: case.output_file.clone(),
                    output_sha256: case.output_sha256.clone(),
                    provenance: case.provenance.clone(),
                })
                .collect(),
        }
    }

    /// Execute every planned command directly with `std::process::Command`.
    ///
    /// `root` is used to resolve relative working-directory and output-file
    /// paths. The executor rejects absolute paths and `..` components, applies
    /// per-case argv and environment entries without invoking a shell, captures
    /// stdout/stderr, enforces each case timeout, requires a successful exit
    /// status, and verifies the declared output artifact SHA-256 pin.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for unsafe paths, spawn/wait/kill
    /// failures, timeouts, nonzero exits, missing output artifacts, or output
    /// digest mismatches.
    pub fn execute_with_root(
        &self,
        root: impl AsRef<Path>,
    ) -> Result<SolverCommandExecutionReport, AerodbError> {
        let root = root.as_ref();
        if !root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_execute_root",
            });
        }
        let mut cases = Vec::with_capacity(self.cases.len());
        for case in &self.cases {
            cases.push(execute_solver_command_case(root, case)?);
        }
        Ok(SolverCommandExecutionReport { cases })
    }

    /// Execute every planned command with a bounded retry policy.
    ///
    /// Each case is attempted up to `policy.max_attempts`. Permanent validation
    /// failures such as unsafe working/output paths are not retryable; only the
    /// failure classes enabled by `policy` can trigger a retry. The returned
    /// report includes failed attempt fields followed by the successful final
    /// attempt for each completed case.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for an invalid policy, invalid
    /// root, a non-retryable failure, or a retryable failure that exhausts the
    /// configured attempt budget.
    pub fn execute_with_root_retrying(
        &self,
        root: impl AsRef<Path>,
        policy: SolverCommandRetryPolicy,
    ) -> Result<SolverCommandRetryExecutionReport, AerodbError> {
        policy.validate()?;
        let root = root.as_ref();
        if !root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_execute_root",
            });
        }
        let mut cases = Vec::with_capacity(self.cases.len());
        for case in &self.cases {
            cases.push(execute_solver_command_case_retrying(root, case, policy)?);
        }
        Ok(SolverCommandRetryExecutionReport { cases })
    }

    /// Render this command plan as a deterministic cluster scheduler script.
    ///
    /// Rendering is offline scheduling metadata only. It validates scheduler
    /// settings and relative case paths, emits a SHA-pinned script artifact,
    /// and returns the submit command vector, but it does not call `sbatch`,
    /// launch MPI ranks, execute solvers, or collect outputs.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid scheduler settings,
    /// unsafe case paths, or an empty command plan.
    pub fn render_cluster_schedule(
        &self,
        template: &SolverClusterScheduleTemplate,
    ) -> Result<SolverClusterSchedule, AerodbError> {
        template.validate()?;
        if self.cases.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_cases",
            });
        }
        for case in &self.cases {
            validate_solver_relative_path(
                &case.working_dir,
                "solver_cluster_schedule_working_dir",
            )?;
            validate_solver_relative_path(
                &case.output_file,
                "solver_cluster_schedule_output_file",
            )?;
        }
        let script_text = match template.scheduler {
            SolverClusterScheduler::Slurm => render_slurm_array_script(self, template),
        };
        Ok(SolverClusterSchedule {
            scheduler: template.scheduler,
            case_count: self.cases.len(),
            script_filename: template.script_filename.clone(),
            submit_command: match template.scheduler {
                SolverClusterScheduler::Slurm => {
                    vec!["sbatch".to_owned(), template.script_filename.clone()]
                }
            },
            script_sha256: sha256_hex(script_text.as_bytes()),
            script_text,
        })
    }

    /// Collect and verify every expected output artifact under a caller root.
    ///
    /// This is the cluster-output counterpart to direct execution's output
    /// verification: it reads the output files declared by the command plan,
    /// rejects unsafe paths, and verifies each SHA-256 pin before returning
    /// per-case evidence.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for missing roots, unsafe output
    /// paths, missing output files, or SHA-256 mismatches.
    pub fn collect_expected_outputs_with_root(
        &self,
        root: impl AsRef<Path>,
    ) -> Result<SolverClusterArtifactCollectionReport, AerodbError> {
        let root = root.as_ref();
        if !root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_artifact_root",
            });
        }
        let mut cases = Vec::with_capacity(self.cases.len());
        for case in &self.cases {
            let output_path = resolve_solver_plan_path(
                root,
                &case.output_file,
                "solver_cluster_artifact_output_file",
            )?;
            let output_bytes = fs::read(&output_path).map_err(|_| AerodbError::InvalidInput {
                field: "solver_cluster_artifact_output_file",
            })?;
            let output_sha256 = sha256_hex(&output_bytes);
            if !output_sha256.eq_ignore_ascii_case(&case.output_sha256) {
                return Err(AerodbError::InvalidInput {
                    field: "solver_cluster_artifact_output_sha256",
                });
            }
            cases.push(SolverClusterArtifactEvidence {
                node_index: case.node_index,
                case_id: case.case_id.clone(),
                output_kind: case.output_kind,
                output_file: case.output_file.clone(),
                output_len: output_bytes.len(),
                output_sha256,
            });
        }
        Ok(SolverClusterArtifactCollectionReport { cases })
    }

    /// Run a site-specific artifact transfer command, then verify outputs.
    ///
    /// The transfer command is supplied by the caller so a deployment can use a
    /// local staging script, `rsync`, `scp`, object-store tooling, or a cluster
    /// wrapper without this crate embedding a remote backend. The command is
    /// executed directly under `root` without a shell, and the expected output
    /// files declared by this command plan are collected and SHA-verified after
    /// it succeeds.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid roots, invalid command
    /// vectors, timeouts, nonzero transfer exits, missing output artifacts, or
    /// output digest mismatches.
    pub fn transfer_expected_outputs_with_root(
        &self,
        root: impl AsRef<Path>,
        transfer: &SolverRemoteArtifactTransferPlan,
    ) -> Result<SolverRemoteArtifactTransferReport, AerodbError> {
        let root = root.as_ref();
        let output = run_cluster_command(
            root,
            &transfer.command,
            transfer.timeout_s,
            &SOLVER_REMOTE_ARTIFACT_TRANSFER_FIELDS,
        )?;
        let artifacts = self.collect_expected_outputs_with_root(root)?;
        Ok(SolverRemoteArtifactTransferReport {
            command: transfer.command.clone(),
            stdout_len: output.stdout.len(),
            stdout_sha256: sha256_hex(&output.stdout),
            stderr_len: output.stderr.len(),
            stderr_sha256: sha256_hex(&output.stderr),
            artifacts,
        })
    }
}

impl SolverCommandExecutionReport {
    /// Execution evidence in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[SolverCommandCaseExecution] {
        &self.cases
    }
}

impl SolverCommandRetryPolicy {
    /// Construct a retry policy for nonzero status and timeout failures.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when `max_attempts` is zero or
    /// exceeds the hard safety cap.
    pub fn transient_failures(max_attempts: u32) -> Result<Self, AerodbError> {
        let policy = Self {
            max_attempts,
            retry_on_spawn_failure: false,
            retry_on_timeout: true,
            retry_on_nonzero_status: true,
            retry_on_output_verification: false,
        };
        policy.validate()?;
        Ok(policy)
    }

    fn validate(self) -> Result<(), AerodbError> {
        if self.max_attempts == 0 || self.max_attempts > MAX_SOLVER_COMMAND_RETRY_ATTEMPTS {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_retry_max_attempts",
            });
        }
        Ok(())
    }

    fn should_retry(self, error: &AerodbError) -> bool {
        let AerodbError::InvalidInput { field } = error else {
            return false;
        };
        match *field {
            "solver_command_execute_spawn" => self.retry_on_spawn_failure,
            "solver_command_execute_timeout" => self.retry_on_timeout,
            "solver_command_execute_status" => self.retry_on_nonzero_status,
            "solver_command_execute_output_file" | "solver_command_execute_output_sha256" => {
                self.retry_on_output_verification
            }
            _ => false,
        }
    }
}

impl SolverCommandRetryExecutionReport {
    /// Retry execution evidence in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[SolverCommandCaseRetryExecution] {
        &self.cases
    }
}

impl SolverClusterPollingPolicy {
    /// Construct a bounded scheduler polling policy.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when `max_polls` or
    /// `query_timeout_s` is zero, or when `max_polls` exceeds the hard safety
    /// cap.
    pub fn new(
        max_polls: u32,
        poll_interval_ms: u64,
        query_timeout_s: u64,
    ) -> Result<Self, AerodbError> {
        let policy = Self {
            max_polls,
            poll_interval_ms,
            query_timeout_s,
        };
        policy.validate()?;
        Ok(policy)
    }

    fn validate(self) -> Result<(), AerodbError> {
        if self.max_polls == 0 || self.max_polls > MAX_SOLVER_CLUSTER_POLL_COUNT {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_max_polls",
            });
        }
        if self.query_timeout_s == 0 {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_query_timeout_s",
            });
        }
        Ok(())
    }
}

impl SolverClusterArtifactCollectionReport {
    /// Verified output artifacts in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[SolverClusterArtifactEvidence] {
        &self.cases
    }
}

impl SolverRemoteArtifactTransferPlan {
    /// Construct a direct-command remote artifact transfer plan.
    #[must_use]
    pub fn direct_command<I, S>(command: I, timeout_s: u64) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        Self {
            command: command.into_iter().map(Into::into).collect(),
            timeout_s,
        }
    }
}

impl SolverRemoteArtifactTransferReport {
    /// SHA-verified artifacts collected after the transfer command completed.
    #[must_use]
    pub fn artifacts(&self) -> &SolverClusterArtifactCollectionReport {
        &self.artifacts
    }
}

impl OpenFoamScalarField {
    /// Parse a recorded OpenFOAM scalar field dictionary.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed dictionary
    /// syntax and [`AerodbError::InvalidInput`] for non-finite scalar values,
    /// duplicate patches, missing required entries, or list-count mismatches.
    pub fn parse_str(text: &str) -> Result<Self, AerodbError> {
        parse_openfoam_scalar_field(text)
    }

    /// Borrow a parsed boundary patch by name.
    #[must_use]
    pub fn patch(&self, name: &str) -> Option<&OpenFoamScalarPatch> {
        self.boundary_patches.get(name)
    }
}

impl OpenFoamVectorField {
    /// Parse a recorded OpenFOAM vector field dictionary.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed dictionary
    /// syntax and [`AerodbError::InvalidInput`] for non-finite vector values,
    /// duplicate patches, missing required entries, or list-count mismatches.
    pub fn parse_str(text: &str) -> Result<Self, AerodbError> {
        parse_openfoam_vector_field(text)
    }

    /// Borrow a parsed boundary patch by name.
    #[must_use]
    pub fn patch(&self, name: &str) -> Option<&OpenFoamVectorPatch> {
        self.boundary_patches.get(name)
    }
}

impl OpenFoamRecordedCaseFields {
    /// Parse and validate a recorded OpenFOAM case field set.
    ///
    /// `field_texts` are keyed by the expected OpenFOAM `FoamFile.object`
    /// names. This catches accidental duplicate or mislabeled files before
    /// downstream ingestion treats them as solver evidence.
    ///
    /// # Errors
    ///
    /// Returns parser errors from individual dictionaries or
    /// [`AerodbError::InvalidInput`] for empty sets, invalid names, duplicates,
    /// or `FoamFile.object` mismatches.
    pub fn from_field_texts(
        scalar_texts: &[(&str, &str)],
        vector_texts: &[(&str, &str)],
    ) -> Result<Self, AerodbError> {
        if scalar_texts.is_empty() && vector_texts.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_fields",
            });
        }

        let mut scalar_fields = BTreeMap::new();
        for (field_name, text) in scalar_texts {
            insert_openfoam_recorded_scalar_field(&mut scalar_fields, field_name, text)?;
        }

        let mut vector_fields = BTreeMap::new();
        for (field_name, text) in vector_texts {
            insert_openfoam_recorded_vector_field(&mut vector_fields, field_name, text)?;
        }

        Ok(Self {
            scalar_fields,
            vector_fields,
        })
    }

    /// Load and validate recorded OpenFOAM field dictionaries from a case root.
    ///
    /// Field paths are resolved under `case_root` and reject absolute paths,
    /// parent-directory traversal, missing roots, and unreadable files.
    ///
    /// # Errors
    ///
    /// Returns parser errors from individual dictionaries or
    /// [`AerodbError::InvalidInput`] for empty sets, invalid roots/paths,
    /// unreadable files, invalid names, duplicates, or `FoamFile.object`
    /// mismatches.
    pub fn from_case_root(
        case_root: impl AsRef<Path>,
        scalar_files: &[OpenFoamCaseFieldFile<'_>],
        vector_files: &[OpenFoamCaseFieldFile<'_>],
    ) -> Result<Self, AerodbError> {
        if scalar_files.is_empty() && vector_files.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_fields",
            });
        }

        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }

        let mut scalar_fields = BTreeMap::new();
        for file in scalar_files {
            let text = read_openfoam_case_field_file(case_root, file)?;
            insert_openfoam_recorded_scalar_field(
                &mut scalar_fields,
                file.object_name,
                text.as_str(),
            )?;
        }

        let mut vector_fields = BTreeMap::new();
        for file in vector_files {
            let text = read_openfoam_case_field_file(case_root, file)?;
            insert_openfoam_recorded_vector_field(
                &mut vector_fields,
                file.object_name,
                text.as_str(),
            )?;
        }

        Ok(Self {
            scalar_fields,
            vector_fields,
        })
    }

    /// Discover scalar and vector OpenFOAM field dictionaries in a case time directory.
    ///
    /// `time_dir` is resolved under `case_root`, read deterministically in
    /// filename order, and only regular files are parsed as field dictionaries.
    /// Each file must carry a supported `FoamFile.class` and a
    /// `FoamFile.object` matching its filename.
    ///
    /// # Errors
    ///
    /// Returns parser errors from individual dictionaries or
    /// [`AerodbError::InvalidInput`] for invalid roots/time directories,
    /// unreadable files, unsupported field classes, filename/object mismatches,
    /// invalid names, duplicates, or empty discovered field sets.
    pub fn discover_time_directory(
        case_root: impl AsRef<Path>,
        time_dir: &str,
    ) -> Result<Self, AerodbError> {
        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }
        let time_path = resolve_solver_plan_path(case_root, time_dir, "openfoam_case_time_dir")?;
        if !time_path.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_time_dir",
            });
        }

        let mut entries = Vec::new();
        for entry in fs::read_dir(&time_path).map_err(|_| AerodbError::InvalidInput {
            field: "openfoam_case_time_dir_read",
        })? {
            let entry = entry.map_err(|_| AerodbError::InvalidInput {
                field: "openfoam_case_time_dir_read",
            })?;
            let file_type = entry.file_type().map_err(|_| AerodbError::InvalidInput {
                field: "openfoam_case_time_dir_read",
            })?;
            if file_type.is_file() {
                let file_name = entry
                    .file_name()
                    .to_str()
                    .ok_or(AerodbError::InvalidInput {
                        field: "openfoam_case_field_file_name",
                    })?
                    .to_owned();
                entries.push((file_name, entry.path()));
            }
        }
        entries.sort_by(|left, right| left.0.cmp(&right.0));

        let mut scalar_fields = BTreeMap::new();
        let mut vector_fields = BTreeMap::new();
        for (file_name, path) in entries {
            let text = fs::read_to_string(path).map_err(|_| AerodbError::InvalidInput {
                field: "openfoam_case_field_file_read",
            })?;
            let (class_name, object_name) = parse_openfoam_field_metadata(&text)?;
            if object_name != file_name {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_case_field_file_object",
                });
            }
            match class_name.as_str() {
                "volScalarField" | "surfaceScalarField" => {
                    let field = OpenFoamScalarField::parse_str(&text)?;
                    insert_openfoam_parsed_scalar_field(&mut scalar_fields, field)?;
                }
                "volVectorField" | "surfaceVectorField" => {
                    let field = OpenFoamVectorField::parse_str(&text)?;
                    insert_openfoam_parsed_vector_field(&mut vector_fields, field)?;
                }
                _ => {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_case_field_class",
                    });
                }
            }
        }

        if scalar_fields.is_empty() && vector_fields.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_fields",
            });
        }

        Ok(Self {
            scalar_fields,
            vector_fields,
        })
    }
}

impl OpenFoamPolyMeshBoundary {
    /// Parse a recorded OpenFOAM `constant/polyMesh/boundary` dictionary.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed dictionary
    /// syntax and [`AerodbError::InvalidInput`] for missing metadata, malformed
    /// patch counts, duplicate patches, or overlapping patch face ranges.
    pub fn parse_str(text: &str) -> Result<Self, AerodbError> {
        parse_openfoam_poly_mesh_boundary(text)
    }

    /// Load the standard `constant/polyMesh/boundary` file under a case root.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for missing roots, unsafe paths,
    /// unreadable files, and parser errors from [`Self::parse_str`].
    pub fn from_case_root(case_root: impl AsRef<Path>) -> Result<Self, AerodbError> {
        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }
        let text = fs::read_to_string(resolve_solver_plan_path(
            case_root,
            "constant/polyMesh/boundary",
            "openfoam_poly_mesh_boundary_file",
        )?)
        .map_err(|_| AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_boundary_file_read",
        })?;
        Self::parse_str(&text)
    }

    /// Borrow a parsed boundary patch by name.
    #[must_use]
    pub fn patch(&self, name: &str) -> Option<&OpenFoamBoundaryPatchTopology> {
        self.patches.get(name)
    }

    /// Validate that every scalar/vector field boundary patch exists in this mesh boundary.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] if any field dictionary references
    /// a boundary patch absent from `constant/polyMesh/boundary`.
    pub fn validate_field_patches(
        &self,
        fields: &OpenFoamRecordedCaseFields,
    ) -> Result<(), AerodbError> {
        for field in fields.scalar_fields.values() {
            for patch_name in field.boundary_patches.keys() {
                if !self.patches.contains_key(patch_name) {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_boundary_field_patch",
                    });
                }
            }
        }
        for field in fields.vector_fields.values() {
            for patch_name in field.boundary_patches.keys() {
                if !self.patches.contains_key(patch_name) {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_boundary_field_patch",
                    });
                }
            }
        }
        Ok(())
    }
}

impl OpenFoamPolyMeshConnectivity {
    /// Parse recorded OpenFOAM `points`, `faces`, `owner`, and `neighbour` files.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed list syntax and
    /// [`AerodbError::InvalidInput`] for wrong metadata, invalid indices,
    /// mismatched owner/neighbour counts, non-finite points, or degenerate faces.
    pub fn from_texts(
        points_text: &str,
        faces_text: &str,
        owner_text: &str,
        neighbour_text: &str,
    ) -> Result<Self, AerodbError> {
        let connectivity = Self {
            points: parse_openfoam_poly_mesh_points(points_text)?,
            faces: parse_openfoam_poly_mesh_faces(faces_text)?,
            owner: parse_openfoam_poly_mesh_label_list(owner_text, "owner")?,
            neighbour: parse_openfoam_poly_mesh_label_list(neighbour_text, "neighbour")?,
        };
        connectivity.validate()?;
        Ok(connectivity)
    }

    /// Load standard OpenFOAM `constant/polyMesh` connectivity files under a case root.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for missing roots, unsafe paths, or
    /// unreadable files, plus parser errors from [`Self::from_texts`].
    pub fn from_case_root(case_root: impl AsRef<Path>) -> Result<Self, AerodbError> {
        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }
        let points_text =
            read_openfoam_poly_mesh_file(case_root, "points", "openfoam_poly_mesh_points_file")?;
        let faces_text =
            read_openfoam_poly_mesh_file(case_root, "faces", "openfoam_poly_mesh_faces_file")?;
        let owner_text =
            read_openfoam_poly_mesh_file(case_root, "owner", "openfoam_poly_mesh_owner_file")?;
        let neighbour_text = read_openfoam_poly_mesh_file(
            case_root,
            "neighbour",
            "openfoam_poly_mesh_neighbour_file",
        )?;
        Self::from_texts(&points_text, &faces_text, &owner_text, &neighbour_text)
    }

    /// Validate boundary patch face ranges against this connectivity.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when a boundary range extends past
    /// the face list or overlaps internal faces covered by `neighbour`.
    pub fn validate_boundary_ranges(
        &self,
        boundary: &OpenFoamPolyMeshBoundary,
    ) -> Result<(), AerodbError> {
        for patch in boundary.patches.values() {
            let end_face =
                patch
                    .start_face
                    .checked_add(patch.n_faces)
                    .ok_or(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_boundary_face_range",
                    })?;
            if end_face > self.faces.len() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_boundary_face_range",
                });
            }
            if patch.n_faces > 0 && patch.start_face < self.neighbour.len() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_boundary_internal_faces",
                });
            }
        }
        Ok(())
    }

    /// Reconstruct face geometry and cell volumes from parsed polyMesh connectivity.
    ///
    /// Face centroids and scalar areas use fan triangulation in recorded face
    /// vertex order. Cell volumes use the OpenFOAM-oriented face-area vector
    /// convention: face vectors point out of the owner cell and into the
    /// neighbour cell for internal faces.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when a face is geometrically
    /// degenerate, a cell face-area closure residual is nonzero beyond tolerance,
    /// or a reconstructed cell volume is non-finite or non-positive.
    pub fn reconstruct_geometry(&self) -> Result<OpenFoamPolyMeshGeometry, AerodbError> {
        let mut faces = Vec::with_capacity(self.faces.len());
        for face in &self.faces {
            faces.push(reconstruct_openfoam_face_geometry(&self.points, face)?);
        }

        let cell_count = inferred_openfoam_cell_count(
            self.owner.iter().chain(self.neighbour.iter()).copied(),
            "openfoam_poly_mesh_cell_volume",
        )?;
        let mut cell_area_vectors = vec![Vector3::zeros(); cell_count];
        let mut cell_area_sums = vec![0.0; cell_count];
        let mut cell_volumes_m3 = vec![0.0; cell_count];

        for (face_index, face_geometry) in faces.iter().enumerate() {
            let contribution = face_geometry.area_vector.dot(&face_geometry.centroid) / 3.0;
            if !contribution.is_finite() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_cell_volume",
                });
            }
            let owner = self.owner[face_index];
            cell_area_vectors[owner] += face_geometry.area_vector;
            cell_area_sums[owner] += face_geometry.area_m2;
            cell_volumes_m3[owner] += contribution;

            if face_index < self.neighbour.len() {
                let neighbour = self.neighbour[face_index];
                cell_area_vectors[neighbour] -= face_geometry.area_vector;
                cell_area_sums[neighbour] += face_geometry.area_m2;
                cell_volumes_m3[neighbour] -= contribution;
            }
        }

        for (cell_index, volume) in cell_volumes_m3.iter().enumerate() {
            let closure_tolerance = (cell_area_sums[cell_index] * 1.0e-10).max(1.0e-12);
            if cell_area_vectors[cell_index].norm() > closure_tolerance {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_cell_closed_surface",
                });
            }
            if !(volume.is_finite() && *volume > 0.0) {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_cell_volume",
                });
            }
        }

        Ok(OpenFoamPolyMeshGeometry {
            faces,
            cell_volumes_m3,
        })
    }

    fn validate(&self) -> Result<(), AerodbError> {
        if self.points.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_points",
            });
        }
        if self.faces.is_empty() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_faces",
            });
        }
        if self.owner.len() != self.faces.len() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_owner_count",
            });
        }
        if self.neighbour.len() > self.faces.len() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_neighbour_count",
            });
        }
        let _ = inferred_openfoam_cell_count(
            self.owner.iter().chain(self.neighbour.iter()).copied(),
            "openfoam_poly_mesh_cell_label",
        )?;
        for face in &self.faces {
            for &point_index in face {
                if point_index >= self.points.len() {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_face_vertex",
                    });
                }
            }
        }
        Ok(())
    }
}

impl OpenFoamPolyMeshZones {
    /// Parse recorded optional OpenFOAM `pointZones`, `faceZones`, and `cellZones` files.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed list syntax and
    /// [`AerodbError::InvalidInput`] for wrong metadata, invalid zone names,
    /// duplicate zones, or missing zone type/label entries.
    pub fn from_texts(
        point_zones_text: Option<&str>,
        face_zones_text: Option<&str>,
        cell_zones_text: Option<&str>,
    ) -> Result<Self, AerodbError> {
        Ok(Self {
            point_zones: match point_zones_text {
                Some(text) => {
                    parse_openfoam_poly_mesh_zones(text, OpenFoamPolyMeshZoneKind::Point)?
                }
                None => BTreeMap::new(),
            },
            face_zones: match face_zones_text {
                Some(text) => parse_openfoam_poly_mesh_zones(text, OpenFoamPolyMeshZoneKind::Face)?,
                None => BTreeMap::new(),
            },
            cell_zones: match cell_zones_text {
                Some(text) => parse_openfoam_poly_mesh_zones(text, OpenFoamPolyMeshZoneKind::Cell)?,
                None => BTreeMap::new(),
            },
        })
    }

    /// Load optional standard OpenFOAM zone files under a case root.
    ///
    /// Missing `pointZones`, `faceZones`, or `cellZones` files are treated as
    /// empty zone sets because OpenFOAM cases commonly omit unused zone files.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for missing roots, unsafe paths, or
    /// unreadable present files, plus parser errors from [`Self::from_texts`].
    pub fn from_case_root(case_root: impl AsRef<Path>) -> Result<Self, AerodbError> {
        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }
        let point_zones_text = read_optional_openfoam_poly_mesh_file(
            case_root,
            "pointZones",
            "openfoam_poly_mesh_point_zones_file",
        )?;
        let face_zones_text = read_optional_openfoam_poly_mesh_file(
            case_root,
            "faceZones",
            "openfoam_poly_mesh_face_zones_file",
        )?;
        let cell_zones_text = read_optional_openfoam_poly_mesh_file(
            case_root,
            "cellZones",
            "openfoam_poly_mesh_cell_zones_file",
        )?;
        Self::from_texts(
            point_zones_text.as_deref(),
            face_zones_text.as_deref(),
            cell_zones_text.as_deref(),
        )
    }

    /// Validate parsed zone labels against parsed polyMesh connectivity counts.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when a zone label references a
    /// missing point, face, or inferred cell, or when a face-zone flip map has a
    /// different length than its face-label list.
    pub fn validate_connectivity(
        &self,
        connectivity: &OpenFoamPolyMeshConnectivity,
    ) -> Result<(), AerodbError> {
        for zone in self.point_zones.values() {
            for &label in &zone.labels {
                if label >= connectivity.points.len() {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_point_zone_label",
                    });
                }
            }
        }
        for zone in self.face_zones.values() {
            if let Some(flip_map) = &zone.flip_map
                && flip_map.len() != zone.labels.len()
            {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_face_zone_flip_map",
                });
            }
            for &label in &zone.labels {
                if label >= connectivity.faces.len() {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_face_zone_label",
                    });
                }
            }
        }
        let cell_count = inferred_openfoam_cell_count(
            connectivity
                .owner
                .iter()
                .chain(connectivity.neighbour.iter())
                .copied(),
            "openfoam_poly_mesh_cell_zone_label",
        )?;
        for zone in self.cell_zones.values() {
            for &label in &zone.labels {
                if label >= cell_count {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_poly_mesh_cell_zone_label",
                    });
                }
            }
        }
        Ok(())
    }
}

impl OpenFoamTurbulenceProperties {
    /// Parse a recorded OpenFOAM `constant/turbulenceProperties` dictionary.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed dictionary
    /// syntax and [`AerodbError::InvalidInput`] for missing or unsupported
    /// simulation type/model metadata.
    pub fn parse_str(text: &str) -> Result<Self, AerodbError> {
        parse_openfoam_turbulence_properties(text)
    }

    /// Load `constant/turbulenceProperties` under an OpenFOAM case root.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for missing roots, unsafe paths, or
    /// unreadable files, plus parser errors from [`Self::parse_str`].
    pub fn from_case_root(case_root: impl AsRef<Path>) -> Result<Self, AerodbError> {
        let case_root = case_root.as_ref();
        if !case_root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_case_root",
            });
        }
        let text = fs::read_to_string(resolve_solver_plan_path(
            case_root,
            "constant/turbulenceProperties",
            "openfoam_turbulence_properties_file",
        )?)
        .map_err(|_| AerodbError::InvalidInput {
            field: "openfoam_turbulence_properties_file_read",
        })?;
        Self::parse_str(&text)
    }

    /// Validate required recorded field dictionaries for this turbulence setup.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when an active RAS/LES model is
    /// unsupported or required `U`/scalar turbulence fields are absent.
    pub fn validate_field_set(
        &self,
        fields: &OpenFoamRecordedCaseFields,
    ) -> Result<(), AerodbError> {
        if self.simulation_type == OpenFoamTurbulenceSimulationType::Laminar
            || self.turbulence == Some(false)
        {
            return Ok(());
        }
        if !fields.vector_fields.contains_key("U") {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_velocity_field",
            });
        }
        for &field_name in self.required_scalar_fields()? {
            if !fields.scalar_fields.contains_key(field_name) {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_turbulence_field",
                });
            }
        }
        Ok(())
    }

    /// Return a parsed scalar turbulence-model coefficient by name.
    #[must_use]
    pub fn coefficient(&self, name: &str) -> Option<f64> {
        self.model_coefficients.get(name).copied()
    }

    /// Validate parsed model coefficients against calibrated expected values.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when an expectation is malformed, a
    /// required coefficient is missing, or the absolute coefficient error
    /// exceeds the declared tolerance.
    pub fn validate_coefficients(
        &self,
        expectations: &[OpenFoamTurbulenceCoefficientExpectation<'_>],
    ) -> Result<(), AerodbError> {
        for expectation in expectations {
            validate_case_id_prefix(expectation.name, "openfoam_turbulence_coefficient_name")?;
            if !(expectation.expected.is_finite()
                && expectation.tolerance.is_finite()
                && expectation.tolerance >= 0.0)
            {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_turbulence_coefficient_tolerance",
                });
            }
            let actual =
                self.model_coefficients
                    .get(expectation.name)
                    .ok_or(AerodbError::InvalidInput {
                        field: "openfoam_turbulence_coefficient",
                    })?;
            if (actual - expectation.expected).abs() > expectation.tolerance {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_turbulence_coefficient_tolerance",
                });
            }
        }
        Ok(())
    }

    fn required_scalar_fields(&self) -> Result<&'static [&'static str], AerodbError> {
        let Some(model_name) = self.model_name.as_deref() else {
            return Ok(&[]);
        };
        match (self.simulation_type, model_name) {
            (OpenFoamTurbulenceSimulationType::Ras, "kOmegaSST" | "kOmega" | "kOmegaSSTLM") => {
                Ok(&["k", "omega", "nut"])
            }
            (
                OpenFoamTurbulenceSimulationType::Ras,
                "kEpsilon" | "realizableKE" | "RNGkEpsilon",
            ) => Ok(&["k", "epsilon", "nut"]),
            (
                OpenFoamTurbulenceSimulationType::Ras,
                "SpalartAllmaras" | "SpalartAllmarasDDES" | "SpalartAllmarasIDDES",
            ) => Ok(&["nuTilda", "nut"]),
            (OpenFoamTurbulenceSimulationType::Les, "Smagorinsky" | "WALE") => Ok(&["nut"]),
            (OpenFoamTurbulenceSimulationType::Les, "kEqn" | "dynamicKEqn") => Ok(&["k", "nut"]),
            _ => Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_model",
            }),
        }
    }
}

impl SolverClusterScheduleTemplate {
    /// Construct a Slurm array schedule template.
    #[must_use]
    pub fn slurm(job_name: impl Into<String>, wall_time_s: u64) -> Self {
        Self {
            scheduler: SolverClusterScheduler::Slurm,
            job_name: job_name.into(),
            script_filename: "openbmp-solver-array.sbatch".to_owned(),
            log_dir: "logs".to_owned(),
            mpi_ranks_per_case: 1,
            cpus_per_task: 1,
            wall_time_s,
            account: None,
            partition: None,
        }
    }

    fn validate(&self) -> Result<(), AerodbError> {
        validate_scheduler_token(&self.job_name, "solver_cluster_schedule_job_name")?;
        validate_solver_plan_filename(
            &self.script_filename,
            "solver_cluster_schedule_script_filename",
        )?;
        validate_solver_relative_path(&self.log_dir, "solver_cluster_schedule_log_dir")?;
        if self.log_dir.chars().any(char::is_whitespace) {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_log_dir",
            });
        }
        if self.mpi_ranks_per_case == 0 {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_mpi_ranks_per_case",
            });
        }
        if self.cpus_per_task == 0 {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_cpus_per_task",
            });
        }
        if self.wall_time_s == 0 {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_wall_time_s",
            });
        }
        if let Some(account) = &self.account {
            validate_scheduler_token(account, "solver_cluster_schedule_account")?;
        }
        if let Some(partition) = &self.partition {
            validate_scheduler_token(partition, "solver_cluster_schedule_partition")?;
        }
        Ok(())
    }
}

impl SolverClusterSchedule {
    /// Write the rendered scheduler script under an existing root directory.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when the root is missing, the
    /// script filename is unsafe, or the file cannot be written.
    pub fn write_script_to_root(&self, root: impl AsRef<Path>) -> Result<(), AerodbError> {
        validate_solver_plan_filename(
            &self.script_filename,
            "solver_cluster_schedule_script_filename",
        )?;
        let root = root.as_ref();
        if !root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "solver_cluster_submit_root",
            });
        }
        let script_path = resolve_solver_plan_path(
            root,
            &self.script_filename,
            "solver_cluster_schedule_script_filename",
        )?;
        fs::write(&script_path, self.script_text.as_bytes()).map_err(|_| {
            AerodbError::InvalidInput {
                field: "solver_cluster_schedule_script_filename",
            }
        })
    }

    /// Write and submit the scheduler script with the schedule's submit command.
    ///
    /// This executes the submit command directly without a shell and parses the
    /// scheduler job id from stdout. It does not monitor job completion or
    /// collect solver artifacts.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid roots, invalid submit
    /// commands, timeouts, nonzero submit exits, or unparseable submit output.
    pub fn submit_with_root(
        &self,
        root: impl AsRef<Path>,
        timeout_s: u64,
    ) -> Result<SolverClusterSubmissionReport, AerodbError> {
        self.write_script_to_root(root.as_ref())?;
        let output = run_cluster_command(
            root.as_ref(),
            &self.submit_command,
            timeout_s,
            &SOLVER_CLUSTER_SUBMIT_FIELDS,
        )?;
        let job_id = match self.scheduler {
            SolverClusterScheduler::Slurm => parse_slurm_submit_job_id(&output.stdout)?,
        };
        Ok(SolverClusterSubmissionReport {
            scheduler: self.scheduler,
            job_id,
            script_filename: self.script_filename.clone(),
            script_sha256: self.script_sha256.clone(),
            stdout_len: output.stdout.len(),
            stdout_sha256: sha256_hex(&output.stdout),
            stderr_len: output.stderr.len(),
            stderr_sha256: sha256_hex(&output.stderr),
        })
    }

    /// Query a scheduler job state with an explicit command vector.
    ///
    /// The caller supplies the full query command so tests and deployments can
    /// choose `sacct`, `squeue`, or a site wrapper without embedding shell
    /// syntax.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid job ids, roots,
    /// commands, timeouts, nonzero query exits, or empty scheduler state output.
    pub fn query_job_state_with_root(
        &self,
        root: impl AsRef<Path>,
        job_id: &str,
        query_command: &[String],
        timeout_s: u64,
    ) -> Result<SolverClusterJobStateReport, AerodbError> {
        validate_scheduler_token(job_id, "solver_cluster_query_job_id")?;
        let output = run_cluster_command(
            root.as_ref(),
            query_command,
            timeout_s,
            &SOLVER_CLUSTER_QUERY_FIELDS,
        )?;
        let raw_state = match self.scheduler {
            SolverClusterScheduler::Slurm => parse_slurm_state_token(&output.stdout)?,
        };
        let status = match self.scheduler {
            SolverClusterScheduler::Slurm => classify_slurm_job_state(&raw_state),
        };
        Ok(SolverClusterJobStateReport {
            scheduler: self.scheduler,
            job_id: job_id.to_owned(),
            status,
            raw_state,
            stdout_len: output.stdout.len(),
            stdout_sha256: sha256_hex(&output.stdout),
            stderr_len: output.stderr.len(),
            stderr_sha256: sha256_hex(&output.stderr),
        })
    }

    /// Query a Slurm job state with `sacct`.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Self::query_job_state_with_root`].
    pub fn query_slurm_job_state_with_root(
        &self,
        root: impl AsRef<Path>,
        job_id: &str,
        timeout_s: u64,
    ) -> Result<SolverClusterJobStateReport, AerodbError> {
        self.query_job_state_with_root(
            root,
            job_id,
            &[
                "sacct".to_owned(),
                "-j".to_owned(),
                job_id.to_owned(),
                "--format=State".to_owned(),
                "--noheader".to_owned(),
                "--parsable2".to_owned(),
            ],
            timeout_s,
        )
    }

    /// Poll scheduler state until a job reaches successful completion.
    ///
    /// The caller supplies the full query command vector, so tests and
    /// deployments can use `sacct`, `squeue`, or a site wrapper. Failed,
    /// cancelled, timed-out, unknown, and exhausted polling states fail closed.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid polling policy,
    /// query-command failures, unknown scheduler states, unsuccessful terminal
    /// states, or exhausting `policy.max_polls` before completion.
    pub fn wait_for_completion_with_root(
        &self,
        root: impl AsRef<Path>,
        job_id: &str,
        query_command: &[String],
        policy: SolverClusterPollingPolicy,
    ) -> Result<SolverClusterCompletionReport, AerodbError> {
        policy.validate()?;
        let root = root.as_ref();
        let mut polls = Vec::with_capacity(policy.max_polls as usize);
        for poll_index in 0..policy.max_polls {
            let state = self.query_job_state_with_root(
                root,
                job_id,
                query_command,
                policy.query_timeout_s,
            )?;
            match state.status {
                SolverClusterJobStatus::Completed => {
                    polls.push(state);
                    return Ok(SolverClusterCompletionReport {
                        scheduler: self.scheduler,
                        job_id: job_id.to_owned(),
                        final_status: SolverClusterJobStatus::Completed,
                        polls,
                    });
                }
                SolverClusterJobStatus::Pending
                | SolverClusterJobStatus::Running
                | SolverClusterJobStatus::Completing => {
                    polls.push(state);
                    if poll_index + 1 < policy.max_polls && policy.poll_interval_ms > 0 {
                        thread::sleep(Duration::from_millis(policy.poll_interval_ms));
                    }
                }
                SolverClusterJobStatus::Failed => {
                    return Err(AerodbError::InvalidInput {
                        field: "solver_cluster_poll_failed",
                    });
                }
                SolverClusterJobStatus::Cancelled => {
                    return Err(AerodbError::InvalidInput {
                        field: "solver_cluster_poll_cancelled",
                    });
                }
                SolverClusterJobStatus::Timeout => {
                    return Err(AerodbError::InvalidInput {
                        field: "solver_cluster_poll_timeout",
                    });
                }
                SolverClusterJobStatus::Unknown => {
                    return Err(AerodbError::InvalidInput {
                        field: "solver_cluster_poll_state",
                    });
                }
            }
        }
        Err(AerodbError::InvalidInput {
            field: "solver_cluster_poll_exhausted",
        })
    }
}

impl Su2EulerCaseTemplate {
    /// Create a SU2-Euler case template with conservative defaults.
    #[must_use]
    pub fn new(
        mesh_filename: impl Into<String>,
        reference_area_m2: f64,
        reference_length_m: f64,
        provenance: impl Into<String>,
    ) -> Self {
        Self {
            program: "SU2_CFD".to_owned(),
            mesh_filename: mesh_filename.into(),
            surface_marker: "body".to_owned(),
            case_dir_prefix: "cases".to_owned(),
            case_id_prefix: "case".to_owned(),
            config_filename: "case.cfg".to_owned(),
            history_filename: "history.csv".to_owned(),
            surface_filename: "surface.csv".to_owned(),
            timeout_s: 3600,
            reference_area_m2,
            reference_length_m,
            moment_reference_m: Vector3::zeros(),
            extra_config: BTreeMap::new(),
            extra_args: Vec::new(),
            env: BTreeMap::new(),
            provenance: provenance.into(),
        }
    }

    /// Validate and render one SU2-Euler config per run-matrix node.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] for invalid paths, command text,
    /// reference geometry, protected config overrides, or malformed extra
    /// config/env entries.
    pub fn render_cases(&self, matrix: &RunMatrix) -> Result<Su2EulerCaseBundle, AerodbError> {
        self.validate()?;
        let mut cases = Vec::with_capacity(matrix.nodes().len());
        for (node_index, node) in matrix.nodes().iter().copied().enumerate() {
            let case_id = format!("{}-{node_index}", self.case_id_prefix);
            let working_dir = join_relative_path(&self.case_dir_prefix, &case_id);
            let config_file = join_relative_path(&working_dir, &self.config_filename);
            let history_file = join_relative_path(&working_dir, &self.history_filename);
            let surface_file = join_relative_path(&working_dir, &self.surface_filename);
            let config_text = render_su2_euler_config(self, node_index, node);
            let config_sha256 = sha256_hex(config_text.as_bytes());
            let mut args = Vec::with_capacity(1 + self.extra_args.len());
            args.push(self.config_filename.clone());
            args.extend(self.extra_args.iter().cloned());
            cases.push(Su2EulerCase {
                node_index,
                node,
                solver: SU2_EULER_SOLVER_LABEL.to_owned(),
                case_id,
                program: self.program.clone(),
                args,
                env: self.env.clone(),
                working_dir,
                config_file,
                config_text,
                config_sha256,
                history_file,
                surface_file,
                timeout_s: self.timeout_s,
                provenance: self.provenance.clone(),
            });
        }
        Ok(Su2EulerCaseBundle { cases })
    }

    fn validate(&self) -> Result<(), AerodbError> {
        validate_solver_command_text(&self.program, "su2_euler_program")?;
        validate_solver_relative_path(&self.mesh_filename, "su2_euler_mesh_filename")?;
        validate_su2_marker(&self.surface_marker)?;
        validate_solver_relative_path(&self.case_dir_prefix, "su2_euler_case_dir_prefix")?;
        validate_case_id_prefix(&self.case_id_prefix, "su2_euler_case_id_prefix")?;
        validate_solver_plan_filename(&self.config_filename, "su2_euler_config_filename")?;
        validate_solver_plan_filename(&self.history_filename, "su2_euler_history_filename")?;
        validate_solver_plan_filename(&self.surface_filename, "su2_euler_surface_filename")?;
        if self.timeout_s == 0 {
            return Err(AerodbError::InvalidInput {
                field: "su2_euler_timeout_s",
            });
        }
        if !(self.reference_area_m2.is_finite() && self.reference_area_m2 > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "su2_euler_reference_area_m2",
            });
        }
        if !(self.reference_length_m.is_finite() && self.reference_length_m > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "su2_euler_reference_length_m",
            });
        }
        if !(self.moment_reference_m.x.is_finite()
            && self.moment_reference_m.y.is_finite()
            && self.moment_reference_m.z.is_finite())
        {
            return Err(AerodbError::InvalidInput {
                field: "su2_euler_moment_reference_m",
            });
        }
        for (key, value) in &self.extra_config {
            validate_su2_config_key(key)?;
            if SU2_EULER_PROTECTED_CONFIG_KEYS.contains(&key.as_str()) {
                return Err(AerodbError::InvalidInput {
                    field: "su2_euler_extra_config_key",
                });
            }
            validate_solver_command_text(value, "su2_euler_extra_config_value")?;
        }
        for arg in &self.extra_args {
            validate_solver_command_text(arg, "su2_euler_arg")?;
        }
        for (key, value) in &self.env {
            validate_solver_env_key(key)?;
            validate_solver_command_text(value, "su2_euler_env_value")?;
        }
        validate_nonblank_solver_case_field(&self.provenance, "su2_euler_provenance")?;
        Ok(())
    }
}

impl Su2EulerCaseBundle {
    /// Generated cases in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[Su2EulerCase] {
        &self.cases
    }

    /// Write generated config files under an existing root directory.
    ///
    /// This creates case directories and writes only the rendered SU2 config
    /// files. Meshes, solver binaries, and solver outputs remain external.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when the root is not a directory or
    /// when case directories/config files cannot be created under that root.
    pub fn write_configs_to_root(&self, root: impl AsRef<Path>) -> Result<(), AerodbError> {
        let root = root.as_ref();
        if !root.is_dir() {
            return Err(AerodbError::InvalidInput {
                field: "su2_euler_case_root",
            });
        }
        for case in &self.cases {
            let working_dir =
                resolve_solver_plan_path(root, &case.working_dir, "su2_euler_case_working_dir")?;
            fs::create_dir_all(&working_dir).map_err(|_| AerodbError::InvalidInput {
                field: "su2_euler_case_working_dir",
            })?;
            let config_path =
                resolve_solver_plan_path(root, &case.config_file, "su2_euler_case_config_file")?;
            fs::write(&config_path, case.config_text.as_bytes()).map_err(|_| {
                AerodbError::InvalidInput {
                    field: "su2_euler_case_config_file",
                }
            })?;
        }
        Ok(())
    }
}

impl SolverCaseManifest {
    /// Parse and validate a strict solver-case manifest TOML document.
    ///
    /// The manifest is metadata only: it records solver artifacts that were
    /// produced out of process and pins their hashes. It does not launch a
    /// solver, evaluate shell commands, or make any artifact a runtime force
    /// source.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedToml`] for TOML/schema failures and
    /// [`AerodbError::InvalidInput`] for row count, node order, condition echo,
    /// blank provenance/path fields, or malformed SHA-256 pins.
    pub fn load_toml_str(matrix: &RunMatrix, s: &str) -> Result<Self, AerodbError> {
        let parsed: SolverCaseManifestToml =
            toml::from_str(s).map_err(|_| AerodbError::MalformedToml {
                reason: "solver-case manifest TOML did not match schema",
            })?;
        if parsed.openbmp.aerodb_solver_case_manifest != SOLVER_CASE_MANIFEST_SCHEMA_VERSION_1 {
            return Err(AerodbError::MalformedToml {
                reason: "unsupported solver-case manifest schema version",
            });
        }
        let expected = matrix.nodes().len();
        if parsed.cases.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_rows",
            });
        }
        let mut cases = Vec::with_capacity(expected);
        for (record_index, case) in parsed.cases.into_iter().enumerate() {
            let node = matrix.nodes()[record_index];
            cases.push(case.into_artifact(record_index, node)?);
        }
        Ok(Self { cases })
    }

    /// Recorded solver artifacts in run-matrix order.
    #[must_use]
    pub fn cases(&self) -> &[SolverCaseArtifact] {
        &self.cases
    }

    /// Verify every declared artifact against its SHA-256 pin.
    ///
    /// The caller supplies the byte lookup so this offline crate does not assume
    /// a filesystem layout. Missing bytes fail closed on `output_file`; digest
    /// mismatches fail closed on `output_sha256`.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any declared artifact is
    /// missing from the lookup or its bytes do not match the manifest pin.
    pub fn verify_artifact_sha256_with_bytes<'a, F>(&self, mut lookup: F) -> Result<(), AerodbError>
    where
        F: FnMut(&SolverCaseArtifact) -> Option<&'a [u8]>,
    {
        for case in &self.cases {
            let bytes = lookup(case).ok_or(AerodbError::InvalidInput {
                field: "solver_case_manifest_output_file",
            })?;
            if !case.output_sha256.eq_ignore_ascii_case(&sha256_hex(bytes)) {
                return Err(AerodbError::InvalidInput {
                    field: "solver_case_manifest_output_sha256",
                });
            }
        }
        Ok(())
    }
}

impl DeckBuilder {
    /// Construct a deck builder from reference geometry.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when either reference value is
    /// non-finite or non-positive.
    pub fn new(reference_area_m2: f64, reference_length_m: f64) -> Result<Self, AerodbError> {
        if !(reference_area_m2.is_finite() && reference_area_m2 > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "reference_area_m2",
            });
        }
        if !(reference_length_m.is_finite() && reference_length_m > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "reference_length_m",
            });
        }
        Ok(Self {
            reference_area_m2,
            reference_length_m,
        })
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from an in-Rust aero method.
    ///
    /// The current `AeroDeck` shape is three-axis, so Reynolds, Knudsen, and
    /// dynamic-pressure axes must be singleton values. Broader N-D database
    /// tables, uncertainty assembly, and external-solver ingestion are later
    /// work packages.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::Aero`] when the method fails, or any
    /// [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_panel_deck<M: AeroMethod + Sync>(
        &self,
        matrix: &RunMatrix,
        method: &M,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let mut samples = matrix
            .nodes()
            .par_iter()
            .enumerate()
            .map(|(index, node)| {
                let force_moment = method.aero_force_moment_body(&node.context())?;
                Ok((
                    index,
                    coefficients_from_force_moment(self, *node, force_moment)?,
                ))
            })
            .collect::<Result<Vec<_>, AerodbError>>()?;
        samples.sort_unstable_by_key(|(index, _)| *index);

        let mut cn = Vec::with_capacity(samples.len());
        let mut cd = Vec::with_capacity(samples.len());
        let mut cm = Vec::with_capacity(samples.len());
        for (_, coeffs) in samples {
            cn.push(coeffs.cn);
            cd.push(coeffs.cd);
            cm.push(coeffs.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Bake a deck with both legacy reduced `(CN, CD, CM)` coefficients and
    /// explicit body-axis `(CX, CY, CZ, Cl, Cm, Cn)` coefficients from an
    /// in-Rust aero method.
    ///
    /// The reduced tables remain attached for existing consumers, while
    /// [`openbmp_aero::DeckLookup`] will prefer the explicit six-channel body
    /// coefficients when composing runtime force and moment.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::Aero`] when the method fails, or any
    /// [`AeroError`] surfaced by [`AeroDeck::new`] /
    /// [`AeroDeck::with_six_coefficient_tables`].
    pub fn bake_panel_six_coefficient_deck<M: AeroMethod + Sync>(
        &self,
        matrix: &RunMatrix,
        method: &M,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let mut samples = matrix
            .nodes()
            .par_iter()
            .enumerate()
            .map(|(index, node)| {
                let force_moment = method.aero_force_moment_body(&node.context())?;
                Ok((
                    index,
                    coefficients_from_force_moment(self, *node, force_moment)?,
                    six_coefficients_from_force_moment(self, *node, force_moment)?,
                ))
            })
            .collect::<Result<Vec<_>, AerodbError>>()?;
        samples.sort_unstable_by_key(|(index, _, _)| *index);

        let mut cn = Vec::with_capacity(samples.len());
        let mut cd = Vec::with_capacity(samples.len());
        let mut cm = Vec::with_capacity(samples.len());
        let mut cx = Vec::with_capacity(samples.len());
        let mut cy = Vec::with_capacity(samples.len());
        let mut cz = Vec::with_capacity(samples.len());
        let mut cl = Vec::with_capacity(samples.len());
        let mut c_m = Vec::with_capacity(samples.len());
        let mut c_n = Vec::with_capacity(samples.len());
        for (_, reduced, six) in samples {
            cn.push(reduced.cn);
            cd.push(reduced.cd);
            cm.push(reduced.cm);
            cx.push(six.cx);
            cy.push(six.cy);
            cz.push(six.cz);
            cl.push(six.cl);
            c_m.push(six.cm);
            c_n.push(six.cn);
        }

        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?
        .with_six_coefficient_tables(cx, cy, cz, cl, c_m, c_n)?)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from a strict normalized
    /// external-solver coefficient CSV.
    ///
    /// The CSV must use [`SOLVER_COEFFICIENT_CSV_SCHEMA_V1`] in every row, have
    /// exactly the expected header, carry sequential `node_index` values, echo
    /// each [`RunNode`] condition exactly, and provide finite `CN`, `CD`, and
    /// `CM` values. The `solver`, `case_id`, and `provenance` columns are
    /// required to be nonblank so downstream provenance can remain auditable.
    ///
    /// This helper is a normalized coefficient front door. Native SU2/OpenFOAM
    /// field-output parsers remain separate adapters that can emit this schema.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for CSV syntax/schema failures,
    /// [`AerodbError::InvalidInput`] for row count, node, condition, or numeric
    /// failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_solver_coefficient_csv_deck(
        &self,
        matrix: &RunMatrix,
        csv_text: &str,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let mut reader = csv::ReaderBuilder::new()
            .trim(csv::Trim::All)
            .from_reader(csv_text.as_bytes());
        let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV header did not parse",
        })?;
        validate_solver_coefficient_csv_header(headers)?;

        let expected = matrix.nodes().len();
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        for (record_index, record) in reader.records().enumerate() {
            let record = record.map_err(|_| AerodbError::MalformedCsv {
                reason: "aero solver coefficient CSV row did not parse",
            })?;
            let node = matrix
                .nodes()
                .get(record_index)
                .ok_or(AerodbError::InvalidInput {
                    field: "solver_coefficient_csv_rows",
                })?;
            let coeffs = solver_coefficients_from_csv_record(&record, record_index, *node)?;
            cn.push(coeffs.cn);
            cd.push(coeffs.cd);
            cm.push(coeffs.cm);
        }
        if cn.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "solver_coefficient_csv_rows",
            });
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Bake a deck with both legacy reduced `(CN, CD, CM)` coefficients and
    /// explicit body-axis `(CX, CY, CZ, Cl, Cm, Cn)` coefficients from a strict
    /// normalized external-solver coefficient CSV.
    ///
    /// The CSV must use [`SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1`] in every row,
    /// have exactly the expected header, carry sequential `node_index` values,
    /// echo each [`RunNode`] condition exactly, and provide finite body-axis
    /// force/moment coefficients. Legacy reduced tables are derived as
    /// `CN = -CZ`, `CD = -CX`, and `CM = Cm` so existing deck consumers remain
    /// compatible while six-channel consumers can use the explicit tables.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for CSV syntax/schema failures,
    /// [`AerodbError::InvalidInput`] for row count, node, condition, or numeric
    /// failures, or any [`AeroError`] surfaced by [`AeroDeck::new`] /
    /// [`AeroDeck::with_six_coefficient_tables`].
    pub fn bake_solver_six_coefficient_csv_deck(
        &self,
        matrix: &RunMatrix,
        csv_text: &str,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let mut reader = csv::ReaderBuilder::new()
            .trim(csv::Trim::All)
            .from_reader(csv_text.as_bytes());
        let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
            reason: "aero solver six coefficient CSV header did not parse",
        })?;
        validate_solver_six_coefficient_csv_header(headers)?;

        let expected = matrix.nodes().len();
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        let mut cx = Vec::with_capacity(expected);
        let mut cy = Vec::with_capacity(expected);
        let mut cz = Vec::with_capacity(expected);
        let mut cl = Vec::with_capacity(expected);
        let mut c_m = Vec::with_capacity(expected);
        let mut c_n = Vec::with_capacity(expected);
        for (record_index, record) in reader.records().enumerate() {
            let record = record.map_err(|_| AerodbError::MalformedCsv {
                reason: "aero solver six coefficient CSV row did not parse",
            })?;
            let node = matrix
                .nodes()
                .get(record_index)
                .ok_or(AerodbError::InvalidInput {
                    field: "solver_six_coefficient_csv_rows",
                })?;
            let six = solver_six_coefficients_from_csv_record(&record, record_index, *node)?;
            cn.push(-six.cz);
            cd.push(-six.cx);
            cm.push(six.cm);
            cx.push(six.cx);
            cy.push(six.cy);
            cz.push(six.cz);
            cl.push(six.cl);
            c_m.push(six.cm);
            c_n.push(six.cn);
        }
        if cx.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "solver_six_coefficient_csv_rows",
            });
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?
        .with_six_coefficient_tables(cx, cy, cz, cl, c_m, c_n)?)
    }

    /// Integrate normalized surface-Cp CSV rows into per-node force/moment load
    /// reports.
    ///
    /// The CSV must use [`SURFACE_CP_CSV_SCHEMA_V1`] in every row, have exactly
    /// the expected header, and contain rows ordered by run-matrix node and then
    /// mesh triangle index. Each row must echo the node condition exactly,
    /// provide a finite `cp`, and carry nonblank solver/case/provenance fields.
    /// Pressures are integrated with the same panel convention as
    /// `LocalInclinationPanels`: `F_i = -Cp_i q A_i n_i`, with moments about the
    /// supplied body-frame CG. `axial_stations_m` may be empty; otherwise it
    /// must contain finite, strictly increasing station coordinates.
    ///
    /// This helper is a normalized surface-pressure front door. Native solver
    /// field-output parsers remain separate adapters that can emit this schema.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedCsv`] for CSV syntax/schema failures and
    /// [`AerodbError::InvalidInput`] for CG, axial stations, row count, node,
    /// panel, condition, provenance, reference scale, or numeric failures.
    pub fn integrate_surface_cp_csv_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        csv_text: &str,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;

        let panel_count = mesh.triangles().len();
        let expected_nodes = matrix.nodes().len();
        let expected_rows =
            expected_nodes
                .checked_mul(panel_count)
                .ok_or(AerodbError::InvalidInput {
                    field: "surface_cp_csv_rows",
                })?;
        let mut reader = csv::ReaderBuilder::new()
            .trim(csv::Trim::All)
            .from_reader(csv_text.as_bytes());
        let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
            reason: "surface Cp CSV header did not parse",
        })?;
        validate_surface_cp_csv_header(headers)?;

        let mut reports = Vec::with_capacity(expected_nodes);
        let mut force = Vector3::zeros();
        let mut moment = Vector3::zeros();
        let mut axial_line_loads = surface_cp_empty_axial_line_loads(axial_stations_m);
        let mut row_count = 0usize;
        for (record_index, record) in reader.records().enumerate() {
            let record = record.map_err(|_| AerodbError::MalformedCsv {
                reason: "surface Cp CSV row did not parse",
            })?;
            if record_index >= expected_rows {
                return Err(AerodbError::InvalidInput {
                    field: "surface_cp_csv_rows",
                });
            }
            let node_index = record_index / panel_count;
            let panel_index = record_index % panel_count;
            let node = matrix.nodes()[node_index];
            let cp = surface_cp_from_csv_record(&record, node_index, panel_index, node)?;
            let panel_load = surface_cp_panel_load(mesh, panel_index, cp, node, cg_body_m)?;
            force += panel_load.force_n_body;
            moment += panel_load.moment_n_m_body;
            if let Some(bin) = surface_cp_axial_station_bin(axial_stations_m, panel_load.centroid.x)
            {
                axial_line_loads[bin].force_n_body += panel_load.force_n_body;
                axial_line_loads[bin].moment_n_m_body += panel_load.moment_n_m_body;
            }
            row_count += 1;

            if panel_index + 1 == panel_count {
                reports.push(surface_cp_load_report_from_force_moment(
                    self,
                    node_index,
                    node,
                    force,
                    moment,
                    cg_body_m,
                    axial_line_loads,
                )?);
                force = Vector3::zeros();
                moment = Vector3::zeros();
                axial_line_loads = surface_cp_empty_axial_line_loads(axial_stations_m);
            }
        }
        if row_count != expected_rows || reports.len() != expected_nodes {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_csv_rows",
            });
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from a normalized surface-Cp
    /// CSV over a [`PanelMesh`].
    ///
    /// The CSV must use [`SURFACE_CP_CSV_SCHEMA_V1`] in every row, have exactly
    /// the expected header, and contain rows ordered by run-matrix node and then
    /// mesh triangle index. Each row must echo the node condition exactly,
    /// provide a finite `cp`, and carry nonblank solver/case/provenance fields.
    /// Pressures are integrated with the same panel convention as
    /// `LocalInclinationPanels`: `F_i = -Cp_i q A_i n_i`, with moments about the
    /// supplied body-frame CG.
    ///
    /// This helper is a normalized surface-pressure front door. Native solver
    /// field-output parsers remain separate adapters that can emit this schema.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for CSV syntax/schema failures,
    /// [`AerodbError::InvalidInput`] for CG, row count, node, panel, condition,
    /// provenance, or numeric failures, or any [`AeroError`] surfaced by
    /// [`AeroDeck::new`].
    pub fn bake_surface_cp_csv_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        csv_text: &str,
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_surface_cp_csv_loads(matrix, mesh, csv_text, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Integrate recorded SU2 surface CSV files into per-node force/moment load
    /// reports.
    ///
    /// Each surface CSV must contain one finite pressure coefficient for every
    /// [`PanelMesh`] vertex, with coordinates matching the mesh exactly. The
    /// parser accepts common SU2-style coordinate headers such as
    /// `Points_0`/`Points_1`/`Points_2` and pressure headers such as
    /// `Pressure_Coefficient`. Triangle pressure is the average of the three
    /// vertex values.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedCsv`] for malformed or incomplete SU2
    /// surface CSV data and [`AerodbError::InvalidInput`] for row count, node
    /// order, provenance, coordinate mapping, duplicate/missing vertices,
    /// axial stations, or non-finite coefficient failures.
    pub fn integrate_su2_surface_cp_csv_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[Su2SurfaceCpCsv<'_>],
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;
        let expected = matrix.nodes().len();
        if surfaces.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_rows",
            });
        }
        let mut reports = Vec::with_capacity(expected);
        for (record_index, surface) in surfaces.iter().enumerate() {
            if surface.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "su2_surface_cp_node_index",
                });
            }
            if surface.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "su2_surface_cp_case_id",
                });
            }
            if surface.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "su2_surface_cp_provenance",
                });
            }
            let vertex_cp = su2_surface_vertex_cp_from_csv(surface.csv_text, mesh)?;
            reports.push(surface_cp_load_report_from_vertex_cp(
                self,
                record_index,
                matrix.nodes()[record_index],
                mesh,
                &vertex_cp,
                cg_body_m,
                axial_stations_m,
            )?);
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from recorded SU2 surface CSV
    /// files.
    ///
    /// This is a solver-specific surface-pressure adapter. It parses one SU2
    /// surface CSV per run-matrix node, integrates vertex Cp over the supplied
    /// [`PanelMesh`], and projects the loads into the current reduced deck
    /// shape.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed SU2 surface CSV
    /// data, [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, coordinate mapping, duplicate/missing vertices, or non-finite
    /// coefficient failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_su2_surface_cp_csv_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[Su2SurfaceCpCsv<'_>],
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_su2_surface_cp_csv_loads(matrix, mesh, surfaces, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Integrate recorded Cart3D ASCII `.triq` files into per-node force/moment
    /// load reports.
    ///
    /// Each `.triq` source must contain the same finite vertices and triangles as
    /// the supplied [`PanelMesh`]. The adapter uses the first scalar at each
    /// vertex as Cp. Triangle pressure is the average of the three vertex values.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedCsv`] for malformed or incomplete Cart3D
    /// TRI/Q data and [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, geometry mismatch, duplicate/missing vertices, axial stations,
    /// or non-finite coefficient failures.
    pub fn integrate_cart3d_triq_surface_cp_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[Cart3dTriqSurfaceCp<'_>],
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;
        let expected = matrix.nodes().len();
        if surfaces.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_rows",
            });
        }
        let mut reports = Vec::with_capacity(expected);
        for (record_index, surface) in surfaces.iter().enumerate() {
            if surface.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "cart3d_triq_surface_cp_node_index",
                });
            }
            if surface.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "cart3d_triq_surface_cp_case_id",
                });
            }
            if surface.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "cart3d_triq_surface_cp_provenance",
                });
            }
            let vertex_cp = cart3d_triq_vertex_cp_from_text(surface.triq_text, mesh)?;
            reports.push(surface_cp_load_report_from_vertex_cp(
                self,
                record_index,
                matrix.nodes()[record_index],
                mesh,
                &vertex_cp,
                cg_body_m,
                axial_stations_m,
            )?);
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from recorded Cart3D ASCII
    /// `.triq` files.
    ///
    /// This parser is scoped to recorded Cart3D surface output. It validates the
    /// ASCII TRI/Q geometry against a [`PanelMesh`], maps the first vertex scalar
    /// to Cp, integrates the pressure loads, and projects the result into the
    /// current reduced deck shape.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed Cart3D TRI/Q data,
    /// [`AerodbError::InvalidInput`] for row count, node order, provenance,
    /// geometry mismatch, duplicate/missing vertices, or non-finite coefficient
    /// failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_cart3d_triq_surface_cp_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[Cart3dTriqSurfaceCp<'_>],
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_cart3d_triq_surface_cp_loads(matrix, mesh, surfaces, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Integrate recorded OpenFOAM sampled-surface Cp files into per-node
    /// force/moment load reports.
    ///
    /// Each text source must contain one finite pressure coefficient for every
    /// [`PanelMesh`] vertex. Non-comment rows are parsed as whitespace-delimited
    /// `x y z Cp`; coordinates must match mesh vertices exactly. Triangle
    /// pressure is the average of the three vertex values.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedCsv`] for malformed or incomplete
    /// OpenFOAM sampled-surface data and [`AerodbError::InvalidInput`] for row
    /// count, node order, provenance, coordinate mapping, duplicate/missing
    /// vertices, axial stations, or non-finite coefficient failures.
    pub fn integrate_openfoam_surface_cp_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[OpenFoamSurfaceCp<'_>],
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;
        let expected = matrix.nodes().len();
        if surfaces.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_rows",
            });
        }
        let mut reports = Vec::with_capacity(expected);
        for (record_index, surface) in surfaces.iter().enumerate() {
            if surface.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_surface_cp_node_index",
                });
            }
            if surface.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_surface_cp_case_id",
                });
            }
            if surface.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_surface_cp_provenance",
                });
            }
            let vertex_cp = openfoam_surface_vertex_cp_from_text(surface.text, mesh)?;
            reports.push(surface_cp_load_report_from_vertex_cp(
                self,
                record_index,
                matrix.nodes()[record_index],
                mesh,
                &vertex_cp,
                cg_body_m,
                axial_stations_m,
            )?);
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from recorded OpenFOAM
    /// sampled-surface Cp files.
    ///
    /// This parser is intentionally scoped to recorded sampled-surface text, not
    /// full OpenFOAM case execution. It maps per-vertex Cp rows to a
    /// [`PanelMesh`], integrates the pressure loads, and projects the result into
    /// the current reduced deck shape.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed OpenFOAM surface
    /// data, [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, coordinate mapping, duplicate/missing vertices, or non-finite
    /// coefficient failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_openfoam_surface_cp_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[OpenFoamSurfaceCp<'_>],
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_openfoam_surface_cp_loads(matrix, mesh, surfaces, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Integrate recorded OpenFOAM scalar-field boundary Cp values into
    /// per-node force/moment load reports.
    ///
    /// The parser consumes one OpenFOAM scalar field dictionary per run-matrix
    /// node, selects the source's `patch_name`, and requires that patch `value`
    /// be either uniform or a nonuniform list matching the mesh panel count.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedOpenFoam`] for malformed field
    /// dictionaries and [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, missing patch/value, value-count mismatch, non-finite loads,
    /// or invalid axial stations.
    pub fn integrate_openfoam_boundary_cp_field_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        fields: &[OpenFoamBoundaryCpField<'_>],
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_boundary_cp_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;
        let expected = matrix.nodes().len();
        if fields.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_boundary_cp_rows",
            });
        }
        let mut reports = Vec::with_capacity(expected);
        for (record_index, source) in fields.iter().enumerate() {
            if source.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_boundary_cp_node_index",
                });
            }
            if source.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_boundary_cp_case_id",
                });
            }
            if source.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_boundary_cp_provenance",
                });
            }
            if source.patch_name.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_boundary_cp_patch_name",
                });
            }
            let panel_cp = openfoam_boundary_patch_cp_from_field(
                source.field_text,
                source.patch_name,
                mesh.triangles().len(),
            )?;
            let mut panel_forces = Vec::with_capacity(panel_cp.len());
            for (panel_index, cp) in panel_cp.into_iter().enumerate() {
                panel_forces.push(
                    surface_cp_panel_load(
                        mesh,
                        panel_index,
                        cp,
                        matrix.nodes()[record_index],
                        cg_body_m,
                    )?
                    .force_n_body,
                );
            }
            reports.push(surface_cp_load_report_from_panel_forces(
                self,
                record_index,
                matrix.nodes()[record_index],
                mesh,
                &panel_forces,
                cg_body_m,
                axial_stations_m,
            )?);
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from recorded OpenFOAM
    /// scalar-field boundary Cp dictionaries.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, plus the same parsing/integration errors as
    /// [`Self::integrate_openfoam_boundary_cp_field_loads`].
    pub fn bake_openfoam_boundary_cp_field_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        fields: &[OpenFoamBoundaryCpField<'_>],
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_openfoam_boundary_cp_field_loads(matrix, mesh, fields, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Integrate recorded SPARTA per-surface-element force files into per-node
    /// force/moment load reports.
    ///
    /// Each text source must contain one finite body-frame force vector for every
    /// [`PanelMesh`] triangle. Rows are whitespace-delimited and may include a
    /// header naming `surf_id`, `fx`, `fy`, and `fz`; without a header, columns are
    /// interpreted in that order. Surface ids may be zero-based or one-based, but
    /// every panel must appear exactly once.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::MalformedCsv`] for malformed SPARTA surface-force
    /// data and [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, duplicate/missing/out-of-range surface ids, axial stations, or
    /// non-finite coefficient failures.
    pub fn integrate_sparta_surface_force_loads(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[SpartaSurfaceForces<'_>],
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<Vec<SurfaceCpLoadReport>, AerodbError> {
        if !(cg_body_m.x.is_finite() && cg_body_m.y.is_finite() && cg_body_m.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_cg_body_m",
            });
        }
        validate_surface_cp_axial_stations(axial_stations_m)?;
        let expected = matrix.nodes().len();
        if surfaces.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_rows",
            });
        }
        let mut reports = Vec::with_capacity(expected);
        for (record_index, surface) in surfaces.iter().enumerate() {
            if surface.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "sparta_surface_forces_node_index",
                });
            }
            if surface.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "sparta_surface_forces_case_id",
                });
            }
            if surface.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "sparta_surface_forces_provenance",
                });
            }
            let panel_forces = sparta_surface_forces_from_text(surface.text, mesh)?;
            reports.push(surface_cp_load_report_from_panel_forces(
                self,
                record_index,
                matrix.nodes()[record_index],
                mesh,
                &panel_forces,
                cg_body_m,
                axial_stations_m,
            )?);
        }
        Ok(reports)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from recorded SPARTA
    /// per-surface-element force files.
    ///
    /// This parser is scoped to recorded SPARTA surface-force output, not SPARTA
    /// subprocess execution. It maps per-surface force vectors to [`PanelMesh`]
    /// triangles, integrates force/moment loads, and projects them into the
    /// current reduced deck shape.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed SPARTA force data,
    /// [`AerodbError::InvalidInput`] for row count, node order, provenance,
    /// duplicate/missing/out-of-range surface ids, or non-finite coefficient
    /// failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_sparta_surface_force_deck(
        &self,
        matrix: &RunMatrix,
        mesh: &PanelMesh,
        surfaces: &[SpartaSurfaceForces<'_>],
        cg_body_m: Vector3<f64>,
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let reports =
            self.integrate_sparta_surface_force_loads(matrix, mesh, surfaces, cg_body_m, &[])?;
        let mut cn = Vec::with_capacity(reports.len());
        let mut cd = Vec::with_capacity(reports.len());
        let mut cm = Vec::with_capacity(reports.len());
        for report in reports {
            cn.push(report.reduced_coefficients.cn);
            cd.push(report.reduced_coefficients.cd);
            cm.push(report.reduced_coefficients.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from SU2 history CSV files.
    ///
    /// The `histories` slice must be in run-matrix order and contain one
    /// completed SU2 history CSV per node. The parser uses the final history row.
    /// It prefers SU2 body-force columns `CFx`, `CFz`, and `CMy`, converting them
    /// to OpenBMP's reduced body-axis coefficients as `CD = -CFx`, `CN = -CFz`,
    /// and `CM = CMy`. If `CFx`/`CFz` are absent, it accepts wind-axis `CD` and
    /// `CL` plus `CMy` only for zero-sideslip nodes and rotates by `alpha_deg`.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed or incomplete SU2
    /// history CSV data, [`AerodbError::InvalidInput`] for row count, node order,
    /// provenance, non-finite coefficient, or unsupported wind-axis conversion
    /// failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_su2_history_csv_deck(
        &self,
        matrix: &RunMatrix,
        histories: &[Su2HistoryCoefficientCsv<'_>],
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let expected = matrix.nodes().len();
        if histories.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "su2_history_rows",
            });
        }
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        for (record_index, history) in histories.iter().enumerate() {
            if history.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "su2_history_node_index",
                });
            }
            if history.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "su2_history_case_id",
                });
            }
            if history.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "su2_history_provenance",
                });
            }
            let coeffs =
                su2_history_coefficients_from_csv(history.csv_text, matrix.nodes()[record_index])?;
            cn.push(coeffs.cn);
            cd.push(coeffs.cd);
            cm.push(coeffs.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }

    /// Bake a reduced `(CN, CD, CM)` [`AeroDeck`] from OpenFOAM forceCoeffs
    /// tables.
    ///
    /// The `histories` slice must be in run-matrix order and contain one
    /// completed OpenFOAM force-coefficients table per node. The parser requires
    /// a comment header with `Cd`, `Cl`, and `CmPitch`, uses the final data row,
    /// and rotates wind-axis `Cd`/`Cl` into OpenBMP's reduced body-axis
    /// coefficients only for zero-sideslip nodes.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::UnsupportedDeckBake`] when non-deck axes are not
    /// singleton, [`AerodbError::MalformedCsv`] for malformed or incomplete
    /// OpenFOAM coefficient data, [`AerodbError::InvalidInput`] for row count,
    /// node order, provenance, non-finite coefficient, or nonzero-sideslip
    /// conversion failures, or any [`AeroError`] surfaced by [`AeroDeck::new`].
    pub fn bake_openfoam_force_coeffs_deck(
        &self,
        matrix: &RunMatrix,
        histories: &[OpenFoamForceCoeffs<'_>],
    ) -> Result<AeroDeck, AerodbError> {
        require_singleton_deck_axes(matrix)?;
        let expected = matrix.nodes().len();
        if histories.len() != expected {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_force_coeffs_rows",
            });
        }
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        for (record_index, history) in histories.iter().enumerate() {
            if history.node_index != record_index {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_force_coeffs_node_index",
                });
            }
            if history.case_id.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_force_coeffs_case_id",
                });
            }
            if history.provenance.trim().is_empty() {
                return Err(AerodbError::InvalidInput {
                    field: "openfoam_force_coeffs_provenance",
                });
            }
            let coeffs =
                openfoam_force_coeffs_from_text(history.text, matrix.nodes()[record_index])?;
            cn.push(coeffs.cn);
            cd.push(coeffs.cd);
            cm.push(coeffs.cm);
        }
        Ok(AeroDeck::new(
            matrix.axes.mach.clone(),
            matrix.axes.alpha_deg.clone(),
            matrix.axes.beta_deg.clone(),
            cn,
            cd,
            cm,
            self.reference_area_m2,
            self.reference_length_m,
        )?)
    }
}

/// Independent one-sigma sources for one coefficient at one deck node.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct CoefficientSigmaBudget {
    /// Wind-tunnel or flight-test evidence contribution.
    pub wind_tunnel: f64,
    /// CFD numerical contribution.
    pub cfd: f64,
    /// Model-form contribution.
    pub model_form: f64,
    /// Extrapolation contribution.
    pub extrapolation: f64,
}

impl CoefficientSigmaBudget {
    /// Construct a coefficient sigma budget from independent one-sigma terms.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any term is non-finite or
    /// negative.
    pub fn new(
        wind_tunnel: f64,
        cfd: f64,
        model_form: f64,
        extrapolation: f64,
    ) -> Result<Self, AerodbError> {
        let budget = Self {
            wind_tunnel,
            cfd,
            model_form,
            extrapolation,
        };
        budget.validate()?;
        Ok(budget)
    }

    /// Replace the CFD contribution with a three-grid GCI estimate.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::Uq`] when the GCI calculation fails.
    pub fn with_cfd_gci(
        mut self,
        fine: f64,
        medium: f64,
        coarse: f64,
        refinement_ratio: f64,
    ) -> Result<Self, AerodbError> {
        self.cfd = GridConvergence::from_three_grids(fine, medium, coarse, refinement_ratio)?
            .absolute_gci_fine;
        self.validate()?;
        Ok(self)
    }

    /// Aggregate the independent sources by RSS.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any source is malformed or
    /// the aggregate is non-finite.
    pub fn rss_one_sigma(&self) -> Result<f64, AerodbError> {
        self.validate()?;
        let sum = self.wind_tunnel * self.wind_tunnel
            + self.cfd * self.cfd
            + self.model_form * self.model_form
            + self.extrapolation * self.extrapolation;
        let sigma = sum.sqrt();
        if sigma.is_finite() {
            Ok(sigma)
        } else {
            Err(AerodbError::InvalidInput {
                field: "coefficient_sigma_budget.rss",
            })
        }
    }

    fn validate(&self) -> Result<(), AerodbError> {
        for (field, value) in [
            ("coefficient_sigma_budget.wind_tunnel", self.wind_tunnel),
            ("coefficient_sigma_budget.cfd", self.cfd),
            ("coefficient_sigma_budget.model_form", self.model_form),
            ("coefficient_sigma_budget.extrapolation", self.extrapolation),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(AerodbError::InvalidInput { field });
            }
        }
        Ok(())
    }
}

/// Reduced `(CN, CD, CM)` sigma budgets for one deck node.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ReducedCoefficientSigmaBudget {
    /// Normal-force coefficient sigma budget.
    pub cn: CoefficientSigmaBudget,
    /// Drag coefficient sigma budget.
    pub cd: CoefficientSigmaBudget,
    /// Pitching-moment coefficient sigma budget.
    pub cm: CoefficientSigmaBudget,
}

impl ReducedCoefficientSigmaBudget {
    /// Aggregate each coefficient budget by RSS.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any budget is malformed.
    pub fn rss(self) -> Result<AeroCoefficients, AerodbError> {
        Ok(AeroCoefficients {
            cn: self.cn.rss_one_sigma()?,
            cd: self.cd.rss_one_sigma()?,
            cm: self.cm.rss_one_sigma()?,
        })
    }
}

/// Six-channel `(CX, CY, CZ, Cl, Cm, Cn)` sigma budgets for one deck node.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct SixCoefficientSigmaBudget {
    /// Body-x force coefficient sigma budget.
    pub cx: CoefficientSigmaBudget,
    /// Body-y force coefficient sigma budget.
    pub cy: CoefficientSigmaBudget,
    /// Body-z force coefficient sigma budget.
    pub cz: CoefficientSigmaBudget,
    /// Roll-moment coefficient sigma budget.
    pub cl: CoefficientSigmaBudget,
    /// Pitch-moment coefficient sigma budget.
    pub cm: CoefficientSigmaBudget,
    /// Yaw-moment coefficient sigma budget.
    pub cn: CoefficientSigmaBudget,
}

impl SixCoefficientSigmaBudget {
    /// Aggregate each six-channel coefficient budget by RSS.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any budget is malformed.
    pub fn rss(self) -> Result<AeroSixCoefficients, AerodbError> {
        Ok(AeroSixCoefficients {
            cx: self.cx.rss_one_sigma()?,
            cy: self.cy.rss_one_sigma()?,
            cz: self.cz.rss_one_sigma()?,
            cl: self.cl.rss_one_sigma()?,
            cm: self.cm.rss_one_sigma()?,
            cn: self.cn.rss_one_sigma()?,
        })
    }
}

/// Validate an [`AeroDeck`] with nominal six-coefficient tables against
/// calibrated reference cases.
///
/// # Errors
///
/// Returns [`AerodbError::InvalidInput`] for empty/malformed validation cases,
/// missing six-coefficient tables, or tolerance failures, and
/// [`AerodbError::Aero`] for deck lookup failures.
pub fn validate_six_coefficient_deck(
    deck: &AeroDeck,
    cases: &[SixCoefficientValidationCase<'_>],
) -> Result<SixCoefficientValidationReport, AerodbError> {
    if cases.is_empty() {
        return Err(AerodbError::InvalidInput {
            field: "six_coefficient_validation_cases",
        });
    }
    if !deck.has_six_coefficient_tables() {
        return Err(AerodbError::InvalidInput {
            field: "six_coefficient_validation_tables",
        });
    }

    let deflections = BTreeMap::new();
    let mut rows = Vec::with_capacity(cases.len());
    for case in cases {
        validate_nonblank_solver_case_field(case.case_id, "six_coefficient_validation_case_id")?;
        validate_nonblank_solver_case_field(
            case.provenance,
            "six_coefficient_validation_provenance",
        )?;
        if !(case.mach.is_finite() && case.alpha_deg.is_finite() && case.beta_deg.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_condition",
            });
        }
        validate_six_coefficients_finite(case.expected, "six_coefficient_validation_expected")?;
        validate_six_coefficient_tolerances(case.tolerance)?;

        let actual = deck
            .lookup_six(case.mach, case.alpha_deg, case.beta_deg, &deflections)?
            .ok_or(AerodbError::InvalidInput {
                field: "six_coefficient_validation_tables",
            })?;
        let absolute_error = six_coefficient_abs_error(actual, case.expected);
        if !six_coefficient_errors_within_tolerance(absolute_error, case.tolerance) {
            return Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_tolerance",
            });
        }

        rows.push(SixCoefficientValidationRow {
            case_id: case.case_id.to_owned(),
            provenance: case.provenance.to_owned(),
            mach: case.mach,
            alpha_deg: case.alpha_deg,
            beta_deg: case.beta_deg,
            expected: case.expected,
            actual,
            absolute_error,
            tolerance: case.tolerance,
        });
    }

    Ok(SixCoefficientValidationReport { cases: rows })
}

/// Attach RSS-reduced sigma tables to a reduced three-axis [`AeroDeck`].
///
/// `budgets` must be row-major over the deck's mach/alpha/beta axes.
///
/// # Errors
///
/// Returns [`AerodbError::UnsupportedDeckBake`] for non-reduced decks,
/// [`AerodbError::InvalidInput`] for length or budget failures, or
/// [`AerodbError::Aero`] when [`AeroDeck::with_sigma_tables`] rejects the
/// produced tables.
pub fn attach_reduced_sigma_tables(
    deck: AeroDeck,
    budgets: &[ReducedCoefficientSigmaBudget],
) -> Result<AeroDeck, AerodbError> {
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "sigma budget assembly currently supports reduced mach/alpha/beta decks only",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "sigma_budget_node_count",
        })?;
    if budgets.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "sigma_budget_length",
        });
    }
    let mut cn = Vec::with_capacity(budgets.len());
    let mut cd = Vec::with_capacity(budgets.len());
    let mut cm = Vec::with_capacity(budgets.len());
    for budget in budgets {
        let sigma = budget.rss()?;
        cn.push(sigma.cn);
        cd.push(sigma.cd);
        cm.push(sigma.cm);
    }
    Ok(deck.with_sigma_tables(cn, cd, cm)?)
}

/// Attach RSS six-channel sigma tables to a reduced three-axis [`AeroDeck`]
/// that already carries nominal six-coefficient tables.
///
/// `budgets` must be row-major over the deck's mach/alpha/beta axes.
///
/// # Errors
///
/// Returns [`AerodbError::UnsupportedDeckBake`] for non-reduced decks or decks
/// without nominal six-coefficient tables, [`AerodbError::InvalidInput`] for
/// length or budget failures, or [`AerodbError::Aero`] when
/// [`AeroDeck::with_six_sigma_tables`] rejects the produced tables.
pub fn attach_six_sigma_tables(
    deck: AeroDeck,
    budgets: &[SixCoefficientSigmaBudget],
) -> Result<AeroDeck, AerodbError> {
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "six sigma budget assembly currently supports reduced mach/alpha/beta decks only",
        });
    }
    if !deck.has_six_coefficient_tables() {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "six sigma budget assembly requires nominal six coefficient tables",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "six_sigma_budget_node_count",
        })?;
    if budgets.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "six_sigma_budget_length",
        });
    }
    let mut cx = Vec::with_capacity(budgets.len());
    let mut cy = Vec::with_capacity(budgets.len());
    let mut cz = Vec::with_capacity(budgets.len());
    let mut cl = Vec::with_capacity(budgets.len());
    let mut cm = Vec::with_capacity(budgets.len());
    let mut cn = Vec::with_capacity(budgets.len());
    for budget in budgets {
        let sigma = budget.rss()?;
        cx.push(sigma.cx);
        cy.push(sigma.cy);
        cz.push(sigma.cz);
        cl.push(sigma.cl);
        cm.push(sigma.cm);
        cn.push(sigma.cn);
    }
    Ok(deck.with_six_sigma_tables(cx, cy, cz, cl, cm, cn)?)
}

/// Attach reduced-deck sigma tables from a strict external-solver GCI TOML
/// document.
///
/// The document is row-major over the deck's mach/alpha/beta axes and each row
/// carries fine/medium/coarse values for CN, CD, and CM. The helper converts
/// those three-grid studies into CFD one-sigma terms, RSS-combines them with
/// optional source terms, and attaches the resulting sigma tables to `deck`.
///
/// # Errors
///
/// Returns [`AerodbError::MalformedToml`] for TOML schema failures,
/// [`AerodbError::UnsupportedDeckBake`] for non-reduced decks, and
/// [`AerodbError::InvalidInput`] / [`AerodbError::Uq`] for invalid row counts,
/// source terms, or GCI studies.
pub fn attach_reduced_sigma_tables_from_gci_toml(
    deck: AeroDeck,
    toml_text: &str,
) -> Result<AeroDeck, AerodbError> {
    let parsed: GciSigmaToml =
        toml::from_str(toml_text).map_err(|_| AerodbError::MalformedToml {
            reason: "aero GCI sigma TOML did not parse against schema",
        })?;
    if parsed.openbmp.aerodb_gci_sigma != GCI_SIGMA_SCHEMA_VERSION_1 {
        return Err(AerodbError::MalformedToml {
            reason: "openbmp.aerodb_gci_sigma schema version must be 1",
        });
    }
    if parsed.provenance.trim().is_empty() {
        return Err(AerodbError::MalformedToml {
            reason: "aero GCI sigma provenance must not be blank",
        });
    }
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "GCI sigma ingestion currently supports reduced mach/alpha/beta decks only",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "gci_sigma_node_count",
        })?;
    if parsed.rows.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "gci_sigma_rows",
        });
    }
    let mut budgets = Vec::with_capacity(parsed.rows.len());
    for row in parsed.rows {
        budgets.push(row.into_budget(parsed.refinement_ratio, parsed.safety_factor)?);
    }
    attach_reduced_sigma_tables(deck, &budgets)
}

/// Attach six-channel sigma tables from a strict external-solver GCI TOML
/// document.
///
/// The document is row-major over the deck's mach/alpha/beta axes and each row
/// carries fine/medium/coarse values for CX, CY, CZ, Cl, Cm, and Cn.
///
/// # Errors
///
/// Returns [`AerodbError::MalformedToml`] for TOML schema failures,
/// [`AerodbError::UnsupportedDeckBake`] for non-reduced decks or decks without
/// nominal six tables, and [`AerodbError::InvalidInput`] /
/// [`AerodbError::Uq`] for invalid row counts, source terms, or GCI studies.
pub fn attach_six_sigma_tables_from_gci_toml(
    deck: AeroDeck,
    toml_text: &str,
) -> Result<AeroDeck, AerodbError> {
    let parsed: SixGciSigmaToml =
        toml::from_str(toml_text).map_err(|_| AerodbError::MalformedToml {
            reason: "aero six GCI sigma TOML did not parse against schema",
        })?;
    if parsed.openbmp.aerodb_six_gci_sigma != SIX_GCI_SIGMA_SCHEMA_VERSION_1 {
        return Err(AerodbError::MalformedToml {
            reason: "openbmp.aerodb_six_gci_sigma schema version must be 1",
        });
    }
    if parsed.provenance.trim().is_empty() {
        return Err(AerodbError::MalformedToml {
            reason: "aero six GCI sigma provenance must not be blank",
        });
    }
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "six GCI sigma ingestion currently supports reduced mach/alpha/beta decks only",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "six_gci_sigma_node_count",
        })?;
    if parsed.rows.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "six_gci_sigma_rows",
        });
    }
    let mut budgets = Vec::with_capacity(parsed.rows.len());
    for row in parsed.rows {
        budgets.push(row.into_budget(parsed.refinement_ratio, parsed.safety_factor)?);
    }
    attach_six_sigma_tables(deck, &budgets)
}

/// Attach reduced-deck sigma tables from a strict external-solver coefficient
/// CSV document.
///
/// The CSV must use [`GCI_SIGMA_CSV_SCHEMA_V1`] in every row, have exactly the
/// expected header, carry sequential `node_index` values in reduced-deck
/// row-major order, and provide fine/medium/coarse CN/CD/CM values plus
/// explicit source terms. `refinement_ratio` and `safety_factor` are shared by
/// every row.
///
/// # Errors
///
/// Returns [`AerodbError::MalformedCsv`] for CSV syntax/schema failures,
/// [`AerodbError::UnsupportedDeckBake`] for non-reduced decks, and
/// [`AerodbError::InvalidInput`] / [`AerodbError::Uq`] for invalid row counts,
/// numeric fields, source terms, or GCI studies.
pub fn attach_reduced_sigma_tables_from_gci_csv(
    deck: AeroDeck,
    csv_text: &str,
    refinement_ratio: f64,
    safety_factor: f64,
) -> Result<AeroDeck, AerodbError> {
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "GCI CSV ingestion currently supports reduced mach/alpha/beta decks only",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "gci_sigma_node_count",
        })?;
    let mut reader = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(csv_text.as_bytes());
    let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
        reason: "aero GCI sigma CSV header did not parse",
    })?;
    validate_gci_sigma_csv_header(headers)?;

    let mut budgets = Vec::with_capacity(expected);
    for (record_index, record) in reader.records().enumerate() {
        let record = record.map_err(|_| AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV row did not parse",
        })?;
        budgets.push(gci_sigma_budget_from_csv_record(
            &record,
            record_index,
            refinement_ratio,
            safety_factor,
        )?);
    }
    if budgets.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "gci_sigma_csv_rows",
        });
    }
    attach_reduced_sigma_tables(deck, &budgets)
}

/// Attach six-channel sigma tables from a strict external-solver coefficient
/// CSV document.
///
/// The CSV must use [`SIX_GCI_SIGMA_CSV_SCHEMA_V1`] in every row, have exactly
/// the expected header, carry sequential `node_index` values in reduced-deck
/// row-major order, and provide fine/medium/coarse CX/CY/CZ/Cl/Cm/Cn values
/// plus explicit source terms.
///
/// # Errors
///
/// Returns [`AerodbError::MalformedCsv`] for CSV syntax/schema failures,
/// [`AerodbError::UnsupportedDeckBake`] for non-reduced decks or decks without
/// nominal six tables, and [`AerodbError::InvalidInput`] /
/// [`AerodbError::Uq`] for invalid row counts, numeric fields, source terms, or
/// GCI studies.
pub fn attach_six_sigma_tables_from_gci_csv(
    deck: AeroDeck,
    csv_text: &str,
    refinement_ratio: f64,
    safety_factor: f64,
) -> Result<AeroDeck, AerodbError> {
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "six GCI CSV ingestion currently supports reduced mach/alpha/beta decks only",
        });
    }
    let expected = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "six_gci_sigma_node_count",
        })?;
    let mut reader = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(csv_text.as_bytes());
    let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
        reason: "aero six GCI sigma CSV header did not parse",
    })?;
    validate_six_gci_sigma_csv_header(headers)?;

    let mut budgets = Vec::with_capacity(expected);
    for (record_index, record) in reader.records().enumerate() {
        let record = record.map_err(|_| AerodbError::MalformedCsv {
            reason: "aero six GCI sigma CSV row did not parse",
        })?;
        budgets.push(six_gci_sigma_budget_from_csv_record(
            &record,
            record_index,
            refinement_ratio,
            safety_factor,
        )?);
    }
    if budgets.len() != expected {
        return Err(AerodbError::InvalidInput {
            field: "six_gci_sigma_csv_rows",
        });
    }
    attach_six_sigma_tables(deck, &budgets)
}

/// Deterministic settings for a Mach-correlated reduced-deck draw.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CorrelatedDeckDrawSettings {
    /// Campaign-level seed used for deterministic MC streams.
    pub campaign_seed: u64,
    /// Sample index in the campaign.
    pub sample_index: u64,
    /// First deterministic RNG dimension id. CN, CD, and CM consume this
    /// dimension and the next two dimensions.
    pub dimension_id_base: u32,
    /// Exponential Mach correlation length, in Mach units.
    pub mach_correlation_length: f64,
    /// Optional exponential angle-of-attack correlation length, in degrees.
    ///
    /// When absent, each Mach residual remains shared across the full
    /// alpha/beta plane for backward-compatible WP-03.6 behavior.
    pub alpha_correlation_length_deg: Option<f64>,
    /// Optional exponential sideslip correlation length, in degrees.
    ///
    /// When absent, each Mach/alpha residual remains shared across beta.
    pub beta_correlation_length_deg: Option<f64>,
}

impl CorrelatedDeckDrawSettings {
    /// Construct validated correlated-draw settings.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when the Mach correlation length
    /// is non-finite or non-positive, or when the base dimension id does not
    /// leave room for the three coefficient streams.
    pub fn new(
        campaign_seed: u64,
        sample_index: u64,
        dimension_id_base: u32,
        mach_correlation_length: f64,
    ) -> Result<Self, AerodbError> {
        let settings = Self {
            campaign_seed,
            sample_index,
            dimension_id_base,
            mach_correlation_length,
            alpha_correlation_length_deg: None,
            beta_correlation_length_deg: None,
        };
        settings.validate()?;
        Ok(settings)
    }

    /// Construct settings with optional alpha/beta covariance lengths.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when any supplied correlation
    /// length is non-finite or non-positive, or when the base dimension id does
    /// not leave room for the required coefficient streams.
    pub fn new_with_axis_correlation_lengths(
        campaign_seed: u64,
        sample_index: u64,
        dimension_id_base: u32,
        mach_correlation_length: f64,
        alpha_correlation_length_deg: Option<f64>,
        beta_correlation_length_deg: Option<f64>,
    ) -> Result<Self, AerodbError> {
        let settings = Self {
            campaign_seed,
            sample_index,
            dimension_id_base,
            mach_correlation_length,
            alpha_correlation_length_deg,
            beta_correlation_length_deg,
        };
        settings.validate()?;
        Ok(settings)
    }

    /// Construct settings, using deck-declared correlation metadata when the
    /// caller does not provide an explicit Mach correlation length.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when neither an explicit length nor
    /// deck metadata is available, when the resolved length is invalid, or when
    /// the dimension id base is invalid.
    pub fn new_with_deck_default(
        campaign_seed: u64,
        sample_index: u64,
        dimension_id_base: u32,
        mach_correlation_length: Option<f64>,
        deck: &AeroDeck,
    ) -> Result<Self, AerodbError> {
        Self::new_with_deck_default_and_axis_correlation_lengths(
            campaign_seed,
            sample_index,
            dimension_id_base,
            mach_correlation_length,
            None,
            None,
            deck,
        )
    }

    /// Construct settings with deck-declared Mach covariance fallback and
    /// optional alpha/beta covariance lengths.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] when neither an explicit Mach
    /// length nor deck metadata is available, when any resolved/supplied
    /// length is invalid, or when the dimension id base is invalid.
    pub fn new_with_deck_default_and_axis_correlation_lengths(
        campaign_seed: u64,
        sample_index: u64,
        dimension_id_base: u32,
        mach_correlation_length: Option<f64>,
        alpha_correlation_length_deg: Option<f64>,
        beta_correlation_length_deg: Option<f64>,
        deck: &AeroDeck,
    ) -> Result<Self, AerodbError> {
        let mach_correlation_length = match mach_correlation_length {
            Some(value) => value,
            None => deck
                .uncertainty_correlation()
                .ok_or(AerodbError::InvalidInput {
                    field: "mach_correlation_length",
                })?
                .mach_length(),
        };
        Self::new_with_axis_correlation_lengths(
            campaign_seed,
            sample_index,
            dimension_id_base,
            mach_correlation_length,
            alpha_correlation_length_deg,
            beta_correlation_length_deg,
        )
    }

    fn validate(self) -> Result<(), AerodbError> {
        if !(self.mach_correlation_length.is_finite() && self.mach_correlation_length > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "mach_correlation_length",
            });
        }
        if self
            .alpha_correlation_length_deg
            .is_some_and(|value| !(value.is_finite() && value > 0.0))
        {
            return Err(AerodbError::InvalidInput {
                field: "alpha_correlation_length_deg",
            });
        }
        if self
            .beta_correlation_length_deg
            .is_some_and(|value| !(value.is_finite() && value > 0.0))
        {
            return Err(AerodbError::InvalidInput {
                field: "beta_correlation_length_deg",
            });
        }
        if self.dimension_id_base.checked_add(2).is_none() {
            return Err(AerodbError::InvalidInput {
                field: "dimension_id_base",
            });
        }
        Ok(())
    }
}

/// Number of deterministic RNG dimensions a correlated deck draw consumes.
///
/// Reduced-only draws consume three streams (`CN`, `CD`, `CM`). Decks carrying
/// six-channel sigma tables also consume six body-axis streams (`CX`, `CY`,
/// `CZ`, `Cl`, `Cm`, `Cn`) after the reduced streams.
#[must_use]
pub fn correlated_deck_draw_required_dimension_count(deck: &AeroDeck) -> u32 {
    if deck.has_six_sigma_tables() { 9 } else { 3 }
}

/// Validate that a first RNG dimension id can reserve the streams needed by a
/// correlated deck draw for `deck`.
///
/// # Errors
///
/// Returns [`AerodbError::InvalidInput`] when `dimension_id_base` cannot reserve
/// the required contiguous dimension range.
pub fn validate_correlated_deck_draw_dimension_capacity(
    deck: &AeroDeck,
    dimension_id_base: u32,
) -> Result<(), AerodbError> {
    let highest_offset = correlated_deck_draw_required_dimension_count(deck) - 1;
    validate_dimension_capacity(dimension_id_base, highest_offset)
}

/// Draw a deterministic correlated deck sample.
///
/// The draw is off by default: when `deck` has no reduced or six-channel sigma
/// tables, this returns an exact clone of `deck`. When reduced sigma tables are
/// present, CN, CD, and CM each receive an independent standard-normal residual
/// field over the Mach/alpha/beta grid, correlated by exponential covariance
/// and Cholesky factorization. When six-channel sigma tables are present, CX,
/// CY, CZ, Cl, Cm, and Cn receive independent residual fields from separate
/// deterministic RNG dimensions. The default settings retain the original
/// same-Mach residual sharing across the alpha/beta plane; optional alpha/beta
/// correlation lengths expand the residuals into separable grid fields. Each
/// perturbation is scaled by the node's own coefficient sigma.
///
/// # Errors
///
/// Returns [`AerodbError::UnsupportedDeckBake`] for non-reduced decks,
/// [`AerodbError::InvalidInput`] for invalid settings or failed covariance
/// factorization, or [`AerodbError::Aero`] for deck lookup/construction errors.
pub fn draw_correlated_aero_deck(
    deck: &AeroDeck,
    settings: CorrelatedDeckDrawSettings,
) -> Result<AeroDeck, AerodbError> {
    settings.validate()?;
    if deck.axis_order().len() != 3 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "correlated deck draw currently supports reduced mach/alpha/beta decks only",
        });
    }
    let draw_reduced = deck.has_sigma_tables();
    let draw_six = deck.has_six_sigma_tables();
    let preserve_six = deck.has_six_coefficient_tables();
    validate_correlated_deck_draw_dimension_capacity(deck, settings.dimension_id_base)?;
    if !(draw_reduced || draw_six) {
        return Ok(deck.clone());
    }

    let reduced_residuals = if draw_reduced {
        Some(correlated_coefficient_residuals(
            deck.mach_grid(),
            deck.alpha_grid_deg(),
            deck.beta_grid_deg(),
            settings,
            deck,
        )?)
    } else {
        None
    };
    let six_residuals = if draw_six {
        Some(correlated_six_residuals(
            deck.mach_grid(),
            deck.alpha_grid_deg(),
            deck.beta_grid_deg(),
            settings,
            3,
            deck,
        )?)
    } else {
        None
    };
    let deflections = BTreeMap::<&str, f64>::new();

    let node_count = deck
        .mach_grid()
        .len()
        .checked_mul(deck.alpha_grid_deg().len())
        .and_then(|value| value.checked_mul(deck.beta_grid_deg().len()))
        .ok_or(AerodbError::InvalidInput {
            field: "draw_node_count",
        })?;
    let mut cn = Vec::with_capacity(node_count);
    let mut cd = Vec::with_capacity(node_count);
    let mut cm = Vec::with_capacity(node_count);
    let mut cn_sigma = Vec::with_capacity(node_count);
    let mut cd_sigma = Vec::with_capacity(node_count);
    let mut cm_sigma = Vec::with_capacity(node_count);
    let mut six_cx = Vec::with_capacity(node_count);
    let mut six_cy = Vec::with_capacity(node_count);
    let mut six_cz = Vec::with_capacity(node_count);
    let mut six_cl = Vec::with_capacity(node_count);
    let mut six_cm = Vec::with_capacity(node_count);
    let mut six_cn = Vec::with_capacity(node_count);
    let mut six_cx_sigma = Vec::with_capacity(node_count);
    let mut six_cy_sigma = Vec::with_capacity(node_count);
    let mut six_cz_sigma = Vec::with_capacity(node_count);
    let mut six_cl_sigma = Vec::with_capacity(node_count);
    let mut six_cm_sigma = Vec::with_capacity(node_count);
    let mut six_cn_sigma = Vec::with_capacity(node_count);

    let mut node_index = 0usize;
    for &mach in deck.mach_grid() {
        for &alpha_deg in deck.alpha_grid_deg() {
            for &beta_deg in deck.beta_grid_deg() {
                let coeffs = deck.lookup(mach, alpha_deg, beta_deg, &deflections)?;
                if let Some([cn_residual, cd_residual, cm_residual]) = &reduced_residuals {
                    let sigma = deck
                        .lookup_sigma(mach, alpha_deg, beta_deg, &deflections)?
                        .ok_or(AerodbError::InvalidInput {
                            field: "deck_sigma_tables",
                        })?;
                    cn.push(coeffs.cn + sigma.cn * cn_residual[node_index]);
                    cd.push(coeffs.cd + sigma.cd * cd_residual[node_index]);
                    cm.push(coeffs.cm + sigma.cm * cm_residual[node_index]);
                    cn_sigma.push(sigma.cn);
                    cd_sigma.push(sigma.cd);
                    cm_sigma.push(sigma.cm);
                } else {
                    cn.push(coeffs.cn);
                    cd.push(coeffs.cd);
                    cm.push(coeffs.cm);
                }

                if preserve_six {
                    let six = deck
                        .lookup_six(mach, alpha_deg, beta_deg, &deflections)?
                        .ok_or(AerodbError::InvalidInput {
                            field: "deck_six_tables",
                        })?;
                    if let Some(
                        [
                            cx_residual,
                            cy_residual,
                            cz_residual,
                            cl_residual,
                            six_cm_residual,
                            six_cn_residual,
                        ],
                    ) = &six_residuals
                    {
                        let sigma = deck
                            .lookup_six_sigma(mach, alpha_deg, beta_deg, &deflections)?
                            .ok_or(AerodbError::InvalidInput {
                                field: "deck_six_sigma_tables",
                            })?;
                        six_cx.push(six.cx + sigma.cx * cx_residual[node_index]);
                        six_cy.push(six.cy + sigma.cy * cy_residual[node_index]);
                        six_cz.push(six.cz + sigma.cz * cz_residual[node_index]);
                        six_cl.push(six.cl + sigma.cl * cl_residual[node_index]);
                        six_cm.push(six.cm + sigma.cm * six_cm_residual[node_index]);
                        six_cn.push(six.cn + sigma.cn * six_cn_residual[node_index]);
                        six_cx_sigma.push(sigma.cx);
                        six_cy_sigma.push(sigma.cy);
                        six_cz_sigma.push(sigma.cz);
                        six_cl_sigma.push(sigma.cl);
                        six_cm_sigma.push(sigma.cm);
                        six_cn_sigma.push(sigma.cn);
                    } else {
                        six_cx.push(six.cx);
                        six_cy.push(six.cy);
                        six_cz.push(six.cz);
                        six_cl.push(six.cl);
                        six_cm.push(six.cm);
                        six_cn.push(six.cn);
                    }
                }
                node_index += 1;
            }
        }
    }

    let drawn = AeroDeck::new(
        deck.mach_grid().to_vec(),
        deck.alpha_grid_deg().to_vec(),
        deck.beta_grid_deg().to_vec(),
        cn,
        cd,
        cm,
        deck.reference_area_m2(),
        deck.reference_length_m(),
    )?
    .with_extrapolation_policy(deck.extrapolation_policy());
    let drawn = if let Some(correlation) = deck.uncertainty_correlation() {
        drawn.with_uncertainty_correlation(correlation)
    } else {
        drawn
    };
    let drawn = if preserve_six {
        drawn.with_six_coefficient_tables(six_cx, six_cy, six_cz, six_cl, six_cm, six_cn)?
    } else {
        drawn
    };
    let drawn = if draw_reduced {
        drawn.with_sigma_tables(cn_sigma, cd_sigma, cm_sigma)?
    } else {
        drawn
    };
    let drawn = if draw_six {
        drawn.with_six_sigma_tables(
            six_cx_sigma,
            six_cy_sigma,
            six_cz_sigma,
            six_cl_sigma,
            six_cm_sigma,
            six_cn_sigma,
        )?
    } else {
        drawn
    };
    Ok(drawn)
}

/// Draw a deterministic Mach-correlated reduced `(CN, CD, CM)` deck sample.
///
/// This compatibility entry point now delegates to
/// [`draw_correlated_aero_deck`], preserving optional six-coefficient tables
/// when they are present.
///
/// # Errors
///
/// Returns the same errors as [`draw_correlated_aero_deck`].
pub fn draw_reduced_correlated_deck(
    deck: &AeroDeck,
    settings: CorrelatedDeckDrawSettings,
) -> Result<AeroDeck, AerodbError> {
    draw_correlated_aero_deck(deck, settings)
}

#[derive(Copy, Clone)]
enum AxisEnvelope {
    AnyFinite,
    NonNegative,
    Positive,
}

fn validate_axis(
    axis: &[f64],
    field: &'static str,
    envelope: AxisEnvelope,
) -> Result<(), AerodbError> {
    if axis.is_empty() {
        return Err(AerodbError::InvalidInput { field });
    }
    let mut previous = None;
    for &value in axis {
        if !value.is_finite() {
            return Err(AerodbError::InvalidInput { field });
        }
        match envelope {
            AxisEnvelope::AnyFinite => {}
            AxisEnvelope::NonNegative if value < 0.0 => {
                return Err(AerodbError::InvalidInput { field });
            }
            AxisEnvelope::Positive if value <= 0.0 => {
                return Err(AerodbError::InvalidInput { field });
            }
            AxisEnvelope::NonNegative | AxisEnvelope::Positive => {}
        }
        if let Some(previous_value) = previous
            && value <= previous_value
        {
            return Err(AerodbError::InvalidInput { field });
        }
        previous = Some(value);
    }
    Ok(())
}

fn matrix_node_count(axes: &RunMatrixAxes) -> Result<usize, AerodbError> {
    [
        axes.mach.len(),
        axes.alpha_deg.len(),
        axes.beta_deg.len(),
        axes.reynolds_length.len(),
        axes.knudsen.len(),
        axes.dynamic_pressure_pa.len(),
    ]
    .into_iter()
    .try_fold(1usize, |acc, len| {
        acc.checked_mul(len).ok_or(AerodbError::InvalidInput {
            field: "axis_size_product",
        })
    })
}

fn require_singleton_deck_axes(matrix: &RunMatrix) -> Result<(), AerodbError> {
    if matrix.axes.reynolds_length.len() != 1 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "reynolds_length axis must be singleton for reduced AeroDeck baking",
        });
    }
    if matrix.axes.knudsen.len() != 1 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "knudsen axis must be singleton for reduced AeroDeck baking",
        });
    }
    if matrix.axes.dynamic_pressure_pa.len() != 1 {
        return Err(AerodbError::UnsupportedDeckBake {
            reason: "dynamic_pressure_pa axis must be singleton for reduced AeroDeck baking",
        });
    }
    Ok(())
}

fn lint_toml_value_for_condition_only_keys(
    value: &toml::Value,
    path: &str,
) -> Result<(), AerodbError> {
    match value {
        toml::Value::Table(table) => {
            for (key, child) in table {
                let child_path = if path.is_empty() {
                    key.clone()
                } else {
                    format!("{path}.{key}")
                };
                if run_matrix_key_has_forbidden_vocabulary(key) {
                    return Err(AerodbError::ForbiddenVocabulary { field: child_path });
                }
                lint_toml_value_for_condition_only_keys(child, &child_path)?;
            }
            Ok(())
        }
        toml::Value::Array(values) => {
            for (index, child) in values.iter().enumerate() {
                lint_toml_value_for_condition_only_keys(child, &format!("{path}[{index}]"))?;
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

fn run_matrix_key_has_forbidden_vocabulary(key: &str) -> bool {
    key.split(|ch: char| !ch.is_ascii_alphanumeric())
        .any(|token| matches!(token, "target" | "aimpoint" | "range"))
}

fn validate_solver_case_condition(
    actual: f64,
    expected: f64,
    field: &'static str,
) -> Result<(), AerodbError> {
    if !(actual.is_finite() && actual.to_bits() == expected.to_bits()) {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn validate_nonblank_solver_case_field(
    value: &str,
    field: &'static str,
) -> Result<(), AerodbError> {
    if value.trim().is_empty() {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn validate_solver_command_text(value: &str, field: &'static str) -> Result<(), AerodbError> {
    if value.trim().is_empty() || value.chars().any(char::is_control) {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn validate_solver_env_key(value: &str) -> Result<(), AerodbError> {
    if value.trim().is_empty() || value.contains('=') || value.chars().any(char::is_control) {
        return Err(AerodbError::InvalidInput {
            field: "solver_command_plan_env_key",
        });
    }
    Ok(())
}

fn validate_solver_relative_path(value: &str, field: &'static str) -> Result<(), AerodbError> {
    let _ = resolve_solver_plan_path(Path::new("."), value, field)?;
    Ok(())
}

fn validate_solver_plan_filename(value: &str, field: &'static str) -> Result<(), AerodbError> {
    validate_solver_command_text(value, field)?;
    let mut components = Path::new(value).components();
    match (components.next(), components.next()) {
        (Some(Component::Normal(_)), None) => Ok(()),
        _ => Err(AerodbError::InvalidInput { field }),
    }
}

fn validate_case_id_prefix(value: &str, field: &'static str) -> Result<(), AerodbError> {
    if value.trim().is_empty()
        || value
            .chars()
            .any(|ch| ch.is_control() || ch.is_whitespace() || matches!(ch, '/' | '\\'))
    {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn validate_scheduler_token(value: &str, field: &'static str) -> Result<(), AerodbError> {
    if value.trim().is_empty()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
    {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn validate_su2_marker(value: &str) -> Result<(), AerodbError> {
    if value.trim().is_empty()
        || value
            .chars()
            .any(|ch| ch.is_control() || ch.is_whitespace() || matches!(ch, '(' | ')' | ','))
    {
        return Err(AerodbError::InvalidInput {
            field: "su2_euler_surface_marker",
        });
    }
    Ok(())
}

fn render_slurm_array_script(
    plan: &SolverCommandPlan,
    template: &SolverClusterScheduleTemplate,
) -> String {
    let mut text = String::new();
    text.push_str("#!/usr/bin/env bash\n");
    text.push_str("set -euo pipefail\n\n");
    text.push_str(&format!("#SBATCH --job-name={}\n", template.job_name));
    text.push_str(&format!("#SBATCH --array=0-{}\n", plan.cases.len() - 1));
    text.push_str(&format!(
        "#SBATCH --time={}\n",
        format_slurm_duration(template.wall_time_s)
    ));
    text.push_str(&format!(
        "#SBATCH --ntasks={}\n",
        template.mpi_ranks_per_case
    ));
    text.push_str(&format!(
        "#SBATCH --cpus-per-task={}\n",
        template.cpus_per_task
    ));
    text.push_str(&format!(
        "#SBATCH --output={}/%x-%A-%a.out\n",
        template.log_dir
    ));
    text.push_str(&format!(
        "#SBATCH --error={}/%x-%A-%a.err\n",
        template.log_dir
    ));
    if let Some(account) = &template.account {
        text.push_str(&format!("#SBATCH --account={account}\n"));
    }
    if let Some(partition) = &template.partition {
        text.push_str(&format!("#SBATCH --partition={partition}\n"));
    }
    text.push_str("\n: \"${OPENBMP_SOLVER_ROOT:?set OPENBMP_SOLVER_ROOT}\"\n");
    text.push_str(&format!(
        "mkdir -p -- \"$OPENBMP_SOLVER_ROOT\"/{}\n\n",
        shell_single_quote(&template.log_dir)
    ));
    text.push_str("case \"${SLURM_ARRAY_TASK_ID:?missing SLURM_ARRAY_TASK_ID}\" in\n");
    for case in &plan.cases {
        text.push_str(&format!("  {})\n", case.node_index));
        text.push_str(&format!("    # openbmp_case_id={}\n", case.case_id));
        text.push_str(&format!("    # openbmp_solver={}\n", case.solver));
        text.push_str(&format!("    # openbmp_timeout_s={}\n", case.timeout_s));
        text.push_str(&format!(
            "    cd -- \"$OPENBMP_SOLVER_ROOT\"/{}\n",
            shell_single_quote(&case.working_dir)
        ));
        for (key, value) in &case.env {
            text.push_str(&format!("    export {key}={}\n", shell_single_quote(value)));
        }
        text.push_str("    srun");
        text.push_str(&format!(" --ntasks={}", template.mpi_ranks_per_case));
        text.push_str(&format!(" --cpus-per-task={}", template.cpus_per_task));
        text.push(' ');
        text.push_str(&shell_single_quote(&case.program));
        for arg in &case.args {
            text.push(' ');
            text.push_str(&shell_single_quote(arg));
        }
        text.push('\n');
        text.push_str("    ;;\n");
    }
    text.push_str("  *)\n");
    text.push_str("    echo \"unknown SLURM_ARRAY_TASK_ID=${SLURM_ARRAY_TASK_ID}\" >&2\n");
    text.push_str("    exit 64\n");
    text.push_str("    ;;\n");
    text.push_str("esac\n");
    text
}

struct ClusterCommandOutput {
    stdout: Vec<u8>,
    stderr: Vec<u8>,
}

struct ClusterCommandFields {
    root: &'static str,
    command: &'static str,
    timeout_s: &'static str,
    spawn: &'static str,
    wait: &'static str,
    timeout: &'static str,
    status: &'static str,
}

const SOLVER_CLUSTER_SUBMIT_FIELDS: ClusterCommandFields = ClusterCommandFields {
    root: "solver_cluster_submit_root",
    command: "solver_cluster_submit_command",
    timeout_s: "solver_cluster_submit_timeout_s",
    spawn: "solver_cluster_submit_spawn",
    wait: "solver_cluster_submit_wait",
    timeout: "solver_cluster_submit_timeout",
    status: "solver_cluster_submit_status",
};

const SOLVER_CLUSTER_QUERY_FIELDS: ClusterCommandFields = ClusterCommandFields {
    root: "solver_cluster_query_root",
    command: "solver_cluster_query_command",
    timeout_s: "solver_cluster_query_timeout_s",
    spawn: "solver_cluster_query_spawn",
    wait: "solver_cluster_query_wait",
    timeout: "solver_cluster_query_timeout",
    status: "solver_cluster_query_status",
};

const SOLVER_REMOTE_ARTIFACT_TRANSFER_FIELDS: ClusterCommandFields = ClusterCommandFields {
    root: "solver_remote_artifact_transfer_root",
    command: "solver_remote_artifact_transfer_command",
    timeout_s: "solver_remote_artifact_transfer_timeout_s",
    spawn: "solver_remote_artifact_transfer_spawn",
    wait: "solver_remote_artifact_transfer_wait",
    timeout: "solver_remote_artifact_transfer_timeout",
    status: "solver_remote_artifact_transfer_status",
};

fn run_cluster_command(
    root: &Path,
    command: &[String],
    timeout_s: u64,
    fields: &ClusterCommandFields,
) -> Result<ClusterCommandOutput, AerodbError> {
    if !root.is_dir() {
        return Err(AerodbError::InvalidInput { field: fields.root });
    }
    if command.is_empty() || timeout_s == 0 {
        return Err(AerodbError::InvalidInput {
            field: if command.is_empty() {
                fields.command
            } else {
                fields.timeout_s
            },
        });
    }
    for value in command {
        validate_solver_command_text(value, fields.command)?;
    }
    let mut child = Command::new(&command[0])
        .args(&command[1..])
        .current_dir(root)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|_| AerodbError::InvalidInput {
            field: fields.spawn,
        })?;
    let deadline = Instant::now()
        .checked_add(Duration::from_secs(timeout_s))
        .ok_or(AerodbError::InvalidInput {
            field: fields.timeout_s,
        })?;
    loop {
        if child
            .try_wait()
            .map_err(|_| AerodbError::InvalidInput { field: fields.wait })?
            .is_some()
        {
            break;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return Err(AerodbError::InvalidInput {
                field: fields.timeout,
            });
        }
        thread::sleep(Duration::from_millis(10));
    }
    let output = child
        .wait_with_output()
        .map_err(|_| AerodbError::InvalidInput { field: fields.wait })?;
    if !output.status.success() {
        return Err(AerodbError::InvalidInput {
            field: fields.status,
        });
    }
    Ok(ClusterCommandOutput {
        stdout: output.stdout,
        stderr: output.stderr,
    })
}

fn parse_slurm_submit_job_id(stdout: &[u8]) -> Result<String, AerodbError> {
    let text = std::str::from_utf8(stdout).map_err(|_| AerodbError::InvalidInput {
        field: "solver_cluster_submit_job_id",
    })?;
    for token in text.split_whitespace() {
        if token.bytes().all(|byte| byte.is_ascii_digit()) {
            return Ok(token.to_owned());
        }
    }
    Err(AerodbError::InvalidInput {
        field: "solver_cluster_submit_job_id",
    })
}

fn parse_slurm_state_token(stdout: &[u8]) -> Result<String, AerodbError> {
    let text = std::str::from_utf8(stdout).map_err(|_| AerodbError::InvalidInput {
        field: "solver_cluster_query_state",
    })?;
    text.split(|ch: char| ch.is_whitespace() || ch == '|')
        .find(|token| !token.trim().is_empty())
        .map(str::trim)
        .map(str::to_owned)
        .ok_or(AerodbError::InvalidInput {
            field: "solver_cluster_query_state",
        })
}

fn classify_slurm_job_state(raw_state: &str) -> SolverClusterJobStatus {
    match raw_state {
        "PENDING" | "CONFIGURING" | "SUSPENDED" => SolverClusterJobStatus::Pending,
        "RUNNING" | "RESIZING" => SolverClusterJobStatus::Running,
        "COMPLETING" => SolverClusterJobStatus::Completing,
        "COMPLETED" => SolverClusterJobStatus::Completed,
        "CANCELLED" | "REVOKED" | "STOPPED" => SolverClusterJobStatus::Cancelled,
        "TIMEOUT" => SolverClusterJobStatus::Timeout,
        "BOOT_FAIL" | "DEADLINE" | "FAILED" | "NODE_FAIL" | "OUT_OF_MEMORY" | "PREEMPTED" => {
            SolverClusterJobStatus::Failed
        }
        _ => SolverClusterJobStatus::Unknown,
    }
}

fn format_slurm_duration(seconds: u64) -> String {
    let hours = seconds / 3600;
    let minutes = (seconds % 3600) / 60;
    let seconds = seconds % 60;
    format!("{hours:02}:{minutes:02}:{seconds:02}")
}

fn shell_single_quote(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\\''"))
}

fn validate_su2_config_key(value: &str) -> Result<(), AerodbError> {
    if value.trim().is_empty()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_')
    {
        return Err(AerodbError::InvalidInput {
            field: "su2_euler_extra_config_key",
        });
    }
    Ok(())
}

fn join_relative_path(parent: &str, child: &str) -> String {
    let mut path = PathBuf::from(parent);
    path.push(child);
    path.to_string_lossy().replace('\\', "/")
}

fn format_su2_float(value: f64) -> String {
    format!("{value:.17e}")
}

fn render_su2_euler_config(
    template: &Su2EulerCaseTemplate,
    node_index: usize,
    node: RunNode,
) -> String {
    let mut text = String::new();
    text.push_str("% Generated by OpenBMP openbmp-aerodb SU2-Euler case generator\n");
    text.push_str(&format!("% node_index= {node_index}\n"));
    text.push_str(&format!("% mach= {}\n", format_su2_float(node.mach)));
    text.push_str(&format!(
        "% alpha_deg= {}\n",
        format_su2_float(node.alpha_deg)
    ));
    text.push_str(&format!(
        "% beta_deg= {}\n",
        format_su2_float(node.beta_deg)
    ));
    text.push_str(&format!(
        "% reynolds_length= {}\n",
        format_su2_float(node.reynolds_length)
    ));
    text.push_str(&format!("% knudsen= {}\n", format_su2_float(node.knudsen)));
    text.push_str(&format!(
        "% dynamic_pressure_pa= {}\n",
        format_su2_float(node.dynamic_pressure_pa)
    ));
    text.push_str("SOLVER= EULER\n");
    text.push_str("MATH_PROBLEM= DIRECT\n");
    text.push_str(&format!("MESH_FILENAME= {}\n", template.mesh_filename));
    text.push_str(&format!("MACH_NUMBER= {}\n", format_su2_float(node.mach)));
    text.push_str(&format!("AOA= {}\n", format_su2_float(node.alpha_deg)));
    text.push_str(&format!(
        "SIDESLIP_ANGLE= {}\n",
        format_su2_float(node.beta_deg)
    ));
    text.push_str(&format!(
        "REF_AREA= {}\n",
        format_su2_float(template.reference_area_m2)
    ));
    text.push_str(&format!(
        "REF_LENGTH= {}\n",
        format_su2_float(template.reference_length_m)
    ));
    text.push_str(&format!(
        "REF_ORIGIN_MOMENT_X= {}\n",
        format_su2_float(template.moment_reference_m.x)
    ));
    text.push_str(&format!(
        "REF_ORIGIN_MOMENT_Y= {}\n",
        format_su2_float(template.moment_reference_m.y)
    ));
    text.push_str(&format!(
        "REF_ORIGIN_MOMENT_Z= {}\n",
        format_su2_float(template.moment_reference_m.z)
    ));
    text.push_str(&format!("MARKER_EULER= ( {} )\n", template.surface_marker));
    text.push_str(&format!(
        "MARKER_PLOTTING= ( {} )\n",
        template.surface_marker
    ));
    text.push_str(&format!(
        "MARKER_MONITORING= ( {} )\n",
        template.surface_marker
    ));
    text.push_str("TABULAR_FORMAT= CSV\n");
    text.push_str(&format!("CONV_FILENAME= {}\n", template.history_filename));
    text.push_str(&format!(
        "SURFACE_FILENAME= {}\n",
        template.surface_filename
    ));
    text.push_str("OUTPUT_FILES= (RESTART, PARAVIEW, SURFACE_CSV)\n");
    for (key, value) in &template.extra_config {
        text.push_str(&format!("{key}= {value}\n"));
    }
    text
}

fn execute_solver_command_case(
    root: &Path,
    case: &SolverCommandCase,
) -> Result<SolverCommandCaseExecution, AerodbError> {
    let working_dir = resolve_solver_plan_path(
        root,
        &case.working_dir,
        "solver_command_execute_working_dir",
    )?;
    if !working_dir.is_dir() {
        return Err(AerodbError::InvalidInput {
            field: "solver_command_execute_working_dir",
        });
    }
    let output_path = resolve_solver_plan_path(
        root,
        &case.output_file,
        "solver_command_execute_output_file",
    )?;
    let mut command = Command::new(&case.program);
    command
        .args(&case.args)
        .envs(case.env.iter())
        .current_dir(&working_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command.spawn().map_err(|_| AerodbError::InvalidInput {
        field: "solver_command_execute_spawn",
    })?;
    let deadline = Instant::now()
        .checked_add(Duration::from_secs(case.timeout_s))
        .ok_or(AerodbError::InvalidInput {
            field: "solver_command_execute_timeout_s",
        })?;
    loop {
        if child
            .try_wait()
            .map_err(|_| AerodbError::InvalidInput {
                field: "solver_command_execute_wait",
            })?
            .is_some()
        {
            break;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return Err(AerodbError::InvalidInput {
                field: "solver_command_execute_timeout",
            });
        }
        thread::sleep(Duration::from_millis(10));
    }
    let output = child
        .wait_with_output()
        .map_err(|_| AerodbError::InvalidInput {
            field: "solver_command_execute_wait",
        })?;
    if !output.status.success() {
        return Err(AerodbError::InvalidInput {
            field: "solver_command_execute_status",
        });
    }
    let output_bytes = fs::read(&output_path).map_err(|_| AerodbError::InvalidInput {
        field: "solver_command_execute_output_file",
    })?;
    let output_sha256 = sha256_hex(&output_bytes);
    if !output_sha256.eq_ignore_ascii_case(&case.output_sha256) {
        return Err(AerodbError::InvalidInput {
            field: "solver_command_execute_output_sha256",
        });
    }
    Ok(SolverCommandCaseExecution {
        node_index: case.node_index,
        case_id: case.case_id.clone(),
        exit_code: output.status.code(),
        stdout_len: output.stdout.len(),
        stdout_sha256: sha256_hex(&output.stdout),
        stderr_len: output.stderr.len(),
        stderr_sha256: sha256_hex(&output.stderr),
        output_file: case.output_file.clone(),
        output_sha256,
    })
}

fn execute_solver_command_case_retrying(
    root: &Path,
    case: &SolverCommandCase,
    policy: SolverCommandRetryPolicy,
) -> Result<SolverCommandCaseRetryExecution, AerodbError> {
    let mut attempts = Vec::new();
    for attempt_index in 1..=policy.max_attempts {
        match execute_solver_command_case(root, case) {
            Ok(execution) => {
                attempts.push(SolverCommandCaseAttempt {
                    attempt_index,
                    success: true,
                    failure_field: None,
                });
                return Ok(SolverCommandCaseRetryExecution {
                    node_index: case.node_index,
                    case_id: case.case_id.clone(),
                    attempts,
                    execution,
                });
            }
            Err(error) => {
                let retry = attempt_index < policy.max_attempts && policy.should_retry(&error);
                attempts.push(SolverCommandCaseAttempt {
                    attempt_index,
                    success: false,
                    failure_field: aerodb_error_field(&error),
                });
                if !retry {
                    return Err(error);
                }
            }
        }
    }
    Err(AerodbError::InvalidInput {
        field: "solver_command_retry_max_attempts",
    })
}

fn aerodb_error_field(error: &AerodbError) -> Option<&'static str> {
    match error {
        AerodbError::InvalidInput { field } => Some(*field),
        AerodbError::MalformedToml { .. }
        | AerodbError::MalformedCsv { .. }
        | AerodbError::MalformedOpenFoam { .. }
        | AerodbError::ForbiddenVocabulary { .. }
        | AerodbError::UnsupportedDeckBake { .. }
        | AerodbError::Aero(_)
        | AerodbError::Uq(_) => None,
    }
}

fn resolve_solver_plan_path(
    root: &Path,
    value: &str,
    field: &'static str,
) -> Result<PathBuf, AerodbError> {
    let path = Path::new(value);
    if path.is_absolute() {
        return Err(AerodbError::InvalidInput { field });
    }
    let mut normalized = PathBuf::new();
    for component in path.components() {
        match component {
            Component::Normal(part) => normalized.push(part),
            Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(AerodbError::InvalidInput { field });
            }
        }
    }
    if normalized.as_os_str().is_empty() {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(root.join(normalized))
}

fn validate_sha256_hex(value: &str, field: &'static str) -> Result<(), AerodbError> {
    if value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        Ok(())
    } else {
        Err(AerodbError::InvalidInput { field })
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>()
}

fn coefficients_from_force_moment(
    builder: &DeckBuilder,
    node: RunNode,
    force_moment: AeroForceMomentBody,
) -> Result<AeroCoefficients, AerodbError> {
    let force_ref = node.dynamic_pressure_pa * builder.reference_area_m2;
    let moment_ref = force_ref * builder.reference_length_m;
    if !(force_ref.is_finite() && force_ref > 0.0 && moment_ref.is_finite() && moment_ref > 0.0) {
        return Err(AerodbError::InvalidInput {
            field: "coefficient_reference",
        });
    }
    let coeffs = AeroCoefficients {
        cn: -force_moment.force_n_body.z / force_ref,
        cd: -force_moment.force_n_body.x / force_ref,
        cm: force_moment.moment_n_m_body.y / moment_ref,
    };
    if !(coeffs.cn.is_finite() && coeffs.cd.is_finite() && coeffs.cm.is_finite()) {
        return Err(AerodbError::InvalidInput {
            field: "baked_coefficients",
        });
    }
    Ok(coeffs)
}

fn six_coefficients_from_force_moment(
    builder: &DeckBuilder,
    node: RunNode,
    force_moment: AeroForceMomentBody,
) -> Result<AeroSixCoefficients, AerodbError> {
    let force_ref = node.dynamic_pressure_pa * builder.reference_area_m2;
    let moment_ref = force_ref * builder.reference_length_m;
    if !(force_ref.is_finite() && force_ref > 0.0 && moment_ref.is_finite() && moment_ref > 0.0) {
        return Err(AerodbError::InvalidInput {
            field: "six_coefficient_reference",
        });
    }
    let coeffs = AeroSixCoefficients {
        cx: force_moment.force_n_body.x / force_ref,
        cy: force_moment.force_n_body.y / force_ref,
        cz: force_moment.force_n_body.z / force_ref,
        cl: force_moment.moment_n_m_body.x / moment_ref,
        cm: force_moment.moment_n_m_body.y / moment_ref,
        cn: force_moment.moment_n_m_body.z / moment_ref,
    };
    if !(coeffs.cx.is_finite()
        && coeffs.cy.is_finite()
        && coeffs.cz.is_finite()
        && coeffs.cl.is_finite()
        && coeffs.cm.is_finite()
        && coeffs.cn.is_finite())
    {
        return Err(AerodbError::InvalidInput {
            field: "baked_six_coefficients",
        });
    }
    Ok(coeffs)
}

fn validate_solver_coefficient_csv_header(headers: &csv::StringRecord) -> Result<(), AerodbError> {
    if headers.len() != SOLVER_COEFFICIENT_CSV_HEADER.len()
        || headers
            .iter()
            .zip(SOLVER_COEFFICIENT_CSV_HEADER)
            .any(|(actual, expected)| actual != expected)
    {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV header does not match schema",
        });
    }
    Ok(())
}

fn validate_solver_six_coefficient_csv_header(
    headers: &csv::StringRecord,
) -> Result<(), AerodbError> {
    if headers.len() != SOLVER_SIX_COEFFICIENT_CSV_HEADER.len()
        || headers
            .iter()
            .zip(SOLVER_SIX_COEFFICIENT_CSV_HEADER)
            .any(|(actual, expected)| actual != expected)
    {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver six coefficient CSV header does not match schema",
        });
    }
    Ok(())
}

fn solver_coefficients_from_csv_record(
    record: &csv::StringRecord,
    expected_index: usize,
    node: RunNode,
) -> Result<AeroCoefficients, AerodbError> {
    if record.len() != SOLVER_COEFFICIENT_CSV_HEADER.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV row has the wrong field count",
        });
    }
    if record.get(0) != Some(SOLVER_COEFFICIENT_CSV_SCHEMA_V1) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV row has an unsupported schema",
        });
    }
    let node_index = parse_solver_coefficient_csv_usize(record, 1)?;
    if node_index != expected_index {
        return Err(AerodbError::InvalidInput {
            field: "solver_coefficient_csv_node_index",
        });
    }
    validate_solver_coefficient_csv_condition(record, 2, node.mach, "solver_coefficient_csv_mach")?;
    validate_solver_coefficient_csv_condition(
        record,
        3,
        node.alpha_deg,
        "solver_coefficient_csv_alpha_deg",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        4,
        node.beta_deg,
        "solver_coefficient_csv_beta_deg",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        5,
        node.reynolds_length,
        "solver_coefficient_csv_reynolds_length",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        6,
        node.knudsen,
        "solver_coefficient_csv_knudsen",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        7,
        node.dynamic_pressure_pa,
        "solver_coefficient_csv_dynamic_pressure_pa",
    )?;
    for (index, field) in [
        (11, "solver_coefficient_csv_solver"),
        (12, "solver_coefficient_csv_case_id"),
        (13, "solver_coefficient_csv_provenance"),
    ] {
        if record
            .get(index)
            .is_none_or(|value| value.trim().is_empty())
        {
            return Err(AerodbError::MalformedCsv { reason: field });
        }
    }
    Ok(AeroCoefficients {
        cn: parse_solver_coefficient_csv_f64(record, 8, "solver_coefficient_csv_cn")?,
        cd: parse_solver_coefficient_csv_f64(record, 9, "solver_coefficient_csv_cd")?,
        cm: parse_solver_coefficient_csv_f64(record, 10, "solver_coefficient_csv_cm")?,
    })
}

fn solver_six_coefficients_from_csv_record(
    record: &csv::StringRecord,
    expected_index: usize,
    node: RunNode,
) -> Result<AeroSixCoefficients, AerodbError> {
    if record.len() != SOLVER_SIX_COEFFICIENT_CSV_HEADER.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver six coefficient CSV row has the wrong field count",
        });
    }
    if record.get(0) != Some(SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero solver six coefficient CSV row has an unsupported schema",
        });
    }
    let node_index = parse_solver_coefficient_csv_usize(record, 1)?;
    if node_index != expected_index {
        return Err(AerodbError::InvalidInput {
            field: "solver_six_coefficient_csv_node_index",
        });
    }
    validate_solver_coefficient_csv_condition(
        record,
        2,
        node.mach,
        "solver_six_coefficient_csv_mach",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        3,
        node.alpha_deg,
        "solver_six_coefficient_csv_alpha_deg",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        4,
        node.beta_deg,
        "solver_six_coefficient_csv_beta_deg",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        5,
        node.reynolds_length,
        "solver_six_coefficient_csv_reynolds_length",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        6,
        node.knudsen,
        "solver_six_coefficient_csv_knudsen",
    )?;
    validate_solver_coefficient_csv_condition(
        record,
        7,
        node.dynamic_pressure_pa,
        "solver_six_coefficient_csv_dynamic_pressure_pa",
    )?;
    for (index, field) in [
        (14, "solver_six_coefficient_csv_solver"),
        (15, "solver_six_coefficient_csv_case_id"),
        (16, "solver_six_coefficient_csv_provenance"),
    ] {
        if record
            .get(index)
            .is_none_or(|value| value.trim().is_empty())
        {
            return Err(AerodbError::MalformedCsv { reason: field });
        }
    }
    Ok(AeroSixCoefficients {
        cx: parse_solver_coefficient_csv_f64(record, 8, "solver_six_coefficient_csv_cx")?,
        cy: parse_solver_coefficient_csv_f64(record, 9, "solver_six_coefficient_csv_cy")?,
        cz: parse_solver_coefficient_csv_f64(record, 10, "solver_six_coefficient_csv_cz")?,
        cl: parse_solver_coefficient_csv_f64(record, 11, "solver_six_coefficient_csv_cl")?,
        cm: parse_solver_coefficient_csv_f64(record, 12, "solver_six_coefficient_csv_cm")?,
        cn: parse_solver_coefficient_csv_f64(record, 13, "solver_six_coefficient_csv_cn")?,
    })
}

fn validate_solver_coefficient_csv_condition(
    record: &csv::StringRecord,
    index: usize,
    expected: f64,
    field: &'static str,
) -> Result<(), AerodbError> {
    let value = parse_solver_coefficient_csv_f64(record, index, field)?;
    if value.to_bits() != expected.to_bits() {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn parse_solver_coefficient_csv_usize(
    record: &csv::StringRecord,
    index: usize,
) -> Result<usize, AerodbError> {
    record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV row is missing a required field",
        })?
        .parse::<usize>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV integer field did not parse",
        })
}

fn parse_solver_coefficient_csv_f64(
    record: &csv::StringRecord,
    index: usize,
    field: &'static str,
) -> Result<f64, AerodbError> {
    let value = record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV row is missing a required field",
        })?
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "aero solver coefficient CSV float field did not parse",
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput { field })
    }
}

fn validate_surface_cp_csv_header(headers: &csv::StringRecord) -> Result<(), AerodbError> {
    if headers.len() != SURFACE_CP_CSV_HEADER.len()
        || headers
            .iter()
            .zip(SURFACE_CP_CSV_HEADER)
            .any(|(actual, expected)| actual != expected)
    {
        return Err(AerodbError::MalformedCsv {
            reason: "surface Cp CSV header does not match schema",
        });
    }
    Ok(())
}

fn surface_cp_from_csv_record(
    record: &csv::StringRecord,
    expected_node_index: usize,
    expected_panel_index: usize,
    node: RunNode,
) -> Result<f64, AerodbError> {
    if record.len() != SURFACE_CP_CSV_HEADER.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "surface Cp CSV row has the wrong field count",
        });
    }
    if record.get(0) != Some(SURFACE_CP_CSV_SCHEMA_V1) {
        return Err(AerodbError::MalformedCsv {
            reason: "surface Cp CSV row has an unsupported schema",
        });
    }
    let node_index = parse_surface_cp_csv_usize(record, 1)?;
    if node_index != expected_node_index {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_csv_node_index",
        });
    }
    let panel_index = parse_surface_cp_csv_usize(record, 2)?;
    if panel_index != expected_panel_index {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_csv_panel_index",
        });
    }
    validate_surface_cp_csv_condition(record, 3, node.mach, "surface_cp_csv_mach")?;
    validate_surface_cp_csv_condition(record, 4, node.alpha_deg, "surface_cp_csv_alpha_deg")?;
    validate_surface_cp_csv_condition(record, 5, node.beta_deg, "surface_cp_csv_beta_deg")?;
    validate_surface_cp_csv_condition(
        record,
        6,
        node.reynolds_length,
        "surface_cp_csv_reynolds_length",
    )?;
    validate_surface_cp_csv_condition(record, 7, node.knudsen, "surface_cp_csv_knudsen")?;
    validate_surface_cp_csv_condition(
        record,
        8,
        node.dynamic_pressure_pa,
        "surface_cp_csv_dynamic_pressure_pa",
    )?;
    for (index, field) in [
        (10, "surface_cp_csv_solver"),
        (11, "surface_cp_csv_case_id"),
        (12, "surface_cp_csv_provenance"),
    ] {
        if record
            .get(index)
            .is_none_or(|value| value.trim().is_empty())
        {
            return Err(AerodbError::MalformedCsv { reason: field });
        }
    }
    parse_csv_f64_at(
        record,
        9,
        "surface Cp CSV cp field did not parse",
        "surface_cp_csv_cp",
    )
}

fn validate_surface_cp_csv_condition(
    record: &csv::StringRecord,
    index: usize,
    expected: f64,
    field: &'static str,
) -> Result<(), AerodbError> {
    let value = parse_csv_f64_at(
        record,
        index,
        "surface Cp CSV float field did not parse",
        field,
    )?;
    if value.to_bits() != expected.to_bits() {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

fn parse_surface_cp_csv_usize(
    record: &csv::StringRecord,
    index: usize,
) -> Result<usize, AerodbError> {
    record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "surface Cp CSV row is missing a required field",
        })?
        .parse::<usize>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "surface Cp CSV integer field did not parse",
        })
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct SurfaceCpPanelLoad {
    force_n_body: Vector3<f64>,
    moment_n_m_body: Vector3<f64>,
    centroid: Vector3<f64>,
}

fn surface_cp_panel_load(
    mesh: &PanelMesh,
    panel_index: usize,
    cp: f64,
    node: RunNode,
    cg_body_m: Vector3<f64>,
) -> Result<SurfaceCpPanelLoad, AerodbError> {
    let [ia, ib, ic] = mesh.triangles()[panel_index];
    let a = mesh.vertices()[ia as usize];
    let b = mesh.vertices()[ib as usize];
    let c = mesh.vertices()[ic as usize];
    let area_vec = (b - a).cross(&(c - a));
    let area = 0.5 * area_vec.norm();
    if !(area.is_finite() && area > 0.0) {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_csv_panel_area",
        });
    }
    let normal = area_vec.normalize();
    let panel_force = -normal * (cp * node.dynamic_pressure_pa * area);
    let centroid = (a + b + c) / 3.0;
    let panel_moment = (centroid - cg_body_m).cross(&panel_force);
    if !(panel_force.x.is_finite()
        && panel_force.y.is_finite()
        && panel_force.z.is_finite()
        && panel_moment.x.is_finite()
        && panel_moment.y.is_finite()
        && panel_moment.z.is_finite())
    {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_csv_loads",
        });
    }
    Ok(SurfaceCpPanelLoad {
        force_n_body: panel_force,
        moment_n_m_body: panel_moment,
        centroid,
    })
}

fn surface_cp_load_report_from_force_moment(
    builder: &DeckBuilder,
    node_index: usize,
    node: RunNode,
    force_n_body: Vector3<f64>,
    moment_n_m_body: Vector3<f64>,
    cg_body_m: Vector3<f64>,
    axial_line_loads: Vec<AxialLineLoad>,
) -> Result<SurfaceCpLoadReport, AerodbError> {
    let force_ref = node.dynamic_pressure_pa * builder.reference_area_m2;
    let moment_ref = force_ref * builder.reference_length_m;
    if !(force_ref.is_finite() && force_ref > 0.0 && moment_ref.is_finite() && moment_ref > 0.0) {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_coefficient_reference",
        });
    }
    let force_coefficient_body = force_n_body / force_ref;
    let moment_coefficient_body = moment_n_m_body / moment_ref;
    if !(force_coefficient_body.x.is_finite()
        && force_coefficient_body.y.is_finite()
        && force_coefficient_body.z.is_finite()
        && moment_coefficient_body.x.is_finite()
        && moment_coefficient_body.y.is_finite()
        && moment_coefficient_body.z.is_finite())
    {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_coefficients",
        });
    }
    let reduced_coefficients = coefficients_from_force_moment(
        builder,
        node,
        AeroForceMomentBody {
            force_n_body,
            moment_n_m_body,
        },
    )?;
    Ok(SurfaceCpLoadReport {
        node_index,
        force_n_body,
        moment_n_m_body,
        cg_body_m,
        force_coefficient_body,
        moment_coefficient_body,
        reduced_coefficients,
        axial_line_loads,
    })
}

fn surface_cp_load_report_from_vertex_cp(
    builder: &DeckBuilder,
    node_index: usize,
    node: RunNode,
    mesh: &PanelMesh,
    vertex_cp: &[f64],
    cg_body_m: Vector3<f64>,
    axial_stations_m: &[f64],
) -> Result<SurfaceCpLoadReport, AerodbError> {
    if vertex_cp.len() != mesh.vertices().len() {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_vertex_count",
        });
    }
    let mut force = Vector3::zeros();
    let mut moment = Vector3::zeros();
    let mut axial_line_loads = surface_cp_empty_axial_line_loads(axial_stations_m);
    for (panel_index, &[ia, ib, ic]) in mesh.triangles().iter().enumerate() {
        let cp = (vertex_cp[ia as usize] + vertex_cp[ib as usize] + vertex_cp[ic as usize]) / 3.0;
        if !cp.is_finite() {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_vertex_cp",
            });
        }
        let panel_load = surface_cp_panel_load(mesh, panel_index, cp, node, cg_body_m)?;
        force += panel_load.force_n_body;
        moment += panel_load.moment_n_m_body;
        if let Some(bin) = surface_cp_axial_station_bin(axial_stations_m, panel_load.centroid.x) {
            axial_line_loads[bin].force_n_body += panel_load.force_n_body;
            axial_line_loads[bin].moment_n_m_body += panel_load.moment_n_m_body;
        }
    }
    surface_cp_load_report_from_force_moment(
        builder,
        node_index,
        node,
        force,
        moment,
        cg_body_m,
        axial_line_loads,
    )
}

fn surface_cp_load_report_from_panel_forces(
    builder: &DeckBuilder,
    node_index: usize,
    node: RunNode,
    mesh: &PanelMesh,
    panel_forces_n_body: &[Vector3<f64>],
    cg_body_m: Vector3<f64>,
    axial_stations_m: &[f64],
) -> Result<SurfaceCpLoadReport, AerodbError> {
    if panel_forces_n_body.len() != mesh.triangles().len() {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_panel_force_count",
        });
    }
    let mut force = Vector3::zeros();
    let mut moment = Vector3::zeros();
    let mut axial_line_loads = surface_cp_empty_axial_line_loads(axial_stations_m);
    for (panel_index, panel_force) in panel_forces_n_body.iter().copied().enumerate() {
        if !(panel_force.x.is_finite() && panel_force.y.is_finite() && panel_force.z.is_finite()) {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_panel_force",
            });
        }
        let [ia, ib, ic] = mesh.triangles()[panel_index];
        let centroid = (mesh.vertices()[ia as usize]
            + mesh.vertices()[ib as usize]
            + mesh.vertices()[ic as usize])
            / 3.0;
        let panel_moment = (centroid - cg_body_m).cross(&panel_force);
        if !(panel_moment.x.is_finite() && panel_moment.y.is_finite() && panel_moment.z.is_finite())
        {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_panel_force_moment",
            });
        }
        force += panel_force;
        moment += panel_moment;
        if let Some(bin) = surface_cp_axial_station_bin(axial_stations_m, centroid.x) {
            axial_line_loads[bin].force_n_body += panel_force;
            axial_line_loads[bin].moment_n_m_body += panel_moment;
        }
    }
    surface_cp_load_report_from_force_moment(
        builder,
        node_index,
        node,
        force,
        moment,
        cg_body_m,
        axial_line_loads,
    )
}

fn validate_surface_cp_axial_stations(axial_stations_m: &[f64]) -> Result<(), AerodbError> {
    if axial_stations_m.is_empty() {
        return Ok(());
    }
    if axial_stations_m.len() < 2 {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_axial_stations_m",
        });
    }
    let mut previous = axial_stations_m[0];
    if !previous.is_finite() {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_axial_stations_m",
        });
    }
    for &station in &axial_stations_m[1..] {
        if !(station.is_finite() && station > previous) {
            return Err(AerodbError::InvalidInput {
                field: "surface_cp_axial_stations_m",
            });
        }
        previous = station;
    }
    Ok(())
}

fn surface_cp_empty_axial_line_loads(axial_stations_m: &[f64]) -> Vec<AxialLineLoad> {
    axial_stations_m
        .windows(2)
        .map(|window| AxialLineLoad {
            x_min_m: window[0],
            x_max_m: window[1],
            force_n_body: Vector3::zeros(),
            moment_n_m_body: Vector3::zeros(),
        })
        .collect()
}

fn surface_cp_axial_station_bin(axial_stations_m: &[f64], x_m: f64) -> Option<usize> {
    if axial_stations_m.len() < 2 {
        return None;
    }
    let last = axial_stations_m.len() - 1;
    (0..last).find(|&index| {
        x_m >= axial_stations_m[index]
            && (x_m < axial_stations_m[index + 1]
                || (index + 1 == last && x_m <= axial_stations_m[index + 1]))
    })
}

fn su2_surface_vertex_cp_from_csv(
    csv_text: &str,
    mesh: &PanelMesh,
) -> Result<Vec<f64>, AerodbError> {
    let mut reader = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(csv_text.as_bytes());
    let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
        reason: "SU2 surface CSV header did not parse",
    })?;
    let x_index = su2_surface_column(headers, &["x", "coord_x", "coordinate_x", "points_0"])
        .ok_or(AerodbError::MalformedCsv {
            reason: "SU2 surface CSV is missing x coordinate",
        })?;
    let y_index = su2_surface_column(headers, &["y", "coord_y", "coordinate_y", "points_1"])
        .ok_or(AerodbError::MalformedCsv {
            reason: "SU2 surface CSV is missing y coordinate",
        })?;
    let z_index = su2_surface_column(headers, &["z", "coord_z", "coordinate_z", "points_2"])
        .ok_or(AerodbError::MalformedCsv {
            reason: "SU2 surface CSV is missing z coordinate",
        })?;
    let cp_index = su2_surface_column(
        headers,
        &[
            "pressure_coefficient",
            "pressurecoefficient",
            "cp",
            "coeff_pressure",
        ],
    )
    .ok_or(AerodbError::MalformedCsv {
        reason: "SU2 surface CSV is missing Pressure_Coefficient",
    })?;

    let mut mesh_vertices = BTreeMap::new();
    for (index, vertex) in mesh.vertices().iter().copied().enumerate() {
        let previous = mesh_vertices.insert(surface_coordinate_key(vertex)?, index);
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_mesh_vertices",
            });
        }
    }
    let mut vertex_cp = vec![None; mesh.vertices().len()];
    let mut row_count = 0usize;
    for record in reader.records() {
        let record = record.map_err(|_| AerodbError::MalformedCsv {
            reason: "SU2 surface CSV row did not parse",
        })?;
        let x = parse_csv_f64_at(
            &record,
            x_index,
            "SU2 surface CSV x coordinate did not parse",
            "su2_surface_cp_x",
        )?;
        let y = parse_csv_f64_at(
            &record,
            y_index,
            "SU2 surface CSV y coordinate did not parse",
            "su2_surface_cp_y",
        )?;
        let z = parse_csv_f64_at(
            &record,
            z_index,
            "SU2 surface CSV z coordinate did not parse",
            "su2_surface_cp_z",
        )?;
        let cp = parse_csv_f64_at(
            &record,
            cp_index,
            "SU2 surface CSV Pressure_Coefficient did not parse",
            "su2_surface_cp_cp",
        )?;
        let key = surface_coordinate_key(Vector3::new(x, y, z))?;
        let vertex_index = *mesh_vertices.get(&key).ok_or(AerodbError::InvalidInput {
            field: "su2_surface_cp_vertex_coordinate",
        })?;
        if vertex_cp[vertex_index].replace(cp).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_duplicate_vertex",
            });
        }
        row_count += 1;
    }
    if row_count != mesh.vertices().len() || vertex_cp.iter().any(Option::is_none) {
        return Err(AerodbError::InvalidInput {
            field: "su2_surface_cp_vertices",
        });
    }
    vertex_cp
        .into_iter()
        .map(|value| {
            value.ok_or(AerodbError::InvalidInput {
                field: "su2_surface_cp_vertices",
            })
        })
        .collect()
}

fn openfoam_surface_vertex_cp_from_text(
    text: &str,
    mesh: &PanelMesh,
) -> Result<Vec<f64>, AerodbError> {
    let mut mesh_vertices = BTreeMap::new();
    for (index, vertex) in mesh.vertices().iter().copied().enumerate() {
        let previous = mesh_vertices.insert(surface_coordinate_key(vertex)?, index);
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_mesh_vertices",
            });
        }
    }
    let mut vertex_cp = vec![None; mesh.vertices().len()];
    let mut row_count = 0usize;
    for raw_line in text.lines() {
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() || matches!(line, "(" | ")" | ");" | "{" | "}") {
            continue;
        }
        let tokens = line.split_whitespace().collect::<Vec<_>>();
        if tokens.len() != 4 {
            return Err(AerodbError::MalformedCsv {
                reason: "OpenFOAM sampled-surface Cp row must have x y z Cp fields",
            });
        }
        let x = parse_openfoam_surface_cp_f64(
            tokens[0],
            "OpenFOAM sampled-surface Cp x coordinate did not parse",
            "openfoam_surface_cp_x",
        )?;
        let y = parse_openfoam_surface_cp_f64(
            tokens[1],
            "OpenFOAM sampled-surface Cp y coordinate did not parse",
            "openfoam_surface_cp_y",
        )?;
        let z = parse_openfoam_surface_cp_f64(
            tokens[2],
            "OpenFOAM sampled-surface Cp z coordinate did not parse",
            "openfoam_surface_cp_z",
        )?;
        let cp = parse_openfoam_surface_cp_f64(
            tokens[3],
            "OpenFOAM sampled-surface Cp value did not parse",
            "openfoam_surface_cp_cp",
        )?;
        let key = surface_coordinate_key(Vector3::new(x, y, z))?;
        let vertex_index = *mesh_vertices.get(&key).ok_or(AerodbError::InvalidInput {
            field: "openfoam_surface_cp_vertex_coordinate",
        })?;
        if vertex_cp[vertex_index].replace(cp).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_duplicate_vertex",
            });
        }
        row_count += 1;
    }
    if row_count != mesh.vertices().len() || vertex_cp.iter().any(Option::is_none) {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_surface_cp_vertices",
        });
    }
    vertex_cp
        .into_iter()
        .map(|value| {
            value.ok_or(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_vertices",
            })
        })
        .collect()
}

fn openfoam_boundary_patch_cp_from_field(
    field_text: &str,
    patch_name: &str,
    panel_count: usize,
) -> Result<Vec<f64>, AerodbError> {
    let field = OpenFoamScalarField::parse_str(field_text)?;
    let patch = field.patch(patch_name).ok_or(AerodbError::InvalidInput {
        field: "openfoam_boundary_cp_patch_name",
    })?;
    let value = patch.value.as_ref().ok_or(AerodbError::InvalidInput {
        field: "openfoam_boundary_cp_patch_value",
    })?;
    openfoam_scalar_values_for_count(value, panel_count, "openfoam_boundary_cp_patch_values")
}

fn openfoam_scalar_values_for_count(
    value: &OpenFoamScalarFieldValue,
    count: usize,
    field: &'static str,
) -> Result<Vec<f64>, AerodbError> {
    match value {
        OpenFoamScalarFieldValue::Uniform(value) => Ok(vec![*value; count]),
        OpenFoamScalarFieldValue::Nonuniform(values) => {
            if values.len() != count {
                return Err(AerodbError::InvalidInput { field });
            }
            Ok(values.clone())
        }
    }
}

fn read_openfoam_case_field_file(
    case_root: &Path,
    file: &OpenFoamCaseFieldFile<'_>,
) -> Result<String, AerodbError> {
    let path = resolve_solver_plan_path(case_root, file.relative_path, "openfoam_case_field_file")?;
    fs::read_to_string(path).map_err(|_| AerodbError::InvalidInput {
        field: "openfoam_case_field_file_read",
    })
}

fn insert_openfoam_recorded_scalar_field(
    scalar_fields: &mut BTreeMap<String, OpenFoamScalarField>,
    field_name: &str,
    text: &str,
) -> Result<(), AerodbError> {
    validate_case_id_prefix(field_name, "openfoam_recorded_case_field_name")?;
    let field = OpenFoamScalarField::parse_str(text)?;
    if field.object_name != field_name {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_recorded_case_field_object",
        });
    }
    insert_openfoam_parsed_scalar_field(scalar_fields, field)
}

fn insert_openfoam_parsed_scalar_field(
    scalar_fields: &mut BTreeMap<String, OpenFoamScalarField>,
    field: OpenFoamScalarField,
) -> Result<(), AerodbError> {
    validate_case_id_prefix(&field.object_name, "openfoam_recorded_case_field_name")?;
    if scalar_fields
        .insert(field.object_name.clone(), field)
        .is_some()
    {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_recorded_case_scalar_duplicate",
        });
    }
    Ok(())
}

fn insert_openfoam_recorded_vector_field(
    vector_fields: &mut BTreeMap<String, OpenFoamVectorField>,
    field_name: &str,
    text: &str,
) -> Result<(), AerodbError> {
    validate_case_id_prefix(field_name, "openfoam_recorded_case_field_name")?;
    let field = OpenFoamVectorField::parse_str(text)?;
    if field.object_name != field_name {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_recorded_case_field_object",
        });
    }
    insert_openfoam_parsed_vector_field(vector_fields, field)
}

fn insert_openfoam_parsed_vector_field(
    vector_fields: &mut BTreeMap<String, OpenFoamVectorField>,
    field: OpenFoamVectorField,
) -> Result<(), AerodbError> {
    validate_case_id_prefix(&field.object_name, "openfoam_recorded_case_field_name")?;
    if vector_fields
        .insert(field.object_name.clone(), field)
        .is_some()
    {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_recorded_case_vector_duplicate",
        });
    }
    Ok(())
}

fn parse_openfoam_poly_mesh_boundary(text: &str) -> Result<OpenFoamPolyMeshBoundary, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut patches = None;
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
            class_name = class;
            object_name = object;
            continue;
        }
        if let Ok(count) = token.parse::<usize>() {
            cursor.next();
            patches = Some(parse_openfoam_boundary_topology_entries(
                &mut cursor,
                count,
            )?);
            continue;
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }

    let class_name = class_name.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_boundary_class",
    })?;
    if class_name != "polyBoundaryMesh" {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_boundary_class",
        });
    }
    let object_name = object_name.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_boundary_object",
    })?;
    if object_name != "boundary" {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_boundary_object",
        });
    }
    let patches = patches.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_boundary_patches",
    })?;
    if patches.is_empty() {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_boundary_patches",
        });
    }

    Ok(OpenFoamPolyMeshBoundary {
        class_name,
        object_name,
        patches,
    })
}

fn parse_openfoam_boundary_topology_entries(
    cursor: &mut OpenFoamTokenCursor,
    count: usize,
) -> Result<BTreeMap<String, OpenFoamBoundaryPatchTopology>, AerodbError> {
    if count == 0 {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_boundary_patches",
        });
    }
    cursor.expect("(")?;
    let mut patches = BTreeMap::new();
    let mut ranges = Vec::with_capacity(count);
    for _ in 0..count {
        let patch_name = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM polyMesh boundary patch name is missing",
        })?;
        validate_case_id_prefix(&patch_name, "openfoam_poly_mesh_boundary_patch_name")?;
        cursor.expect("{")?;
        let mut patch_type = None;
        let mut n_faces = None;
        let mut start_face = None;
        while let Some(token) = cursor.peek() {
            if token == "}" {
                cursor.next();
                break;
            }
            let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM polyMesh boundary patch entry is incomplete",
            })?;
            match key.as_str() {
                "type" => {
                    patch_type = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM polyMesh boundary patch type is incomplete",
                    })?);
                    cursor.expect(";")?;
                }
                "nFaces" => {
                    n_faces = Some(parse_openfoam_usize_token(
                        &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                            reason: "OpenFOAM polyMesh boundary nFaces is incomplete",
                        })?,
                        "openfoam_poly_mesh_boundary_n_faces",
                    )?);
                    cursor.expect(";")?;
                }
                "startFace" => {
                    start_face = Some(parse_openfoam_usize_token(
                        &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                            reason: "OpenFOAM polyMesh boundary startFace is incomplete",
                        })?,
                        "openfoam_poly_mesh_boundary_start_face",
                    )?);
                    cursor.expect(";")?;
                }
                _ => skip_openfoam_entry(cursor)?,
            }
        }

        let topology = OpenFoamBoundaryPatchTopology {
            patch_type: patch_type.ok_or(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_patch_type",
            })?,
            n_faces: n_faces.ok_or(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_n_faces",
            })?,
            start_face: start_face.ok_or(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_start_face",
            })?,
        };
        let end_face =
            topology
                .start_face
                .checked_add(topology.n_faces)
                .ok_or(AerodbError::InvalidInput {
                    field: "openfoam_poly_mesh_boundary_face_range",
                })?;
        ranges.push((topology.start_face, end_face));
        if patches.insert(patch_name, topology).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_duplicate_patch",
            });
        }
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    validate_openfoam_boundary_topology_ranges(&mut ranges)?;
    Ok(patches)
}

fn validate_openfoam_boundary_topology_ranges(
    ranges: &mut [(usize, usize)],
) -> Result<(), AerodbError> {
    ranges.sort_unstable();
    let mut previous_end = None;
    for &(start, end) in ranges.iter() {
        if end < start {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_face_range",
            });
        }
        if let Some(previous_end) = previous_end
            && start < previous_end
        {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_face_range",
            });
        }
        previous_end = Some(end);
    }
    Ok(())
}

fn read_openfoam_poly_mesh_file(
    case_root: &Path,
    object_name: &str,
    field: &'static str,
) -> Result<String, AerodbError> {
    let relative_path = format!("constant/polyMesh/{object_name}");
    fs::read_to_string(resolve_solver_plan_path(case_root, &relative_path, field)?)
        .map_err(|_| AerodbError::InvalidInput { field })
}

fn read_optional_openfoam_poly_mesh_file(
    case_root: &Path,
    object_name: &str,
    field: &'static str,
) -> Result<Option<String>, AerodbError> {
    let relative_path = format!("constant/polyMesh/{object_name}");
    let path = resolve_solver_plan_path(case_root, &relative_path, field)?;
    if !path.exists() {
        return Ok(None);
    }
    fs::read_to_string(path)
        .map(Some)
        .map_err(|_| AerodbError::InvalidInput { field })
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum OpenFoamPolyMeshZoneKind {
    Point,
    Face,
    Cell,
}

impl OpenFoamPolyMeshZoneKind {
    fn object_name(self) -> &'static str {
        match self {
            Self::Point => "pointZones",
            Self::Face => "faceZones",
            Self::Cell => "cellZones",
        }
    }

    fn expected_class(self) -> &'static str {
        match self {
            Self::Point => "pointZoneList",
            Self::Face => "faceZoneList",
            Self::Cell => "cellZoneList",
        }
    }

    fn label_key(self) -> &'static str {
        match self {
            Self::Point => "pointLabels",
            Self::Face => "faceLabels",
            Self::Cell => "cellLabels",
        }
    }
}

fn reconstruct_openfoam_face_geometry(
    points: &[Vector3<f64>],
    face: &[usize],
) -> Result<OpenFoamPolyMeshFaceGeometry, AerodbError> {
    let anchor = points[face[0]];
    let mut area_vector = Vector3::zeros();
    let mut scalar_area = 0.0;
    let mut weighted_centroid = Vector3::zeros();
    for vertex_index in 1..(face.len() - 1) {
        let a = anchor;
        let b = points[face[vertex_index]];
        let c = points[face[vertex_index + 1]];
        let triangle_area_vector = 0.5 * (b - a).cross(&(c - a));
        let triangle_area = triangle_area_vector.norm();
        if !(triangle_area.is_finite() && triangle_area > 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_geometry",
            });
        }
        area_vector += triangle_area_vector;
        scalar_area += triangle_area;
        weighted_centroid += ((a + b + c) / 3.0) * triangle_area;
    }

    let area_vector_norm = area_vector.norm();
    if !(area_vector_norm.is_finite() && area_vector_norm > 0.0 && scalar_area.is_finite()) {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_face_geometry",
        });
    }
    let centroid = weighted_centroid / scalar_area;
    if !openfoam_vector3_is_finite(&centroid) {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_face_geometry",
        });
    }

    Ok(OpenFoamPolyMeshFaceGeometry {
        centroid,
        area_vector,
        area_m2: scalar_area,
        unit_normal: area_vector / area_vector_norm,
    })
}

fn inferred_openfoam_cell_count<I>(labels: I, field: &'static str) -> Result<usize, AerodbError>
where
    I: Iterator<Item = usize>,
{
    labels
        .max()
        .ok_or(AerodbError::InvalidInput { field })?
        .checked_add(1)
        .ok_or(AerodbError::InvalidInput { field })
}

fn openfoam_vector3_is_finite(value: &Vector3<f64>) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn parse_openfoam_poly_mesh_points(text: &str) -> Result<Vec<Vector3<f64>>, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut points = None;
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
            class_name = class;
            object_name = object;
            continue;
        }
        if let Ok(count) = token.parse::<usize>() {
            cursor.next();
            points = Some(parse_openfoam_vector3_list(
                &mut cursor,
                count,
                "openfoam_poly_mesh_point",
            )?);
            continue;
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }
    validate_openfoam_poly_mesh_file_metadata(
        class_name,
        object_name,
        "vectorField",
        "points",
        "openfoam_poly_mesh_points_class",
        "openfoam_poly_mesh_points_object",
    )?;
    let points = points.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_points",
    })?;
    if points.is_empty() {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_points",
        });
    }
    Ok(points)
}

fn parse_openfoam_vector3_list(
    cursor: &mut OpenFoamTokenCursor,
    count: usize,
    field: &'static str,
) -> Result<Vec<Vector3<f64>>, AerodbError> {
    cursor.expect("(")?;
    let mut values = Vec::with_capacity(count);
    for _ in 0..count {
        values.push(parse_openfoam_vector3(cursor, field)?);
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    Ok(values)
}

fn parse_openfoam_poly_mesh_faces(text: &str) -> Result<Vec<Vec<usize>>, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut faces = None;
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
            class_name = class;
            object_name = object;
            continue;
        }
        if let Ok(count) = token.parse::<usize>() {
            cursor.next();
            faces = Some(parse_openfoam_face_list(&mut cursor, count)?);
            continue;
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }
    validate_openfoam_poly_mesh_file_metadata(
        class_name,
        object_name,
        "faceList",
        "faces",
        "openfoam_poly_mesh_faces_class",
        "openfoam_poly_mesh_faces_object",
    )?;
    let faces = faces.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_faces",
    })?;
    if faces.is_empty() {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_faces",
        });
    }
    Ok(faces)
}

fn parse_openfoam_face_list(
    cursor: &mut OpenFoamTokenCursor,
    count: usize,
) -> Result<Vec<Vec<usize>>, AerodbError> {
    cursor.expect("(")?;
    let mut faces = Vec::with_capacity(count);
    for _ in 0..count {
        let vertex_count = parse_openfoam_usize_token(
            &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM face vertex count is incomplete",
            })?,
            "openfoam_poly_mesh_face_vertices",
        )?;
        if vertex_count < 3 {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_vertices",
            });
        }
        cursor.expect("(")?;
        let mut face = Vec::with_capacity(vertex_count);
        for _ in 0..vertex_count {
            face.push(parse_openfoam_usize_token(
                &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM face vertex index is incomplete",
                })?,
                "openfoam_poly_mesh_face_vertex",
            )?);
        }
        cursor.expect(")")?;
        faces.push(face);
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    Ok(faces)
}

fn parse_openfoam_poly_mesh_label_list(
    text: &str,
    object_name: &'static str,
) -> Result<Vec<usize>, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut parsed_object_name = None;
    let mut labels = None;
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
            class_name = class;
            parsed_object_name = object;
            continue;
        }
        if let Ok(count) = token.parse::<usize>() {
            cursor.next();
            labels = Some(parse_openfoam_usize_list(
                &mut cursor,
                count,
                if object_name == "owner" {
                    "openfoam_poly_mesh_owner"
                } else {
                    "openfoam_poly_mesh_neighbour"
                },
            )?);
            continue;
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }
    validate_openfoam_poly_mesh_file_metadata(
        class_name,
        parsed_object_name,
        "labelList",
        object_name,
        if object_name == "owner" {
            "openfoam_poly_mesh_owner_class"
        } else {
            "openfoam_poly_mesh_neighbour_class"
        },
        if object_name == "owner" {
            "openfoam_poly_mesh_owner_object"
        } else {
            "openfoam_poly_mesh_neighbour_object"
        },
    )?;
    labels.ok_or(AerodbError::InvalidInput {
        field: if object_name == "owner" {
            "openfoam_poly_mesh_owner"
        } else {
            "openfoam_poly_mesh_neighbour"
        },
    })
}

fn parse_openfoam_usize_list(
    cursor: &mut OpenFoamTokenCursor,
    count: usize,
    field: &'static str,
) -> Result<Vec<usize>, AerodbError> {
    cursor.expect("(")?;
    let mut values = Vec::with_capacity(count);
    for _ in 0..count {
        values.push(parse_openfoam_usize_token(
            &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM label list entry is incomplete",
            })?,
            field,
        )?);
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    Ok(values)
}

fn validate_openfoam_poly_mesh_file_metadata(
    class_name: Option<String>,
    object_name: Option<String>,
    expected_class: &'static str,
    expected_object: &'static str,
    class_field: &'static str,
    object_field: &'static str,
) -> Result<(), AerodbError> {
    let class_name = class_name.ok_or(AerodbError::InvalidInput { field: class_field })?;
    if class_name != expected_class {
        return Err(AerodbError::InvalidInput { field: class_field });
    }
    let object_name = object_name.ok_or(AerodbError::InvalidInput {
        field: object_field,
    })?;
    if object_name != expected_object {
        return Err(AerodbError::InvalidInput {
            field: object_field,
        });
    }
    Ok(())
}

fn parse_openfoam_poly_mesh_zones(
    text: &str,
    kind: OpenFoamPolyMeshZoneKind,
) -> Result<BTreeMap<String, OpenFoamPolyMeshZone>, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut zones = None;
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
            class_name = class;
            object_name = object;
            continue;
        }
        if let Ok(count) = token.parse::<usize>() {
            cursor.next();
            zones = Some(parse_openfoam_poly_mesh_zone_entries(
                &mut cursor,
                count,
                kind,
            )?);
            continue;
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }
    validate_openfoam_poly_mesh_zone_metadata(class_name, object_name, kind)?;
    zones.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_zone_list",
    })
}

fn validate_openfoam_poly_mesh_zone_metadata(
    class_name: Option<String>,
    object_name: Option<String>,
    kind: OpenFoamPolyMeshZoneKind,
) -> Result<(), AerodbError> {
    let class_name = class_name.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_zone_class",
    })?;
    if class_name != "regIOobject" && class_name != kind.expected_class() {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_zone_class",
        });
    }
    let object_name = object_name.ok_or(AerodbError::InvalidInput {
        field: "openfoam_poly_mesh_zone_object",
    })?;
    if object_name != kind.object_name() {
        return Err(AerodbError::InvalidInput {
            field: "openfoam_poly_mesh_zone_object",
        });
    }
    Ok(())
}

fn parse_openfoam_poly_mesh_zone_entries(
    cursor: &mut OpenFoamTokenCursor,
    count: usize,
    kind: OpenFoamPolyMeshZoneKind,
) -> Result<BTreeMap<String, OpenFoamPolyMeshZone>, AerodbError> {
    cursor.expect("(")?;
    let mut zones = BTreeMap::new();
    for _ in 0..count {
        let zone_name = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM polyMesh zone name is incomplete",
        })?;
        validate_case_id_prefix(&zone_name, "openfoam_poly_mesh_zone_name")?;
        cursor.expect("{")?;
        let mut zone_type = None;
        let mut labels = None;
        let mut flip_map = None;
        let mut closed = false;
        while let Some(token) = cursor.peek() {
            if token == "}" {
                cursor.next();
                closed = true;
                break;
            }
            let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM polyMesh zone entry is incomplete",
            })?;
            if key == "type" {
                zone_type = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM polyMesh zone type is incomplete",
                })?);
                cursor.expect(";")?;
            } else if key == kind.label_key() {
                labels = Some(parse_openfoam_counted_usize_entry(
                    cursor,
                    "openfoam_poly_mesh_zone_labels",
                )?);
            } else if key == "flipMap" && kind == OpenFoamPolyMeshZoneKind::Face {
                flip_map = Some(parse_openfoam_counted_bool_entry(
                    cursor,
                    "openfoam_poly_mesh_face_zone_flip_map",
                )?);
            } else {
                skip_openfoam_entry(cursor)?;
            }
        }
        if !closed {
            return Err(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM polyMesh zone block did not close",
            });
        }
        let zone = OpenFoamPolyMeshZone {
            zone_type: zone_type.ok_or(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_zone_type",
            })?,
            labels: labels.ok_or(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_zone_labels",
            })?,
            flip_map,
        };
        if zones.insert(zone_name, zone).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_zone_duplicate",
            });
        }
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    Ok(zones)
}

fn parse_openfoam_counted_usize_entry(
    cursor: &mut OpenFoamTokenCursor,
    field: &'static str,
) -> Result<Vec<usize>, AerodbError> {
    let count = parse_openfoam_counted_list_count(
        cursor,
        "OpenFOAM counted label list is missing a count",
    )?;
    parse_openfoam_usize_list(cursor, count, field)
}

fn parse_openfoam_counted_bool_entry(
    cursor: &mut OpenFoamTokenCursor,
    field: &'static str,
) -> Result<Vec<bool>, AerodbError> {
    let count =
        parse_openfoam_counted_list_count(cursor, "OpenFOAM counted bool list is missing a count")?;
    cursor.expect("(")?;
    let mut values = Vec::with_capacity(count);
    for _ in 0..count {
        values.push(parse_openfoam_zone_bool_token(
            &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM counted bool list entry is incomplete",
            })?,
            field,
        )?);
    }
    cursor.expect(")")?;
    if cursor.peek() == Some(";") {
        cursor.next();
    }
    Ok(values)
}

fn parse_openfoam_counted_list_count(
    cursor: &mut OpenFoamTokenCursor,
    missing_count_reason: &'static str,
) -> Result<usize, AerodbError> {
    while let Some(token) = cursor.next() {
        if let Ok(count) = token.parse::<usize>() {
            return Ok(count);
        }
        if matches!(token.as_str(), "{" | "}" | "(" | ")" | ";") {
            return Err(AerodbError::MalformedOpenFoam {
                reason: missing_count_reason,
            });
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: missing_count_reason,
    })
}

fn parse_openfoam_zone_bool_token(token: &str, field: &'static str) -> Result<bool, AerodbError> {
    match token {
        "1" => Ok(true),
        "0" => Ok(false),
        _ => parse_openfoam_bool_token(token, field),
    }
}

fn parse_openfoam_turbulence_properties(
    text: &str,
) -> Result<OpenFoamTurbulenceProperties, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut simulation_type = None;
    let mut ras = None;
    let mut les = None;
    while let Some(token) = cursor.peek() {
        match token {
            "FoamFile" => {
                cursor.next();
                let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
                class_name = class;
                object_name = object;
            }
            "simulationType" => {
                cursor.next();
                let token = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM turbulenceProperties simulationType is incomplete",
                })?;
                simulation_type = Some(parse_openfoam_turbulence_simulation_type(&token)?);
                cursor.expect(";")?;
            }
            "RAS" => {
                cursor.next();
                ras = Some(parse_openfoam_turbulence_model_block(
                    &mut cursor,
                    OpenFoamTurbulenceSimulationType::Ras,
                )?);
            }
            "LES" => {
                cursor.next();
                les = Some(parse_openfoam_turbulence_model_block(
                    &mut cursor,
                    OpenFoamTurbulenceSimulationType::Les,
                )?);
            }
            _ => {
                cursor.next();
                skip_openfoam_entry(&mut cursor)?;
            }
        }
    }
    validate_openfoam_poly_mesh_file_metadata(
        class_name.clone(),
        object_name.clone(),
        "dictionary",
        "turbulenceProperties",
        "openfoam_turbulence_properties_class",
        "openfoam_turbulence_properties_object",
    )?;
    let simulation_type = simulation_type.ok_or(AerodbError::InvalidInput {
        field: "openfoam_turbulence_simulation_type",
    })?;
    let (model_name, turbulence, print_coeffs, model_coefficients) = match simulation_type {
        OpenFoamTurbulenceSimulationType::Laminar => (None, None, None, BTreeMap::new()),
        OpenFoamTurbulenceSimulationType::Ras => {
            let block = ras.ok_or(AerodbError::InvalidInput {
                field: "openfoam_turbulence_model",
            })?;
            (
                Some(block.model_name),
                Some(block.turbulence),
                Some(block.print_coeffs),
                block.coefficients,
            )
        }
        OpenFoamTurbulenceSimulationType::Les => {
            let block = les.ok_or(AerodbError::InvalidInput {
                field: "openfoam_turbulence_model",
            })?;
            (
                Some(block.model_name),
                Some(block.turbulence),
                Some(block.print_coeffs),
                block.coefficients,
            )
        }
    };
    Ok(OpenFoamTurbulenceProperties {
        class_name: class_name.unwrap_or_default(),
        object_name: object_name.unwrap_or_default(),
        simulation_type,
        model_name,
        turbulence,
        print_coeffs,
        model_coefficients,
    })
}

fn parse_openfoam_turbulence_simulation_type(
    token: &str,
) -> Result<OpenFoamTurbulenceSimulationType, AerodbError> {
    match token {
        "laminar" => Ok(OpenFoamTurbulenceSimulationType::Laminar),
        "RAS" => Ok(OpenFoamTurbulenceSimulationType::Ras),
        "LES" => Ok(OpenFoamTurbulenceSimulationType::Les),
        _ => Err(AerodbError::InvalidInput {
            field: "openfoam_turbulence_simulation_type",
        }),
    }
}

fn parse_openfoam_turbulence_model_block(
    cursor: &mut OpenFoamTokenCursor,
    simulation_type: OpenFoamTurbulenceSimulationType,
) -> Result<OpenFoamTurbulenceModelBlock, AerodbError> {
    cursor.expect("{")?;
    let mut model_name = None;
    let mut turbulence = None;
    let mut print_coeffs = None;
    let mut coefficients = BTreeMap::new();
    let mut closed = false;
    while let Some(token) = cursor.peek() {
        if token == "}" {
            cursor.next();
            if cursor.peek() == Some(";") {
                cursor.next();
            }
            closed = true;
            break;
        }
        let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM turbulence model entry is incomplete",
        })?;
        match key.as_str() {
            "RASModel" if simulation_type == OpenFoamTurbulenceSimulationType::Ras => {
                model_name = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM RASModel entry is incomplete",
                })?);
                cursor.expect(";")?;
            }
            "LESModel" if simulation_type == OpenFoamTurbulenceSimulationType::Les => {
                model_name = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM LESModel entry is incomplete",
                })?);
                cursor.expect(";")?;
            }
            "turbulence" => {
                turbulence = Some(parse_openfoam_bool_token(
                    &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM turbulence flag is incomplete",
                    })?,
                    "openfoam_turbulence_active",
                )?);
                cursor.expect(";")?;
            }
            "printCoeffs" => {
                print_coeffs = Some(parse_openfoam_bool_token(
                    &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM printCoeffs flag is incomplete",
                    })?,
                    "openfoam_turbulence_print_coeffs",
                )?);
                cursor.expect(";")?;
            }
            _ if key.ends_with("Coeffs") => {
                let parsed = parse_openfoam_turbulence_coefficients_block(cursor)?;
                for (coefficient_name, coefficient_value) in parsed {
                    if coefficients
                        .insert(coefficient_name, coefficient_value)
                        .is_some()
                    {
                        return Err(AerodbError::InvalidInput {
                            field: "openfoam_turbulence_coefficient_duplicate",
                        });
                    }
                }
            }
            _ => skip_openfoam_turbulence_model_entry(cursor)?,
        }
    }
    if !closed {
        return Err(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM turbulence model block did not close",
        });
    }
    Ok(OpenFoamTurbulenceModelBlock {
        model_name: model_name.ok_or(AerodbError::InvalidInput {
            field: "openfoam_turbulence_model",
        })?,
        turbulence: turbulence.ok_or(AerodbError::InvalidInput {
            field: "openfoam_turbulence_active",
        })?,
        print_coeffs: print_coeffs.ok_or(AerodbError::InvalidInput {
            field: "openfoam_turbulence_print_coeffs",
        })?,
        coefficients,
    })
}

struct OpenFoamTurbulenceModelBlock {
    model_name: String,
    turbulence: bool,
    print_coeffs: bool,
    coefficients: BTreeMap<String, f64>,
}

fn parse_openfoam_turbulence_coefficients_block(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<BTreeMap<String, f64>, AerodbError> {
    cursor.expect("{")?;
    let mut coefficients = BTreeMap::new();
    let mut closed = false;
    while let Some(token) = cursor.peek() {
        if token == "}" {
            cursor.next();
            if cursor.peek() == Some(";") {
                cursor.next();
            }
            closed = true;
            break;
        }
        let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM turbulence coefficient entry is incomplete",
        })?;
        validate_case_id_prefix(&key, "openfoam_turbulence_coefficient_name")?;
        match cursor.peek() {
            Some("{") => skip_openfoam_braced_block(cursor)?,
            Some("(") | Some("[") => skip_openfoam_entry(cursor)?,
            Some(_) => {
                let value_token = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM turbulence coefficient value is incomplete",
                })?;
                cursor.expect(";")?;
                let value =
                    parse_openfoam_f64_token(&value_token, "openfoam_turbulence_coefficient")?;
                if coefficients.insert(key, value).is_some() {
                    return Err(AerodbError::InvalidInput {
                        field: "openfoam_turbulence_coefficient_duplicate",
                    });
                }
            }
            None => {
                return Err(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM turbulence coefficient value is incomplete",
                });
            }
        }
    }
    if !closed {
        return Err(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM turbulence coefficient block did not close",
        });
    }
    Ok(coefficients)
}

fn skip_openfoam_turbulence_model_entry(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<(), AerodbError> {
    if cursor.peek() == Some("{") {
        skip_openfoam_braced_block(cursor)
    } else {
        skip_openfoam_entry(cursor)
    }
}

fn skip_openfoam_braced_block(cursor: &mut OpenFoamTokenCursor) -> Result<(), AerodbError> {
    cursor.expect("{")?;
    let mut depth = 1usize;
    while let Some(token) = cursor.next() {
        match token.as_str() {
            "{" => depth = depth.saturating_add(1),
            "}" => {
                depth -= 1;
                if depth == 0 {
                    if cursor.peek() == Some(";") {
                        cursor.next();
                    }
                    return Ok(());
                }
            }
            _ => {}
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: "OpenFOAM braced block did not close",
    })
}

fn parse_openfoam_bool_token(token: &str, field: &'static str) -> Result<bool, AerodbError> {
    match token {
        "on" | "true" | "yes" => Ok(true),
        "off" | "false" | "no" => Ok(false),
        _ => Err(AerodbError::InvalidInput { field }),
    }
}

fn parse_openfoam_field_metadata(text: &str) -> Result<(String, String), AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    while let Some(token) = cursor.peek() {
        if token == "FoamFile" {
            cursor.next();
            let (class_name, object_name) = parse_openfoam_foamfile_block(&mut cursor)?;
            return Ok((
                class_name.ok_or(AerodbError::InvalidInput {
                    field: "openfoam_case_field_class",
                })?,
                object_name.ok_or(AerodbError::InvalidInput {
                    field: "openfoam_case_field_object",
                })?,
            ));
        }
        cursor.next();
        skip_openfoam_entry(&mut cursor)?;
    }
    Err(AerodbError::InvalidInput {
        field: "openfoam_case_field_class",
    })
}

fn parse_openfoam_scalar_field(text: &str) -> Result<OpenFoamScalarField, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut dimensions = None;
    let mut internal_field = None;
    let mut boundary_patches = None;
    while let Some(token) = cursor.peek() {
        match token {
            "FoamFile" => {
                cursor.next();
                let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
                class_name = class;
                object_name = object;
            }
            "dimensions" => {
                cursor.next();
                dimensions = Some(parse_openfoam_dimensions(
                    &mut cursor,
                    "openfoam_scalar_field_dimensions",
                )?);
            }
            "internalField" => {
                cursor.next();
                internal_field = Some(parse_openfoam_scalar_value(&mut cursor)?);
            }
            "boundaryField" => {
                cursor.next();
                boundary_patches = Some(parse_openfoam_boundary_field(&mut cursor)?);
            }
            _ => {
                cursor.next();
                skip_openfoam_entry(&mut cursor)?;
            }
        }
    }
    Ok(OpenFoamScalarField {
        class_name: class_name.ok_or(AerodbError::InvalidInput {
            field: "openfoam_scalar_field_class",
        })?,
        object_name: object_name.ok_or(AerodbError::InvalidInput {
            field: "openfoam_scalar_field_object",
        })?,
        dimensions: dimensions.ok_or(AerodbError::InvalidInput {
            field: "openfoam_scalar_field_dimensions",
        })?,
        internal_field: internal_field.ok_or(AerodbError::InvalidInput {
            field: "openfoam_scalar_field_internal_field",
        })?,
        boundary_patches: boundary_patches.ok_or(AerodbError::InvalidInput {
            field: "openfoam_scalar_field_boundary_field",
        })?,
    })
}

fn parse_openfoam_vector_field(text: &str) -> Result<OpenFoamVectorField, AerodbError> {
    let tokens = tokenize_openfoam_field(text)?;
    let mut cursor = OpenFoamTokenCursor::new(tokens);
    let mut class_name = None;
    let mut object_name = None;
    let mut dimensions = None;
    let mut internal_field = None;
    let mut boundary_patches = None;
    while let Some(token) = cursor.peek() {
        match token {
            "FoamFile" => {
                cursor.next();
                let (class, object) = parse_openfoam_foamfile_block(&mut cursor)?;
                class_name = class;
                object_name = object;
            }
            "dimensions" => {
                cursor.next();
                dimensions = Some(parse_openfoam_dimensions(
                    &mut cursor,
                    "openfoam_vector_field_dimensions",
                )?);
            }
            "internalField" => {
                cursor.next();
                internal_field = Some(parse_openfoam_vector_value(&mut cursor)?);
            }
            "boundaryField" => {
                cursor.next();
                boundary_patches = Some(parse_openfoam_vector_boundary_field(&mut cursor)?);
            }
            _ => {
                cursor.next();
                skip_openfoam_entry(&mut cursor)?;
            }
        }
    }
    Ok(OpenFoamVectorField {
        class_name: class_name.ok_or(AerodbError::InvalidInput {
            field: "openfoam_vector_field_class",
        })?,
        object_name: object_name.ok_or(AerodbError::InvalidInput {
            field: "openfoam_vector_field_object",
        })?,
        dimensions: dimensions.ok_or(AerodbError::InvalidInput {
            field: "openfoam_vector_field_dimensions",
        })?,
        internal_field: internal_field.ok_or(AerodbError::InvalidInput {
            field: "openfoam_vector_field_internal_field",
        })?,
        boundary_patches: boundary_patches.ok_or(AerodbError::InvalidInput {
            field: "openfoam_vector_field_boundary_field",
        })?,
    })
}

fn tokenize_openfoam_field(text: &str) -> Result<Vec<String>, AerodbError> {
    let mut tokens = Vec::new();
    let mut current = String::new();
    let mut chars = text.chars().peekable();
    while let Some(ch) = chars.next() {
        if ch == '/' && chars.peek() == Some(&'/') {
            chars.next();
            for next in chars.by_ref() {
                if next == '\n' {
                    break;
                }
            }
            continue;
        }
        if ch == '/' && chars.peek() == Some(&'*') {
            chars.next();
            let mut closed = false;
            let mut previous = '\0';
            for next in chars.by_ref() {
                if previous == '*' && next == '/' {
                    closed = true;
                    break;
                }
                previous = next;
            }
            if !closed {
                return Err(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM block comment did not close",
                });
            }
            continue;
        }
        if ch.is_whitespace() {
            if !current.is_empty() {
                tokens.push(std::mem::take(&mut current));
            }
            continue;
        }
        if matches!(ch, '{' | '}' | '(' | ')' | '[' | ']' | ';') {
            if !current.is_empty() {
                tokens.push(std::mem::take(&mut current));
            }
            tokens.push(ch.to_string());
            continue;
        }
        if ch == '"' {
            if !current.is_empty() {
                tokens.push(std::mem::take(&mut current));
            }
            let mut quoted = String::new();
            let mut closed = false;
            for next in chars.by_ref() {
                if next == '"' {
                    closed = true;
                    break;
                }
                quoted.push(next);
            }
            if !closed {
                return Err(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM quoted string did not close",
                });
            }
            tokens.push(quoted);
            continue;
        }
        current.push(ch);
    }
    if !current.is_empty() {
        tokens.push(current);
    }
    Ok(tokens)
}

struct OpenFoamTokenCursor {
    tokens: Vec<String>,
    index: usize,
}

impl OpenFoamTokenCursor {
    fn new(tokens: Vec<String>) -> Self {
        Self { tokens, index: 0 }
    }

    fn peek(&self) -> Option<&str> {
        self.tokens.get(self.index).map(String::as_str)
    }

    fn next(&mut self) -> Option<String> {
        let token = self.tokens.get(self.index).cloned();
        if token.is_some() {
            self.index += 1;
        }
        token
    }

    fn expect(&mut self, expected: &str) -> Result<(), AerodbError> {
        match self.next() {
            Some(token) if token == expected => Ok(()),
            _ => Err(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM field dictionary token did not match expected syntax",
            }),
        }
    }
}

fn parse_openfoam_foamfile_block(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<(Option<String>, Option<String>), AerodbError> {
    cursor.expect("{")?;
    let mut class_name = None;
    let mut object_name = None;
    while let Some(token) = cursor.peek() {
        if token == "}" {
            cursor.next();
            return Ok((class_name, object_name));
        }
        let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM FoamFile entry is incomplete",
        })?;
        let value = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM FoamFile value is incomplete",
        })?;
        cursor.expect(";")?;
        match key.as_str() {
            "class" => class_name = Some(value),
            "object" => object_name = Some(value),
            _ => {}
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: "OpenFOAM FoamFile block did not close",
    })
}

fn parse_openfoam_dimensions(
    cursor: &mut OpenFoamTokenCursor,
    field: &'static str,
) -> Result<[f64; 7], AerodbError> {
    cursor.expect("[")?;
    let mut values = [0.0; 7];
    for value in &mut values {
        *value = parse_openfoam_f64_token(
            &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM dimensions entry is incomplete",
            })?,
            field,
        )?;
    }
    cursor.expect("]")?;
    cursor.expect(";")?;
    Ok(values)
}

fn parse_openfoam_boundary_field(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<BTreeMap<String, OpenFoamScalarPatch>, AerodbError> {
    cursor.expect("{")?;
    let mut patches = BTreeMap::new();
    while let Some(token) = cursor.peek() {
        if token == "}" {
            cursor.next();
            if cursor.peek() == Some(";") {
                cursor.next();
            }
            return Ok(patches);
        }
        let patch_name = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM boundary patch name is missing",
        })?;
        cursor.expect("{")?;
        let mut patch_type = None;
        let mut value = None;
        while let Some(patch_token) = cursor.peek() {
            if patch_token == "}" {
                cursor.next();
                break;
            }
            let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM boundary patch entry is incomplete",
            })?;
            match key.as_str() {
                "type" => {
                    patch_type = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM boundary patch type is incomplete",
                    })?);
                    cursor.expect(";")?;
                }
                "value" => value = Some(parse_openfoam_scalar_value(cursor)?),
                _ => skip_openfoam_entry(cursor)?,
            }
        }
        let previous = patches.insert(
            patch_name,
            OpenFoamScalarPatch {
                patch_type: patch_type.ok_or(AerodbError::InvalidInput {
                    field: "openfoam_scalar_field_patch_type",
                })?,
                value,
            },
        );
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_scalar_field_duplicate_patch",
            });
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: "OpenFOAM boundaryField block did not close",
    })
}

fn parse_openfoam_vector_boundary_field(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<BTreeMap<String, OpenFoamVectorPatch>, AerodbError> {
    cursor.expect("{")?;
    let mut patches = BTreeMap::new();
    while let Some(token) = cursor.peek() {
        if token == "}" {
            cursor.next();
            if cursor.peek() == Some(";") {
                cursor.next();
            }
            return Ok(patches);
        }
        let patch_name = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM boundary patch name is missing",
        })?;
        cursor.expect("{")?;
        let mut patch_type = None;
        let mut value = None;
        while let Some(patch_token) = cursor.peek() {
            if patch_token == "}" {
                cursor.next();
                break;
            }
            let key = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM boundary patch entry is incomplete",
            })?;
            match key.as_str() {
                "type" => {
                    patch_type = Some(cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM boundary patch type is incomplete",
                    })?);
                    cursor.expect(";")?;
                }
                "value" => value = Some(parse_openfoam_vector_value(cursor)?),
                _ => skip_openfoam_entry(cursor)?,
            }
        }
        let previous = patches.insert(
            patch_name,
            OpenFoamVectorPatch {
                patch_type: patch_type.ok_or(AerodbError::InvalidInput {
                    field: "openfoam_vector_field_patch_type",
                })?,
                value,
            },
        );
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "openfoam_vector_field_duplicate_patch",
            });
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: "OpenFOAM boundaryField block did not close",
    })
}

fn parse_openfoam_scalar_value(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<OpenFoamScalarFieldValue, AerodbError> {
    match cursor.next().as_deref() {
        Some("uniform") => {
            let value = parse_openfoam_f64_token(
                &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM uniform scalar value is incomplete",
                })?,
                "openfoam_scalar_field_value",
            )?;
            cursor.expect(";")?;
            Ok(OpenFoamScalarFieldValue::Uniform(value))
        }
        Some("nonuniform") => {
            let mut count = None;
            while let Some(token) = cursor.next() {
                if let Ok(parsed) = token.parse::<usize>() {
                    count = Some(parsed);
                    break;
                }
                if token == "(" || token == ";" {
                    return Err(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM nonuniform scalar list is missing a count",
                    });
                }
            }
            let count = count.ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM nonuniform scalar list is missing a count",
            })?;
            cursor.expect("(")?;
            let mut values = Vec::with_capacity(count);
            for _ in 0..count {
                let token = cursor.next().ok_or(AerodbError::MalformedOpenFoam {
                    reason: "OpenFOAM nonuniform scalar list is incomplete",
                })?;
                if token == ")" {
                    return Err(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM nonuniform scalar list is incomplete",
                    });
                }
                values.push(parse_openfoam_f64_token(
                    &token,
                    "openfoam_scalar_field_value",
                )?);
            }
            cursor.expect(")")?;
            cursor.expect(";")?;
            Ok(OpenFoamScalarFieldValue::Nonuniform(values))
        }
        _ => Err(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM scalar field value must be uniform or nonuniform",
        }),
    }
}

fn parse_openfoam_vector_value(
    cursor: &mut OpenFoamTokenCursor,
) -> Result<OpenFoamVectorFieldValue, AerodbError> {
    match cursor.next().as_deref() {
        Some("uniform") => {
            let value = parse_openfoam_vector3(cursor, "openfoam_vector_field_value")?;
            cursor.expect(";")?;
            Ok(OpenFoamVectorFieldValue::Uniform(value))
        }
        Some("nonuniform") => {
            let mut count = None;
            while let Some(token) = cursor.next() {
                if let Ok(parsed) = token.parse::<usize>() {
                    count = Some(parsed);
                    break;
                }
                if token == "(" || token == ";" {
                    return Err(AerodbError::MalformedOpenFoam {
                        reason: "OpenFOAM nonuniform vector list is missing a count",
                    });
                }
            }
            let count = count.ok_or(AerodbError::MalformedOpenFoam {
                reason: "OpenFOAM nonuniform vector list is missing a count",
            })?;
            cursor.expect("(")?;
            let mut values = Vec::with_capacity(count);
            for _ in 0..count {
                values.push(parse_openfoam_vector3(
                    cursor,
                    "openfoam_vector_field_value",
                )?);
            }
            cursor.expect(")")?;
            cursor.expect(";")?;
            Ok(OpenFoamVectorFieldValue::Nonuniform(values))
        }
        _ => Err(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM vector field value must be uniform or nonuniform",
        }),
    }
}

fn parse_openfoam_vector3(
    cursor: &mut OpenFoamTokenCursor,
    field: &'static str,
) -> Result<Vector3<f64>, AerodbError> {
    cursor.expect("(")?;
    let x = parse_openfoam_f64_token(
        &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM vector x component is incomplete",
        })?,
        field,
    )?;
    let y = parse_openfoam_f64_token(
        &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM vector y component is incomplete",
        })?,
        field,
    )?;
    let z = parse_openfoam_f64_token(
        &cursor.next().ok_or(AerodbError::MalformedOpenFoam {
            reason: "OpenFOAM vector z component is incomplete",
        })?,
        field,
    )?;
    cursor.expect(")")?;
    Ok(Vector3::new(x, y, z))
}

fn parse_openfoam_usize_token(token: &str, field: &'static str) -> Result<usize, AerodbError> {
    token
        .parse::<usize>()
        .map_err(|_| AerodbError::InvalidInput { field })
}

fn skip_openfoam_entry(cursor: &mut OpenFoamTokenCursor) -> Result<(), AerodbError> {
    let mut depth = 0usize;
    while let Some(token) = cursor.next() {
        match token.as_str() {
            "{" | "(" | "[" => depth = depth.saturating_add(1),
            "}" | ")" | "]" => {
                if depth == 0 {
                    return Ok(());
                }
                depth -= 1;
            }
            ";" if depth == 0 => return Ok(()),
            _ => {}
        }
    }
    Err(AerodbError::MalformedOpenFoam {
        reason: "OpenFOAM dictionary entry did not terminate",
    })
}

fn parse_openfoam_f64_token(token: &str, field: &'static str) -> Result<f64, AerodbError> {
    let value = token
        .parse::<f64>()
        .map_err(|_| AerodbError::InvalidInput { field })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput { field })
    }
}

fn cart3d_triq_vertex_cp_from_text(text: &str, mesh: &PanelMesh) -> Result<Vec<f64>, AerodbError> {
    let tokens = tokenize_cart3d_triq(text);
    let mut cursor = 0usize;
    let vertex_count = parse_cart3d_triq_usize(&tokens, &mut cursor, "Cart3D TRI/Q vertex count")?;
    let face_count = parse_cart3d_triq_usize(&tokens, &mut cursor, "Cart3D TRI/Q face count")?;
    let scalar_count = parse_cart3d_triq_usize(&tokens, &mut cursor, "Cart3D TRI/Q scalar count")?;
    if vertex_count != mesh.vertices().len() {
        return Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_vertices",
        });
    }
    if face_count != mesh.triangles().len() {
        return Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_faces",
        });
    }
    if scalar_count == 0 {
        return Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_scalars",
        });
    }

    let mut mesh_vertices = BTreeMap::new();
    for (index, vertex) in mesh.vertices().iter().copied().enumerate() {
        let previous = mesh_vertices.insert(surface_coordinate_key(vertex)?, index);
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_mesh_vertices",
            });
        }
    }

    let mut triq_vertex_keys = Vec::with_capacity(vertex_count);
    let mut vertex_cp = vec![None; vertex_count];
    for _ in 0..vertex_count {
        let x = parse_cart3d_triq_f64(
            &tokens,
            &mut cursor,
            "Cart3D TRI/Q x coordinate",
            "cart3d_triq_surface_cp_x",
        )?;
        let y = parse_cart3d_triq_f64(
            &tokens,
            &mut cursor,
            "Cart3D TRI/Q y coordinate",
            "cart3d_triq_surface_cp_y",
        )?;
        let z = parse_cart3d_triq_f64(
            &tokens,
            &mut cursor,
            "Cart3D TRI/Q z coordinate",
            "cart3d_triq_surface_cp_z",
        )?;
        let key = surface_coordinate_key(Vector3::new(x, y, z))?;
        let mesh_vertex_index = *mesh_vertices.get(&key).ok_or(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_vertex_coordinate",
        })?;
        if vertex_cp[mesh_vertex_index].is_some() {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_duplicate_vertex",
            });
        }
        triq_vertex_keys.push(key);
    }

    let mut triq_face_keys = BTreeMap::new();
    for _ in 0..face_count {
        let face = [
            parse_cart3d_triq_face_index(&tokens, &mut cursor, vertex_count)?,
            parse_cart3d_triq_face_index(&tokens, &mut cursor, vertex_count)?,
            parse_cart3d_triq_face_index(&tokens, &mut cursor, vertex_count)?,
        ];
        let key = sorted_surface_triangle_key([
            triq_vertex_keys[face[0]],
            triq_vertex_keys[face[1]],
            triq_vertex_keys[face[2]],
        ]);
        let previous = triq_face_keys.insert(key, ());
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_duplicate_face",
            });
        }
    }
    validate_cart3d_triq_mesh_faces(mesh, &triq_face_keys)?;

    for _ in 0..face_count {
        parse_cart3d_triq_usize(&tokens, &mut cursor, "Cart3D TRI/Q component id")?;
    }

    for key in triq_vertex_keys {
        let cp = parse_cart3d_triq_f64(
            &tokens,
            &mut cursor,
            "Cart3D TRI/Q Cp scalar",
            "cart3d_triq_surface_cp_cp",
        )?;
        let mesh_vertex_index = *mesh_vertices.get(&key).ok_or(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_vertex_coordinate",
        })?;
        if vertex_cp[mesh_vertex_index].replace(cp).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_duplicate_vertex",
            });
        }
        for _ in 1..scalar_count {
            parse_cart3d_triq_f64(
                &tokens,
                &mut cursor,
                "Cart3D TRI/Q scalar",
                "cart3d_triq_surface_cp_scalar",
            )?;
        }
    }
    if cursor != tokens.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "Cart3D TRI/Q has trailing fields",
        });
    }
    if vertex_cp.iter().any(Option::is_none) {
        return Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_vertices",
        });
    }
    vertex_cp
        .into_iter()
        .map(|value| {
            value.ok_or(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_vertices",
            })
        })
        .collect()
}

fn tokenize_cart3d_triq(text: &str) -> Vec<&str> {
    text.lines()
        .flat_map(|line| {
            line.split('#')
                .next()
                .unwrap_or_default()
                .split_whitespace()
        })
        .collect()
}

fn parse_cart3d_triq_usize(
    tokens: &[&str],
    cursor: &mut usize,
    context: &'static str,
) -> Result<usize, AerodbError> {
    let token = tokens.get(*cursor).ok_or(AerodbError::MalformedCsv {
        reason: "Cart3D TRI/Q ended before a required field",
    })?;
    *cursor += 1;
    token
        .parse::<usize>()
        .map_err(|_| AerodbError::MalformedCsv { reason: context })
}

fn parse_cart3d_triq_face_index(
    tokens: &[&str],
    cursor: &mut usize,
    vertex_count: usize,
) -> Result<usize, AerodbError> {
    let index = parse_cart3d_triq_usize(tokens, cursor, "Cart3D TRI/Q face index")?;
    if index == 0 || index > vertex_count {
        return Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_face_index",
        });
    }
    Ok(index - 1)
}

fn parse_cart3d_triq_f64(
    tokens: &[&str],
    cursor: &mut usize,
    context: &'static str,
    invalid_field: &'static str,
) -> Result<f64, AerodbError> {
    let token = tokens.get(*cursor).ok_or(AerodbError::MalformedCsv {
        reason: "Cart3D TRI/Q ended before a required field",
    })?;
    *cursor += 1;
    let value = token
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv { reason: context })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: invalid_field,
        })
    }
}

fn validate_cart3d_triq_mesh_faces(
    mesh: &PanelMesh,
    triq_face_keys: &BTreeMap<[(u64, u64, u64); 3], ()>,
) -> Result<(), AerodbError> {
    let mut mesh_face_keys = BTreeMap::new();
    for &[ia, ib, ic] in mesh.triangles() {
        let key = sorted_surface_triangle_key([
            surface_coordinate_key(mesh.vertices()[ia as usize])?,
            surface_coordinate_key(mesh.vertices()[ib as usize])?,
            surface_coordinate_key(mesh.vertices()[ic as usize])?,
        ]);
        let previous = mesh_face_keys.insert(key, ());
        if previous.is_some() {
            return Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_mesh_faces",
            });
        }
    }
    if &mesh_face_keys == triq_face_keys {
        Ok(())
    } else {
        Err(AerodbError::InvalidInput {
            field: "cart3d_triq_surface_cp_faces",
        })
    }
}

fn sorted_surface_triangle_key(mut keys: [(u64, u64, u64); 3]) -> [(u64, u64, u64); 3] {
    keys.sort();
    keys
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct SpartaSurfaceForceColumns {
    surf_id: usize,
    fx: usize,
    fy: usize,
    fz: usize,
}

fn sparta_surface_forces_from_text(
    text: &str,
    mesh: &PanelMesh,
) -> Result<Vec<Vector3<f64>>, AerodbError> {
    let default_columns = SpartaSurfaceForceColumns {
        surf_id: 0,
        fx: 1,
        fy: 2,
        fz: 3,
    };
    let mut columns = None;
    let mut rows = Vec::<(usize, Vector3<f64>)>::new();
    for raw_line in text.lines() {
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let tokens = line.split_whitespace().collect::<Vec<_>>();
        let row_columns = columns.unwrap_or(default_columns);
        if tokens
            .get(row_columns.surf_id)
            .and_then(|value| value.parse::<usize>().ok())
            .is_none()
        {
            if rows.is_empty() {
                columns = Some(sparta_surface_force_columns(&tokens).ok_or(
                    AerodbError::MalformedCsv {
                        reason: "SPARTA surface-force header is missing surf_id, fx, fy, or fz",
                    },
                )?);
                continue;
            }
            return Err(AerodbError::MalformedCsv {
                reason: "SPARTA surface-force data row did not start with a surface id",
            });
        }
        let columns = columns.unwrap_or(default_columns);
        let max_index = columns
            .surf_id
            .max(columns.fx)
            .max(columns.fy)
            .max(columns.fz);
        if tokens.len() <= max_index {
            return Err(AerodbError::MalformedCsv {
                reason: "SPARTA surface-force row has too few fields",
            });
        }
        let surf_id = parse_sparta_surface_usize(tokens[columns.surf_id])?;
        let force = Vector3::new(
            parse_sparta_surface_f64(tokens[columns.fx], "sparta_surface_forces_fx")?,
            parse_sparta_surface_f64(tokens[columns.fy], "sparta_surface_forces_fy")?,
            parse_sparta_surface_f64(tokens[columns.fz], "sparta_surface_forces_fz")?,
        );
        rows.push((surf_id, force));
    }

    let panel_count = mesh.triangles().len();
    if rows.len() != panel_count {
        return Err(AerodbError::InvalidInput {
            field: "sparta_surface_forces_rows",
        });
    }
    let zero_based = rows.iter().any(|(surf_id, _force)| *surf_id == 0);
    let mut panel_forces = vec![None; panel_count];
    for (surf_id, force) in rows {
        let panel_index = if zero_based {
            if surf_id >= panel_count {
                return Err(AerodbError::InvalidInput {
                    field: "sparta_surface_forces_surface_id",
                });
            }
            surf_id
        } else {
            if surf_id == 0 || surf_id > panel_count {
                return Err(AerodbError::InvalidInput {
                    field: "sparta_surface_forces_surface_id",
                });
            }
            surf_id - 1
        };
        if panel_forces[panel_index].replace(force).is_some() {
            return Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_duplicate_surface",
            });
        }
    }
    if panel_forces.iter().any(Option::is_none) {
        return Err(AerodbError::InvalidInput {
            field: "sparta_surface_forces_surfaces",
        });
    }
    panel_forces
        .into_iter()
        .map(|value| {
            value.ok_or(AerodbError::InvalidInput {
                field: "sparta_surface_forces_surfaces",
            })
        })
        .collect()
}

fn sparta_surface_force_columns(tokens: &[&str]) -> Option<SpartaSurfaceForceColumns> {
    let surf_id = sparta_surface_force_column(
        tokens,
        &[
            "surf_id",
            "surface_id",
            "surf",
            "surface",
            "id",
            "panel",
            "panel_index",
        ],
    )?;
    let fx = sparta_surface_force_column(tokens, &["fx", "force_x", "forcex"])?;
    let fy = sparta_surface_force_column(tokens, &["fy", "force_y", "forcey"])?;
    let fz = sparta_surface_force_column(tokens, &["fz", "force_z", "forcez"])?;
    Some(SpartaSurfaceForceColumns {
        surf_id,
        fx,
        fy,
        fz,
    })
}

fn sparta_surface_force_column(tokens: &[&str], names: &[&str]) -> Option<usize> {
    tokens.iter().position(|token| {
        let normalized = normalize_csv_header(token);
        names
            .iter()
            .any(|name| normalized == normalize_csv_header(name))
    })
}

fn parse_sparta_surface_usize(value: &str) -> Result<usize, AerodbError> {
    value
        .parse::<usize>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "SPARTA surface-force surface id did not parse",
        })
}

fn parse_sparta_surface_f64(value: &str, invalid_field: &'static str) -> Result<f64, AerodbError> {
    let value = value
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "SPARTA surface-force component did not parse",
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: invalid_field,
        })
    }
}

fn parse_openfoam_surface_cp_f64(
    value: &str,
    malformed_reason: &'static str,
    invalid_field: &'static str,
) -> Result<f64, AerodbError> {
    let value = value
        .trim_matches(|ch| matches!(ch, '(' | ')' | ';' | ','))
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: malformed_reason,
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: invalid_field,
        })
    }
}

fn su2_surface_column(headers: &csv::StringRecord, names: &[&str]) -> Option<usize> {
    headers.iter().position(|header| {
        let normalized = normalize_csv_header(header);
        names
            .iter()
            .any(|name| normalized == normalize_csv_header(name))
    })
}

fn normalize_csv_header(header: &str) -> String {
    header
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

fn surface_coordinate_key(vertex: Vector3<f64>) -> Result<(u64, u64, u64), AerodbError> {
    if !(vertex.x.is_finite() && vertex.y.is_finite() && vertex.z.is_finite()) {
        return Err(AerodbError::InvalidInput {
            field: "surface_cp_vertex_coordinate",
        });
    }
    Ok((
        canonical_f64_bits(vertex.x),
        canonical_f64_bits(vertex.y),
        canonical_f64_bits(vertex.z),
    ))
}

fn canonical_f64_bits(value: f64) -> u64 {
    if value == 0.0 {
        0.0_f64.to_bits()
    } else {
        value.to_bits()
    }
}

fn su2_history_coefficients_from_csv(
    csv_text: &str,
    node: RunNode,
) -> Result<AeroCoefficients, AerodbError> {
    let mut reader = csv::ReaderBuilder::new()
        .trim(csv::Trim::All)
        .from_reader(csv_text.as_bytes());
    let headers = reader.headers().map_err(|_| AerodbError::MalformedCsv {
        reason: "SU2 history CSV header did not parse",
    })?;
    let cfx_index = su2_history_column(headers, "CFx");
    let cfz_index = su2_history_column(headers, "CFz");
    let cd_index = su2_history_column(headers, "CD");
    let cl_index = su2_history_column(headers, "CL");
    let cmy_index = su2_history_column(headers, "CMy").ok_or(AerodbError::MalformedCsv {
        reason: "SU2 history CSV is missing CMy",
    })?;
    let mut final_record = None;
    for record in reader.records() {
        final_record = Some(record.map_err(|_| AerodbError::MalformedCsv {
            reason: "SU2 history CSV row did not parse",
        })?);
    }
    let final_record = final_record.ok_or(AerodbError::MalformedCsv {
        reason: "SU2 history CSV has no data rows",
    })?;
    let cm = parse_csv_f64_at(
        &final_record,
        cmy_index,
        "SU2 history CSV CMy field did not parse",
        "su2_history_cmy",
    )?;
    let coeffs = match (cfx_index, cfz_index) {
        (Some(cfx_index), Some(cfz_index)) => {
            let cfx = parse_csv_f64_at(
                &final_record,
                cfx_index,
                "SU2 history CSV CFx field did not parse",
                "su2_history_cfx",
            )?;
            let cfz = parse_csv_f64_at(
                &final_record,
                cfz_index,
                "SU2 history CSV CFz field did not parse",
                "su2_history_cfz",
            )?;
            AeroCoefficients {
                cn: -cfz,
                cd: -cfx,
                cm,
            }
        }
        _ => {
            let cd_index = cd_index.ok_or(AerodbError::MalformedCsv {
                reason: "SU2 history CSV is missing CD",
            })?;
            let cl_index = cl_index.ok_or(AerodbError::MalformedCsv {
                reason: "SU2 history CSV is missing CL",
            })?;
            let wind_cd = parse_csv_f64_at(
                &final_record,
                cd_index,
                "SU2 history CSV CD field did not parse",
                "su2_history_cd",
            )?;
            let cl = parse_csv_f64_at(
                &final_record,
                cl_index,
                "SU2 history CSV CL field did not parse",
                "su2_history_cl",
            )?;
            wind_axis_lift_drag_to_reduced_coefficients(
                wind_cd,
                cl,
                cm,
                node,
                "su2_history_beta_requires_body_force_columns",
                "su2_history_coefficients",
            )?
        }
    };
    if coeffs.cn.is_finite() && coeffs.cd.is_finite() && coeffs.cm.is_finite() {
        Ok(coeffs)
    } else {
        Err(AerodbError::InvalidInput {
            field: "su2_history_coefficients",
        })
    }
}

fn su2_history_column(headers: &csv::StringRecord, name: &str) -> Option<usize> {
    headers.iter().position(|header| header.trim() == name)
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct OpenFoamForceCoeffColumns {
    cd: usize,
    cl: usize,
    cm_pitch: usize,
}

fn openfoam_force_coeffs_from_text(
    text: &str,
    node: RunNode,
) -> Result<AeroCoefficients, AerodbError> {
    let mut columns = None;
    let mut final_coeffs = None;
    for raw_line in text.lines() {
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let tokens = comment.split_whitespace().collect::<Vec<_>>();
            if let Some(header_columns) = openfoam_force_coeff_header_columns(&tokens) {
                columns = Some(header_columns);
            }
            continue;
        }
        let columns = columns.ok_or(AerodbError::MalformedCsv {
            reason: "OpenFOAM forceCoeffs data appeared before the column header",
        })?;
        let tokens = line.split_whitespace().collect::<Vec<_>>();
        let max_index = columns.cd.max(columns.cl).max(columns.cm_pitch);
        if tokens.len() <= max_index {
            return Err(AerodbError::MalformedCsv {
                reason: "OpenFOAM forceCoeffs row has too few fields",
            });
        }
        let wind_cd = parse_openfoam_force_coeff_f64(
            tokens[columns.cd],
            "OpenFOAM forceCoeffs Cd field did not parse",
            "openfoam_force_coeffs_cd",
        )?;
        let cl = parse_openfoam_force_coeff_f64(
            tokens[columns.cl],
            "OpenFOAM forceCoeffs Cl field did not parse",
            "openfoam_force_coeffs_cl",
        )?;
        let cm = parse_openfoam_force_coeff_f64(
            tokens[columns.cm_pitch],
            "OpenFOAM forceCoeffs CmPitch field did not parse",
            "openfoam_force_coeffs_cmpitch",
        )?;
        final_coeffs = Some(wind_axis_lift_drag_to_reduced_coefficients(
            wind_cd,
            cl,
            cm,
            node,
            "openfoam_force_coeffs_beta_requires_zero_sideslip",
            "openfoam_force_coeffs_coefficients",
        )?);
    }
    columns.ok_or(AerodbError::MalformedCsv {
        reason: "OpenFOAM forceCoeffs header is missing Cd, Cl, or CmPitch",
    })?;
    final_coeffs.ok_or(AerodbError::MalformedCsv {
        reason: "OpenFOAM forceCoeffs has no data rows",
    })
}

fn openfoam_force_coeff_header_columns(tokens: &[&str]) -> Option<OpenFoamForceCoeffColumns> {
    let cd = openfoam_force_coeff_column(tokens, "Cd")?;
    let cl = openfoam_force_coeff_column(tokens, "Cl")?;
    let cm_pitch = openfoam_force_coeff_column(tokens, "CmPitch")?;
    Some(OpenFoamForceCoeffColumns { cd, cl, cm_pitch })
}

fn openfoam_force_coeff_column(tokens: &[&str], name: &str) -> Option<usize> {
    tokens.iter().position(|token| *token == name)
}

fn parse_openfoam_force_coeff_f64(
    value: &str,
    malformed_reason: &'static str,
    invalid_field: &'static str,
) -> Result<f64, AerodbError> {
    let value = value
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: malformed_reason,
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: invalid_field,
        })
    }
}

fn wind_axis_lift_drag_to_reduced_coefficients(
    wind_cd: f64,
    cl: f64,
    cm: f64,
    node: RunNode,
    beta_field: &'static str,
    coefficient_field: &'static str,
) -> Result<AeroCoefficients, AerodbError> {
    if node.beta_deg.to_bits() != 0.0_f64.to_bits() {
        return Err(AerodbError::InvalidInput { field: beta_field });
    }
    let alpha = node.alpha_deg.to_radians();
    let coeffs = AeroCoefficients {
        cn: wind_cd * alpha.sin() + cl * alpha.cos(),
        cd: wind_cd * alpha.cos() - cl * alpha.sin(),
        cm,
    };
    if coeffs.cn.is_finite() && coeffs.cd.is_finite() && coeffs.cm.is_finite() {
        Ok(coeffs)
    } else {
        Err(AerodbError::InvalidInput {
            field: coefficient_field,
        })
    }
}

fn parse_csv_f64_at(
    record: &csv::StringRecord,
    index: usize,
    malformed_reason: &'static str,
    invalid_field: &'static str,
) -> Result<f64, AerodbError> {
    let value = record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "CSV row is missing a required field",
        })?
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: malformed_reason,
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: invalid_field,
        })
    }
}

fn correlated_mach_residuals(
    mach_grid: &[f64],
    settings: CorrelatedDeckDrawSettings,
    coefficient_offset: u32,
) -> Result<Vec<f64>, AerodbError> {
    let dimension_id = settings
        .dimension_id_base
        .checked_add(coefficient_offset)
        .ok_or(AerodbError::InvalidInput {
            field: "dimension_id_base",
        })?;
    let lower = mach_correlation_cholesky(mach_grid, settings.mach_correlation_length)?;
    let mut rng = DeterministicRng::for_mc_sample(
        settings.campaign_seed,
        settings.sample_index,
        dimension_id,
    );
    let z = (0..mach_grid.len())
        .map(|_| standard_normal(&mut rng))
        .collect::<Vec<_>>();
    let mut residuals = vec![0.0; mach_grid.len()];
    for i in 0..mach_grid.len() {
        let mut sum = 0.0;
        for j in 0..=i {
            sum += lower[i * mach_grid.len() + j] * z[j];
        }
        if !sum.is_finite() {
            return Err(AerodbError::InvalidInput {
                field: "mach_correlated_residual",
            });
        }
        residuals[i] = sum;
    }
    Ok(residuals)
}

fn correlated_grid_residuals(
    mach_grid: &[f64],
    alpha_grid_deg: &[f64],
    beta_grid_deg: &[f64],
    settings: CorrelatedDeckDrawSettings,
    coefficient_offset: u32,
) -> Result<Vec<f64>, AerodbError> {
    let node_count = mach_grid
        .len()
        .checked_mul(alpha_grid_deg.len())
        .and_then(|value| value.checked_mul(beta_grid_deg.len()))
        .ok_or(AerodbError::InvalidInput {
            field: "correlated_residual_count",
        })?;
    if settings.alpha_correlation_length_deg.is_none()
        && settings.beta_correlation_length_deg.is_none()
    {
        let mach_residuals = correlated_mach_residuals(mach_grid, settings, coefficient_offset)?;
        let plane_count = alpha_grid_deg
            .len()
            .checked_mul(beta_grid_deg.len())
            .ok_or(AerodbError::InvalidInput {
                field: "correlated_residual_count",
            })?;
        let mut residuals = Vec::with_capacity(node_count);
        for mach_residual in mach_residuals {
            residuals.extend(std::iter::repeat_n(mach_residual, plane_count));
        }
        return Ok(residuals);
    }

    let dimension_id = settings
        .dimension_id_base
        .checked_add(coefficient_offset)
        .ok_or(AerodbError::InvalidInput {
            field: "dimension_id_base",
        })?;
    let mach_lower = mach_correlation_cholesky(mach_grid, settings.mach_correlation_length)?;
    let alpha_lower = settings
        .alpha_correlation_length_deg
        .map(|length| {
            axis_correlation_cholesky(alpha_grid_deg, length, "alpha_correlation_cholesky")
        })
        .transpose()?;
    let beta_lower = settings
        .beta_correlation_length_deg
        .map(|length| axis_correlation_cholesky(beta_grid_deg, length, "beta_correlation_cholesky"))
        .transpose()?;
    let alpha_basis_count = alpha_lower.as_ref().map_or(1, |_| alpha_grid_deg.len());
    let beta_basis_count = beta_lower.as_ref().map_or(1, |_| beta_grid_deg.len());
    let basis_count = mach_grid
        .len()
        .checked_mul(alpha_basis_count)
        .and_then(|value| value.checked_mul(beta_basis_count))
        .ok_or(AerodbError::InvalidInput {
            field: "correlated_residual_count",
        })?;
    let mut rng = DeterministicRng::for_mc_sample(
        settings.campaign_seed,
        settings.sample_index,
        dimension_id,
    );
    let z = (0..basis_count)
        .map(|_| standard_normal(&mut rng))
        .collect::<Vec<_>>();
    let mut residuals = Vec::with_capacity(node_count);
    for mach_index in 0..mach_grid.len() {
        for alpha_index in 0..alpha_grid_deg.len() {
            for beta_index in 0..beta_grid_deg.len() {
                let mut sum = 0.0;
                for mach_basis_index in 0..=mach_index {
                    let mach_coeff = mach_lower[mach_index * mach_grid.len() + mach_basis_index];
                    let alpha_basis_limit = if alpha_lower.is_some() {
                        alpha_index + 1
                    } else {
                        1
                    };
                    for alpha_basis_index in 0..alpha_basis_limit {
                        let alpha_coeff = alpha_lower.as_ref().map_or(1.0, |lower| {
                            lower[alpha_index * alpha_grid_deg.len() + alpha_basis_index]
                        });
                        let beta_basis_limit = if beta_lower.is_some() {
                            beta_index + 1
                        } else {
                            1
                        };
                        for beta_basis_index in 0..beta_basis_limit {
                            let beta_coeff = beta_lower.as_ref().map_or(1.0, |lower| {
                                lower[beta_index * beta_grid_deg.len() + beta_basis_index]
                            });
                            let z_index = (mach_basis_index * alpha_basis_count
                                + alpha_basis_index)
                                * beta_basis_count
                                + beta_basis_index;
                            sum += mach_coeff * alpha_coeff * beta_coeff * z[z_index];
                        }
                    }
                }
                if !sum.is_finite() {
                    return Err(AerodbError::InvalidInput {
                        field: "grid_correlated_residual",
                    });
                }
                residuals.push(sum);
            }
        }
    }
    Ok(residuals)
}

fn validate_dimension_capacity(
    dimension_id_base: u32,
    highest_offset: u32,
) -> Result<(), AerodbError> {
    dimension_id_base
        .checked_add(highest_offset)
        .map(|_| ())
        .ok_or(AerodbError::InvalidInput {
            field: "dimension_id_base",
        })
}

#[allow(clippy::needless_range_loop)] // Three coefficient vectors are mixed by fixed-index matrix math.
fn correlated_coefficient_residuals(
    mach_grid: &[f64],
    alpha_grid_deg: &[f64],
    beta_grid_deg: &[f64],
    settings: CorrelatedDeckDrawSettings,
    deck: &AeroDeck,
) -> Result<[Vec<f64>; 3], AerodbError> {
    let mut residuals = [
        correlated_grid_residuals(mach_grid, alpha_grid_deg, beta_grid_deg, settings, 0)?,
        correlated_grid_residuals(mach_grid, alpha_grid_deg, beta_grid_deg, settings, 1)?,
        correlated_grid_residuals(mach_grid, alpha_grid_deg, beta_grid_deg, settings, 2)?,
    ];
    let Some(coefficient_correlation) = deck
        .uncertainty_correlation()
        .and_then(|correlation| correlation.coefficient_correlation())
    else {
        return Ok(residuals);
    };
    let lower = coefficient_correlation_cholesky::<3>(
        coefficient_correlation,
        "coefficient_correlation_cholesky",
    )?;
    for node_index in 0..residuals[0].len() {
        let z = [
            residuals[0][node_index],
            residuals[1][node_index],
            residuals[2][node_index],
        ];
        for coefficient_index in 0..3 {
            let mut value = 0.0;
            for (basis_index, basis_value) in z.iter().enumerate().take(coefficient_index + 1) {
                value += lower[coefficient_index][basis_index] * basis_value;
            }
            if !value.is_finite() {
                return Err(AerodbError::InvalidInput {
                    field: "coefficient_correlated_residual",
                });
            }
            residuals[coefficient_index][node_index] = value;
        }
    }
    Ok(residuals)
}

#[allow(clippy::needless_range_loop)] // Six coefficient vectors are mixed by fixed-index matrix math.
fn correlated_six_residuals(
    mach_grid: &[f64],
    alpha_grid_deg: &[f64],
    beta_grid_deg: &[f64],
    settings: CorrelatedDeckDrawSettings,
    coefficient_offset_base: u32,
    deck: &AeroDeck,
) -> Result<[Vec<f64>; 6], AerodbError> {
    let mut residuals = [
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base,
        )?,
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base + 1,
        )?,
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base + 2,
        )?,
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base + 3,
        )?,
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base + 4,
        )?,
        correlated_grid_residuals(
            mach_grid,
            alpha_grid_deg,
            beta_grid_deg,
            settings,
            coefficient_offset_base + 5,
        )?,
    ];
    let Some(coefficient_correlation) = deck
        .uncertainty_correlation()
        .and_then(|correlation| correlation.six_coefficient_correlation())
    else {
        return Ok(residuals);
    };
    let lower = coefficient_correlation_cholesky::<6>(
        coefficient_correlation,
        "six_coefficient_correlation_cholesky",
    )?;
    for node_index in 0..residuals[0].len() {
        let z = [
            residuals[0][node_index],
            residuals[1][node_index],
            residuals[2][node_index],
            residuals[3][node_index],
            residuals[4][node_index],
            residuals[5][node_index],
        ];
        for coefficient_index in 0..6 {
            let mut value = 0.0;
            for (basis_index, basis_value) in z.iter().enumerate().take(coefficient_index + 1) {
                value += lower[coefficient_index][basis_index] * basis_value;
            }
            if !value.is_finite() {
                return Err(AerodbError::InvalidInput {
                    field: "six_coefficient_correlated_residual",
                });
            }
            residuals[coefficient_index][node_index] = value;
        }
    }
    Ok(residuals)
}

fn mach_correlation_cholesky(
    mach_grid: &[f64],
    mach_correlation_length: f64,
) -> Result<Vec<f64>, AerodbError> {
    axis_correlation_cholesky(
        mach_grid,
        mach_correlation_length,
        "mach_correlation_cholesky",
    )
}

fn axis_correlation_cholesky(
    axis: &[f64],
    correlation_length: f64,
    field: &'static str,
) -> Result<Vec<f64>, AerodbError> {
    let n = axis.len();
    let mut lower = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = if i == j {
                1.0
            } else {
                (-(axis[i] - axis[j]).abs() / correlation_length).exp()
            };
            for k in 0..j {
                sum -= lower[i * n + k] * lower[j * n + k];
            }
            if i == j {
                if !sum.is_finite() || sum < -1.0e-12 {
                    return Err(AerodbError::InvalidInput { field });
                }
                lower[i * n + j] = sum.max(0.0).sqrt();
            } else {
                let diagonal = lower[j * n + j];
                if !(diagonal.is_finite() && diagonal > 0.0) {
                    return Err(AerodbError::InvalidInput { field });
                }
                lower[i * n + j] = sum / diagonal;
            }
        }
    }
    Ok(lower)
}

#[allow(clippy::needless_range_loop)] // Fixed-size Cholesky/math reads clearest with explicit indices.
fn coefficient_correlation_cholesky<const N: usize>(
    matrix: [[f64; N]; N],
    field: &'static str,
) -> Result<[[f64; N]; N], AerodbError> {
    let mut lower = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let mut sum = matrix[i][j];
            for k in 0..j {
                sum -= lower[i][k] * lower[j][k];
            }
            if i == j {
                if !(sum.is_finite() && sum > 0.0) {
                    return Err(AerodbError::InvalidInput { field });
                }
                lower[i][j] = sum.sqrt();
            } else {
                let diagonal = lower[j][j];
                if !(diagonal.is_finite() && diagonal > 0.0) {
                    return Err(AerodbError::InvalidInput { field });
                }
                lower[i][j] = sum / diagonal;
            }
        }
    }
    Ok(lower)
}

fn standard_normal(rng: &mut DeterministicRng) -> f64 {
    let u1 = uniform01(rng).max(f64::MIN_POSITIVE);
    let u2 = uniform01(rng);
    (-2.0 * u1.ln()).sqrt() * (TWO_PI * u2).cos()
}

fn uniform01(rng: &mut DeterministicRng) -> f64 {
    const SCALE: f64 = 1.0 / ((1u64 << 53) as f64);
    ((rng.next_u64() >> 11) as f64) * SCALE
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunMatrixToml {
    openbmp: RunMatrixMarker,
    seed: u64,
    grid: RunMatrixGridToml,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunMatrixMarker {
    aerodb_run_matrix: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunMatrixGridToml {
    mach: Vec<f64>,
    alpha_deg: Vec<f64>,
    beta_deg: Vec<f64>,
    reynolds_length: Vec<f64>,
    knudsen: Vec<f64>,
    dynamic_pressure_pa: Vec<f64>,
}

impl RunMatrixGridToml {
    fn into_axes(self) -> RunMatrixAxes {
        RunMatrixAxes {
            mach: self.mach,
            alpha_deg: self.alpha_deg,
            beta_deg: self.beta_deg,
            reynolds_length: self.reynolds_length,
            knudsen: self.knudsen,
            dynamic_pressure_pa: self.dynamic_pressure_pa,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCaseManifestToml {
    openbmp: SolverCaseManifestMarker,
    cases: Vec<SolverCaseArtifactToml>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCaseManifestMarker {
    aerodb_solver_case_manifest: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCommandPlanToml {
    openbmp: SolverCommandPlanMarker,
    cases: Vec<SolverCommandCaseToml>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCommandPlanMarker {
    aerodb_solver_command_plan: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCaseArtifactToml {
    node_index: usize,
    mach: f64,
    alpha_deg: f64,
    beta_deg: f64,
    reynolds_length: f64,
    knudsen: f64,
    dynamic_pressure_pa: f64,
    solver: String,
    case_id: String,
    output_kind: SolverCaseOutputKind,
    output_file: String,
    output_sha256: String,
    provenance: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverCommandCaseToml {
    node_index: usize,
    mach: f64,
    alpha_deg: f64,
    beta_deg: f64,
    reynolds_length: f64,
    knudsen: f64,
    dynamic_pressure_pa: f64,
    solver: String,
    case_id: String,
    program: String,
    #[serde(default)]
    args: Vec<String>,
    #[serde(default)]
    env: BTreeMap<String, String>,
    working_dir: String,
    timeout_s: u64,
    output_kind: SolverCaseOutputKind,
    output_file: String,
    output_sha256: String,
    provenance: String,
}

impl SolverCaseArtifactToml {
    fn into_artifact(
        self,
        expected_index: usize,
        node: RunNode,
    ) -> Result<SolverCaseArtifact, AerodbError> {
        if self.node_index != expected_index {
            return Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_node_index",
            });
        }
        validate_solver_case_condition(self.mach, node.mach, "solver_case_manifest_mach")?;
        validate_solver_case_condition(
            self.alpha_deg,
            node.alpha_deg,
            "solver_case_manifest_alpha_deg",
        )?;
        validate_solver_case_condition(
            self.beta_deg,
            node.beta_deg,
            "solver_case_manifest_beta_deg",
        )?;
        validate_solver_case_condition(
            self.reynolds_length,
            node.reynolds_length,
            "solver_case_manifest_reynolds_length",
        )?;
        validate_solver_case_condition(self.knudsen, node.knudsen, "solver_case_manifest_knudsen")?;
        validate_solver_case_condition(
            self.dynamic_pressure_pa,
            node.dynamic_pressure_pa,
            "solver_case_manifest_dynamic_pressure_pa",
        )?;
        validate_nonblank_solver_case_field(&self.solver, "solver_case_manifest_solver")?;
        validate_nonblank_solver_case_field(&self.case_id, "solver_case_manifest_case_id")?;
        validate_nonblank_solver_case_field(&self.output_file, "solver_case_manifest_output_file")?;
        validate_sha256_hex(&self.output_sha256, "solver_case_manifest_output_sha256")?;
        validate_nonblank_solver_case_field(&self.provenance, "solver_case_manifest_provenance")?;
        Ok(SolverCaseArtifact {
            node_index: self.node_index,
            node,
            solver: self.solver,
            case_id: self.case_id,
            output_kind: self.output_kind,
            output_file: self.output_file,
            output_sha256: self.output_sha256,
            provenance: self.provenance,
        })
    }
}

impl SolverCommandCaseToml {
    fn into_case(
        self,
        expected_index: usize,
        node: RunNode,
    ) -> Result<SolverCommandCase, AerodbError> {
        if self.node_index != expected_index {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_plan_node_index",
            });
        }
        validate_solver_case_condition(self.mach, node.mach, "solver_command_plan_mach")?;
        validate_solver_case_condition(
            self.alpha_deg,
            node.alpha_deg,
            "solver_command_plan_alpha_deg",
        )?;
        validate_solver_case_condition(
            self.beta_deg,
            node.beta_deg,
            "solver_command_plan_beta_deg",
        )?;
        validate_solver_case_condition(
            self.reynolds_length,
            node.reynolds_length,
            "solver_command_plan_reynolds_length",
        )?;
        validate_solver_case_condition(self.knudsen, node.knudsen, "solver_command_plan_knudsen")?;
        validate_solver_case_condition(
            self.dynamic_pressure_pa,
            node.dynamic_pressure_pa,
            "solver_command_plan_dynamic_pressure_pa",
        )?;
        validate_nonblank_solver_case_field(&self.solver, "solver_command_plan_solver")?;
        validate_nonblank_solver_case_field(&self.case_id, "solver_command_plan_case_id")?;
        validate_solver_command_text(&self.program, "solver_command_plan_program")?;
        for arg in &self.args {
            validate_solver_command_text(arg, "solver_command_plan_arg")?;
        }
        for (key, value) in &self.env {
            validate_solver_env_key(key)?;
            validate_solver_command_text(value, "solver_command_plan_env_value")?;
        }
        validate_solver_command_text(&self.working_dir, "solver_command_plan_working_dir")?;
        if self.timeout_s == 0 {
            return Err(AerodbError::InvalidInput {
                field: "solver_command_plan_timeout_s",
            });
        }
        validate_solver_command_text(&self.output_file, "solver_command_plan_output_file")?;
        validate_sha256_hex(&self.output_sha256, "solver_command_plan_output_sha256")?;
        validate_nonblank_solver_case_field(&self.provenance, "solver_command_plan_provenance")?;
        Ok(SolverCommandCase {
            node_index: self.node_index,
            node,
            solver: self.solver,
            case_id: self.case_id,
            program: self.program,
            args: self.args,
            env: self.env,
            working_dir: self.working_dir,
            timeout_s: self.timeout_s,
            output_kind: self.output_kind,
            output_file: self.output_file,
            output_sha256: self.output_sha256,
            provenance: self.provenance,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GciSigmaToml {
    openbmp: GciSigmaMarker,
    provenance: String,
    refinement_ratio: f64,
    #[serde(default = "default_gci_safety_factor")]
    safety_factor: f64,
    rows: Vec<GciSigmaRowToml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GciSigmaMarker {
    aerodb_gci_sigma: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SixGciSigmaToml {
    openbmp: SixGciSigmaMarker,
    provenance: String,
    refinement_ratio: f64,
    #[serde(default = "default_gci_safety_factor")]
    safety_factor: f64,
    rows: Vec<SixGciSigmaRowToml>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SixGciSigmaMarker {
    aerodb_six_gci_sigma: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GciSigmaRowToml {
    cn: GciCoefficientToml,
    cd: GciCoefficientToml,
    cm: GciCoefficientToml,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SixGciSigmaRowToml {
    cx: GciCoefficientToml,
    cy: GciCoefficientToml,
    cz: GciCoefficientToml,
    cl: GciCoefficientToml,
    cm: GciCoefficientToml,
    cn: GciCoefficientToml,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GciCoefficientToml {
    fine: f64,
    medium: f64,
    coarse: f64,
    #[serde(default)]
    wind_tunnel: f64,
    #[serde(default)]
    model_form: f64,
    #[serde(default)]
    extrapolation: f64,
}

impl GciSigmaRowToml {
    fn into_budget(
        self,
        refinement_ratio: f64,
        safety_factor: f64,
    ) -> Result<ReducedCoefficientSigmaBudget, AerodbError> {
        Ok(ReducedCoefficientSigmaBudget {
            cn: self.cn.into_budget(refinement_ratio, safety_factor)?,
            cd: self.cd.into_budget(refinement_ratio, safety_factor)?,
            cm: self.cm.into_budget(refinement_ratio, safety_factor)?,
        })
    }
}

impl SixGciSigmaRowToml {
    fn into_budget(
        self,
        refinement_ratio: f64,
        safety_factor: f64,
    ) -> Result<SixCoefficientSigmaBudget, AerodbError> {
        Ok(SixCoefficientSigmaBudget {
            cx: self.cx.into_budget(refinement_ratio, safety_factor)?,
            cy: self.cy.into_budget(refinement_ratio, safety_factor)?,
            cz: self.cz.into_budget(refinement_ratio, safety_factor)?,
            cl: self.cl.into_budget(refinement_ratio, safety_factor)?,
            cm: self.cm.into_budget(refinement_ratio, safety_factor)?,
            cn: self.cn.into_budget(refinement_ratio, safety_factor)?,
        })
    }
}

impl GciCoefficientToml {
    fn into_budget(
        self,
        refinement_ratio: f64,
        safety_factor: f64,
    ) -> Result<CoefficientSigmaBudget, AerodbError> {
        let cfd = GridConvergence::from_three_grids_with_safety_factor(
            self.fine,
            self.medium,
            self.coarse,
            refinement_ratio,
            safety_factor,
        )?
        .absolute_gci_fine;
        CoefficientSigmaBudget::new(self.wind_tunnel, cfd, self.model_form, self.extrapolation)
    }
}

const fn default_gci_safety_factor() -> f64 {
    GridConvergence::THREE_GRID_SAFETY_FACTOR
}

fn validate_gci_sigma_csv_header(headers: &csv::StringRecord) -> Result<(), AerodbError> {
    if headers.len() != GCI_SIGMA_CSV_HEADER.len()
        || headers
            .iter()
            .zip(GCI_SIGMA_CSV_HEADER)
            .any(|(actual, expected)| actual != expected)
    {
        return Err(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV header does not match schema",
        });
    }
    Ok(())
}

fn validate_six_gci_sigma_csv_header(headers: &csv::StringRecord) -> Result<(), AerodbError> {
    if headers.len() != SIX_GCI_SIGMA_CSV_HEADER.len()
        || headers
            .iter()
            .zip(SIX_GCI_SIGMA_CSV_HEADER)
            .any(|(actual, expected)| actual != expected)
    {
        return Err(AerodbError::MalformedCsv {
            reason: "aero six GCI sigma CSV header does not match schema",
        });
    }
    Ok(())
}

fn validate_six_coefficients_finite(
    coefficients: AeroSixCoefficients,
    field: &'static str,
) -> Result<(), AerodbError> {
    if [
        coefficients.cx,
        coefficients.cy,
        coefficients.cz,
        coefficients.cl,
        coefficients.cm,
        coefficients.cn,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        Ok(())
    } else {
        Err(AerodbError::InvalidInput { field })
    }
}

fn validate_six_coefficient_tolerances(tolerance: AeroSixCoefficients) -> Result<(), AerodbError> {
    if [
        tolerance.cx,
        tolerance.cy,
        tolerance.cz,
        tolerance.cl,
        tolerance.cm,
        tolerance.cn,
    ]
    .into_iter()
    .all(|value| value.is_finite() && value >= 0.0)
    {
        Ok(())
    } else {
        Err(AerodbError::InvalidInput {
            field: "six_coefficient_validation_tolerance",
        })
    }
}

fn six_coefficient_abs_error(
    actual: AeroSixCoefficients,
    expected: AeroSixCoefficients,
) -> AeroSixCoefficients {
    AeroSixCoefficients {
        cx: (actual.cx - expected.cx).abs(),
        cy: (actual.cy - expected.cy).abs(),
        cz: (actual.cz - expected.cz).abs(),
        cl: (actual.cl - expected.cl).abs(),
        cm: (actual.cm - expected.cm).abs(),
        cn: (actual.cn - expected.cn).abs(),
    }
}

fn six_coefficient_errors_within_tolerance(
    error: AeroSixCoefficients,
    tolerance: AeroSixCoefficients,
) -> bool {
    error.cx <= tolerance.cx
        && error.cy <= tolerance.cy
        && error.cz <= tolerance.cz
        && error.cl <= tolerance.cl
        && error.cm <= tolerance.cm
        && error.cn <= tolerance.cn
}

fn gci_sigma_budget_from_csv_record(
    record: &csv::StringRecord,
    expected_index: usize,
    refinement_ratio: f64,
    safety_factor: f64,
) -> Result<ReducedCoefficientSigmaBudget, AerodbError> {
    if record.len() != GCI_SIGMA_CSV_HEADER.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV row has the wrong field count",
        });
    }
    if record.get(0) != Some(GCI_SIGMA_CSV_SCHEMA_V1) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV row has an unsupported schema",
        });
    }
    let node_index = parse_gci_sigma_csv_usize(record, 1)?;
    if node_index != expected_index {
        return Err(AerodbError::InvalidInput {
            field: "gci_sigma_csv_node_index",
        });
    }
    if record.get(20).is_none_or(|value| value.trim().is_empty()) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV provenance must not be blank",
        });
    }
    Ok(ReducedCoefficientSigmaBudget {
        cn: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 2, 11)?,
        cd: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 5, 14)?,
        cm: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 8, 17)?,
    })
}

fn six_gci_sigma_budget_from_csv_record(
    record: &csv::StringRecord,
    expected_index: usize,
    refinement_ratio: f64,
    safety_factor: f64,
) -> Result<SixCoefficientSigmaBudget, AerodbError> {
    if record.len() != SIX_GCI_SIGMA_CSV_HEADER.len() {
        return Err(AerodbError::MalformedCsv {
            reason: "aero six GCI sigma CSV row has the wrong field count",
        });
    }
    if record.get(0) != Some(SIX_GCI_SIGMA_CSV_SCHEMA_V1) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero six GCI sigma CSV row has an unsupported schema",
        });
    }
    let node_index = parse_gci_sigma_csv_usize(record, 1)?;
    if node_index != expected_index {
        return Err(AerodbError::InvalidInput {
            field: "six_gci_sigma_csv_node_index",
        });
    }
    if record.get(38).is_none_or(|value| value.trim().is_empty()) {
        return Err(AerodbError::MalformedCsv {
            reason: "aero six GCI sigma CSV provenance must not be blank",
        });
    }
    Ok(SixCoefficientSigmaBudget {
        cx: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 2, 20)?,
        cy: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 5, 23)?,
        cz: gci_coefficient_budget_from_csv_record(record, refinement_ratio, safety_factor, 8, 26)?,
        cl: gci_coefficient_budget_from_csv_record(
            record,
            refinement_ratio,
            safety_factor,
            11,
            29,
        )?,
        cm: gci_coefficient_budget_from_csv_record(
            record,
            refinement_ratio,
            safety_factor,
            14,
            32,
        )?,
        cn: gci_coefficient_budget_from_csv_record(
            record,
            refinement_ratio,
            safety_factor,
            17,
            35,
        )?,
    })
}

fn gci_coefficient_budget_from_csv_record(
    record: &csv::StringRecord,
    refinement_ratio: f64,
    safety_factor: f64,
    triplet_start: usize,
    source_start: usize,
) -> Result<CoefficientSigmaBudget, AerodbError> {
    let cfd = GridConvergence::from_three_grids_with_safety_factor(
        parse_gci_sigma_csv_f64(record, triplet_start)?,
        parse_gci_sigma_csv_f64(record, triplet_start + 1)?,
        parse_gci_sigma_csv_f64(record, triplet_start + 2)?,
        refinement_ratio,
        safety_factor,
    )?
    .absolute_gci_fine;
    CoefficientSigmaBudget::new(
        parse_gci_sigma_csv_f64(record, source_start)?,
        cfd,
        parse_gci_sigma_csv_f64(record, source_start + 1)?,
        parse_gci_sigma_csv_f64(record, source_start + 2)?,
    )
}

fn parse_gci_sigma_csv_usize(
    record: &csv::StringRecord,
    index: usize,
) -> Result<usize, AerodbError> {
    record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV row is missing a required field",
        })?
        .parse::<usize>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV integer field did not parse",
        })
}

fn parse_gci_sigma_csv_f64(record: &csv::StringRecord, index: usize) -> Result<f64, AerodbError> {
    let value = record
        .get(index)
        .ok_or(AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV row is missing a required field",
        })?
        .parse::<f64>()
        .map_err(|_| AerodbError::MalformedCsv {
            reason: "aero GCI sigma CSV float field did not parse",
        })?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AerodbError::InvalidInput {
            field: "gci_sigma_csv_float",
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;
    use openbmp_aero::AeroForceMomentBody;

    fn basic_axes() -> RunMatrixAxes {
        RunMatrixAxes {
            mach: vec![2.0, 4.0],
            alpha_deg: vec![-5.0, 5.0],
            beta_deg: vec![0.0, 3.0],
            reynolds_length: vec![1.0e6],
            knudsen: vec![0.0],
            dynamic_pressure_pa: vec![100.0],
        }
    }

    fn zero_beta_axes() -> RunMatrixAxes {
        let mut axes = basic_axes();
        axes.beta_deg = vec![0.0];
        axes
    }

    fn single_node_axes() -> RunMatrixAxes {
        RunMatrixAxes {
            mach: vec![2.0],
            alpha_deg: vec![0.0],
            beta_deg: vec![0.0],
            reynolds_length: vec![1.0e6],
            knudsen: vec![0.0],
            dynamic_pressure_pa: vec![100.0],
        }
    }

    fn temp_aerodb_dir(name: &str) -> std::path::PathBuf {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let index = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let root = std::env::temp_dir().join(format!(
            "openbmp_aerodb_{name}_{}_{}",
            std::process::id(),
            index
        ));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        root
    }

    #[derive(Copy, Clone, Debug)]
    struct LinearMethod {
        reference_area_m2: f64,
        reference_length_m: f64,
    }

    #[derive(Copy, Clone, Debug)]
    struct SixAxisMethod {
        reference_area_m2: f64,
        reference_length_m: f64,
    }

    impl AeroMethod for LinearMethod {
        fn aero_force_moment_body(
            &self,
            ctx: &AeroContext,
        ) -> Result<AeroForceMomentBody, AeroError> {
            let cn = 0.01 * ctx.alpha_deg + 0.001 * ctx.beta_deg;
            let cd = 0.1 + 0.01 * ctx.mach;
            let cm = -0.02 * ctx.alpha_deg;
            let force_ref = ctx.dynamic_pressure_pa * self.reference_area_m2;
            Ok(AeroForceMomentBody {
                force_n_body: Vector3::new(-cd * force_ref, 0.0, -cn * force_ref),
                moment_n_m_body: Vector3::new(0.0, cm * force_ref * self.reference_length_m, 0.0),
            })
        }
    }

    impl AeroMethod for SixAxisMethod {
        fn aero_force_moment_body(
            &self,
            ctx: &AeroContext,
        ) -> Result<AeroForceMomentBody, AeroError> {
            let force_ref = ctx.dynamic_pressure_pa * self.reference_area_m2;
            let moment_ref = force_ref * self.reference_length_m;
            Ok(AeroForceMomentBody {
                force_n_body: Vector3::new(
                    (0.10 + 0.01 * ctx.mach) * force_ref,
                    (-0.20 + 0.001 * ctx.beta_deg) * force_ref,
                    (0.30 + 0.002 * ctx.alpha_deg) * force_ref,
                ),
                moment_n_m_body: Vector3::new(
                    (-0.010 + 0.001 * ctx.alpha_deg) * moment_ref,
                    (0.020 + 0.001 * ctx.mach) * moment_ref,
                    (-0.030 + 0.001 * ctx.beta_deg) * moment_ref,
                ),
            })
        }
    }

    fn bake_basic_deck() -> (RunMatrix, AeroDeck) {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = LinearMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder.bake_panel_deck(&matrix, &method).unwrap();
        (matrix, deck)
    }

    fn bake_basic_six_deck() -> (RunMatrix, AeroDeck) {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = SixAxisMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder
            .bake_panel_six_coefficient_deck(&matrix, &method)
            .unwrap();
        (matrix, deck)
    }

    fn expected_six_axis_coefficients(
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
    ) -> AeroSixCoefficients {
        AeroSixCoefficients {
            cx: 0.10 + 0.01 * mach,
            cy: -0.20 + 0.001 * beta_deg,
            cz: 0.30 + 0.002 * alpha_deg,
            cl: -0.010 + 0.001 * alpha_deg,
            cm: 0.020 + 0.001 * mach,
            cn: -0.030 + 0.001 * beta_deg,
        }
    }

    fn uniform_six_tolerance(value: f64) -> AeroSixCoefficients {
        AeroSixCoefficients {
            cx: value,
            cy: value,
            cz: value,
            cl: value,
            cm: value,
            cn: value,
        }
    }

    fn attach_uniform_six_sigma_tables(deck: AeroDeck, node_count: usize, sigma: f64) -> AeroDeck {
        deck.with_six_sigma_tables(
            vec![sigma; node_count],
            vec![sigma; node_count],
            vec![sigma; node_count],
            vec![sigma; node_count],
            vec![sigma; node_count],
            vec![sigma; node_count],
        )
        .unwrap()
    }

    fn flat_plate_mesh() -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(1.0, 1.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap()
    }

    #[test]
    fn run_matrix_orders_nodes_with_beta_fastest_for_singleton_extra_axes() {
        let matrix = RunMatrix::from_axes(basic_axes(), 42).unwrap();
        let triples = matrix
            .nodes()
            .iter()
            .map(|node| (node.mach, node.alpha_deg, node.beta_deg))
            .collect::<Vec<_>>();

        assert_eq!(
            triples,
            vec![
                (2.0, -5.0, 0.0),
                (2.0, -5.0, 3.0),
                (2.0, 5.0, 0.0),
                (2.0, 5.0, 3.0),
                (4.0, -5.0, 0.0),
                (4.0, -5.0, 3.0),
                (4.0, 5.0, 0.0),
                (4.0, 5.0, 3.0),
            ]
        );
        assert_eq!(matrix.seed(), 42);
    }

    #[test]
    fn deck_builder_bakes_panel_sweep_in_deck_row_major_order() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = LinearMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder.bake_panel_deck(&matrix, &method).unwrap();

        let mut expected_cn = Vec::new();
        let mut expected_cd = Vec::new();
        let mut expected_cm = Vec::new();
        for node in matrix.nodes() {
            let force_moment = method.aero_force_moment_body(&node.context()).unwrap();
            let coeffs = coefficients_from_force_moment(&builder, *node, force_moment).unwrap();
            expected_cn.push(coeffs.cn);
            expected_cd.push(coeffs.cd);
            expected_cm.push(coeffs.cm);
        }
        let expected = AeroDeck::new(
            matrix.axes().mach.clone(),
            matrix.axes().alpha_deg.clone(),
            matrix.axes().beta_deg.clone(),
            expected_cn,
            expected_cd,
            expected_cm,
            builder.reference_area_m2,
            builder.reference_length_m,
        )
        .unwrap();
        assert_eq!(deck, expected);

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.053, epsilon = 1e-12);
        assert_relative_eq!(coeffs.cd, 0.14, epsilon = 1e-12);
        assert_relative_eq!(coeffs.cm, -0.1, epsilon = 1e-12);
    }

    #[test]
    fn deck_builder_bakes_panel_sweep_with_six_body_axis_coefficients() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = SixAxisMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder
            .bake_panel_six_coefficient_deck(&matrix, &method)
            .unwrap();

        let deflections = std::collections::BTreeMap::new();
        let six = deck
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let reduced = deck.lookup(4.0, 5.0, 3.0, &deflections).unwrap();

        assert!(deck.has_six_coefficient_tables());
        assert_relative_eq!(six.cx, 0.14, epsilon = 1e-12);
        assert_relative_eq!(six.cy, -0.197, epsilon = 1e-12);
        assert_relative_eq!(six.cz, 0.31, epsilon = 1e-12);
        assert_relative_eq!(six.cl, -0.005, epsilon = 1e-12);
        assert_relative_eq!(six.cm, 0.024, epsilon = 1e-12);
        assert_relative_eq!(six.cn, -0.027, epsilon = 1e-12);
        assert_relative_eq!(reduced.cn, -0.31, epsilon = 1e-12);
        assert_relative_eq!(reduced.cd, -0.14, epsilon = 1e-12);
        assert_relative_eq!(reduced.cm, 0.024, epsilon = 1e-12);
    }

    #[test]
    fn six_coefficient_validation_accepts_calibrated_reference_cases() {
        let (_, deck) = bake_basic_six_deck();
        let cases = [
            SixCoefficientValidationCase {
                case_id: "node-case",
                provenance: "synthetic exact-node six coefficient validation",
                mach: 4.0,
                alpha_deg: 5.0,
                beta_deg: 3.0,
                expected: expected_six_axis_coefficients(4.0, 5.0, 3.0),
                tolerance: uniform_six_tolerance(1.0e-12),
            },
            SixCoefficientValidationCase {
                case_id: "interior-case",
                provenance: "synthetic interpolated six coefficient validation",
                mach: 3.0,
                alpha_deg: 0.0,
                beta_deg: 1.5,
                expected: expected_six_axis_coefficients(3.0, 0.0, 1.5),
                tolerance: uniform_six_tolerance(1.0e-12),
            },
        ];

        let report = validate_six_coefficient_deck(&deck, &cases).unwrap();

        assert_eq!(report.cases.len(), 2);
        assert_eq!(report.cases[0].case_id, "node-case");
        assert_relative_eq!(report.cases[0].actual.cx, 0.14, epsilon = 1e-12);
        assert_relative_eq!(report.cases[1].actual.cx, 0.13, epsilon = 1e-12);
        assert_relative_eq!(report.cases[1].actual.cy, -0.1985, epsilon = 1e-12);
        assert_relative_eq!(report.cases[1].absolute_error.cx, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn six_coefficient_validation_rejects_missing_tables_bad_inputs_and_tolerance() {
        let (_, six_deck) = bake_basic_six_deck();
        let (_, reduced_deck) = bake_basic_deck();
        let valid_case = SixCoefficientValidationCase {
            case_id: "node-case",
            provenance: "synthetic exact-node six coefficient validation",
            mach: 4.0,
            alpha_deg: 5.0,
            beta_deg: 3.0,
            expected: expected_six_axis_coefficients(4.0, 5.0, 3.0),
            tolerance: uniform_six_tolerance(1.0e-12),
        };

        assert!(matches!(
            validate_six_coefficient_deck(&six_deck, &[]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_cases"
            })
        ));
        assert!(matches!(
            validate_six_coefficient_deck(&reduced_deck, &[valid_case]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_tables"
            })
        ));

        let mut mismatch = valid_case;
        mismatch.expected.cx += 0.01;
        assert!(matches!(
            validate_six_coefficient_deck(&six_deck, &[mismatch]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_tolerance"
            })
        ));

        let mut bad_expected = valid_case;
        bad_expected.expected.cn = f64::NAN;
        assert!(matches!(
            validate_six_coefficient_deck(&six_deck, &[bad_expected]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_expected"
            })
        ));

        let mut bad_tolerance = valid_case;
        bad_tolerance.tolerance.cy = -1.0;
        assert!(matches!(
            validate_six_coefficient_deck(&six_deck, &[bad_tolerance]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_tolerance"
            })
        ));

        let mut blank_case_id = valid_case;
        blank_case_id.case_id = "";
        assert!(matches!(
            validate_six_coefficient_deck(&six_deck, &[blank_case_id]),
            Err(AerodbError::InvalidInput {
                field: "six_coefficient_validation_case_id"
            })
        ));
    }

    #[test]
    fn deck_builder_rejects_non_singleton_extra_axes_for_reduced_decks() {
        let mut axes = basic_axes();
        axes.reynolds_length = vec![1.0e6, 2.0e6];
        let matrix = RunMatrix::from_axes(axes, 0).unwrap();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let method = LinearMethod {
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };

        assert!(matches!(
            builder.bake_panel_deck(&matrix, &method),
            Err(AerodbError::UnsupportedDeckBake { .. }),
        ));
    }

    #[test]
    fn reduced_sigma_budget_attaches_rss_tables_to_baked_deck() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = LinearMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder.bake_panel_deck(&matrix, &method).unwrap();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(3.0, 4.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.0, 0.0, 12.0, 5.0).unwrap(),
            cm: CoefficientSigmaBudget::new(1.0, 2.0, 2.0, 0.0).unwrap(),
        };
        let deck = attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        assert!(deck.has_sigma_tables());

        let sigma = deck
            .lookup_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        assert_relative_eq!(sigma.cn, 5.0, epsilon = 1e-12);
        assert_relative_eq!(sigma.cd, 13.0, epsilon = 1e-12);
        assert_relative_eq!(sigma.cm, 3.0, epsilon = 1e-12);
    }

    #[test]
    fn six_sigma_budget_attaches_rss_tables_to_baked_six_deck() {
        let (matrix, deck) = bake_basic_six_deck();
        let budget = SixCoefficientSigmaBudget {
            cx: CoefficientSigmaBudget::new(3.0, 4.0, 0.0, 0.0).unwrap(),
            cy: CoefficientSigmaBudget::new(0.0, 0.0, 12.0, 5.0).unwrap(),
            cz: CoefficientSigmaBudget::new(1.0, 2.0, 2.0, 0.0).unwrap(),
            cl: CoefficientSigmaBudget::new(0.0, 0.0, 0.5, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.2, 0.1, 0.2, 0.4).unwrap(),
            cn: CoefficientSigmaBudget::new(0.0, 0.0, 0.0, 0.7).unwrap(),
        };
        let deck = attach_six_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        assert!(deck.has_six_sigma_tables());

        let sigma = deck
            .lookup_six_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        assert_relative_eq!(sigma.cx, 5.0, epsilon = 1e-12);
        assert_relative_eq!(sigma.cy, 13.0, epsilon = 1e-12);
        assert_relative_eq!(sigma.cz, 3.0, epsilon = 1e-12);
        assert_relative_eq!(sigma.cl, 0.5, epsilon = 1e-12);
        assert_relative_eq!(
            sigma.cm,
            (0.2_f64 * 0.2 + 0.1 * 0.1 + 0.2 * 0.2 + 0.4 * 0.4).sqrt(),
            epsilon = 1e-12
        );
        assert_relative_eq!(sigma.cn, 0.7, epsilon = 1e-12);

        let (_matrix, reduced_deck) = bake_basic_deck();
        assert!(matches!(
            attach_six_sigma_tables(reduced_deck, &vec![budget; 8]),
            Err(AerodbError::UnsupportedDeckBake { .. })
        ));
    }

    #[test]
    fn coefficient_sigma_budget_uses_gci_as_cfd_source() {
        let fine = 1.01;
        let medium = 1.04;
        let coarse = 1.16;
        let budget = CoefficientSigmaBudget::new(0.1, 0.0, 0.2, 0.0)
            .unwrap()
            .with_cfd_gci(fine, medium, coarse, 2.0)
            .unwrap();
        let gci = GridConvergence::from_three_grids(fine, medium, coarse, 2.0).unwrap();
        let expected =
            (0.1_f64 * 0.1 + gci.absolute_gci_fine * gci.absolute_gci_fine + 0.2 * 0.2).sqrt();
        assert_relative_eq!(budget.cfd, gci.absolute_gci_fine, epsilon = 1e-15);
        assert_relative_eq!(budget.rss_one_sigma().unwrap(), expected, epsilon = 1e-15);
    }

    fn gci_sigma_row_toml(index: usize) -> String {
        let offset = index as f64 * 0.01;
        format!(
            r#"
[[rows]]
cn = {{ fine = {}, medium = {}, coarse = {}, wind_tunnel = 0.1, model_form = 0.2 }}
cd = {{ fine = {}, medium = {}, coarse = {}, extrapolation = 0.05 }}
cm = {{ fine = {}, medium = {}, coarse = {} }}
"#,
            1.01 + offset,
            1.04 + offset,
            1.16 + offset,
            0.51 + offset,
            0.54 + offset,
            0.66 + offset,
            -0.21 - offset,
            -0.24 - offset,
            -0.36 - offset,
        )
    }

    fn gci_sigma_toml(row_count: usize) -> String {
        let mut text = String::from(
            r#"
openbmp.aerodb_gci_sigma = 1
provenance = "synthetic external solver GCI rows"
refinement_ratio = 2.0
safety_factor = 1.25
"#,
        );
        for index in 0..row_count {
            text.push_str(&gci_sigma_row_toml(index));
        }
        text
    }

    fn gci_sigma_csv_row(index: usize) -> String {
        let offset = index as f64 * 0.01;
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},0.1,0.2,0.0,0.0,0.0,0.05,0.0,0.0,0.0,synthetic external solver row {index}\n",
            GCI_SIGMA_CSV_SCHEMA_V1,
            index,
            1.01 + offset,
            1.04 + offset,
            1.16 + offset,
            0.51 + offset,
            0.54 + offset,
            0.66 + offset,
            -0.21 - offset,
            -0.24 - offset,
            -0.36 - offset,
        )
    }

    fn gci_sigma_csv(row_count: usize) -> String {
        let mut text = GCI_SIGMA_CSV_HEADER.join(",");
        text.push('\n');
        for index in 0..row_count {
            text.push_str(&gci_sigma_csv_row(index));
        }
        text
    }

    fn six_gci_sigma_row_toml(index: usize) -> String {
        let offset = index as f64 * 0.01;
        format!(
            r#"
[[rows]]
cx = {{ fine = {}, medium = {}, coarse = {}, wind_tunnel = 0.1, model_form = 0.2 }}
cy = {{ fine = {}, medium = {}, coarse = {}, extrapolation = 0.05 }}
cz = {{ fine = {}, medium = {}, coarse = {} }}
cl = {{ fine = {}, medium = {}, coarse = {}, model_form = 0.03 }}
cm = {{ fine = {}, medium = {}, coarse = {}, wind_tunnel = 0.04 }}
cn = {{ fine = {}, medium = {}, coarse = {}, extrapolation = 0.02 }}
"#,
            1.01 + offset,
            1.04 + offset,
            1.16 + offset,
            0.51 + offset,
            0.54 + offset,
            0.66 + offset,
            -0.21 - offset,
            -0.24 - offset,
            -0.36 - offset,
            0.31 + offset,
            0.34 + offset,
            0.46 + offset,
            -0.41 - offset,
            -0.44 - offset,
            -0.56 - offset,
            0.71 + offset,
            0.74 + offset,
            0.86 + offset,
        )
    }

    fn six_gci_sigma_toml(row_count: usize) -> String {
        let mut text = String::from(
            r#"
openbmp.aerodb_six_gci_sigma = 1
provenance = "synthetic external solver six GCI rows"
refinement_ratio = 2.0
safety_factor = 1.25
"#,
        );
        for index in 0..row_count {
            text.push_str(&six_gci_sigma_row_toml(index));
        }
        text
    }

    fn six_gci_sigma_csv_row(index: usize) -> String {
        let offset = index as f64 * 0.01;
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0.1,0.2,0.0,0.0,0.0,0.05,0.0,0.0,0.0,0.0,0.03,0.0,0.04,0.0,0.0,0.0,0.0,0.02,synthetic external solver six row {index}\n",
            SIX_GCI_SIGMA_CSV_SCHEMA_V1,
            index,
            1.01 + offset,
            1.04 + offset,
            1.16 + offset,
            0.51 + offset,
            0.54 + offset,
            0.66 + offset,
            -0.21 - offset,
            -0.24 - offset,
            -0.36 - offset,
            0.31 + offset,
            0.34 + offset,
            0.46 + offset,
            -0.41 - offset,
            -0.44 - offset,
            -0.56 - offset,
            0.71 + offset,
            0.74 + offset,
            0.86 + offset,
        )
    }

    fn six_gci_sigma_csv(row_count: usize) -> String {
        let mut text = SIX_GCI_SIGMA_CSV_HEADER.join(",");
        text.push('\n');
        for index in 0..row_count {
            text.push_str(&six_gci_sigma_csv_row(index));
        }
        text
    }

    fn solver_coefficient_csv_row(index: usize, node: RunNode) -> String {
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},su2-euler,case-{index},synthetic solver coefficient row {index}\n",
            SOLVER_COEFFICIENT_CSV_SCHEMA_V1,
            index,
            node.mach,
            node.alpha_deg,
            node.beta_deg,
            node.reynolds_length,
            node.knudsen,
            node.dynamic_pressure_pa,
            0.2 + index as f64 * 0.01,
            0.4 + index as f64 * 0.01,
            -0.05 - index as f64 * 0.001,
        )
    }

    fn solver_coefficient_csv(matrix: &RunMatrix) -> String {
        let mut text = SOLVER_COEFFICIENT_CSV_HEADER.join(",");
        text.push('\n');
        for (index, node) in matrix.nodes().iter().copied().enumerate() {
            text.push_str(&solver_coefficient_csv_row(index, node));
        }
        text
    }

    fn solver_six_coefficient_csv_row(index: usize, node: RunNode) -> String {
        format!(
            "{},{},{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},su2-euler,case-{index},synthetic solver six coefficient row {index}\n",
            SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1,
            index,
            node.mach,
            node.alpha_deg,
            node.beta_deg,
            node.reynolds_length,
            node.knudsen,
            node.dynamic_pressure_pa,
            0.10 + 0.01 * node.mach,
            -0.20 + 0.001 * node.beta_deg,
            0.30 + 0.002 * node.alpha_deg,
            -0.010 + 0.001 * node.alpha_deg,
            0.020 + 0.001 * node.mach,
            -0.030 + 0.001 * node.beta_deg,
        )
    }

    fn solver_six_coefficient_csv(matrix: &RunMatrix) -> String {
        let mut text = SOLVER_SIX_COEFFICIENT_CSV_HEADER.join(",");
        text.push('\n');
        for (index, node) in matrix.nodes().iter().copied().enumerate() {
            text.push_str(&solver_six_coefficient_csv_row(index, node));
        }
        text
    }

    fn solver_case_manifest_toml(matrix: &RunMatrix) -> String {
        let mut text = String::from(
            r#"
[openbmp]
aerodb_solver_case_manifest = 1
"#,
        );
        for (index, node) in matrix.nodes().iter().copied().enumerate() {
            text.push_str(&format!(
                r#"
[[cases]]
node_index = {index}
mach = {}
alpha_deg = {}
beta_deg = {}
reynolds_length = {}
knudsen = {}
dynamic_pressure_pa = {}
solver = "su2-euler"
case_id = "case-{index}"
output_kind = "surface_cp_csv"
output_file = "cases/case-{index}/surface_cp.csv"
output_sha256 = "{:064x}"
provenance = "synthetic solver case {index}"
"#,
                node.mach,
                node.alpha_deg,
                node.beta_deg,
                node.reynolds_length,
                node.knudsen,
                node.dynamic_pressure_pa,
                index + 1,
            ));
        }
        text
    }

    fn solver_command_plan_toml(matrix: &RunMatrix) -> String {
        let mut text = String::from(
            r#"
[openbmp]
aerodb_solver_command_plan = 1
"#,
        );
        for (index, node) in matrix.nodes().iter().copied().enumerate() {
            text.push_str(&format!(
                r#"
[[cases]]
node_index = {index}
mach = {}
alpha_deg = {}
beta_deg = {}
reynolds_length = {}
knudsen = {}
dynamic_pressure_pa = {}
solver = "su2-euler"
case_id = "case-{index}"
program = "SU2_CFD"
args = ["case-{index}.cfg", "--no-color"]
env = {{ OMP_NUM_THREADS = "1", OPENBMP_CASE = "case-{index}" }}
working_dir = "cases/case-{index}"
timeout_s = 3600
output_kind = "normalized_six_coefficients_csv"
output_file = "cases/case-{index}/solver_six_coefficients.csv"
output_sha256 = "{:064x}"
provenance = "synthetic solver command case {index}"
"#,
                node.mach,
                node.alpha_deg,
                node.beta_deg,
                node.reynolds_length,
                node.knudsen,
                node.dynamic_pressure_pa,
                index + 1,
            ));
        }
        text
    }

    fn live_solver_command_plan_toml(matrix: &RunMatrix, script: &str, timeout_s: u64) -> String {
        let output_bytes = b"live solver output";
        let mut text = String::from(
            r#"
[openbmp]
aerodb_solver_command_plan = 1
"#,
        );
        for (index, node) in matrix.nodes().iter().copied().enumerate() {
            text.push_str(&format!(
                r#"
[[cases]]
node_index = {index}
mach = {}
alpha_deg = {}
beta_deg = {}
reynolds_length = {}
knudsen = {}
dynamic_pressure_pa = {}
solver = "python-fixture"
case_id = "case-{index}"
program = "python3"
args = ["-c", {script:?}]
env = {{ OPENBMP_CASE = "case-{index}" }}
working_dir = "cases/case-{index}"
timeout_s = {timeout_s}
output_kind = "normalized_six_coefficients_csv"
output_file = "cases/case-{index}/solver_six_coefficients.csv"
output_sha256 = "{}"
provenance = "synthetic live solver command case {index}"
"#,
                node.mach,
                node.alpha_deg,
                node.beta_deg,
                node.reynolds_length,
                node.knudsen,
                node.dynamic_pressure_pa,
                sha256_hex(output_bytes),
            ));
        }
        text
    }

    fn surface_cp_csv_row(node_index: usize, panel_index: usize, node: RunNode) -> String {
        format!(
            "{},{},{},{},{},{},{},{},{},{},su2-surface,case-{node_index},synthetic surface Cp row {node_index}-{panel_index}\n",
            SURFACE_CP_CSV_SCHEMA_V1,
            node_index,
            panel_index,
            node.mach,
            node.alpha_deg,
            node.beta_deg,
            node.reynolds_length,
            node.knudsen,
            node.dynamic_pressure_pa,
            0.2 + node_index as f64 * 0.01,
        )
    }

    fn surface_cp_csv(matrix: &RunMatrix, mesh: &PanelMesh) -> String {
        let mut text = SURFACE_CP_CSV_HEADER.join(",");
        text.push('\n');
        for (node_index, node) in matrix.nodes().iter().copied().enumerate() {
            for panel_index in 0..mesh.triangles().len() {
                text.push_str(&surface_cp_csv_row(node_index, panel_index, node));
            }
        }
        text
    }

    fn su2_surface_cp_csv(mesh: &PanelMesh, cp: f64) -> String {
        let mut text =
            String::from("Global_Index,Points_0,Points_1,Points_2,Pressure_Coefficient\n");
        for (row_index, vertex_index) in [2usize, 0, 3, 1].into_iter().enumerate() {
            let vertex = mesh.vertices()[vertex_index];
            text.push_str(&format!(
                "{row_index},{},{},{},{}\n",
                vertex.x, vertex.y, vertex.z, cp
            ));
        }
        text
    }

    fn cart3d_triq_text(mesh: &PanelMesh, cp: f64) -> String {
        let vertex_order = [2usize, 0, 3, 1];
        let mut mesh_to_triq_vertex = BTreeMap::new();
        let mut text = format!("{} {} 1\n", mesh.vertices().len(), mesh.triangles().len());
        for (local_index, mesh_vertex_index) in vertex_order.into_iter().enumerate() {
            mesh_to_triq_vertex.insert(mesh_vertex_index, local_index + 1);
            let vertex = mesh.vertices()[mesh_vertex_index];
            text.push_str(&format!("{} {} {}\n", vertex.x, vertex.y, vertex.z));
        }
        for triangle in mesh.triangles() {
            text.push_str(&format!(
                "{} {} {}\n",
                mesh_to_triq_vertex[&(triangle[0] as usize)],
                mesh_to_triq_vertex[&(triangle[1] as usize)],
                mesh_to_triq_vertex[&(triangle[2] as usize)]
            ));
        }
        for _ in mesh.triangles() {
            text.push_str("1\n");
        }
        for _ in vertex_order {
            text.push_str(&format!("{cp}\n"));
        }
        text
    }

    fn cart3d_triq_texts(matrix: &RunMatrix, mesh: &PanelMesh) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| cart3d_triq_text(mesh, 0.2 + index as f64 * 0.01))
            .collect()
    }

    fn cart3d_triq_cases<'a>(texts: &'a [String]) -> Vec<Cart3dTriqSurfaceCp<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, triq_text)| Cart3dTriqSurfaceCp {
                node_index: index,
                case_id: "synthetic-cart3d-triq-case",
                provenance: "synthetic Cart3D TRI/Q fixture",
                triq_text,
            })
            .collect()
    }

    fn su2_surface_cp_texts(matrix: &RunMatrix, mesh: &PanelMesh) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| su2_surface_cp_csv(mesh, 0.2 + index as f64 * 0.01))
            .collect()
    }

    fn su2_surface_cp_cases<'a>(texts: &'a [String]) -> Vec<Su2SurfaceCpCsv<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, csv_text)| Su2SurfaceCpCsv {
                node_index: index,
                case_id: "synthetic-su2-surface-case",
                provenance: "synthetic SU2 surface fixture",
                csv_text,
            })
            .collect()
    }

    fn su2_history_csv(cn: f64, cd: f64, cm: f64) -> String {
        format!(
            "Inner_Iter,CFx,CFz,CMy\n0,-0.001,-0.002,0.0\n50,{},{},{}\n",
            -cd, -cn, cm
        )
    }

    fn su2_history_texts(matrix: &RunMatrix) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| {
                su2_history_csv(
                    0.2 + index as f64 * 0.01,
                    0.4 + index as f64 * 0.01,
                    -0.05 - index as f64 * 0.001,
                )
            })
            .collect()
    }

    fn su2_history_cases<'a>(texts: &'a [String]) -> Vec<Su2HistoryCoefficientCsv<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, csv_text)| Su2HistoryCoefficientCsv {
                node_index: index,
                case_id: "synthetic-su2-case",
                provenance: "synthetic SU2 history fixture",
                csv_text,
            })
            .collect()
    }

    fn openfoam_force_coeffs_text(wind_cd: f64, cl: f64, cm_pitch: f64) -> String {
        format!(
            "# Force coefficients\n# dragDir (1 0 0)\n# Time Cd Cs Cl CmRoll CmPitch CmYaw\n0.0 0.001 0.0 0.002 0.0 0.0 0.0\n1.0 {wind_cd} 0.0 {cl} 0.0 {cm_pitch} 0.0\n",
        )
    }

    fn openfoam_surface_cp_text(mesh: &PanelMesh, cp: f64) -> String {
        let mut text = String::from("# sampledSurface raw point Cp\n# x y z Cp\n(\n");
        for vertex_index in [2usize, 0, 3, 1] {
            let vertex = mesh.vertices()[vertex_index];
            text.push_str(&format!(
                "({} {} {}) {};\n",
                vertex.x, vertex.y, vertex.z, cp
            ));
        }
        text.push_str(")\n");
        text
    }

    fn openfoam_boundary_cp_field_text(cp: f64, panel_count: usize) -> String {
        let mut values = String::new();
        for _ in 0..panel_count {
            values.push_str(&format!("    {cp}\n"));
        }
        format!(
            r#"
FoamFile
{{
    version 2.0;
    format ascii;
    class volScalarField;
    location "0";
    object Cp;
}}
dimensions [0 0 0 0 0 0 0];
internalField uniform 0;
boundaryField
{{
    body
    {{
        type fixedValue;
        value nonuniform List<scalar>
        {panel_count}
        (
{values}        );
    }}
    freestream
    {{
        type fixedValue;
        value uniform -0.1;
    }}
    outlet
    {{
        type zeroGradient;
    }}
}}
"#
        )
    }

    fn openfoam_vector_field_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class volVectorField;
    location "0";
    object U;
}
dimensions [0 1 -1 0 0 0 0];
internalField uniform (10 0 0);
boundaryField
{
    inlet
    {
        type fixedValue;
        value uniform (10 1 0);
    }
    body
    {
        type noSlip;
        value nonuniform List<vector>
        2
        (
            (0 0 0)
            (0.5 0 0)
        );
    }
    outlet
    {
        type zeroGradient;
    }
}
"#
        .to_owned()
    }

    fn openfoam_turbulence_scalar_field_text(object_name: &str, value: f64) -> String {
        format!(
            r#"
FoamFile
{{
    version 2.0;
    format ascii;
    class volScalarField;
    location "0";
    object {object_name};
}}
dimensions [0 2 -2 0 0 0 0];
internalField uniform {value};
boundaryField
{{
    body
    {{
        type nutkWallFunction;
        value uniform 0;
    }}
    freestream
    {{
        type calculated;
        value uniform {value};
    }}
}}
"#
        )
    }

    fn openfoam_turbulence_properties_ras_text(model_name: &str) -> String {
        format!(
            r#"
FoamFile
{{
    version 2.0;
    format ascii;
    class dictionary;
    location "constant";
    object turbulenceProperties;
}}
simulationType RAS;
RAS
{{
    RASModel {model_name};
    turbulence on;
    printCoeffs on;
}}
"#
        )
    }

    fn openfoam_turbulence_properties_ras_coeff_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class dictionary;
    location "constant";
    object turbulenceProperties;
}
simulationType RAS;
RAS
{
    RASModel kOmegaSST;
    turbulence on;
    printCoeffs on;
    kOmegaSSTCoeffs
    {
        betaStar 0.09;
        a1 0.31;
        beta1 0.075;
        ignoredNested
        {
            value 123;
        }
    }
}
"#
        .to_owned()
    }

    fn openfoam_turbulence_properties_laminar_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class dictionary;
    location "constant";
    object turbulenceProperties;
}
simulationType laminar;
"#
        .to_owned()
    }

    fn openfoam_tensor_field_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class volTensorField;
    location "0";
    object tau;
}
dimensions [0 2 -2 0 0 0 0];
internalField uniform (1 0 0 0 1 0 0 0 1);
boundaryField
{
    body
    {
        type calculated;
    }
}
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_boundary_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class polyBoundaryMesh;
    location "constant/polyMesh";
    object boundary;
}
4
(
    inlet
    {
        type patch;
        nFaces 8;
        startFace 100;
    }
    body
    {
        type wall;
        inGroups 1(wall);
        nFaces 12;
        startFace 108;
    }
    freestream
    {
        type patch;
        nFaces 4;
        startFace 120;
    }
    outlet
    {
        type patch;
        nFaces 6;
        startFace 124;
    }
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_connectivity_boundary_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class polyBoundaryMesh;
    location "constant/polyMesh";
    object boundary;
}
2
(
    body
    {
        type wall;
        nFaces 2;
        startFace 2;
    }
    outlet
    {
        type patch;
        nFaces 2;
        startFace 4;
    }
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_points_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class vectorField;
    location "constant/polyMesh";
    object points;
}
5
(
    (0 0 0)
    (1 0 0)
    (1 1 0)
    (0 1 0)
    (0.5 0.5 1)
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_faces_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class faceList;
    location "constant/polyMesh";
    object faces;
}
6
(
    3(0 1 4)
    3(2 3 4)
    3(0 3 4)
    3(1 2 4)
    4(0 1 2 3)
    3(0 2 4)
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_owner_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class labelList;
    location "constant/polyMesh";
    object owner;
}
6
(
    0
    1
    0
    0
    1
    1
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_neighbour_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class labelList;
    location "constant/polyMesh";
    object neighbour;
}
2
(
    1
    0
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_tetra_points_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class vectorField;
    location "constant/polyMesh";
    object points;
}
4
(
    (0 0 0)
    (1 0 0)
    (0 1 0)
    (0 0 1)
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_tetra_faces_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class faceList;
    location "constant/polyMesh";
    object faces;
}
4
(
    3(0 2 1)
    3(0 1 3)
    3(0 3 2)
    3(1 2 3)
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_tetra_owner_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class labelList;
    location "constant/polyMesh";
    object owner;
}
4
(
    0
    0
    0
    0
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_empty_neighbour_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class labelList;
    location "constant/polyMesh";
    object neighbour;
}
0
(
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_point_zones_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class pointZoneList;
    location "constant/polyMesh";
    object pointZones;
}
1
(
    nose
    {
        type pointZone;
        pointLabels 2(0 4);
    }
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_face_zones_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class faceZoneList;
    location "constant/polyMesh";
    object faceZones;
}
1
(
    wake
    {
        type faceZone;
        faceLabels List<label> 2 (2 3);
        flipMap List<bool> 2 (0 1);
    }
)
"#
        .to_owned()
    }

    fn openfoam_poly_mesh_cell_zones_text() -> String {
        r#"
FoamFile
{
    version 2.0;
    format ascii;
    class cellZoneList;
    location "constant/polyMesh";
    object cellZones;
}
1
(
    core
    {
        type cellZone;
        cellLabels 2(0 1);
    }
)
"#
        .to_owned()
    }

    fn sparta_surface_forces_text(mesh: &PanelMesh, node_index: usize) -> String {
        let cp_equivalent = 0.2 + node_index as f64 * 0.01;
        let panel_force_z = -0.5 * 100.0 * cp_equivalent;
        let mut text = String::from("# SPARTA compute surf fx/fy/fz fixture\nsurf_id fx fy fz\n");
        for panel_id in (1..=mesh.triangles().len()).rev() {
            text.push_str(&format!("{panel_id} 0.0 0.0 {panel_force_z}\n"));
        }
        text
    }

    fn sparta_surface_forces_texts(matrix: &RunMatrix, mesh: &PanelMesh) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| sparta_surface_forces_text(mesh, index))
            .collect()
    }

    fn sparta_surface_force_cases<'a>(texts: &'a [String]) -> Vec<SpartaSurfaceForces<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, text)| SpartaSurfaceForces {
                node_index: index,
                case_id: "synthetic-sparta-surface-case",
                provenance: "synthetic SPARTA surface-force fixture",
                text,
            })
            .collect()
    }

    fn openfoam_surface_cp_texts(matrix: &RunMatrix, mesh: &PanelMesh) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| openfoam_surface_cp_text(mesh, 0.2 + index as f64 * 0.01))
            .collect()
    }

    fn openfoam_surface_cp_cases<'a>(texts: &'a [String]) -> Vec<OpenFoamSurfaceCp<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, text)| OpenFoamSurfaceCp {
                node_index: index,
                case_id: "synthetic-openfoam-surface-case",
                provenance: "synthetic OpenFOAM sampled surface fixture",
                text,
            })
            .collect()
    }

    fn openfoam_boundary_cp_field_texts(matrix: &RunMatrix, mesh: &PanelMesh) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| {
                openfoam_boundary_cp_field_text(0.2 + index as f64 * 0.01, mesh.triangles().len())
            })
            .collect()
    }

    fn openfoam_boundary_cp_field_cases<'a>(
        texts: &'a [String],
    ) -> Vec<OpenFoamBoundaryCpField<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, field_text)| OpenFoamBoundaryCpField {
                node_index: index,
                case_id: "synthetic-openfoam-boundary-case",
                provenance: "synthetic OpenFOAM boundary scalar field fixture",
                patch_name: "body",
                field_text,
            })
            .collect()
    }

    fn openfoam_force_coeffs_texts(matrix: &RunMatrix) -> Vec<String> {
        matrix
            .nodes()
            .iter()
            .enumerate()
            .map(|(index, _node)| {
                openfoam_force_coeffs_text(
                    0.4 + index as f64 * 0.01,
                    0.2 + index as f64 * 0.01,
                    -0.05 - index as f64 * 0.001,
                )
            })
            .collect()
    }

    fn openfoam_force_coeffs_cases<'a>(texts: &'a [String]) -> Vec<OpenFoamForceCoeffs<'a>> {
        texts
            .iter()
            .enumerate()
            .map(|(index, text)| OpenFoamForceCoeffs {
                node_index: index,
                case_id: "synthetic-openfoam-case",
                provenance: "synthetic OpenFOAM forceCoeffs fixture",
                text,
            })
            .collect()
    }

    #[test]
    fn solver_case_manifest_parses_row_major_artifact_pins() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let manifest =
            SolverCaseManifest::load_toml_str(&matrix, &solver_case_manifest_toml(&matrix))
                .unwrap();

        assert_eq!(manifest.cases().len(), matrix.nodes().len());
        assert_eq!(manifest.cases()[0].node_index, 0);
        assert_eq!(manifest.cases()[0].node, matrix.nodes()[0]);
        assert_eq!(
            manifest.cases()[0].output_kind,
            SolverCaseOutputKind::SurfaceCpCsv
        );
        assert_eq!(
            manifest.cases()[7].output_file,
            "cases/case-7/surface_cp.csv"
        );
        assert_eq!(manifest.cases()[7].output_sha256.len(), 64);

        let openfoam_surface_manifest_toml = solver_case_manifest_toml(&matrix).replace(
            "output_kind = \"surface_cp_csv\"",
            "output_kind = \"open_foam_surface_cp\"",
        );
        let openfoam_surface_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &openfoam_surface_manifest_toml).unwrap();
        assert_eq!(
            openfoam_surface_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::OpenFoamSurfaceCp
        );

        let openfoam_boundary_manifest_toml = solver_case_manifest_toml(&matrix).replace(
            "output_kind = \"surface_cp_csv\"",
            "output_kind = \"open_foam_boundary_cp_field\"",
        );
        let openfoam_boundary_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &openfoam_boundary_manifest_toml).unwrap();
        assert_eq!(
            openfoam_boundary_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::OpenFoamBoundaryCpField
        );

        let openfoam_scalar_manifest_toml = solver_case_manifest_toml(&matrix).replace(
            "output_kind = \"surface_cp_csv\"",
            "output_kind = \"open_foam_scalar_field\"",
        );
        let openfoam_scalar_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &openfoam_scalar_manifest_toml).unwrap();
        assert_eq!(
            openfoam_scalar_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::OpenFoamScalarField
        );

        let openfoam_vector_manifest_toml = solver_case_manifest_toml(&matrix).replace(
            "output_kind = \"surface_cp_csv\"",
            "output_kind = \"open_foam_vector_field\"",
        );
        let openfoam_vector_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &openfoam_vector_manifest_toml).unwrap();
        assert_eq!(
            openfoam_vector_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::OpenFoamVectorField
        );

        let six_coefficients_manifest_toml = solver_case_manifest_toml(&matrix)
            .replace("surface_cp_csv", "normalized_six_coefficients_csv");
        let six_coefficients_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &six_coefficients_manifest_toml).unwrap();
        assert_eq!(
            six_coefficients_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::NormalizedSixCoefficientsCsv
        );

        let cart3d_triq_manifest_toml =
            solver_case_manifest_toml(&matrix).replace("surface_cp_csv", "cart3d_triq");
        let cart3d_triq_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &cart3d_triq_manifest_toml).unwrap();
        assert_eq!(
            cart3d_triq_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::Cart3dTriq
        );

        let sparta_surface_manifest_toml =
            solver_case_manifest_toml(&matrix).replace("surface_cp_csv", "sparta_surface_forces");
        let sparta_surface_manifest =
            SolverCaseManifest::load_toml_str(&matrix, &sparta_surface_manifest_toml).unwrap();
        assert_eq!(
            sparta_surface_manifest.cases()[0].output_kind,
            SolverCaseOutputKind::SpartaSurfaceForces
        );
    }

    #[test]
    fn solver_case_manifest_rejects_node_condition_and_hash_mismatch() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let toml = solver_case_manifest_toml(&matrix);
        let bad_index = toml.replacen("node_index = 0", "node_index = 7", 1);
        assert!(matches!(
            SolverCaseManifest::load_toml_str(&matrix, &bad_index),
            Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_node_index"
            })
        ));

        let bad_condition = toml.replacen("mach = 2\n", "mach = 4\n", 1);
        assert!(matches!(
            SolverCaseManifest::load_toml_str(&matrix, &bad_condition),
            Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_mach"
            })
        ));

        let first_hash = format!("{:064x}", 1);
        let bad_hash = toml.replacen(&first_hash, "not-a-sha", 1);
        assert!(matches!(
            SolverCaseManifest::load_toml_str(&matrix, &bad_hash),
            Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_output_sha256"
            })
        ));
    }

    #[test]
    fn solver_case_manifest_verifies_artifact_sha256_pins() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mut toml = solver_case_manifest_toml(&matrix);
        let mut artifacts = BTreeMap::<String, Vec<u8>>::new();
        for index in 0..matrix.nodes().len() {
            let bytes = format!("synthetic solver artifact {index}").into_bytes();
            toml = toml.replacen(&format!("{:064x}", index + 1), &sha256_hex(&bytes), 1);
            artifacts.insert(format!("cases/case-{index}/surface_cp.csv"), bytes);
        }
        let manifest = SolverCaseManifest::load_toml_str(&matrix, &toml).unwrap();

        manifest
            .verify_artifact_sha256_with_bytes(|case| {
                artifacts.get(&case.output_file).map(Vec::as_slice)
            })
            .unwrap();

        artifacts.get_mut("cases/case-0/surface_cp.csv").unwrap()[0] ^= 0xff;
        assert!(matches!(
            manifest.verify_artifact_sha256_with_bytes(|case| {
                artifacts.get(&case.output_file).map(Vec::as_slice)
            }),
            Err(AerodbError::InvalidInput {
                field: "solver_case_manifest_output_sha256"
            })
        ));
    }

    #[test]
    fn solver_command_plan_parses_argv_env_and_projects_artifact_manifest() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mut toml = solver_command_plan_toml(&matrix);
        let mut artifacts = BTreeMap::<String, Vec<u8>>::new();
        for index in 0..matrix.nodes().len() {
            let bytes = format!("synthetic solver command output {index}").into_bytes();
            toml = toml.replacen(&format!("{:064x}", index + 1), &sha256_hex(&bytes), 1);
            artifacts.insert(
                format!("cases/case-{index}/solver_six_coefficients.csv"),
                bytes,
            );
        }

        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();

        assert_eq!(plan.cases().len(), matrix.nodes().len());
        assert_eq!(plan.cases()[0].node, matrix.nodes()[0]);
        assert_eq!(plan.cases()[0].program, "SU2_CFD");
        assert_eq!(
            plan.cases()[0].args.as_slice(),
            ["case-0.cfg", "--no-color"]
        );
        assert_eq!(
            plan.cases()[0]
                .env
                .get("OMP_NUM_THREADS")
                .map(String::as_str),
            Some("1")
        );
        assert_eq!(plan.cases()[0].timeout_s, 3600);
        assert_eq!(
            plan.cases()[0].output_kind,
            SolverCaseOutputKind::NormalizedSixCoefficientsCsv
        );

        let manifest = plan.expected_artifact_manifest();
        assert_eq!(
            manifest.cases()[7].output_file,
            "cases/case-7/solver_six_coefficients.csv"
        );
        manifest
            .verify_artifact_sha256_with_bytes(|case| {
                artifacts.get(&case.output_file).map(Vec::as_slice)
            })
            .unwrap();
    }

    #[test]
    fn solver_command_plan_rejects_node_condition_command_env_timeout_and_hash_mismatch() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let toml = solver_command_plan_toml(&matrix);

        let bad_index = toml.replacen("node_index = 0", "node_index = 7", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &bad_index),
            Err(AerodbError::InvalidInput {
                field: "solver_command_plan_node_index"
            })
        ));

        let bad_condition = toml.replacen("mach = 2\n", "mach = 4\n", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &bad_condition),
            Err(AerodbError::InvalidInput {
                field: "solver_command_plan_mach"
            })
        ));

        let blank_program = toml.replacen("program = \"SU2_CFD\"", "program = \" \"", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &blank_program),
            Err(AerodbError::InvalidInput {
                field: "solver_command_plan_program"
            })
        ));

        let bad_env = toml.replacen("OMP_NUM_THREADS", "OMP=NUM_THREADS", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &bad_env),
            Err(AerodbError::MalformedToml { .. })
                | Err(AerodbError::InvalidInput {
                    field: "solver_command_plan_env_key"
                })
        ));

        let zero_timeout = toml.replacen("timeout_s = 3600", "timeout_s = 0", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &zero_timeout),
            Err(AerodbError::InvalidInput {
                field: "solver_command_plan_timeout_s"
            })
        ));

        let first_hash = format!("{:064x}", 1);
        let bad_hash = toml.replacen(&first_hash, "not-a-sha", 1);
        assert!(matches!(
            SolverCommandPlan::load_toml_str(&matrix, &bad_hash),
            Err(AerodbError::InvalidInput {
                field: "solver_command_plan_output_sha256"
            })
        ));
    }

    #[test]
    fn solver_command_plan_execute_runs_direct_command_and_verifies_output_hash() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let root = temp_aerodb_dir("solver_execute_success");
        std::fs::create_dir_all(root.join("cases/case-0")).unwrap();
        let script = "from pathlib import Path; import os; Path('solver_six_coefficients.csv').write_bytes(b'live solver output'); print(os.environ['OPENBMP_CASE'])";
        let plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(&matrix, script, 5),
        )
        .unwrap();

        let report = plan.execute_with_root(&root).unwrap();

        assert_eq!(report.cases().len(), 1);
        assert_eq!(report.cases()[0].node_index, 0);
        assert_eq!(report.cases()[0].case_id, "case-0");
        assert_eq!(report.cases()[0].exit_code, Some(0));
        assert_eq!(report.cases()[0].stdout_len, "case-0\n".len());
        assert_eq!(report.cases()[0].stdout_sha256, sha256_hex(b"case-0\n"));
        assert_eq!(report.cases()[0].stderr_len, 0);
        assert_eq!(report.cases()[0].stderr_sha256, sha256_hex(b""));
        assert_eq!(
            report.cases()[0].output_sha256,
            sha256_hex(b"live solver output")
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_command_plan_execute_rejects_status_timeout_hash_and_unsafe_paths() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();

        let status_root = temp_aerodb_dir("solver_execute_status");
        std::fs::create_dir_all(status_root.join("cases/case-0")).unwrap();
        let status_plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(&matrix, "import sys; sys.exit(7)", 5),
        )
        .unwrap();
        assert!(matches!(
            status_plan.execute_with_root(&status_root),
            Err(AerodbError::InvalidInput {
                field: "solver_command_execute_status"
            })
        ));
        std::fs::remove_dir_all(status_root).unwrap();

        let timeout_root = temp_aerodb_dir("solver_execute_timeout");
        std::fs::create_dir_all(timeout_root.join("cases/case-0")).unwrap();
        let timeout_plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(&matrix, "import time; time.sleep(2)", 1),
        )
        .unwrap();
        assert!(matches!(
            timeout_plan.execute_with_root(&timeout_root),
            Err(AerodbError::InvalidInput {
                field: "solver_command_execute_timeout"
            })
        ));
        std::fs::remove_dir_all(timeout_root).unwrap();

        let hash_root = temp_aerodb_dir("solver_execute_hash");
        std::fs::create_dir_all(hash_root.join("cases/case-0")).unwrap();
        let hash_plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(
                &matrix,
                "from pathlib import Path; Path('solver_six_coefficients.csv').write_bytes(b'wrong output')",
                5,
            ),
        )
        .unwrap();
        assert!(matches!(
            hash_plan.execute_with_root(&hash_root),
            Err(AerodbError::InvalidInput {
                field: "solver_command_execute_output_sha256"
            })
        ));
        std::fs::remove_dir_all(hash_root).unwrap();

        let unsafe_root = temp_aerodb_dir("solver_execute_path");
        std::fs::create_dir_all(unsafe_root.join("cases/case-0")).unwrap();
        let unsafe_toml = live_solver_command_plan_toml(
            &matrix,
            "from pathlib import Path; Path('solver_six_coefficients.csv').write_bytes(b'live solver output')",
            5,
        )
        .replacen("working_dir = \"cases/case-0\"", "working_dir = \"../outside\"", 1);
        let unsafe_plan = SolverCommandPlan::load_toml_str(&matrix, &unsafe_toml).unwrap();
        assert!(matches!(
            unsafe_plan.execute_with_root(&unsafe_root),
            Err(AerodbError::InvalidInput {
                field: "solver_command_execute_working_dir"
            })
        ));
        std::fs::remove_dir_all(unsafe_root).unwrap();
    }

    #[test]
    fn solver_command_plan_retry_executes_until_success_and_records_attempts() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let root = temp_aerodb_dir("solver_retry_success");
        std::fs::create_dir_all(root.join("cases/case-0")).unwrap();
        let script = "from pathlib import Path; import sys; marker = Path('attempt.marker'); exists = marker.exists(); marker.write_text('seen') if not exists else None; sys.exit(7) if not exists else (Path('solver_six_coefficients.csv').write_bytes(b'live solver output') and print('retry-ok'))";
        let plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(&matrix, script, 5),
        )
        .unwrap();
        let policy = SolverCommandRetryPolicy::transient_failures(2).unwrap();

        let report = plan.execute_with_root_retrying(&root, policy).unwrap();

        assert_eq!(report.cases().len(), 1);
        let case = &report.cases()[0];
        assert_eq!(case.node_index, 0);
        assert_eq!(case.case_id, "case-0");
        assert_eq!(case.attempts.len(), 2);
        assert_eq!(case.attempts[0].attempt_index, 1);
        assert!(!case.attempts[0].success);
        assert_eq!(
            case.attempts[0].failure_field,
            Some("solver_command_execute_status")
        );
        assert_eq!(case.attempts[1].attempt_index, 2);
        assert!(case.attempts[1].success);
        assert_eq!(case.attempts[1].failure_field, None);
        assert_eq!(case.execution.stdout_sha256, sha256_hex(b"retry-ok\n"));
        assert_eq!(
            case.execution.output_sha256,
            sha256_hex(b"live solver output")
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_command_plan_retry_can_retry_output_verification_when_enabled() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let root = temp_aerodb_dir("solver_retry_hash");
        std::fs::create_dir_all(root.join("cases/case-0")).unwrap();
        let script = "from pathlib import Path; marker = Path('hash.marker'); exists = marker.exists(); marker.write_text('seen') if not exists else None; Path('solver_six_coefficients.csv').write_bytes(b'live solver output' if exists else b'wrong output')";
        let plan = SolverCommandPlan::load_toml_str(
            &matrix,
            &live_solver_command_plan_toml(&matrix, script, 5),
        )
        .unwrap();
        let mut policy = SolverCommandRetryPolicy::transient_failures(2).unwrap();
        policy.retry_on_output_verification = true;

        let report = plan.execute_with_root_retrying(&root, policy).unwrap();

        let case = &report.cases()[0];
        assert_eq!(case.attempts.len(), 2);
        assert_eq!(
            case.attempts[0].failure_field,
            Some("solver_command_execute_output_sha256")
        );
        assert!(case.attempts[1].success);
        assert_eq!(
            case.execution.output_sha256,
            sha256_hex(b"live solver output")
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_command_plan_retry_rejects_policy_and_nonretryable_path_failures() {
        assert!(matches!(
            SolverCommandRetryPolicy::transient_failures(0),
            Err(AerodbError::InvalidInput {
                field: "solver_command_retry_max_attempts"
            })
        ));
        assert!(matches!(
            SolverCommandRetryPolicy::transient_failures(MAX_SOLVER_COMMAND_RETRY_ATTEMPTS + 1),
            Err(AerodbError::InvalidInput {
                field: "solver_command_retry_max_attempts"
            })
        ));

        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let unsafe_root = temp_aerodb_dir("solver_retry_path");
        std::fs::create_dir_all(unsafe_root.join("cases/case-0")).unwrap();
        let unsafe_toml = live_solver_command_plan_toml(
            &matrix,
            "from pathlib import Path; Path('solver_six_coefficients.csv').write_bytes(b'live solver output')",
            5,
        )
        .replacen("working_dir = \"cases/case-0\"", "working_dir = \"../outside\"", 1);
        let unsafe_plan = SolverCommandPlan::load_toml_str(&matrix, &unsafe_toml).unwrap();
        let policy = SolverCommandRetryPolicy::transient_failures(3).unwrap();
        assert!(matches!(
            unsafe_plan.execute_with_root_retrying(&unsafe_root, policy),
            Err(AerodbError::InvalidInput {
                field: "solver_command_execute_working_dir"
            })
        ));
        std::fs::remove_dir_all(unsafe_root).unwrap();
    }

    #[test]
    fn solver_cluster_schedule_renders_deterministic_slurm_array_script() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let plan =
            SolverCommandPlan::load_toml_str(&matrix, &solver_command_plan_toml(&matrix)).unwrap();
        let mut template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3661);
        template.mpi_ranks_per_case = 4;
        template.cpus_per_task = 2;
        template.account = Some("aero123".into());
        template.partition = Some("compute".into());

        let schedule = plan.render_cluster_schedule(&template).unwrap();
        let repeated = plan.render_cluster_schedule(&template).unwrap();

        assert_eq!(schedule, repeated);
        assert_eq!(schedule.scheduler, SolverClusterScheduler::Slurm);
        assert_eq!(schedule.case_count, matrix.nodes().len());
        assert_eq!(
            schedule.submit_command.as_slice(),
            ["sbatch", "openbmp-solver-array.sbatch"]
        );
        assert_eq!(
            schedule.script_sha256,
            sha256_hex(schedule.script_text.as_bytes())
        );
        assert!(
            schedule
                .script_text
                .contains("#SBATCH --job-name=openbmp-aero\n")
        );
        assert!(schedule.script_text.contains("#SBATCH --array=0-7\n"));
        assert!(schedule.script_text.contains("#SBATCH --time=01:01:01\n"));
        assert!(schedule.script_text.contains("#SBATCH --ntasks=4\n"));
        assert!(schedule.script_text.contains("#SBATCH --cpus-per-task=2\n"));
        assert!(schedule.script_text.contains("#SBATCH --account=aero123\n"));
        assert!(
            schedule
                .script_text
                .contains("#SBATCH --partition=compute\n")
        );
        assert!(
            schedule
                .script_text
                .contains("cd -- \"$OPENBMP_SOLVER_ROOT\"/'cases/case-0'\n")
        );
        assert!(
            schedule
                .script_text
                .contains("export OPENBMP_CASE='case-0'\n")
        );
        assert!(
            schedule.script_text.contains(
                "srun --ntasks=4 --cpus-per-task=2 'SU2_CFD' 'case-0.cfg' '--no-color'\n"
            )
        );
        assert!(schedule.script_text.contains("# openbmp_timeout_s=3600\n"));
    }

    #[test]
    fn solver_cluster_schedule_shell_quotes_command_arguments() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let toml = solver_command_plan_toml(&matrix)
            .replace("program = \"SU2_CFD\"", "program = \"solver'safe\"")
            .replace("--no-color", "--flag'safe")
            .replace("OPENBMP_CASE = \"case-0\"", "OPENBMP_CASE = \"case'0\"");
        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);

        let schedule = plan.render_cluster_schedule(&template).unwrap();

        assert!(
            schedule
                .script_text
                .contains("export OPENBMP_CASE='case'\\''0'\n")
        );
        assert!(schedule.script_text.contains(
            "srun --ntasks=1 --cpus-per-task=1 'solver'\\''safe' 'case-0.cfg' '--flag'\\''safe'\n"
        ));
    }

    #[test]
    fn solver_cluster_schedule_rejects_invalid_template_and_unsafe_case_paths() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let plan =
            SolverCommandPlan::load_toml_str(&matrix, &solver_command_plan_toml(&matrix)).unwrap();

        let mut bad_job = SolverClusterScheduleTemplate::slurm("bad job", 3600);
        assert!(matches!(
            plan.render_cluster_schedule(&bad_job),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_job_name"
            })
        ));

        bad_job.job_name = "openbmp-aero".into();
        bad_job.log_dir = "../logs".into();
        assert!(matches!(
            plan.render_cluster_schedule(&bad_job),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_log_dir"
            })
        ));

        let mut zero_ranks = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        zero_ranks.mpi_ranks_per_case = 0;
        assert!(matches!(
            plan.render_cluster_schedule(&zero_ranks),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_mpi_ranks_per_case"
            })
        ));

        let unsafe_toml = solver_command_plan_toml(&matrix).replacen(
            "working_dir = \"cases/case-0\"",
            "working_dir = \"../outside\"",
            1,
        );
        let unsafe_plan = SolverCommandPlan::load_toml_str(&matrix, &unsafe_toml).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        assert!(matches!(
            unsafe_plan.render_cluster_schedule(&template),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_schedule_working_dir"
            })
        ));
    }

    #[test]
    fn solver_cluster_schedule_submit_writes_script_and_parses_slurm_job_id() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let plan =
            SolverCommandPlan::load_toml_str(&matrix, &solver_command_plan_toml(&matrix)).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        let mut schedule = plan.render_cluster_schedule(&template).unwrap();
        schedule.submit_command = vec!["python3".into(), "fake_submit.py".into()];
        let root = temp_aerodb_dir("solver_cluster_submit");
        std::fs::write(
            root.join("fake_submit.py"),
            "from pathlib import Path; assert Path('openbmp-solver-array.sbatch').read_text().startswith('#!/usr/bin/env bash'); print('Submitted batch job 4242')",
        )
        .unwrap();

        let report = schedule.submit_with_root(&root, 5).unwrap();

        assert_eq!(report.scheduler, SolverClusterScheduler::Slurm);
        assert_eq!(report.job_id, "4242");
        assert_eq!(report.script_filename, "openbmp-solver-array.sbatch");
        assert_eq!(report.script_sha256, schedule.script_sha256);
        assert_eq!(
            report.stdout_sha256,
            sha256_hex(b"Submitted batch job 4242\n")
        );
        assert_eq!(report.stderr_len, 0);
        assert_eq!(
            std::fs::read_to_string(root.join("openbmp-solver-array.sbatch")).unwrap(),
            schedule.script_text
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_cluster_schedule_query_parses_slurm_job_states() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let plan =
            SolverCommandPlan::load_toml_str(&matrix, &solver_command_plan_toml(&matrix)).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        let schedule = plan.render_cluster_schedule(&template).unwrap();
        let root = temp_aerodb_dir("solver_cluster_query");
        std::fs::write(root.join("fake_state.py"), "print('RUNNING|0:0')").unwrap();
        let query_command = vec!["python3".to_owned(), "fake_state.py".to_owned()];

        let report = schedule
            .query_job_state_with_root(&root, "4242", &query_command, 5)
            .unwrap();

        assert_eq!(report.scheduler, SolverClusterScheduler::Slurm);
        assert_eq!(report.job_id, "4242");
        assert_eq!(report.raw_state, "RUNNING");
        assert_eq!(report.status, SolverClusterJobStatus::Running);
        assert_eq!(report.stdout_sha256, sha256_hex(b"RUNNING|0:0\n"));
        assert_eq!(report.stderr_len, 0);
        assert_eq!(
            classify_slurm_job_state("PENDING"),
            SolverClusterJobStatus::Pending
        );
        assert_eq!(
            classify_slurm_job_state("COMPLETING"),
            SolverClusterJobStatus::Completing
        );
        assert_eq!(
            classify_slurm_job_state("COMPLETED"),
            SolverClusterJobStatus::Completed
        );
        assert_eq!(
            classify_slurm_job_state("FAILED"),
            SolverClusterJobStatus::Failed
        );
        assert_eq!(
            classify_slurm_job_state("CANCELLED"),
            SolverClusterJobStatus::Cancelled
        );
        assert_eq!(
            classify_slurm_job_state("TIMEOUT"),
            SolverClusterJobStatus::Timeout
        );
        assert_eq!(
            classify_slurm_job_state("SITE_CUSTOM"),
            SolverClusterJobStatus::Unknown
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_cluster_schedule_submit_and_query_fail_closed() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let plan =
            SolverCommandPlan::load_toml_str(&matrix, &solver_command_plan_toml(&matrix)).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        let mut schedule = plan.render_cluster_schedule(&template).unwrap();
        let root = temp_aerodb_dir("solver_cluster_submit_fail");

        schedule.submit_command.clear();
        assert!(matches!(
            schedule.submit_with_root(&root, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_submit_command"
            })
        ));

        schedule.submit_command = vec!["python3".into(), "fake_bad_submit.py".into()];
        std::fs::write(root.join("fake_bad_submit.py"), "print('no job id here')").unwrap();
        assert!(matches!(
            schedule.submit_with_root(&root, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_submit_job_id"
            })
        ));
        assert!(matches!(
            schedule.submit_with_root(&root, 0),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_submit_timeout_s"
            })
        ));

        let query_command = Vec::<String>::new();
        assert!(matches!(
            schedule.query_job_state_with_root(&root, "4242", &query_command, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_query_command"
            })
        ));
        std::fs::write(root.join("fake_empty_state.py"), "print('')").unwrap();
        let empty_state_command = vec!["python3".to_owned(), "fake_empty_state.py".to_owned()];
        assert!(matches!(
            schedule.query_job_state_with_root(&root, "4242", &empty_state_command, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_query_state"
            })
        ));
        assert!(matches!(
            schedule.query_job_state_with_root(&root, "bad job", &empty_state_command, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_query_job_id"
            })
        ));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_cluster_schedule_polls_completion_and_collects_artifacts() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let output_bytes = b"cluster solver output";
        let toml = solver_command_plan_toml(&matrix).replacen(
            &format!("{:064x}", 1),
            &sha256_hex(output_bytes),
            1,
        );
        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        let schedule = plan.render_cluster_schedule(&template).unwrap();
        let root = temp_aerodb_dir("solver_cluster_completion");
        std::fs::create_dir_all(root.join("cases/case-0")).unwrap();
        std::fs::write(
            root.join("cases/case-0/solver_six_coefficients.csv"),
            output_bytes,
        )
        .unwrap();
        std::fs::write(
            root.join("fake_poll.py"),
            "from pathlib import Path; p=Path('poll.count'); n=int(p.read_text()) if p.exists() else 0; states=['PENDING','RUNNING','COMPLETED']; print(states[min(n, len(states)-1)]); p.write_text(str(n+1))",
        )
        .unwrap();
        let query_command = vec!["python3".to_owned(), "fake_poll.py".to_owned()];
        let policy = SolverClusterPollingPolicy::new(4, 0, 5).unwrap();

        let completion = schedule
            .wait_for_completion_with_root(&root, "4242", &query_command, policy)
            .unwrap();
        let artifacts = plan.collect_expected_outputs_with_root(&root).unwrap();

        assert_eq!(completion.scheduler, SolverClusterScheduler::Slurm);
        assert_eq!(completion.job_id, "4242");
        assert_eq!(completion.final_status, SolverClusterJobStatus::Completed);
        assert_eq!(completion.polls.len(), 3);
        assert_eq!(completion.polls[0].status, SolverClusterJobStatus::Pending);
        assert_eq!(completion.polls[1].status, SolverClusterJobStatus::Running);
        assert_eq!(
            completion.polls[2].status,
            SolverClusterJobStatus::Completed
        );

        assert_eq!(artifacts.cases().len(), 1);
        assert_eq!(artifacts.cases()[0].node_index, 0);
        assert_eq!(artifacts.cases()[0].case_id, "case-0");
        assert_eq!(
            artifacts.cases()[0].output_kind,
            SolverCaseOutputKind::NormalizedSixCoefficientsCsv
        );
        assert_eq!(
            artifacts.cases()[0].output_file,
            "cases/case-0/solver_six_coefficients.csv"
        );
        assert_eq!(artifacts.cases()[0].output_len, output_bytes.len());
        assert_eq!(artifacts.cases()[0].output_sha256, sha256_hex(output_bytes));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_cluster_schedule_poll_and_artifact_collection_fail_closed() {
        assert!(matches!(
            SolverClusterPollingPolicy::new(0, 0, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_max_polls"
            })
        ));
        assert!(matches!(
            SolverClusterPollingPolicy::new(MAX_SOLVER_CLUSTER_POLL_COUNT + 1, 0, 5),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_max_polls"
            })
        ));
        assert!(matches!(
            SolverClusterPollingPolicy::new(1, 0, 0),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_query_timeout_s"
            })
        ));

        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let output_bytes = b"cluster solver output";
        let toml = solver_command_plan_toml(&matrix).replacen(
            &format!("{:064x}", 1),
            &sha256_hex(output_bytes),
            1,
        );
        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();
        let template = SolverClusterScheduleTemplate::slurm("openbmp-aero", 3600);
        let schedule = plan.render_cluster_schedule(&template).unwrap();
        let root = temp_aerodb_dir("solver_cluster_poll_fail");

        std::fs::write(root.join("fake_running.py"), "print('RUNNING')").unwrap();
        let running_query = vec!["python3".to_owned(), "fake_running.py".to_owned()];
        assert!(matches!(
            schedule.wait_for_completion_with_root(
                &root,
                "4242",
                &running_query,
                SolverClusterPollingPolicy::new(2, 0, 5).unwrap(),
            ),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_exhausted"
            })
        ));

        std::fs::write(root.join("fake_failed.py"), "print('FAILED')").unwrap();
        let failed_query = vec!["python3".to_owned(), "fake_failed.py".to_owned()];
        assert!(matches!(
            schedule.wait_for_completion_with_root(
                &root,
                "4242",
                &failed_query,
                SolverClusterPollingPolicy::new(2, 0, 5).unwrap(),
            ),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_poll_failed"
            })
        ));

        assert!(matches!(
            plan.collect_expected_outputs_with_root(&root),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_artifact_output_file"
            })
        ));

        std::fs::create_dir_all(root.join("cases/case-0")).unwrap();
        std::fs::write(
            root.join("cases/case-0/solver_six_coefficients.csv"),
            b"wrong cluster output",
        )
        .unwrap();
        assert!(matches!(
            plan.collect_expected_outputs_with_root(&root),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_artifact_output_sha256"
            })
        ));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_remote_artifact_transfer_stages_and_verifies_expected_outputs() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let output_bytes = b"remote staged solver output";
        let toml = solver_command_plan_toml(&matrix).replacen(
            &format!("{:064x}", 1),
            &sha256_hex(output_bytes),
            1,
        );
        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();
        let root = temp_aerodb_dir("solver_remote_artifact_transfer_success");
        std::fs::write(
            root.join("fake_transfer.py"),
            "from pathlib import Path\np=Path('cases/case-0')\np.mkdir(parents=True, exist_ok=True)\np.joinpath('solver_six_coefficients.csv').write_bytes(b'remote staged solver output')\nprint('staged case-0')\n",
        )
        .unwrap();
        let transfer =
            SolverRemoteArtifactTransferPlan::direct_command(["python3", "fake_transfer.py"], 5);

        let report = plan
            .transfer_expected_outputs_with_root(&root, &transfer)
            .unwrap();

        assert_eq!(
            report.command,
            vec!["python3".to_owned(), "fake_transfer.py".to_owned()]
        );
        assert_eq!(report.stdout_len, "staged case-0\n".len());
        assert_eq!(report.stdout_sha256, sha256_hex(b"staged case-0\n"));
        assert_eq!(report.stderr_len, 0);
        assert_eq!(report.stderr_sha256, sha256_hex(b""));
        assert_eq!(report.artifacts().cases().len(), 1);
        assert_eq!(report.artifacts().cases()[0].node_index, 0);
        assert_eq!(report.artifacts().cases()[0].case_id, "case-0");
        assert_eq!(
            report.artifacts().cases()[0].output_file,
            "cases/case-0/solver_six_coefficients.csv"
        );
        assert_eq!(report.artifacts().cases()[0].output_len, output_bytes.len());
        assert_eq!(
            report.artifacts().cases()[0].output_sha256,
            sha256_hex(output_bytes)
        );

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn solver_remote_artifact_transfer_rejects_command_timeout_status_and_hash_failures() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let output_bytes = b"remote staged solver output";
        let toml = solver_command_plan_toml(&matrix).replacen(
            &format!("{:064x}", 1),
            &sha256_hex(output_bytes),
            1,
        );
        let plan = SolverCommandPlan::load_toml_str(&matrix, &toml).unwrap();
        let root = temp_aerodb_dir("solver_remote_artifact_transfer_fail");

        let empty_transfer =
            SolverRemoteArtifactTransferPlan::direct_command(Vec::<String>::new(), 5);
        assert!(matches!(
            plan.transfer_expected_outputs_with_root(&root, &empty_transfer),
            Err(AerodbError::InvalidInput {
                field: "solver_remote_artifact_transfer_command"
            })
        ));

        let timeout_transfer =
            SolverRemoteArtifactTransferPlan::direct_command(["python3", "missing.py"], 0);
        assert!(matches!(
            plan.transfer_expected_outputs_with_root(&root, &timeout_transfer),
            Err(AerodbError::InvalidInput {
                field: "solver_remote_artifact_transfer_timeout_s"
            })
        ));

        std::fs::write(root.join("fake_status.py"), "import sys\nsys.exit(7)\n").unwrap();
        let status_transfer =
            SolverRemoteArtifactTransferPlan::direct_command(["python3", "fake_status.py"], 5);
        assert!(matches!(
            plan.transfer_expected_outputs_with_root(&root, &status_transfer),
            Err(AerodbError::InvalidInput {
                field: "solver_remote_artifact_transfer_status"
            })
        ));

        std::fs::write(
            root.join("fake_bad_hash.py"),
            "from pathlib import Path\np=Path('cases/case-0')\np.mkdir(parents=True, exist_ok=True)\np.joinpath('solver_six_coefficients.csv').write_bytes(b'wrong remote output')\n",
        )
        .unwrap();
        let hash_transfer =
            SolverRemoteArtifactTransferPlan::direct_command(["python3", "fake_bad_hash.py"], 5);
        assert!(matches!(
            plan.transfer_expected_outputs_with_root(&root, &hash_transfer),
            Err(AerodbError::InvalidInput {
                field: "solver_cluster_artifact_output_sha256"
            })
        ));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn su2_euler_case_generation_renders_deterministic_configs() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mut template = Su2EulerCaseTemplate::new(
            "mesh/benchmark.su2",
            12.5,
            3.25,
            "synthetic SU2-Euler case generation",
        );
        template.moment_reference_m = Vector3::new(1.0, -0.5, 0.25);
        template
            .extra_config
            .insert("CFL_NUMBER".into(), "5.0".into());
        template.extra_args.push("--no-color".into());
        template.env.insert("OMP_NUM_THREADS".into(), "2".into());

        let bundle = template.render_cases(&matrix).unwrap();
        let repeated = template.render_cases(&matrix).unwrap();

        assert_eq!(bundle, repeated);
        assert_eq!(bundle.cases().len(), matrix.nodes().len());
        let first = &bundle.cases()[0];
        assert_eq!(first.node_index, 0);
        assert_eq!(first.node, matrix.nodes()[0]);
        assert_eq!(first.solver, "su2-euler");
        assert_eq!(first.case_id, "case-0");
        assert_eq!(first.working_dir, "cases/case-0");
        assert_eq!(first.config_file, "cases/case-0/case.cfg");
        assert_eq!(first.history_file, "cases/case-0/history.csv");
        assert_eq!(first.surface_file, "cases/case-0/surface.csv");
        assert_eq!(first.args.as_slice(), ["case.cfg", "--no-color"]);
        assert_eq!(
            first.env.get("OMP_NUM_THREADS").map(String::as_str),
            Some("2")
        );
        assert_eq!(
            first.config_sha256,
            sha256_hex(first.config_text.as_bytes())
        );
        assert!(first.config_text.contains("SOLVER= EULER\n"));
        assert!(
            first
                .config_text
                .contains("MESH_FILENAME= mesh/benchmark.su2\n")
        );
        assert!(first.config_text.contains(&format!(
            "MACH_NUMBER= {}\n",
            format_su2_float(matrix.nodes()[0].mach)
        )));
        assert!(first.config_text.contains(&format!(
            "AOA= {}\n",
            format_su2_float(matrix.nodes()[0].alpha_deg)
        )));
        assert!(first.config_text.contains("MARKER_EULER= ( body )\n"));
        assert!(first.config_text.contains("CFL_NUMBER= 5.0\n"));
        assert!(first.config_text.contains("% knudsen= "));
        assert!(bundle.cases()[7].config_file.ends_with("case-7/case.cfg"));
    }

    #[test]
    fn su2_euler_case_generation_materializes_configs_under_root() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let template = Su2EulerCaseTemplate::new(
            "mesh/benchmark.su2",
            12.5,
            3.25,
            "synthetic SU2-Euler materialization",
        );
        let bundle = template.render_cases(&matrix).unwrap();
        let root = temp_aerodb_dir("su2_case_generation");

        bundle.write_configs_to_root(&root).unwrap();

        let config_path = root.join(&bundle.cases()[0].config_file);
        let written = std::fs::read_to_string(&config_path).unwrap();
        assert_eq!(written, bundle.cases()[0].config_text);
        assert!(root.join("cases/case-0").is_dir());

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn su2_euler_case_generation_rejects_invalid_template_fields() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let base = Su2EulerCaseTemplate::new(
            "mesh/benchmark.su2",
            12.5,
            3.25,
            "synthetic SU2-Euler validation",
        );

        let mut bad_mesh = base.clone();
        bad_mesh.mesh_filename = "../mesh.su2".into();
        assert!(matches!(
            bad_mesh.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_mesh_filename"
            })
        ));

        let mut bad_case_prefix = base.clone();
        bad_case_prefix.case_id_prefix = "bad prefix".into();
        assert!(matches!(
            bad_case_prefix.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_case_id_prefix"
            })
        ));

        let mut bad_filename = base.clone();
        bad_filename.config_filename = "nested/case.cfg".into();
        assert!(matches!(
            bad_filename.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_config_filename"
            })
        ));

        let mut bad_reference = base.clone();
        bad_reference.reference_area_m2 = 0.0;
        assert!(matches!(
            bad_reference.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_reference_area_m2"
            })
        ));

        let mut bad_timeout = base.clone();
        bad_timeout.timeout_s = 0;
        assert!(matches!(
            bad_timeout.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_timeout_s"
            })
        ));

        let mut protected_key = base.clone();
        protected_key
            .extra_config
            .insert("MACH_NUMBER".into(), "9.0".into());
        assert!(matches!(
            protected_key.render_cases(&matrix),
            Err(AerodbError::InvalidInput {
                field: "su2_euler_extra_config_key"
            })
        ));
    }

    #[test]
    fn solver_coefficient_csv_ingestion_bakes_reduced_deck() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let deck = builder
            .bake_solver_coefficient_csv_deck(&matrix, &solver_coefficient_csv(&matrix))
            .unwrap();

        assert_eq!(deck.mach_grid(), matrix.axes().mach.as_slice());
        assert_eq!(deck.alpha_grid_deg(), matrix.axes().alpha_deg.as_slice());
        assert_eq!(deck.beta_grid_deg(), matrix.axes().beta_deg.as_slice());
        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.47, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, -0.057, epsilon = 1e-15);
    }

    #[test]
    fn solver_coefficient_csv_ingestion_rejects_header_node_and_condition_mismatch() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let csv = solver_coefficient_csv(&matrix);
        let bad_header = csv.replacen("schema,node_index", "node_index,schema", 1);
        assert!(matches!(
            builder.bake_solver_coefficient_csv_deck(&matrix, &bad_header),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let bad_index = csv.replacen(
            &format!("{SOLVER_COEFFICIENT_CSV_SCHEMA_V1},1,"),
            &format!("{SOLVER_COEFFICIENT_CSV_SCHEMA_V1},7,"),
            1,
        );
        assert!(matches!(
            builder.bake_solver_coefficient_csv_deck(&matrix, &bad_index),
            Err(AerodbError::InvalidInput {
                field: "solver_coefficient_csv_node_index"
            })
        ));

        let bad_condition = csv.replacen(
            &format!("{SOLVER_COEFFICIENT_CSV_SCHEMA_V1},0,2,"),
            &format!("{SOLVER_COEFFICIENT_CSV_SCHEMA_V1},0,4,"),
            1,
        );
        assert!(matches!(
            builder.bake_solver_coefficient_csv_deck(&matrix, &bad_condition),
            Err(AerodbError::InvalidInput {
                field: "solver_coefficient_csv_mach"
            })
        ));
    }

    #[test]
    fn solver_six_coefficient_csv_ingestion_bakes_six_deck() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let deck = builder
            .bake_solver_six_coefficient_csv_deck(&matrix, &solver_six_coefficient_csv(&matrix))
            .unwrap();

        assert!(deck.has_six_coefficient_tables());
        let deflections = std::collections::BTreeMap::new();
        let six = deck
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let reduced = deck.lookup(4.0, 5.0, 3.0, &deflections).unwrap();

        assert_relative_eq!(six.cx, 0.14, epsilon = 1e-15);
        assert_relative_eq!(six.cy, -0.197, epsilon = 1e-15);
        assert_relative_eq!(six.cz, 0.31, epsilon = 1e-15);
        assert_relative_eq!(six.cl, -0.005, epsilon = 1e-15);
        assert_relative_eq!(six.cm, 0.024, epsilon = 1e-15);
        assert_relative_eq!(six.cn, -0.027, epsilon = 1e-15);
        assert_relative_eq!(reduced.cn, -0.31, epsilon = 1e-15);
        assert_relative_eq!(reduced.cd, -0.14, epsilon = 1e-15);
        assert_relative_eq!(reduced.cm, 0.024, epsilon = 1e-15);
    }

    #[test]
    fn solver_six_coefficient_csv_ingestion_rejects_header_node_condition_and_numeric_mismatch() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let csv = solver_six_coefficient_csv(&matrix);
        let bad_header = csv.replacen("schema,node_index", "node_index,schema", 1);
        assert!(matches!(
            builder.bake_solver_six_coefficient_csv_deck(&matrix, &bad_header),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let bad_index = csv.replacen(
            &format!("{SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1},1,"),
            &format!("{SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1},7,"),
            1,
        );
        assert!(matches!(
            builder.bake_solver_six_coefficient_csv_deck(&matrix, &bad_index),
            Err(AerodbError::InvalidInput {
                field: "solver_six_coefficient_csv_node_index"
            })
        ));

        let bad_condition = csv.replacen(
            &format!("{SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1},0,2,"),
            &format!("{SOLVER_SIX_COEFFICIENT_CSV_SCHEMA_V1},0,4,"),
            1,
        );
        assert!(matches!(
            builder.bake_solver_six_coefficient_csv_deck(&matrix, &bad_condition),
            Err(AerodbError::InvalidInput {
                field: "solver_six_coefficient_csv_mach"
            })
        ));

        let bad_numeric = csv.replacen(",0.120000,", ",NaN,", 1);
        assert!(matches!(
            builder.bake_solver_six_coefficient_csv_deck(&matrix, &bad_numeric),
            Err(AerodbError::InvalidInput {
                field: "solver_six_coefficient_csv_cx"
            })
        ));
    }

    #[test]
    fn surface_cp_csv_ingestion_bakes_reduced_deck_from_panel_pressures() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();

        let deck = builder
            .bake_surface_cp_csv_deck(
                &matrix,
                &mesh,
                &surface_cp_csv(&matrix, &mesh),
                Vector3::zeros(),
            )
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);
    }

    #[test]
    fn surface_cp_csv_integration_reports_six_coefficients_and_axial_bins() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();

        let reports = builder
            .integrate_surface_cp_csv_loads(
                &matrix,
                &mesh,
                &surface_cp_csv(&matrix, &mesh),
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();

        assert_eq!(reports.len(), matrix.nodes().len());
        let report = &reports[0];
        assert_eq!(report.node_index, 0);
        assert_relative_eq!(report.force_n_body.x, 0.0, epsilon = 1e-15);
        assert_relative_eq!(report.force_n_body.y, 0.0, epsilon = 1e-15);
        assert_relative_eq!(report.force_n_body.z, -20.0, epsilon = 1e-15);
        assert_relative_eq!(report.force_coefficient_body.z, -0.2, epsilon = 1e-15);
        assert_relative_eq!(report.moment_coefficient_body.x, -0.1, epsilon = 1e-15);
        assert_relative_eq!(report.moment_coefficient_body.y, 0.1, epsilon = 1e-15);
        assert_relative_eq!(report.reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_relative_eq!(report.reduced_coefficients.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(report.reduced_coefficients.cm, 0.1, epsilon = 1e-15);
        assert_eq!(report.axial_line_loads.len(), 2);
        assert_relative_eq!(
            report.axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            report.axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );

        assert!(matches!(
            builder.integrate_surface_cp_csv_loads(
                &matrix,
                &mesh,
                &surface_cp_csv(&matrix, &mesh),
                Vector3::zeros(),
                &[0.0, 0.0],
            ),
            Err(AerodbError::InvalidInput {
                field: "surface_cp_axial_stations_m"
            })
        ));
    }

    #[test]
    fn surface_cp_csv_ingestion_rejects_header_panel_condition_and_row_count() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let csv = surface_cp_csv(&matrix, &mesh);
        let bad_header = csv.replacen("schema,node_index", "node_index,schema", 1);
        assert!(matches!(
            builder.bake_surface_cp_csv_deck(&matrix, &mesh, &bad_header, Vector3::zeros()),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let bad_panel_index = csv.replacen(
            &format!("{SURFACE_CP_CSV_SCHEMA_V1},0,1,"),
            &format!("{SURFACE_CP_CSV_SCHEMA_V1},0,7,"),
            1,
        );
        assert!(matches!(
            builder.bake_surface_cp_csv_deck(&matrix, &mesh, &bad_panel_index, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "surface_cp_csv_panel_index"
            })
        ));

        let bad_condition = csv.replacen(
            &format!("{SURFACE_CP_CSV_SCHEMA_V1},0,0,2,"),
            &format!("{SURFACE_CP_CSV_SCHEMA_V1},0,0,4,"),
            1,
        );
        assert!(matches!(
            builder.bake_surface_cp_csv_deck(&matrix, &mesh, &bad_condition, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "surface_cp_csv_mach"
            })
        ));

        let missing_last_row = csv
            .lines()
            .take(csv.lines().count() - 1)
            .collect::<Vec<_>>()
            .join("\n");
        assert!(matches!(
            builder.bake_surface_cp_csv_deck(&matrix, &mesh, &missing_last_row, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "surface_cp_csv_rows"
            })
        ));
    }

    #[test]
    fn su2_surface_cp_csv_ingestion_bakes_deck_and_reports_loads() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = su2_surface_cp_texts(&matrix, &mesh);
        let surfaces = su2_surface_cp_cases(&texts);

        let deck = builder
            .bake_su2_surface_cp_csv_deck(&matrix, &mesh, &surfaces, Vector3::zeros())
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);

        let reports = builder
            .integrate_su2_surface_cp_csv_loads(
                &matrix,
                &mesh,
                &surfaces,
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();
        assert_eq!(reports[0].node_index, 0);
        assert_relative_eq!(reports[0].reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_eq!(reports[0].axial_line_loads.len(), 2);
        assert_relative_eq!(
            reports[0].axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            reports[0].axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn su2_surface_cp_csv_ingestion_rejects_node_columns_and_vertex_mapping() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = su2_surface_cp_texts(&matrix, &mesh);
        let mut surfaces = su2_surface_cp_cases(&texts);
        surfaces[1].node_index = 7;
        assert!(matches!(
            builder.bake_su2_surface_cp_csv_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_node_index"
            })
        ));

        let mut missing_cp = su2_surface_cp_texts(&matrix, &mesh);
        missing_cp[0] = missing_cp[0].replacen("Pressure_Coefficient", "Pressure", 1);
        let surfaces = su2_surface_cp_cases(&missing_cp);
        assert!(matches!(
            builder.bake_su2_surface_cp_csv_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let mut duplicate_vertex = su2_surface_cp_texts(&matrix, &mesh);
        let first = mesh.vertices()[0];
        let second = mesh.vertices()[1];
        duplicate_vertex[0] = duplicate_vertex[0].replacen(
            &format!("{},{},{},0.2", second.x, second.y, second.z),
            &format!("{},{},{},0.2", first.x, first.y, first.z),
            1,
        );
        let surfaces = su2_surface_cp_cases(&duplicate_vertex);
        assert!(matches!(
            builder.bake_su2_surface_cp_csv_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "su2_surface_cp_duplicate_vertex"
            })
        ));
    }

    #[test]
    fn cart3d_triq_surface_cp_ingestion_bakes_deck_and_reports_loads() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = cart3d_triq_texts(&matrix, &mesh);
        let surfaces = cart3d_triq_cases(&texts);

        let deck = builder
            .bake_cart3d_triq_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros())
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);

        let reports = builder
            .integrate_cart3d_triq_surface_cp_loads(
                &matrix,
                &mesh,
                &surfaces,
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();
        assert_eq!(reports[0].node_index, 0);
        assert_relative_eq!(reports[0].reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_eq!(reports[0].axial_line_loads.len(), 2);
        assert_relative_eq!(
            reports[0].axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            reports[0].axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn cart3d_triq_surface_cp_ingestion_rejects_node_geometry_and_scalar_rows() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = cart3d_triq_texts(&matrix, &mesh);
        let mut surfaces = cart3d_triq_cases(&texts);
        surfaces[1].node_index = 7;
        assert!(matches!(
            builder.bake_cart3d_triq_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_node_index"
            })
        ));

        let mut no_scalars = cart3d_triq_texts(&matrix, &mesh);
        no_scalars[0] = no_scalars[0].replacen("4 2 1", "4 2 0", 1);
        let surfaces = cart3d_triq_cases(&no_scalars);
        assert!(matches!(
            builder.bake_cart3d_triq_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_scalars"
            })
        ));

        let mut face_mismatch = cart3d_triq_texts(&matrix, &mesh);
        face_mismatch[0] = face_mismatch[0].replacen("2 4 1", "2 4 4", 1);
        let surfaces = cart3d_triq_cases(&face_mismatch);
        assert!(matches!(
            builder.bake_cart3d_triq_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "cart3d_triq_surface_cp_faces"
            })
        ));

        let mut missing_cp = cart3d_triq_texts(&matrix, &mesh);
        missing_cp[0] = missing_cp[0]
            .lines()
            .take(missing_cp[0].lines().count() - 1)
            .collect::<Vec<_>>()
            .join("\n");
        let surfaces = cart3d_triq_cases(&missing_cp);
        assert!(matches!(
            builder.bake_cart3d_triq_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::MalformedCsv { .. })
        ));
    }

    #[test]
    fn openfoam_surface_cp_ingestion_bakes_deck_and_reports_loads() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = openfoam_surface_cp_texts(&matrix, &mesh);
        let surfaces = openfoam_surface_cp_cases(&texts);

        let deck = builder
            .bake_openfoam_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros())
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);

        let reports = builder
            .integrate_openfoam_surface_cp_loads(
                &matrix,
                &mesh,
                &surfaces,
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();
        assert_eq!(reports[0].node_index, 0);
        assert_relative_eq!(reports[0].reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_eq!(reports[0].axial_line_loads.len(), 2);
        assert_relative_eq!(
            reports[0].axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            reports[0].axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn openfoam_surface_cp_ingestion_rejects_node_rows_and_vertex_mapping() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = openfoam_surface_cp_texts(&matrix, &mesh);
        let mut surfaces = openfoam_surface_cp_cases(&texts);
        surfaces[1].node_index = 7;
        assert!(matches!(
            builder.bake_openfoam_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_node_index"
            })
        ));

        let mut malformed_row = openfoam_surface_cp_texts(&matrix, &mesh);
        malformed_row[0] = malformed_row[0].replacen("0.2;", "0.2 99.0;", 1);
        let surfaces = openfoam_surface_cp_cases(&malformed_row);
        assert!(matches!(
            builder.bake_openfoam_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let mut duplicate_vertex = openfoam_surface_cp_texts(&matrix, &mesh);
        let first = mesh.vertices()[0];
        let second = mesh.vertices()[1];
        duplicate_vertex[0] = duplicate_vertex[0].replacen(
            &format!("({} {} {})", second.x, second.y, second.z),
            &format!("({} {} {})", first.x, first.y, first.z),
            1,
        );
        let surfaces = openfoam_surface_cp_cases(&duplicate_vertex);
        assert!(matches!(
            builder.bake_openfoam_surface_cp_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "openfoam_surface_cp_duplicate_vertex"
            })
        ));
    }

    #[test]
    fn openfoam_case_field_parser_reads_scalar_dictionary_and_boundary_values() {
        let text = openfoam_boundary_cp_field_text(0.2, 2);
        let field = OpenFoamScalarField::parse_str(&text).unwrap();

        assert_eq!(field.class_name, "volScalarField");
        assert_eq!(field.object_name, "Cp");
        assert_eq!(field.dimensions, [0.0; 7]);
        assert_eq!(field.internal_field, OpenFoamScalarFieldValue::Uniform(0.0));
        let body = field.patch("body").unwrap();
        assert_eq!(body.patch_type, "fixedValue");
        assert_eq!(
            body.value,
            Some(OpenFoamScalarFieldValue::Nonuniform(vec![0.2, 0.2]))
        );
        let freestream = field.patch("freestream").unwrap();
        assert_eq!(
            freestream.value,
            Some(OpenFoamScalarFieldValue::Uniform(-0.1))
        );
        assert!(field.patch("outlet").unwrap().value.is_none());
    }

    #[test]
    fn openfoam_case_field_parser_reads_vector_dictionary_and_case_field_set() {
        let vector_text = openfoam_vector_field_text();
        let vector_field = OpenFoamVectorField::parse_str(&vector_text).unwrap();

        assert_eq!(vector_field.class_name, "volVectorField");
        assert_eq!(vector_field.object_name, "U");
        assert_eq!(
            vector_field.dimensions,
            [0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
        );
        assert_eq!(
            vector_field.internal_field,
            OpenFoamVectorFieldValue::Uniform(Vector3::new(10.0, 0.0, 0.0))
        );
        let inlet = vector_field.patch("inlet").unwrap();
        assert_eq!(inlet.patch_type, "fixedValue");
        assert_eq!(
            inlet.value,
            Some(OpenFoamVectorFieldValue::Uniform(Vector3::new(
                10.0, 1.0, 0.0
            )))
        );
        let body = vector_field.patch("body").unwrap();
        assert_eq!(body.patch_type, "noSlip");
        assert_eq!(
            body.value,
            Some(OpenFoamVectorFieldValue::Nonuniform(vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.5, 0.0, 0.0),
            ]))
        );
        assert!(vector_field.patch("outlet").unwrap().value.is_none());

        let nut_text = openfoam_turbulence_scalar_field_text("nut", 1.0e-5);
        let fields = OpenFoamRecordedCaseFields::from_field_texts(
            &[("nut", nut_text.as_str())],
            &[("U", vector_text.as_str())],
        )
        .unwrap();

        assert!(fields.scalar_fields.contains_key("nut"));
        assert!(fields.vector_fields.contains_key("U"));
    }

    #[test]
    fn openfoam_case_field_loader_reads_recorded_case_directory_fields() {
        let root = temp_aerodb_dir("openfoam_case_fields_success");
        std::fs::create_dir_all(root.join("0")).unwrap();
        std::fs::write(
            root.join("0/nut"),
            openfoam_turbulence_scalar_field_text("nut", 1.0e-5),
        )
        .unwrap();
        std::fs::write(root.join("0/U"), openfoam_vector_field_text()).unwrap();

        let fields = OpenFoamRecordedCaseFields::from_case_root(
            &root,
            &[OpenFoamCaseFieldFile {
                object_name: "nut",
                relative_path: "0/nut",
            }],
            &[OpenFoamCaseFieldFile {
                object_name: "U",
                relative_path: "0/U",
            }],
        )
        .unwrap();

        assert_eq!(fields.scalar_fields["nut"].object_name, "nut");
        assert_eq!(fields.vector_fields["U"].object_name, "U");

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_case_field_discovery_reads_time_directory_fields() {
        let root = temp_aerodb_dir("openfoam_case_field_discovery_success");
        std::fs::create_dir_all(root.join("0/uniform")).unwrap();
        std::fs::write(root.join("0/U"), openfoam_vector_field_text()).unwrap();
        std::fs::write(
            root.join("0/nut"),
            openfoam_turbulence_scalar_field_text("nut", 1.0e-5),
        )
        .unwrap();

        let fields = OpenFoamRecordedCaseFields::discover_time_directory(&root, "0").unwrap();

        assert_eq!(fields.scalar_fields.keys().collect::<Vec<_>>(), vec!["nut"]);
        assert_eq!(fields.vector_fields.keys().collect::<Vec<_>>(), vec!["U"]);

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_case_field_discovery_rejects_bad_time_directory_inputs() {
        let root = temp_aerodb_dir("openfoam_case_field_discovery_reject");
        std::fs::create_dir_all(root.join("0")).unwrap();

        assert!(matches!(
            OpenFoamRecordedCaseFields::discover_time_directory(&root, "../0"),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_time_dir"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::discover_time_directory(&root, "1"),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_time_dir"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::discover_time_directory(&root, "0"),
            Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_fields"
            })
        ));

        std::fs::write(root.join("0/tau"), openfoam_tensor_field_text()).unwrap();
        assert!(matches!(
            OpenFoamRecordedCaseFields::discover_time_directory(&root, "0"),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_field_class"
            })
        ));
        std::fs::remove_file(root.join("0/tau")).unwrap();

        std::fs::write(
            root.join("0/k"),
            openfoam_turbulence_scalar_field_text("nut", 1.0e-5),
        )
        .unwrap();
        assert!(matches!(
            OpenFoamRecordedCaseFields::discover_time_directory(&root, "0"),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_field_file_object"
            })
        ));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_poly_mesh_boundary_parser_reads_topology_and_validates_fields() {
        let boundary =
            OpenFoamPolyMeshBoundary::parse_str(&openfoam_poly_mesh_boundary_text()).unwrap();

        assert_eq!(boundary.class_name, "polyBoundaryMesh");
        assert_eq!(boundary.object_name, "boundary");
        assert_eq!(boundary.patch("body").unwrap().patch_type, "wall");
        assert_eq!(boundary.patch("body").unwrap().n_faces, 12);
        assert_eq!(boundary.patch("body").unwrap().start_face, 108);

        let root = temp_aerodb_dir("openfoam_poly_mesh_boundary_success");
        std::fs::create_dir_all(root.join("constant/polyMesh")).unwrap();
        std::fs::create_dir_all(root.join("0")).unwrap();
        std::fs::write(
            root.join("constant/polyMesh/boundary"),
            openfoam_poly_mesh_boundary_text(),
        )
        .unwrap();
        std::fs::write(root.join("0/U"), openfoam_vector_field_text()).unwrap();
        std::fs::write(
            root.join("0/nut"),
            openfoam_turbulence_scalar_field_text("nut", 1.0e-5),
        )
        .unwrap();

        let loaded_boundary = OpenFoamPolyMeshBoundary::from_case_root(&root).unwrap();
        let fields = OpenFoamRecordedCaseFields::discover_time_directory(&root, "0").unwrap();
        loaded_boundary.validate_field_patches(&fields).unwrap();

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_poly_mesh_boundary_parser_rejects_bad_topology_and_patch_mismatches() {
        let duplicate_patch = openfoam_poly_mesh_boundary_text().replacen("outlet", "body", 1);
        assert!(matches!(
            OpenFoamPolyMeshBoundary::parse_str(&duplicate_patch),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_duplicate_patch"
            })
        ));

        let overlapping_range = openfoam_poly_mesh_boundary_text().replacen(
            "nFaces 4;\n        startFace 120;",
            "nFaces 4;\n        startFace 110;",
            1,
        );
        assert!(matches!(
            OpenFoamPolyMeshBoundary::parse_str(&overlapping_range),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_face_range"
            })
        ));

        let missing_count = openfoam_poly_mesh_boundary_text().replacen("nFaces 12;", "", 1);
        assert!(matches!(
            OpenFoamPolyMeshBoundary::parse_str(&missing_count),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_n_faces"
            })
        ));

        let boundary_without_freestream =
            openfoam_poly_mesh_boundary_text().replace("freestream", "farfield");
        let boundary = OpenFoamPolyMeshBoundary::parse_str(&boundary_without_freestream).unwrap();
        let nut_text = openfoam_turbulence_scalar_field_text("nut", 1.0e-5);
        let fields =
            OpenFoamRecordedCaseFields::from_field_texts(&[("nut", nut_text.as_str())], &[])
                .unwrap();
        assert!(matches!(
            boundary.validate_field_patches(&fields),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_field_patch"
            })
        ));
    }

    #[test]
    fn openfoam_poly_mesh_connectivity_parser_reads_lists_and_validates_boundary_ranges() {
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_points_text(),
            &openfoam_poly_mesh_faces_text(),
            &openfoam_poly_mesh_owner_text(),
            &openfoam_poly_mesh_neighbour_text(),
        )
        .unwrap();

        assert_eq!(connectivity.points.len(), 5);
        assert_eq!(connectivity.faces.len(), 6);
        assert_eq!(connectivity.faces[4], vec![0, 1, 2, 3]);
        assert_eq!(connectivity.owner.len(), connectivity.faces.len());
        assert_eq!(connectivity.neighbour, vec![1, 0]);

        let boundary =
            OpenFoamPolyMeshBoundary::parse_str(&openfoam_poly_mesh_connectivity_boundary_text())
                .unwrap();
        connectivity.validate_boundary_ranges(&boundary).unwrap();

        let root = temp_aerodb_dir("openfoam_poly_mesh_connectivity_success");
        std::fs::create_dir_all(root.join("constant/polyMesh")).unwrap();
        std::fs::write(
            root.join("constant/polyMesh/points"),
            openfoam_poly_mesh_points_text(),
        )
        .unwrap();
        std::fs::write(
            root.join("constant/polyMesh/faces"),
            openfoam_poly_mesh_faces_text(),
        )
        .unwrap();
        std::fs::write(
            root.join("constant/polyMesh/owner"),
            openfoam_poly_mesh_owner_text(),
        )
        .unwrap();
        std::fs::write(
            root.join("constant/polyMesh/neighbour"),
            openfoam_poly_mesh_neighbour_text(),
        )
        .unwrap();

        let loaded = OpenFoamPolyMeshConnectivity::from_case_root(&root).unwrap();
        assert_eq!(loaded, connectivity);

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_poly_mesh_connectivity_parser_rejects_bad_indices_counts_and_ranges() {
        let bad_vertex = openfoam_poly_mesh_faces_text().replacen("3(0 2 4)", "3(0 2 99)", 1);
        assert!(matches!(
            OpenFoamPolyMeshConnectivity::from_texts(
                &openfoam_poly_mesh_points_text(),
                &bad_vertex,
                &openfoam_poly_mesh_owner_text(),
                &openfoam_poly_mesh_neighbour_text(),
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_vertex"
            })
        ));

        let short_face = openfoam_poly_mesh_faces_text().replacen("3(0 1 4)", "2(0 1)", 1);
        assert!(matches!(
            OpenFoamPolyMeshConnectivity::from_texts(
                &openfoam_poly_mesh_points_text(),
                &short_face,
                &openfoam_poly_mesh_owner_text(),
                &openfoam_poly_mesh_neighbour_text(),
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_vertices"
            })
        ));

        let bad_owner_count = openfoam_poly_mesh_owner_text().replacen("\n6\n", "\n5\n", 1);
        assert!(matches!(
            OpenFoamPolyMeshConnectivity::from_texts(
                &openfoam_poly_mesh_points_text(),
                &openfoam_poly_mesh_faces_text(),
                &bad_owner_count,
                &openfoam_poly_mesh_neighbour_text(),
            ),
            Err(AerodbError::MalformedOpenFoam { .. })
        ));

        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_points_text(),
            &openfoam_poly_mesh_faces_text(),
            &openfoam_poly_mesh_owner_text(),
            &openfoam_poly_mesh_neighbour_text(),
        )
        .unwrap();

        let internal_boundary = openfoam_poly_mesh_connectivity_boundary_text().replacen(
            "startFace 2;",
            "startFace 1;",
            1,
        );
        let boundary = OpenFoamPolyMeshBoundary::parse_str(&internal_boundary).unwrap();
        assert!(matches!(
            connectivity.validate_boundary_ranges(&boundary),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_internal_faces"
            })
        ));

        let out_of_range_boundary = openfoam_poly_mesh_connectivity_boundary_text().replacen(
            "outlet\n    {\n        type patch;\n        nFaces 2;\n        startFace 4;",
            "outlet\n    {\n        type patch;\n        nFaces 4;\n        startFace 4;",
            1,
        );
        let boundary = OpenFoamPolyMeshBoundary::parse_str(&out_of_range_boundary).unwrap();
        assert!(matches!(
            connectivity.validate_boundary_ranges(&boundary),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_boundary_face_range"
            })
        ));
    }

    #[test]
    fn openfoam_poly_mesh_geometry_reconstructs_faces_and_cell_volumes() {
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_tetra_points_text(),
            &openfoam_poly_mesh_tetra_faces_text(),
            &openfoam_poly_mesh_tetra_owner_text(),
            &openfoam_poly_mesh_empty_neighbour_text(),
        )
        .unwrap();

        let geometry = connectivity.reconstruct_geometry().unwrap();
        assert_eq!(geometry.faces.len(), 4);
        assert_eq!(geometry.cell_volumes_m3.len(), 1);
        assert_relative_eq!(geometry.cell_volumes_m3[0], 1.0 / 6.0, epsilon = 1e-12);

        let slant = &geometry.faces[3];
        assert_relative_eq!(slant.centroid.x, 1.0 / 3.0, epsilon = 1e-12);
        assert_relative_eq!(slant.centroid.y, 1.0 / 3.0, epsilon = 1e-12);
        assert_relative_eq!(slant.centroid.z, 1.0 / 3.0, epsilon = 1e-12);
        assert_relative_eq!(slant.area_vector.x, 0.5, epsilon = 1e-12);
        assert_relative_eq!(slant.area_vector.y, 0.5, epsilon = 1e-12);
        assert_relative_eq!(slant.area_vector.z, 0.5, epsilon = 1e-12);
        assert_relative_eq!(slant.area_m2, 3.0_f64.sqrt() / 2.0, epsilon = 1e-12);
        assert_relative_eq!(slant.unit_normal.x, 1.0 / 3.0_f64.sqrt(), epsilon = 1e-12);
    }

    #[test]
    fn openfoam_poly_mesh_geometry_rejects_degenerate_open_and_negative_cells() {
        let degenerate_face =
            openfoam_poly_mesh_tetra_faces_text().replacen("3(0 2 1)", "3(0 0 1)", 1);
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_tetra_points_text(),
            &degenerate_face,
            &openfoam_poly_mesh_tetra_owner_text(),
            &openfoam_poly_mesh_empty_neighbour_text(),
        )
        .unwrap();
        assert!(matches!(
            connectivity.reconstruct_geometry(),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_geometry"
            })
        ));

        let open_cell = openfoam_poly_mesh_tetra_faces_text().replacen("3(1 2 3)", "3(1 3 2)", 1);
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_tetra_points_text(),
            &open_cell,
            &openfoam_poly_mesh_tetra_owner_text(),
            &openfoam_poly_mesh_empty_neighbour_text(),
        )
        .unwrap();
        assert!(matches!(
            connectivity.reconstruct_geometry(),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_cell_closed_surface"
            })
        ));

        let negative_cell = openfoam_poly_mesh_tetra_faces_text()
            .replace("3(0 2 1)", "3(0 1 2)")
            .replace("3(0 1 3)", "3(0 3 1)")
            .replace("3(0 3 2)", "3(0 2 3)")
            .replace("3(1 2 3)", "3(1 3 2)");
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_tetra_points_text(),
            &negative_cell,
            &openfoam_poly_mesh_tetra_owner_text(),
            &openfoam_poly_mesh_empty_neighbour_text(),
        )
        .unwrap();
        assert!(matches!(
            connectivity.reconstruct_geometry(),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_cell_volume"
            })
        ));
    }

    #[test]
    fn openfoam_poly_mesh_zones_parser_reads_zones_and_validates_connectivity() {
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_points_text(),
            &openfoam_poly_mesh_faces_text(),
            &openfoam_poly_mesh_owner_text(),
            &openfoam_poly_mesh_neighbour_text(),
        )
        .unwrap();
        let zones = OpenFoamPolyMeshZones::from_texts(
            Some(&openfoam_poly_mesh_point_zones_text()),
            Some(&openfoam_poly_mesh_face_zones_text()),
            Some(&openfoam_poly_mesh_cell_zones_text()),
        )
        .unwrap();

        assert_eq!(zones.point_zones["nose"].zone_type, "pointZone");
        assert_eq!(zones.point_zones["nose"].labels, vec![0, 4]);
        assert_eq!(zones.face_zones["wake"].zone_type, "faceZone");
        assert_eq!(zones.face_zones["wake"].labels, vec![2, 3]);
        assert_eq!(zones.face_zones["wake"].flip_map, Some(vec![false, true]));
        assert_eq!(zones.cell_zones["core"].labels, vec![0, 1]);
        zones.validate_connectivity(&connectivity).unwrap();

        let root = temp_aerodb_dir("openfoam_poly_mesh_zones_success");
        std::fs::create_dir_all(root.join("constant/polyMesh")).unwrap();
        std::fs::write(
            root.join("constant/polyMesh/pointZones"),
            openfoam_poly_mesh_point_zones_text(),
        )
        .unwrap();
        std::fs::write(
            root.join("constant/polyMesh/faceZones"),
            openfoam_poly_mesh_face_zones_text(),
        )
        .unwrap();
        std::fs::write(
            root.join("constant/polyMesh/cellZones"),
            openfoam_poly_mesh_cell_zones_text(),
        )
        .unwrap();

        let loaded = OpenFoamPolyMeshZones::from_case_root(&root).unwrap();
        assert_eq!(loaded, zones);

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_poly_mesh_zones_parser_rejects_bad_zone_connectivity() {
        let connectivity = OpenFoamPolyMeshConnectivity::from_texts(
            &openfoam_poly_mesh_points_text(),
            &openfoam_poly_mesh_faces_text(),
            &openfoam_poly_mesh_owner_text(),
            &openfoam_poly_mesh_neighbour_text(),
        )
        .unwrap();

        let bad_point_zone = openfoam_poly_mesh_point_zones_text()
            .replace("pointLabels 2(0 4);", "pointLabels 2(0 99);");
        let zones = OpenFoamPolyMeshZones::from_texts(Some(&bad_point_zone), None, None).unwrap();
        assert!(matches!(
            zones.validate_connectivity(&connectivity),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_point_zone_label"
            })
        ));

        let bad_face_zone = openfoam_poly_mesh_face_zones_text().replace(
            "faceLabels List<label> 2 (2 3);",
            "faceLabels List<label> 2 (2 99);",
        );
        let zones = OpenFoamPolyMeshZones::from_texts(None, Some(&bad_face_zone), None).unwrap();
        assert!(matches!(
            zones.validate_connectivity(&connectivity),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_zone_label"
            })
        ));

        let bad_cell_zone = openfoam_poly_mesh_cell_zones_text()
            .replace("cellLabels 2(0 1);", "cellLabels 2(0 2);");
        let zones = OpenFoamPolyMeshZones::from_texts(None, None, Some(&bad_cell_zone)).unwrap();
        assert!(matches!(
            zones.validate_connectivity(&connectivity),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_cell_zone_label"
            })
        ));

        let bad_flip_map = openfoam_poly_mesh_face_zones_text()
            .replace("flipMap List<bool> 2 (0 1);", "flipMap List<bool> 1 (0);");
        let zones = OpenFoamPolyMeshZones::from_texts(None, Some(&bad_flip_map), None).unwrap();
        assert!(matches!(
            zones.validate_connectivity(&connectivity),
            Err(AerodbError::InvalidInput {
                field: "openfoam_poly_mesh_face_zone_flip_map"
            })
        ));
    }

    #[test]
    fn openfoam_turbulence_properties_parser_validates_ras_field_set() {
        let properties = OpenFoamTurbulenceProperties::parse_str(
            &openfoam_turbulence_properties_ras_text("kOmegaSST"),
        )
        .unwrap();

        assert_eq!(properties.class_name, "dictionary");
        assert_eq!(properties.object_name, "turbulenceProperties");
        assert_eq!(
            properties.simulation_type,
            OpenFoamTurbulenceSimulationType::Ras
        );
        assert_eq!(properties.model_name.as_deref(), Some("kOmegaSST"));
        assert_eq!(properties.turbulence, Some(true));
        assert_eq!(properties.print_coeffs, Some(true));

        let k = openfoam_turbulence_scalar_field_text("k", 1.0);
        let omega = openfoam_turbulence_scalar_field_text("omega", 2.0);
        let nut = openfoam_turbulence_scalar_field_text("nut", 1.0e-5);
        let u = openfoam_vector_field_text();
        let fields = OpenFoamRecordedCaseFields::from_field_texts(
            &[
                ("k", k.as_str()),
                ("omega", omega.as_str()),
                ("nut", nut.as_str()),
            ],
            &[("U", u.as_str())],
        )
        .unwrap();
        properties.validate_field_set(&fields).unwrap();

        let root = temp_aerodb_dir("openfoam_turbulence_properties_success");
        std::fs::create_dir_all(root.join("constant")).unwrap();
        std::fs::write(
            root.join("constant/turbulenceProperties"),
            openfoam_turbulence_properties_ras_text("kOmegaSST"),
        )
        .unwrap();
        let loaded = OpenFoamTurbulenceProperties::from_case_root(&root).unwrap();
        assert_eq!(loaded, properties);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_turbulence_properties_parser_validates_model_coefficients() {
        let properties = OpenFoamTurbulenceProperties::parse_str(
            &openfoam_turbulence_properties_ras_coeff_text(),
        )
        .unwrap();

        assert_relative_eq!(
            properties.coefficient("betaStar").unwrap(),
            0.09,
            epsilon = 1e-15
        );
        assert_relative_eq!(properties.coefficient("a1").unwrap(), 0.31, epsilon = 1e-15);
        assert!(properties.coefficient("ignoredNested").is_none());
        properties
            .validate_coefficients(&[
                OpenFoamTurbulenceCoefficientExpectation {
                    name: "betaStar",
                    expected: 0.09,
                    tolerance: 1.0e-12,
                },
                OpenFoamTurbulenceCoefficientExpectation {
                    name: "a1",
                    expected: 0.31,
                    tolerance: 1.0e-12,
                },
            ])
            .unwrap();

        assert!(matches!(
            properties.validate_coefficients(&[OpenFoamTurbulenceCoefficientExpectation {
                name: "sigmaOmega2",
                expected: 0.856,
                tolerance: 1.0e-12,
            }]),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_coefficient"
            })
        ));
        assert!(matches!(
            properties.validate_coefficients(&[OpenFoamTurbulenceCoefficientExpectation {
                name: "betaStar",
                expected: 0.08,
                tolerance: 1.0e-12,
            }]),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_coefficient_tolerance"
            })
        ));
        assert!(matches!(
            properties.validate_coefficients(&[OpenFoamTurbulenceCoefficientExpectation {
                name: "betaStar",
                expected: 0.09,
                tolerance: -1.0,
            }]),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_coefficient_tolerance"
            })
        ));

        let duplicate = openfoam_turbulence_properties_ras_coeff_text()
            .replace("a1 0.31;", "a1 0.31;\n        a1 0.32;");
        assert!(matches!(
            OpenFoamTurbulenceProperties::parse_str(&duplicate),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_coefficient_duplicate"
            })
        ));

        let nonfinite = openfoam_turbulence_properties_ras_coeff_text()
            .replace("betaStar 0.09;", "betaStar NaN;");
        assert!(matches!(
            OpenFoamTurbulenceProperties::parse_str(&nonfinite),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_coefficient"
            })
        ));
    }

    #[test]
    fn openfoam_turbulence_properties_parser_rejects_missing_fields_and_models() {
        let properties = OpenFoamTurbulenceProperties::parse_str(
            &openfoam_turbulence_properties_ras_text("kOmegaSST"),
        )
        .unwrap();
        let k = openfoam_turbulence_scalar_field_text("k", 1.0);
        let nut = openfoam_turbulence_scalar_field_text("nut", 1.0e-5);
        let u = openfoam_vector_field_text();
        let missing_omega = OpenFoamRecordedCaseFields::from_field_texts(
            &[("k", k.as_str()), ("nut", nut.as_str())],
            &[("U", u.as_str())],
        )
        .unwrap();
        assert!(matches!(
            properties.validate_field_set(&missing_omega),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_field"
            })
        ));

        let missing_u = OpenFoamRecordedCaseFields::from_field_texts(
            &[
                ("k", k.as_str()),
                (
                    "omega",
                    openfoam_turbulence_scalar_field_text("omega", 2.0).as_str(),
                ),
                ("nut", nut.as_str()),
            ],
            &[],
        )
        .unwrap();
        assert!(matches!(
            properties.validate_field_set(&missing_u),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_velocity_field"
            })
        ));

        let unsupported = OpenFoamTurbulenceProperties::parse_str(
            &openfoam_turbulence_properties_ras_text("unknownRAS"),
        )
        .unwrap();
        let omega = openfoam_turbulence_scalar_field_text("omega", 2.0);
        let fields = OpenFoamRecordedCaseFields::from_field_texts(
            &[
                ("k", k.as_str()),
                ("omega", omega.as_str()),
                ("nut", nut.as_str()),
            ],
            &[("U", u.as_str())],
        )
        .unwrap();
        assert!(matches!(
            unsupported.validate_field_set(&fields),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_model"
            })
        ));

        let missing_ras_block = openfoam_turbulence_properties_ras_text("kOmegaSST").replace(
            r#"RAS
{
    RASModel kOmegaSST;
    turbulence on;
    printCoeffs on;
}"#,
            "",
        );
        assert!(matches!(
            OpenFoamTurbulenceProperties::parse_str(&missing_ras_block),
            Err(AerodbError::InvalidInput {
                field: "openfoam_turbulence_model"
            })
        ));

        let laminar =
            OpenFoamTurbulenceProperties::parse_str(&openfoam_turbulence_properties_laminar_text())
                .unwrap();
        let empty_fields = OpenFoamRecordedCaseFields {
            scalar_fields: BTreeMap::new(),
            vector_fields: BTreeMap::new(),
        };
        laminar.validate_field_set(&empty_fields).unwrap();
    }

    #[test]
    fn openfoam_case_field_loader_rejects_bad_roots_paths_and_objects() {
        let root = temp_aerodb_dir("openfoam_case_fields_reject");
        std::fs::create_dir_all(root.join("0")).unwrap();
        std::fs::write(
            root.join("0/nut"),
            openfoam_turbulence_scalar_field_text("nut", 1.0e-5),
        )
        .unwrap();

        assert!(matches!(
            OpenFoamRecordedCaseFields::from_case_root(
                root.join("missing"),
                &[OpenFoamCaseFieldFile {
                    object_name: "nut",
                    relative_path: "0/nut",
                }],
                &[],
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_root"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_case_root(
                &root,
                &[OpenFoamCaseFieldFile {
                    object_name: "nut",
                    relative_path: "../0/nut",
                }],
                &[],
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_field_file"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_case_root(
                &root,
                &[OpenFoamCaseFieldFile {
                    object_name: "nut",
                    relative_path: "0/missing",
                }],
                &[],
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_case_field_file_read"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_case_root(
                &root,
                &[OpenFoamCaseFieldFile {
                    object_name: "k",
                    relative_path: "0/nut",
                }],
                &[],
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_field_object"
            })
        ));

        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn openfoam_case_field_parser_rejects_vector_and_field_set_mismatches() {
        let vector_text = openfoam_vector_field_text();
        let bad_count = vector_text.replacen("\n        2\n", "\n        3\n", 1);
        assert!(matches!(
            OpenFoamVectorField::parse_str(&bad_count),
            Err(AerodbError::MalformedOpenFoam { .. })
        ));

        let bad_value = vector_text.replacen("(0.5 0 0)", "(NaN 0 0)", 1);
        assert!(matches!(
            OpenFoamVectorField::parse_str(&bad_value),
            Err(AerodbError::InvalidInput {
                field: "openfoam_vector_field_value"
            })
        ));

        let nut_text = openfoam_turbulence_scalar_field_text("nut", 1.0e-5);
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_field_texts(&[("k", nut_text.as_str())], &[]),
            Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_field_object"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_field_texts(
                &[("nut", nut_text.as_str()), ("nut", nut_text.as_str())],
                &[]
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_scalar_duplicate"
            })
        ));
        assert!(matches!(
            OpenFoamRecordedCaseFields::from_field_texts(&[], &[]),
            Err(AerodbError::InvalidInput {
                field: "openfoam_recorded_case_fields"
            })
        ));
    }

    #[test]
    fn openfoam_boundary_cp_field_ingestion_bakes_deck_and_reports_loads() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = openfoam_boundary_cp_field_texts(&matrix, &mesh);
        let fields = openfoam_boundary_cp_field_cases(&texts);

        let deck = builder
            .bake_openfoam_boundary_cp_field_deck(&matrix, &mesh, &fields, Vector3::zeros())
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);

        let reports = builder
            .integrate_openfoam_boundary_cp_field_loads(
                &matrix,
                &mesh,
                &fields,
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();
        assert_eq!(reports[0].node_index, 0);
        assert_relative_eq!(reports[0].reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_eq!(reports[0].axial_line_loads.len(), 2);
        assert_relative_eq!(
            reports[0].axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            reports[0].axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn openfoam_boundary_cp_field_ingestion_rejects_malformed_fields() {
        let matrix = RunMatrix::from_axes(single_node_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = vec![openfoam_boundary_cp_field_text(0.2, mesh.triangles().len())];

        let mut bad_patch = openfoam_boundary_cp_field_cases(&texts);
        bad_patch[0].patch_name = "missing";
        assert!(matches!(
            builder.bake_openfoam_boundary_cp_field_deck(
                &matrix,
                &mesh,
                &bad_patch,
                Vector3::zeros()
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_boundary_cp_patch_name"
            })
        ));

        let mut bad_count_texts = texts.clone();
        bad_count_texts[0] = bad_count_texts[0].replacen("\n        2\n", "\n        3\n", 1);
        let bad_count = openfoam_boundary_cp_field_cases(&bad_count_texts);
        assert!(matches!(
            builder.bake_openfoam_boundary_cp_field_deck(
                &matrix,
                &mesh,
                &bad_count,
                Vector3::zeros()
            ),
            Err(AerodbError::MalformedOpenFoam { .. })
        ));

        let mut bad_value_texts = texts.clone();
        bad_value_texts[0] = bad_value_texts[0].replacen("    0.2\n", "    NaN\n", 1);
        let bad_value = openfoam_boundary_cp_field_cases(&bad_value_texts);
        assert!(matches!(
            builder.bake_openfoam_boundary_cp_field_deck(
                &matrix,
                &mesh,
                &bad_value,
                Vector3::zeros()
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_scalar_field_value"
            })
        ));

        let mut missing_value_texts = texts.clone();
        missing_value_texts[0] = missing_value_texts[0].replace(
            r#"        value nonuniform List<scalar>
        2
        (
    0.2
    0.2
        );"#,
            "",
        );
        let missing_value = openfoam_boundary_cp_field_cases(&missing_value_texts);
        assert!(matches!(
            builder.bake_openfoam_boundary_cp_field_deck(
                &matrix,
                &mesh,
                &missing_value,
                Vector3::zeros()
            ),
            Err(AerodbError::InvalidInput {
                field: "openfoam_boundary_cp_patch_value"
            })
        ));
    }

    #[test]
    fn sparta_surface_forces_ingestion_bakes_deck_and_reports_loads() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = sparta_surface_forces_texts(&matrix, &mesh);
        let surfaces = sparta_surface_force_cases(&texts);

        let deck = builder
            .bake_sparta_surface_force_deck(&matrix, &mesh, &surfaces, Vector3::zeros())
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.0, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, 0.135, epsilon = 1e-15);

        let reports = builder
            .integrate_sparta_surface_force_loads(
                &matrix,
                &mesh,
                &surfaces,
                Vector3::zeros(),
                &[0.0, 0.5, 1.0],
            )
            .unwrap();
        assert_eq!(reports[0].node_index, 0);
        assert_relative_eq!(reports[0].force_n_body.z, -20.0, epsilon = 1e-15);
        assert_relative_eq!(reports[0].reduced_coefficients.cn, 0.2, epsilon = 1e-15);
        assert_eq!(reports[0].axial_line_loads.len(), 2);
        assert_relative_eq!(
            reports[0].axial_line_loads[0].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
        assert_relative_eq!(
            reports[0].axial_line_loads[1].force_n_body.z,
            -10.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn sparta_surface_forces_ingestion_rejects_node_header_and_surface_ids() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let mesh = flat_plate_mesh();
        let builder = DeckBuilder::new(1.0, 1.0).unwrap();
        let texts = sparta_surface_forces_texts(&matrix, &mesh);
        let mut surfaces = sparta_surface_force_cases(&texts);
        surfaces[1].node_index = 7;
        assert!(matches!(
            builder.bake_sparta_surface_force_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_node_index"
            })
        ));

        let mut bad_header = sparta_surface_forces_texts(&matrix, &mesh);
        bad_header[0] = bad_header[0].replacen("surf_id fx fy fz", "surf force_y force_z", 1);
        let surfaces = sparta_surface_force_cases(&bad_header);
        assert!(matches!(
            builder.bake_sparta_surface_force_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let mut duplicate_surface = sparta_surface_forces_texts(&matrix, &mesh);
        duplicate_surface[0] = duplicate_surface[0].replacen("2 0.0 0.0", "1 0.0 0.0", 1);
        let surfaces = sparta_surface_force_cases(&duplicate_surface);
        assert!(matches!(
            builder.bake_sparta_surface_force_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_duplicate_surface"
            })
        ));

        let mut nonfinite_force = sparta_surface_forces_texts(&matrix, &mesh);
        nonfinite_force[0] = nonfinite_force[0].replacen("-10", "NaN", 1);
        let surfaces = sparta_surface_force_cases(&nonfinite_force);
        assert!(matches!(
            builder.bake_sparta_surface_force_deck(&matrix, &mesh, &surfaces, Vector3::zeros()),
            Err(AerodbError::InvalidInput {
                field: "sparta_surface_forces_fz"
            })
        ));
    }

    #[test]
    fn su2_history_csv_ingestion_bakes_reduced_deck_from_body_force_history() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let texts = su2_history_texts(&matrix);
        let histories = su2_history_cases(&texts);

        let deck = builder
            .bake_su2_history_csv_deck(&matrix, &histories)
            .unwrap();

        let coeffs = deck
            .lookup(4.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_relative_eq!(coeffs.cn, 0.27, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cd, 0.47, epsilon = 1e-15);
        assert_relative_eq!(coeffs.cm, -0.057, epsilon = 1e-15);
    }

    #[test]
    fn su2_history_csv_ingestion_rejects_node_missing_columns_and_beta_fallback() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let texts = su2_history_texts(&matrix);
        let mut histories = su2_history_cases(&texts);
        histories[1].node_index = 7;
        assert!(matches!(
            builder.bake_su2_history_csv_deck(&matrix, &histories),
            Err(AerodbError::InvalidInput {
                field: "su2_history_node_index"
            })
        ));

        let mut missing_moment = su2_history_texts(&matrix);
        missing_moment[0] = "Inner_Iter,CFx,CFz\n0,-0.4,-0.2\n".to_owned();
        let histories = su2_history_cases(&missing_moment);
        assert!(matches!(
            builder.bake_su2_history_csv_deck(&matrix, &histories),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let mut wind_axis_with_beta = su2_history_texts(&matrix);
        wind_axis_with_beta[1] = "Inner_Iter,CD,CL,CMy\n0,0.4,0.2,-0.05\n".to_owned();
        let histories = su2_history_cases(&wind_axis_with_beta);
        assert!(matches!(
            builder.bake_su2_history_csv_deck(&matrix, &histories),
            Err(AerodbError::InvalidInput {
                field: "su2_history_beta_requires_body_force_columns"
            })
        ));
    }

    #[test]
    fn openfoam_force_coeffs_ingestion_bakes_reduced_deck_from_final_row() {
        let matrix = RunMatrix::from_axes(zero_beta_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let texts = openfoam_force_coeffs_texts(&matrix);
        let histories = openfoam_force_coeffs_cases(&texts);

        let deck = builder
            .bake_openfoam_force_coeffs_deck(&matrix, &histories)
            .unwrap();

        let node_index = 3;
        let node = matrix.nodes()[node_index];
        let alpha = node.alpha_deg.to_radians();
        let wind_cd = 0.4 + node_index as f64 * 0.01;
        let cl = 0.2 + node_index as f64 * 0.01;
        let coeffs = deck
            .lookup(
                node.mach,
                node.alpha_deg,
                node.beta_deg,
                &std::collections::BTreeMap::new(),
            )
            .unwrap();
        assert_relative_eq!(
            coeffs.cn,
            wind_cd * alpha.sin() + cl * alpha.cos(),
            epsilon = 1e-15
        );
        assert_relative_eq!(
            coeffs.cd,
            wind_cd * alpha.cos() - cl * alpha.sin(),
            epsilon = 1e-15
        );
        assert_relative_eq!(coeffs.cm, -0.053, epsilon = 1e-15);
    }

    #[test]
    fn openfoam_force_coeffs_ingestion_rejects_node_header_and_beta_fallback() {
        let matrix = RunMatrix::from_axes(zero_beta_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let texts = openfoam_force_coeffs_texts(&matrix);
        let mut histories = openfoam_force_coeffs_cases(&texts);
        histories[1].node_index = 7;
        assert!(matches!(
            builder.bake_openfoam_force_coeffs_deck(&matrix, &histories),
            Err(AerodbError::InvalidInput {
                field: "openfoam_force_coeffs_node_index"
            })
        ));

        let mut missing_header = openfoam_force_coeffs_texts(&matrix);
        missing_header[0] = "# Force coefficients\n0.0 0.4 0.0 0.2 0.0 -0.05\n".to_owned();
        let histories = openfoam_force_coeffs_cases(&missing_header);
        assert!(matches!(
            builder.bake_openfoam_force_coeffs_deck(&matrix, &histories),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let beta_matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let texts = openfoam_force_coeffs_texts(&beta_matrix);
        let histories = openfoam_force_coeffs_cases(&texts);
        assert!(matches!(
            builder.bake_openfoam_force_coeffs_deck(&beta_matrix, &histories),
            Err(AerodbError::InvalidInput {
                field: "openfoam_force_coeffs_beta_requires_zero_sideslip"
            })
        ));
    }

    #[test]
    fn gci_sigma_toml_ingestion_attaches_reduced_sigma_tables() {
        let (_matrix, deck) = bake_basic_deck();
        let deck = attach_reduced_sigma_tables_from_gci_toml(deck, &gci_sigma_toml(8)).unwrap();

        assert!(deck.has_sigma_tables());
        let sigma = deck
            .lookup_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let cn_gci =
            GridConvergence::from_three_grids_with_safety_factor(1.01, 1.04, 1.16, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        let cd_gci =
            GridConvergence::from_three_grids_with_safety_factor(0.51, 0.54, 0.66, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        let cm_gci =
            GridConvergence::from_three_grids_with_safety_factor(-0.21, -0.24, -0.36, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        assert_relative_eq!(
            sigma.cn,
            (0.1_f64 * 0.1 + cn_gci * cn_gci + 0.2 * 0.2).sqrt(),
            epsilon = 1e-15
        );
        assert_relative_eq!(
            sigma.cd,
            (cd_gci * cd_gci + 0.05_f64 * 0.05).sqrt(),
            epsilon = 1e-15
        );
        assert_relative_eq!(sigma.cm, cm_gci, epsilon = 1e-15);
    }

    #[test]
    fn gci_sigma_toml_ingestion_rejects_bad_rows_and_triplets() {
        let (_matrix, deck) = bake_basic_deck();
        assert!(matches!(
            attach_reduced_sigma_tables_from_gci_toml(deck.clone(), &gci_sigma_toml(1)),
            Err(AerodbError::InvalidInput {
                field: "gci_sigma_rows"
            })
        ));

        let malformed_triplet = gci_sigma_toml(8).replace(
            "cd = { fine = 0.51, medium = 0.54, coarse = 0.66, extrapolation = 0.05 }",
            "cd = { fine = 0.51, medium = 0.54, coarse = 0.52, extrapolation = 0.05 }",
        );
        assert!(matches!(
            attach_reduced_sigma_tables_from_gci_toml(deck, &malformed_triplet),
            Err(AerodbError::Uq(_))
        ));
    }

    #[test]
    fn gci_sigma_csv_ingestion_attaches_reduced_sigma_tables() {
        let (_matrix, deck) = bake_basic_deck();
        let deck = attach_reduced_sigma_tables_from_gci_csv(
            deck,
            &gci_sigma_csv(8),
            2.0,
            GridConvergence::THREE_GRID_SAFETY_FACTOR,
        )
        .unwrap();

        let sigma = deck
            .lookup_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let cn_gci = GridConvergence::from_three_grids(1.01, 1.04, 1.16, 2.0)
            .unwrap()
            .absolute_gci_fine;
        assert_relative_eq!(
            sigma.cn,
            (0.1_f64 * 0.1 + cn_gci * cn_gci + 0.2 * 0.2).sqrt(),
            epsilon = 1e-15
        );
    }

    #[test]
    fn gci_sigma_csv_ingestion_rejects_header_and_node_mismatch() {
        let (_matrix, deck) = bake_basic_deck();
        let bad_header = gci_sigma_csv(8).replacen("schema,node_index", "node_index,schema", 1);
        assert!(matches!(
            attach_reduced_sigma_tables_from_gci_csv(
                deck.clone(),
                &bad_header,
                2.0,
                GridConvergence::THREE_GRID_SAFETY_FACTOR,
            ),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let bad_index = gci_sigma_csv(8).replacen(
            &format!("{GCI_SIGMA_CSV_SCHEMA_V1},1,"),
            &format!("{GCI_SIGMA_CSV_SCHEMA_V1},7,"),
            1,
        );
        assert!(matches!(
            attach_reduced_sigma_tables_from_gci_csv(
                deck,
                &bad_index,
                2.0,
                GridConvergence::THREE_GRID_SAFETY_FACTOR,
            ),
            Err(AerodbError::InvalidInput {
                field: "gci_sigma_csv_node_index"
            })
        ));
    }

    #[test]
    fn six_gci_sigma_toml_ingestion_attaches_six_sigma_tables() {
        let (_matrix, deck) = bake_basic_six_deck();
        let deck = attach_six_sigma_tables_from_gci_toml(deck, &six_gci_sigma_toml(8)).unwrap();

        assert!(deck.has_six_sigma_tables());
        let sigma = deck
            .lookup_six_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let cx_gci =
            GridConvergence::from_three_grids_with_safety_factor(1.01, 1.04, 1.16, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        let cy_gci =
            GridConvergence::from_three_grids_with_safety_factor(0.51, 0.54, 0.66, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        let cn_gci =
            GridConvergence::from_three_grids_with_safety_factor(0.71, 0.74, 0.86, 2.0, 1.25)
                .unwrap()
                .absolute_gci_fine;
        assert_relative_eq!(
            sigma.cx,
            (0.1_f64 * 0.1 + cx_gci * cx_gci + 0.2 * 0.2).sqrt(),
            epsilon = 1e-15
        );
        assert_relative_eq!(
            sigma.cy,
            (cy_gci * cy_gci + 0.05_f64 * 0.05).sqrt(),
            epsilon = 1e-15
        );
        assert_relative_eq!(
            sigma.cn,
            (cn_gci * cn_gci + 0.02_f64 * 0.02).sqrt(),
            epsilon = 1e-15
        );
    }

    #[test]
    fn six_gci_sigma_toml_ingestion_rejects_bad_rows_and_triplets() {
        let (_matrix, deck) = bake_basic_six_deck();
        assert!(matches!(
            attach_six_sigma_tables_from_gci_toml(deck.clone(), &six_gci_sigma_toml(1)),
            Err(AerodbError::InvalidInput {
                field: "six_gci_sigma_rows"
            })
        ));

        let malformed_triplet = six_gci_sigma_toml(8).replace(
            "cy = { fine = 0.51, medium = 0.54, coarse = 0.66, extrapolation = 0.05 }",
            "cy = { fine = 0.51, medium = 0.54, coarse = 0.52, extrapolation = 0.05 }",
        );
        assert!(matches!(
            attach_six_sigma_tables_from_gci_toml(deck, &malformed_triplet),
            Err(AerodbError::Uq(_))
        ));
    }

    #[test]
    fn six_gci_sigma_csv_ingestion_attaches_six_sigma_tables() {
        let (_matrix, deck) = bake_basic_six_deck();
        let deck = attach_six_sigma_tables_from_gci_csv(
            deck,
            &six_gci_sigma_csv(8),
            2.0,
            GridConvergence::THREE_GRID_SAFETY_FACTOR,
        )
        .unwrap();

        assert!(deck.has_six_sigma_tables());
        let sigma = deck
            .lookup_six_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let cx_gci = GridConvergence::from_three_grids(1.01, 1.04, 1.16, 2.0)
            .unwrap()
            .absolute_gci_fine;
        assert_relative_eq!(
            sigma.cx,
            (0.1_f64 * 0.1 + cx_gci * cx_gci + 0.2 * 0.2).sqrt(),
            epsilon = 1e-15
        );
    }

    #[test]
    fn six_gci_sigma_csv_ingestion_rejects_header_and_node_mismatch() {
        let (_matrix, deck) = bake_basic_six_deck();
        let bad_header = six_gci_sigma_csv(8).replacen("schema,node_index", "node_index,schema", 1);
        assert!(matches!(
            attach_six_sigma_tables_from_gci_csv(
                deck.clone(),
                &bad_header,
                2.0,
                GridConvergence::THREE_GRID_SAFETY_FACTOR,
            ),
            Err(AerodbError::MalformedCsv { .. })
        ));

        let bad_index = six_gci_sigma_csv(8).replacen(
            &format!("{SIX_GCI_SIGMA_CSV_SCHEMA_V1},1,"),
            &format!("{SIX_GCI_SIGMA_CSV_SCHEMA_V1},7,"),
            1,
        );
        assert!(matches!(
            attach_six_sigma_tables_from_gci_csv(
                deck,
                &bad_index,
                2.0,
                GridConvergence::THREE_GRID_SAFETY_FACTOR,
            ),
            Err(AerodbError::InvalidInput {
                field: "six_gci_sigma_csv_node_index"
            })
        ));
    }

    #[test]
    fn reduced_sigma_budget_rejects_bad_lengths_and_negative_sources() {
        let matrix = RunMatrix::from_axes(basic_axes(), 7).unwrap();
        let builder = DeckBuilder::new(2.0, 3.0).unwrap();
        let method = LinearMethod {
            reference_area_m2: builder.reference_area_m2,
            reference_length_m: builder.reference_length_m,
        };
        let deck = builder.bake_panel_deck(&matrix, &method).unwrap();

        assert!(matches!(
            CoefficientSigmaBudget::new(0.0, -1.0, 0.0, 0.0),
            Err(AerodbError::InvalidInput { .. }),
        ));
        assert!(matches!(
            attach_reduced_sigma_tables(deck, &[]),
            Err(AerodbError::InvalidInput {
                field: "sigma_budget_length"
            }),
        ));
    }

    #[test]
    fn correlated_deck_draw_returns_nominal_without_sigma_tables() {
        let (_matrix, deck) = bake_basic_deck();
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();

        let drawn = draw_reduced_correlated_deck(&deck, settings).unwrap();

        assert_eq!(drawn, deck);
    }

    #[test]
    fn correlated_deck_draw_is_deterministic_and_preserves_sigma_tables() {
        let (matrix, deck) = bake_basic_deck();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(0.02, 0.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.01, 0.0, 0.0, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.005, 0.0, 0.0, 0.0).unwrap(),
        };
        let correlation = openbmp_aero::AeroUncertaintyCorrelation::exponential_mach(1.5).unwrap();
        let deck = attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()])
            .unwrap()
            .with_uncertainty_correlation(correlation);
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();

        let drawn_a = draw_reduced_correlated_deck(&deck, settings).unwrap();
        let drawn_b = draw_reduced_correlated_deck(&deck, settings).unwrap();
        let drawn_c = draw_reduced_correlated_deck(
            &deck,
            CorrelatedDeckDrawSettings::new(0xA3E0, 5, 120, 1.5).unwrap(),
        )
        .unwrap();

        assert_eq!(drawn_a, drawn_b);
        assert_ne!(drawn_a, drawn_c);
        assert!(drawn_a.has_sigma_tables());
        assert_eq!(drawn_a.uncertainty_correlation(), Some(correlation));
        let nominal_sigma = deck
            .lookup_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let drawn_sigma = drawn_a
            .lookup_sigma(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        assert_relative_eq!(drawn_sigma.cn, nominal_sigma.cn, epsilon = 1e-15);
        assert_relative_eq!(drawn_sigma.cd, nominal_sigma.cd, epsilon = 1e-15);
        assert_relative_eq!(drawn_sigma.cm, nominal_sigma.cm, epsilon = 1e-15);

        let nominal = deck
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        let drawn = drawn_a
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert!((drawn.cn - nominal.cn).abs() > 1e-12);
    }

    #[test]
    fn correlated_deck_draw_preserves_nominal_six_tables_without_six_sigma() {
        let (matrix, deck) = bake_basic_six_deck();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(0.02, 0.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.01, 0.0, 0.0, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.005, 0.0, 0.0, 0.0).unwrap(),
        };
        let deck = attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();
        let deflections = std::collections::BTreeMap::new();

        let drawn = draw_correlated_aero_deck(&deck, settings).unwrap();
        let nominal_six = deck.lookup_six(4.0, 5.0, 3.0, &deflections).unwrap();
        let drawn_six = drawn.lookup_six(4.0, 5.0, 3.0, &deflections).unwrap();
        let nominal_reduced = deck.lookup(4.0, 5.0, 3.0, &deflections).unwrap();
        let drawn_reduced = drawn.lookup(4.0, 5.0, 3.0, &deflections).unwrap();

        assert!(drawn.has_six_coefficient_tables());
        assert_eq!(drawn_six, nominal_six);
        assert!((drawn_reduced.cn - nominal_reduced.cn).abs() > 1e-12);
    }

    #[test]
    fn correlated_deck_draw_perturbs_six_sigma_tables_deterministically() {
        let (matrix, deck) = bake_basic_six_deck();
        let deck = attach_uniform_six_sigma_tables(deck, matrix.nodes().len(), 0.025);
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();
        let deflections = std::collections::BTreeMap::new();

        let drawn_a = draw_correlated_aero_deck(&deck, settings).unwrap();
        let drawn_b = draw_correlated_aero_deck(&deck, settings).unwrap();
        let drawn_c = draw_correlated_aero_deck(
            &deck,
            CorrelatedDeckDrawSettings::new(0xA3E0, 5, 120, 1.5).unwrap(),
        )
        .unwrap();

        assert_eq!(drawn_a, drawn_b);
        assert_ne!(drawn_a, drawn_c);
        assert!(drawn_a.has_six_sigma_tables());
        let nominal_six = deck
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let drawn_six = drawn_a
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let drawn_sigma = drawn_a
            .lookup_six_sigma(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let nominal_reduced = deck.lookup(4.0, 5.0, 3.0, &deflections).unwrap();
        let drawn_reduced = drawn_a.lookup(4.0, 5.0, 3.0, &deflections).unwrap();

        assert!((drawn_six.cx - nominal_six.cx).abs() > 1e-12);
        assert!((drawn_six.cn - nominal_six.cn).abs() > 1e-12);
        assert_relative_eq!(drawn_sigma.cx, 0.025, epsilon = 1e-15);
        assert_relative_eq!(drawn_sigma.cn, 0.025, epsilon = 1e-15);
        assert_relative_eq!(drawn_reduced.cn, nominal_reduced.cn, epsilon = 1e-15);
        assert_relative_eq!(drawn_reduced.cd, nominal_reduced.cd, epsilon = 1e-15);
        assert_relative_eq!(drawn_reduced.cm, nominal_reduced.cm, epsilon = 1e-15);
    }

    #[test]
    fn correlated_deck_draw_applies_six_coefficient_correlation_metadata() {
        let (matrix, deck) = bake_basic_six_deck();
        let deck_without_coefficient_correlation =
            attach_uniform_six_sigma_tables(deck, matrix.nodes().len(), 0.025);
        let six_coefficient_correlation = [
            [1.0, 0.75, 0.0, 0.0, 0.0, 0.0],
            [0.75, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, -0.25],
            [0.0, 0.0, 0.0, 0.0, -0.25, 1.0],
        ];
        let correlation = openbmp_aero::AeroUncertaintyCorrelation::exponential_mach(1.5)
            .unwrap()
            .with_six_coefficient_correlation(six_coefficient_correlation)
            .unwrap();
        let deck_with_coefficient_correlation = deck_without_coefficient_correlation
            .clone()
            .with_uncertainty_correlation(correlation);
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();
        let deflections = std::collections::BTreeMap::new();

        let independent =
            draw_correlated_aero_deck(&deck_without_coefficient_correlation, settings).unwrap();
        let correlated =
            draw_correlated_aero_deck(&deck_with_coefficient_correlation, settings).unwrap();
        let correlated_again =
            draw_correlated_aero_deck(&deck_with_coefficient_correlation, settings).unwrap();

        assert_eq!(correlated, correlated_again);
        assert_eq!(correlated.uncertainty_correlation(), Some(correlation));
        let independent_six = independent
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();
        let correlated_six = correlated
            .lookup_six(4.0, 5.0, 3.0, &deflections)
            .unwrap()
            .unwrap();

        assert_relative_eq!(correlated_six.cx, independent_six.cx, epsilon = 1e-15);
        assert_ne!(correlated_six.cy.to_bits(), independent_six.cy.to_bits());
        assert_ne!(correlated_six.cn.to_bits(), independent_six.cn.to_bits());
    }

    #[test]
    fn correlated_deck_draw_settings_can_use_deck_correlation_metadata() {
        let (_matrix, deck) = bake_basic_deck();
        let correlation = openbmp_aero::AeroUncertaintyCorrelation::exponential_mach(1.5).unwrap();
        let deck = deck.with_uncertainty_correlation(correlation);

        let settings =
            CorrelatedDeckDrawSettings::new_with_deck_default(0xA3E0, 4, 120, None, &deck).unwrap();

        assert_eq!(
            settings.mach_correlation_length.to_bits(),
            1.5_f64.to_bits()
        );

        let (_matrix, deck_without_metadata) = bake_basic_deck();
        assert!(matches!(
            CorrelatedDeckDrawSettings::new_with_deck_default(
                0xA3E0,
                4,
                120,
                None,
                &deck_without_metadata,
            ),
            Err(AerodbError::InvalidInput {
                field: "mach_correlation_length"
            })
        ));
    }

    #[test]
    fn correlated_deck_draw_applies_coefficient_correlation_metadata() {
        let (matrix, deck) = bake_basic_deck();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(0.02, 0.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.01, 0.0, 0.0, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.005, 0.0, 0.0, 0.0).unwrap(),
        };
        let deck_without_coefficient_correlation =
            attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        let coefficient_correlation = [[1.0, 0.75, 0.0], [0.75, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let correlation = openbmp_aero::AeroUncertaintyCorrelation::exponential_mach(1.5)
            .unwrap()
            .with_coefficient_correlation(coefficient_correlation)
            .unwrap();
        let deck_with_coefficient_correlation = deck_without_coefficient_correlation
            .clone()
            .with_uncertainty_correlation(correlation);
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();

        let uncorrelated =
            draw_reduced_correlated_deck(&deck_without_coefficient_correlation, settings).unwrap();
        let correlated =
            draw_reduced_correlated_deck(&deck_with_coefficient_correlation, settings).unwrap();

        assert_eq!(correlated.uncertainty_correlation(), Some(correlation));
        let uncorrelated_coeffs = uncorrelated
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        let correlated_coeffs = correlated
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert!((correlated_coeffs.cd - uncorrelated_coeffs.cd).abs() > 1.0e-12);
    }

    #[test]
    fn correlated_deck_draw_shares_mach_residual_across_alpha_beta_plane() {
        let (matrix, deck) = bake_basic_deck();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(0.5, 0.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.0, 0.0, 0.0, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.0, 0.0, 0.0, 0.0).unwrap(),
        };
        let deck = attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        let settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();

        let drawn = draw_reduced_correlated_deck(&deck, settings).unwrap();
        let nominal_low = deck
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        let nominal_high = deck
            .lookup(2.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();
        let drawn_low = drawn
            .lookup(2.0, -5.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        let drawn_high = drawn
            .lookup(2.0, 5.0, 3.0, &std::collections::BTreeMap::new())
            .unwrap();

        let delta_low = drawn_low.cn - nominal_low.cn;
        let delta_high = drawn_high.cn - nominal_high.cn;
        assert_relative_eq!(delta_low, delta_high, epsilon = 1e-14);
    }

    #[test]
    fn correlated_deck_draw_can_use_alpha_beta_grid_covariance() {
        let (matrix, deck) = bake_basic_deck();
        let budget = ReducedCoefficientSigmaBudget {
            cn: CoefficientSigmaBudget::new(0.5, 0.0, 0.0, 0.0).unwrap(),
            cd: CoefficientSigmaBudget::new(0.0, 0.0, 0.0, 0.0).unwrap(),
            cm: CoefficientSigmaBudget::new(0.0, 0.0, 0.0, 0.0).unwrap(),
        };
        let deck = attach_reduced_sigma_tables(deck, &vec![budget; matrix.nodes().len()]).unwrap();
        let deflections = std::collections::BTreeMap::new();
        let default_settings = CorrelatedDeckDrawSettings::new(0xA3E0, 4, 120, 1.5).unwrap();
        let grid_settings = CorrelatedDeckDrawSettings::new_with_axis_correlation_lengths(
            0xA3E0,
            4,
            120,
            1.5,
            Some(4.0),
            Some(3.0),
        )
        .unwrap();

        let default_drawn = draw_reduced_correlated_deck(&deck, default_settings).unwrap();
        let grid_drawn_a = draw_reduced_correlated_deck(&deck, grid_settings).unwrap();
        let grid_drawn_b = draw_reduced_correlated_deck(&deck, grid_settings).unwrap();
        let grid_drawn_c = draw_reduced_correlated_deck(
            &deck,
            CorrelatedDeckDrawSettings::new_with_axis_correlation_lengths(
                0xA3E0,
                5,
                120,
                1.5,
                Some(4.0),
                Some(3.0),
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(grid_drawn_a, grid_drawn_b);
        assert_ne!(grid_drawn_a, grid_drawn_c);

        let nominal_low = deck.lookup(2.0, -5.0, 0.0, &deflections).unwrap();
        let nominal_high = deck.lookup(2.0, 5.0, 3.0, &deflections).unwrap();
        let default_low = default_drawn.lookup(2.0, -5.0, 0.0, &deflections).unwrap();
        let default_high = default_drawn.lookup(2.0, 5.0, 3.0, &deflections).unwrap();
        let grid_low = grid_drawn_a.lookup(2.0, -5.0, 0.0, &deflections).unwrap();
        let grid_high = grid_drawn_a.lookup(2.0, 5.0, 3.0, &deflections).unwrap();

        let default_delta_low = default_low.cn - nominal_low.cn;
        let default_delta_high = default_high.cn - nominal_high.cn;
        let grid_delta_low = grid_low.cn - nominal_low.cn;
        let grid_delta_high = grid_high.cn - nominal_high.cn;
        assert_relative_eq!(default_delta_low, default_delta_high, epsilon = 1e-14);
        assert_ne!(grid_delta_low.to_bits(), grid_delta_high.to_bits());
    }

    #[test]
    fn correlated_deck_draw_rejects_invalid_settings_and_non_reduced_decks() {
        assert!(matches!(
            CorrelatedDeckDrawSettings::new(1, 0, 0, 0.0),
            Err(AerodbError::InvalidInput {
                field: "mach_correlation_length"
            }),
        ));
        assert!(matches!(
            CorrelatedDeckDrawSettings::new(1, 0, u32::MAX - 1, 1.0),
            Err(AerodbError::InvalidInput {
                field: "dimension_id_base"
            }),
        ));
        assert!(matches!(
            CorrelatedDeckDrawSettings::new_with_axis_correlation_lengths(
                1,
                0,
                0,
                1.0,
                Some(0.0),
                None,
            ),
            Err(AerodbError::InvalidInput {
                field: "alpha_correlation_length_deg"
            }),
        ));
        assert!(matches!(
            CorrelatedDeckDrawSettings::new_with_axis_correlation_lengths(
                1,
                0,
                0,
                1.0,
                None,
                Some(f64::NAN),
            ),
            Err(AerodbError::InvalidInput {
                field: "beta_correlation_length_deg"
            }),
        ));

        let deck = AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "fin_deg".to_string(),
            ],
            vec![vec![2.0], vec![0.0], vec![0.0], vec![0.0]],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            1.0,
            1.0,
        )
        .unwrap();
        assert!(matches!(
            draw_reduced_correlated_deck(
                &deck,
                CorrelatedDeckDrawSettings::new(1, 0, 0, 1.0).unwrap(),
            ),
            Err(AerodbError::UnsupportedDeckBake { .. }),
        ));

        let (matrix, six_deck) = bake_basic_six_deck();
        let six_deck = attach_uniform_six_sigma_tables(six_deck, matrix.nodes().len(), 0.01);
        assert!(matches!(
            draw_correlated_aero_deck(
                &six_deck,
                CorrelatedDeckDrawSettings::new(1, 0, u32::MAX - 7, 1.0).unwrap(),
            ),
            Err(AerodbError::InvalidInput {
                field: "dimension_id_base"
            }),
        ));
    }

    #[test]
    fn correlated_deck_draw_reports_required_dimension_count() {
        let (_matrix, reduced_deck) = bake_basic_deck();
        assert_eq!(
            correlated_deck_draw_required_dimension_count(&reduced_deck),
            3
        );
        assert!(
            validate_correlated_deck_draw_dimension_capacity(&reduced_deck, u32::MAX - 2,).is_ok()
        );

        let (matrix, six_deck) = bake_basic_six_deck();
        let six_deck = attach_uniform_six_sigma_tables(six_deck, matrix.nodes().len(), 0.01);
        assert_eq!(correlated_deck_draw_required_dimension_count(&six_deck), 9);
        assert!(validate_correlated_deck_draw_dimension_capacity(&six_deck, u32::MAX - 8,).is_ok());
        assert!(matches!(
            validate_correlated_deck_draw_dimension_capacity(&six_deck, u32::MAX - 7),
            Err(AerodbError::InvalidInput {
                field: "dimension_id_base"
            })
        ));
    }

    #[test]
    fn run_matrix_toml_rejects_target_aimpoint_fields() {
        let toml = r#"
openbmp.aerodb_run_matrix = 1
seed = 1
target_aimpoint_eci_m = [0.0, 0.0, 0.0]

[grid]
mach = [2.0]
alpha_deg = [0.0]
beta_deg = [0.0]
reynolds_length = [1000000.0]
knudsen = [0.0]
dynamic_pressure_pa = [100.0]
"#;
        assert!(matches!(
            RunMatrix::load_toml_str(toml),
            Err(AerodbError::ForbiddenVocabulary { .. }),
        ));
    }

    #[test]
    fn run_matrix_condition_only_lint_accepts_reynolds_and_rejects_forbidden_keys() {
        let valid = r#"
openbmp.aerodb_run_matrix = 1
seed = 1

[grid]
mach = [2.0]
alpha_deg = [0.0]
beta_deg = [0.0]
reynolds_length = [1000000.0]
knudsen = [0.0]
dynamic_pressure_pa = [100.0]
"#;
        lint_condition_only_run_matrix_toml(valid).unwrap();

        for forbidden in [
            "target = \"pad\"\n",
            "[grid]\nrange_m = [100.0]\n",
            "[case]\naimpoint = [0.0, 0.0, 0.0]\n",
        ] {
            assert!(
                matches!(
                    lint_condition_only_run_matrix_toml(forbidden),
                    Err(AerodbError::ForbiddenVocabulary { .. }),
                ),
                "{forbidden}"
            );
        }
    }

    #[test]
    fn run_node_rejects_invalid_conditions() {
        assert!(matches!(
            RunNode::new(1.0, 0.0, 0.0, 0.0, 1.0e6, 0.0),
            Err(AerodbError::InvalidInput {
                field: "dynamic_pressure_pa"
            }),
        ));
        assert!(matches!(
            RunNode::new(1.0, 0.0, 0.0, 100.0, -1.0, 0.0),
            Err(AerodbError::InvalidInput {
                field: "reynolds_length"
            }),
        ));
    }
}
