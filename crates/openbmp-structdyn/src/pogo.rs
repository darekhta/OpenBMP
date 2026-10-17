//! Structural-side POGO frequency-response and reduced eigenvalue helpers.
//!
//! This module evaluates structural modal receptance, sampled open-loop Nyquist
//! clearance for caller-provided feed transfer samples, reduced first-order
//! feed-loop roots, and finite-dimensional lumped feedline-coupled roots.
//! Scenario parsing and runner gating live above this crate.

use num_complex::Complex64;
use serde::Deserialize;
use std::collections::BTreeSet;

use crate::StructDynError;

const PUBLIC_POGO_CASE_HISTORY_DATASET_ID: &str = "openbmp.feed_system.public_pogo_case_history.v1";

/// Typed public POGO case-history deck.
///
/// This captures the qualitative public NASA case-history evidence used to
/// bound the POGO helper's validation claim. It is not a calibrated vehicle
/// model or a quantitative margin oracle.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoCaseHistoryDeck {
    /// Dataset identifier from the deck metadata.
    pub dataset_id: String,
    /// Source class from the deck metadata.
    pub source_class: String,
    /// Validation label from the deck metadata.
    pub validation: String,
    /// Provenance pointer from the deck metadata.
    pub provenance: String,
    /// Public source records referenced by case rows.
    pub sources: Vec<PogoCaseHistorySource>,
    /// Qualitative case-history rows.
    pub cases: Vec<PogoCaseHistoryCase>,
}

/// One public source record in a POGO case-history deck.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoCaseHistorySource {
    /// Stable source id used by case rows.
    pub id: String,
    /// Public document identifier.
    pub document_id: String,
    /// Public report number.
    pub report_number: String,
    /// Whether the source is declared public.
    pub public: bool,
}

/// One qualitative POGO case-history row.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoCaseHistoryCase {
    /// Stable case id.
    pub id: String,
    /// Vehicle family label.
    pub vehicle_family: String,
    /// Stage or subsystem label.
    pub stage: String,
    /// Referenced source ids.
    pub source_ids: Vec<String>,
    /// Public observed frequency band, Hz.
    pub public_frequency_band_hz: [f64; 2],
    /// Public peak acceleration, g, when reported.
    pub public_peak_acceleration_g: Option<f64>,
    /// Whether public records report observed POGO.
    pub observed_pogo: bool,
    /// Declared accumulator/suppressor trend.
    pub accumulator_trend: String,
    /// Public mitigation description.
    pub mitigation: String,
    /// Qualitative model expectation.
    pub model_expectation: String,
}

/// Parse and validate the public qualitative POGO case-history TOML deck.
///
/// # Errors
///
/// Returns [`StructDynError`] when the deck cannot be parsed, has the wrong
/// magic or metadata, carries duplicate source/case ids, references unknown
/// sources, or declares invalid case-history bands/trends.
pub fn parse_public_pogo_case_history_toml(
    input: &str,
) -> Result<PogoCaseHistoryDeck, StructDynError> {
    let raw: RawPogoCaseHistoryDeck =
        toml::from_str(input).map_err(|_| StructDynError::InvalidMatrix {
            field: "pogo.case_history",
            reason: "invalid TOML",
        })?;
    if raw.openbmp_feed_pogo_case_history != 1 {
        return Err(StructDynError::InvalidNumber {
            field: "pogo.case_history.magic",
        });
    }
    require_nonempty_str("pogo.case_history.meta.dataset_id", &raw.meta.dataset_id)?;
    require_nonempty_str(
        "pogo.case_history.meta.source_class",
        &raw.meta.source_class,
    )?;
    require_nonempty_str("pogo.case_history.meta.validation", &raw.meta.validation)?;
    require_nonempty_str("pogo.case_history.meta.provenance", &raw.meta.provenance)?;
    if raw.meta.dataset_id != PUBLIC_POGO_CASE_HISTORY_DATASET_ID {
        return Err(StructDynError::InvalidString {
            field: "pogo.case_history.meta.dataset_id",
        });
    }
    if raw.meta.source_class != "public-case-history" {
        return Err(StructDynError::InvalidString {
            field: "pogo.case_history.meta.source_class",
        });
    }
    if raw.source.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.case_history.source",
            expected: 1,
            actual: 0,
        });
    }
    if raw.case.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.case_history.case",
            expected: 1,
            actual: 0,
        });
    }

    let mut source_ids = BTreeSet::new();
    let mut sources = Vec::with_capacity(raw.source.len());
    for source in raw.source {
        require_nonempty_str("pogo.case_history.source.id", &source.id)?;
        require_nonempty_str("pogo.case_history.source.document_id", &source.document_id)?;
        require_nonempty_str(
            "pogo.case_history.source.report_number",
            &source.report_number,
        )?;
        if !source.public {
            return Err(StructDynError::InvalidString {
                field: "pogo.case_history.source.public",
            });
        }
        if !source_ids.insert(source.id.clone()) {
            return Err(StructDynError::InvalidString {
                field: "pogo.case_history.source.id",
            });
        }
        sources.push(PogoCaseHistorySource {
            id: source.id,
            document_id: source.document_id,
            report_number: source.report_number,
            public: source.public,
        });
    }

    let mut case_ids = BTreeSet::new();
    let mut cases = Vec::with_capacity(raw.case.len());
    for case in raw.case {
        require_nonempty_str("pogo.case_history.case.id", &case.id)?;
        require_nonempty_str(
            "pogo.case_history.case.vehicle_family",
            &case.vehicle_family,
        )?;
        require_nonempty_str("pogo.case_history.case.stage", &case.stage)?;
        require_nonempty_str("pogo.case_history.case.mitigation", &case.mitigation)?;
        require_nonempty_str(
            "pogo.case_history.case.model_expectation",
            &case.model_expectation,
        )?;
        if !case_ids.insert(case.id.clone()) {
            return Err(StructDynError::InvalidString {
                field: "pogo.case_history.case.id",
            });
        }
        if !case.observed_pogo {
            return Err(StructDynError::InvalidString {
                field: "pogo.case_history.case.observed_pogo",
            });
        }
        if case.accumulator_trend != "raises_margin" {
            return Err(StructDynError::InvalidString {
                field: "pogo.case_history.case.accumulator_trend",
            });
        }
        if case.source_ids.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "pogo.case_history.case.source_ids",
                expected: 1,
                actual: 0,
            });
        }
        for source_id in &case.source_ids {
            require_nonempty_str("pogo.case_history.case.source_ids", source_id)?;
            if !source_ids.contains(source_id) {
                return Err(StructDynError::InvalidString {
                    field: "pogo.case_history.case.source_ids",
                });
            }
        }
        let [band_min, band_max] = case.public_frequency_band_hz;
        require_positive_finite("pogo.case_history.case.public_frequency_band_hz", band_min)?;
        require_positive_finite("pogo.case_history.case.public_frequency_band_hz", band_max)?;
        if band_min >= band_max {
            return Err(StructDynError::InvalidNumber {
                field: "pogo.case_history.case.public_frequency_band_hz",
            });
        }
        if let Some(peak_g) = case.public_peak_acceleration_g {
            require_positive_finite("pogo.case_history.case.public_peak_acceleration_g", peak_g)?;
        }

        cases.push(PogoCaseHistoryCase {
            id: case.id,
            vehicle_family: case.vehicle_family,
            stage: case.stage,
            source_ids: case.source_ids,
            public_frequency_band_hz: case.public_frequency_band_hz,
            public_peak_acceleration_g: case.public_peak_acceleration_g,
            observed_pogo: case.observed_pogo,
            accumulator_trend: case.accumulator_trend,
            mitigation: case.mitigation,
            model_expectation: case.model_expectation,
        });
    }

    Ok(PogoCaseHistoryDeck {
        dataset_id: raw.meta.dataset_id,
        source_class: raw.meta.source_class,
        validation: raw.meta.validation,
        provenance: raw.meta.provenance,
        sources,
        cases,
    })
}

#[derive(Debug, Deserialize)]
struct RawPogoCaseHistoryDeck {
    openbmp_feed_pogo_case_history: u32,
    meta: RawPogoCaseHistoryMeta,
    source: Vec<RawPogoCaseHistorySource>,
    case: Vec<RawPogoCaseHistoryCase>,
}

#[derive(Debug, Deserialize)]
struct RawPogoCaseHistoryMeta {
    dataset_id: String,
    source_class: String,
    provenance: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
struct RawPogoCaseHistorySource {
    id: String,
    document_id: String,
    report_number: String,
    public: bool,
}

#[derive(Debug, Deserialize)]
struct RawPogoCaseHistoryCase {
    id: String,
    vehicle_family: String,
    stage: String,
    source_ids: Vec<String>,
    observed_pogo: bool,
    public_frequency_band_hz: [f64; 2],
    public_peak_acceleration_g: Option<f64>,
    mitigation: String,
    accumulator_trend: String,
    model_expectation: String,
}

/// One longitudinal structural mode used by the sampled POGO loop margin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalPogoMode {
    /// Modal natural frequency, rad/s.
    pub natural_frequency_rad_s: f64,
    /// Modal damping ratio.
    pub damping_ratio: f64,
    /// Modal mass, kg.
    pub modal_mass_kg: f64,
    /// Dimensionless generalized coupling factor between feed force and this
    /// modal coordinate.
    pub coupling_factor: f64,
}

impl LongitudinalPogoMode {
    /// Construct and validate one longitudinal POGO mode.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field is non-finite, natural
    /// frequency or modal mass are non-positive, or damping is negative.
    pub fn new(
        natural_frequency_rad_s: f64,
        damping_ratio: f64,
        modal_mass_kg: f64,
        coupling_factor: f64,
    ) -> Result<Self, StructDynError> {
        let mode = Self {
            natural_frequency_rad_s,
            damping_ratio,
            modal_mass_kg,
            coupling_factor,
        };
        mode.require_valid()?;
        Ok(mode)
    }

    /// Validate modal fields.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field violates the constructor
    /// contract.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_positive_finite(
            "pogo.mode.natural_frequency_rad_s",
            self.natural_frequency_rad_s,
        )?;
        require_nonnegative_finite("pogo.mode.damping_ratio", self.damping_ratio)?;
        require_positive_finite("pogo.mode.modal_mass_kg", self.modal_mass_kg)?;
        require_finite("pogo.mode.coupling_factor", self.coupling_factor)
    }

    /// Modal receptance from generalized feed force to collocated displacement,
    /// evaluated at `s = j omega`.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when `omega_rad_s` is invalid or an undamped
    /// resonance would create a singular receptance.
    pub fn receptance_at(&self, omega_rad_s: f64) -> Result<Complex64, StructDynError> {
        self.require_valid()?;
        require_nonnegative_finite("pogo.omega_rad_s", omega_rad_s)?;

        let omega_n = self.natural_frequency_rad_s;
        let real = self.modal_mass_kg * (omega_n * omega_n - omega_rad_s * omega_rad_s);
        let imag = self.modal_mass_kg * (2.0 * self.damping_ratio * omega_n * omega_rad_s);
        let denominator = Complex64::new(real, imag);
        if denominator.norm_sqr() == 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "pogo.receptance.denominator",
            });
        }
        let numerator = self.coupling_factor * self.coupling_factor;
        let receptance = Complex64::new(numerator, 0.0) / denominator;
        require_finite_complex("pogo.receptance", receptance)?;
        Ok(receptance)
    }
}

/// One caller-provided feed transfer sample on the POGO frequency grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedTransferSample {
    /// Angular frequency, rad/s.
    pub omega_rad_s: f64,
    /// Signed complex feed transfer from structural displacement back to
    /// generalized modal force, N/m. Multiplying by structural receptance
    /// produces a dimensionless open-loop value.
    pub transfer_n_per_m: Complex64,
}

impl FeedTransferSample {
    /// Construct and validate one feed transfer sample.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the frequency is negative or any complex
    /// component is non-finite.
    pub fn new(omega_rad_s: f64, transfer_n_per_m: Complex64) -> Result<Self, StructDynError> {
        let sample = Self {
            omega_rad_s,
            transfer_n_per_m,
        };
        sample.require_valid()?;
        Ok(sample)
    }

    /// Validate one feed transfer sample.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the frequency or transfer is invalid.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_nonnegative_finite("pogo.feed.omega_rad_s", self.omega_rad_s)?;
        require_finite_complex("pogo.feed.transfer_n_per_m", self.transfer_n_per_m)
    }
}

/// One lossless 1-D feedline segment for POGO transfer-matrix assembly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFeedlineSegment {
    /// Segment length, m.
    pub length_m: f64,
    /// Fluid acoustic wave speed, m/s.
    pub wave_speed_m_s: f64,
    /// Fluid density, kg/m^3.
    pub density_kg_m3: f64,
    /// Cross-sectional flow area, m^2.
    pub cross_section_area_m2: f64,
}

impl PogoFeedlineSegment {
    /// Construct and validate one lossless feedline segment.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any segment field is non-finite or
    /// non-positive.
    pub fn new(
        length_m: f64,
        wave_speed_m_s: f64,
        density_kg_m3: f64,
        cross_section_area_m2: f64,
    ) -> Result<Self, StructDynError> {
        let segment = Self {
            length_m,
            wave_speed_m_s,
            density_kg_m3,
            cross_section_area_m2,
        };
        segment.require_valid()?;
        Ok(segment)
    }

    /// Validate feedline segment fields.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any segment field violates the
    /// constructor contract.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_positive_finite("pogo.feedline.length_m", self.length_m)?;
        require_positive_finite("pogo.feedline.wave_speed_m_s", self.wave_speed_m_s)?;
        require_positive_finite("pogo.feedline.density_kg_m3", self.density_kg_m3)?;
        require_positive_finite(
            "pogo.feedline.cross_section_area_m2",
            self.cross_section_area_m2,
        )
    }

    /// Characteristic pressure/volume-flow impedance, Pa/(m^3/s).
    #[must_use]
    pub fn characteristic_impedance_pa_s_per_m3(self) -> f64 {
        self.density_kg_m3 * self.wave_speed_m_s / self.cross_section_area_m2
    }

    /// Lumped inertance, Pa*s^2/m^3.
    #[must_use]
    pub fn inertance_pa_s2_per_m3(self) -> f64 {
        self.density_kg_m3 * self.length_m / self.cross_section_area_m2
    }

    /// Lumped acoustic compliance, m^3/Pa.
    #[must_use]
    pub fn compliance_m3_per_pa(self) -> f64 {
        self.cross_section_area_m2 * self.length_m
            / (self.density_kg_m3 * self.wave_speed_m_s * self.wave_speed_m_s)
    }
}

/// Two-port transfer matrix for feedline state `[pressure_pa, volume_flow_m3_s]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFeedlineTransferMatrix {
    /// Pressure-to-pressure transfer term.
    pub a: Complex64,
    /// Volume-flow-to-pressure transfer term.
    pub b: Complex64,
    /// Pressure-to-volume-flow transfer term.
    pub c: Complex64,
    /// Volume-flow-to-volume-flow transfer term.
    pub d: Complex64,
}

impl PogoFeedlineTransferMatrix {
    /// Identity transfer matrix.
    #[must_use]
    pub fn identity() -> Self {
        Self {
            a: Complex64::new(1.0, 0.0),
            b: Complex64::new(0.0, 0.0),
            c: Complex64::new(0.0, 0.0),
            d: Complex64::new(1.0, 0.0),
        }
    }

    /// Compose this matrix with `rhs` in feedline order.
    #[must_use]
    pub fn cascade(self, rhs: Self) -> Self {
        Self {
            a: self.a * rhs.a + self.b * rhs.c,
            b: self.a * rhs.b + self.b * rhs.d,
            c: self.c * rhs.a + self.d * rhs.c,
            d: self.c * rhs.b + self.d * rhs.d,
        }
    }

    /// Determinant of the two-port transfer matrix.
    #[must_use]
    pub fn determinant(self) -> Complex64 {
        self.a * self.d - self.b * self.c
    }
}

/// One sampled residual from the uniform distributed-feedline benchmark.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoDistributedFeedlineBenchmarkSample {
    /// Angular frequency, rad/s.
    pub omega_rad_s: f64,
    /// Matrix from cascading the declared uniform segments.
    pub cascaded_matrix: PogoFeedlineTransferMatrix,
    /// Closed-form single-line matrix for the same total length and medium.
    pub closed_form_matrix: PogoFeedlineTransferMatrix,
    /// Residual of the lossless reciprocal determinant invariant, `|det(M)-1|`.
    pub determinant_residual: f64,
    /// Maximum relative matrix-entry residual against the single-line closed
    /// form.
    pub transfer_matrix_residual: f64,
    /// Residual of the impedance-normalized off-diagonal symmetry,
    /// `|b/Zc - c*Zc|`.
    pub normalized_off_diagonal_residual: f64,
}

/// Uniform distributed-feedline benchmark report across a frequency grid.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoDistributedFeedlineBenchmarkReport {
    samples: Vec<PogoDistributedFeedlineBenchmarkSample>,
    /// Maximum determinant residual across all samples.
    pub max_determinant_residual: f64,
    /// Maximum relative residual against the closed-form single-line matrix.
    pub max_transfer_matrix_residual: f64,
    /// Maximum impedance-normalized off-diagonal residual across all samples.
    pub max_normalized_off_diagonal_residual: f64,
}

impl PogoDistributedFeedlineBenchmarkReport {
    /// Evaluated benchmark samples in frequency-grid order.
    #[must_use]
    pub fn samples(&self) -> &[PogoDistributedFeedlineBenchmarkSample] {
        &self.samples
    }
}

/// Assemble the lossless distributed feedline transfer matrix at `s = j omega`.
///
/// Each segment maps downstream pressure/volume-flow to upstream
/// pressure/volume-flow:
///
/// ```text
/// [p_up]   [ cos(kL)    j Zc sin(kL)] [p_down]
/// [Q_up] = [j/Zc sin(kL)  cos(kL)   ] [Q_down]
/// ```
///
/// where `k = omega / wave_speed` and `Zc = rho * wave_speed / area`.
///
/// # Errors
///
/// Returns [`StructDynError`] when the segment list is empty, angular frequency
/// is negative or non-finite, any segment is invalid, or any assembled matrix
/// component is non-finite.
pub fn assemble_feedline_transfer_matrix(
    segments: &[PogoFeedlineSegment],
    omega_rad_s: f64,
) -> Result<PogoFeedlineTransferMatrix, StructDynError> {
    if segments.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.feedline.segments",
            expected: 1,
            actual: 0,
        });
    }
    require_nonnegative_finite("pogo.feedline.omega_rad_s", omega_rad_s)?;
    let mut matrix = PogoFeedlineTransferMatrix::identity();
    for segment in segments {
        segment.require_valid()?;
        let theta = omega_rad_s * segment.length_m / segment.wave_speed_m_s;
        require_finite("pogo.feedline.theta", theta)?;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let zc = segment.characteristic_impedance_pa_s_per_m3();
        let segment_matrix = PogoFeedlineTransferMatrix {
            a: Complex64::new(cos_theta, 0.0),
            b: Complex64::new(0.0, zc * sin_theta),
            c: Complex64::new(0.0, sin_theta / zc),
            d: Complex64::new(cos_theta, 0.0),
        };
        require_feedline_matrix_finite(segment_matrix)?;
        matrix = matrix.cascade(segment_matrix);
        require_feedline_matrix_finite(matrix)?;
    }
    Ok(matrix)
}

/// Evaluate a uniform distributed-feedline analytic benchmark.
///
/// The benchmark is intentionally limited to a uniform lossless medium: every
/// declared segment must share wave speed, density, and cross-sectional area.
/// It compares the declaration-ordered segment cascade against the closed-form
/// transfer matrix for one equivalent line with the same total length, and also
/// records the determinant and impedance-normalized symmetry invariants of a
/// reciprocal lossless line.
///
/// # Errors
///
/// Returns [`StructDynError`] when the segment list is empty, segments are
/// invalid or non-uniform, the frequency grid is empty/not strictly increasing,
/// or any benchmark residual is non-finite.
pub fn evaluate_uniform_distributed_feedline_benchmark(
    segments: &[PogoFeedlineSegment],
    frequency_grid_rad_s: &[f64],
) -> Result<PogoDistributedFeedlineBenchmarkReport, StructDynError> {
    let equivalent = equivalent_uniform_feedline_segment(segments)?;
    validate_frequency_grid(
        "pogo.feedline.benchmark.frequency_grid_rad_s",
        frequency_grid_rad_s,
    )?;

    let characteristic_impedance = equivalent.characteristic_impedance_pa_s_per_m3();
    let mut samples = Vec::with_capacity(frequency_grid_rad_s.len());
    let mut max_determinant_residual = 0.0_f64;
    let mut max_transfer_matrix_residual = 0.0_f64;
    let mut max_normalized_off_diagonal_residual = 0.0_f64;

    for omega_rad_s in frequency_grid_rad_s {
        let cascaded_matrix = assemble_feedline_transfer_matrix(segments, *omega_rad_s)?;
        let closed_form_matrix = assemble_feedline_transfer_matrix(&[equivalent], *omega_rad_s)?;
        let determinant_residual =
            complex_abs(cascaded_matrix.determinant() - Complex64::new(1.0, 0.0));
        let transfer_matrix_residual =
            feedline_matrix_relative_residual(cascaded_matrix, closed_form_matrix)?;
        let normalized_off_diagonal_residual = complex_abs(
            cascaded_matrix.b / characteristic_impedance
                - cascaded_matrix.c * characteristic_impedance,
        );
        require_all_finite_array(
            "pogo.feedline.benchmark.residual",
            &[
                determinant_residual,
                transfer_matrix_residual,
                normalized_off_diagonal_residual,
            ],
        )?;

        max_determinant_residual = max_determinant_residual.max(determinant_residual);
        max_transfer_matrix_residual = max_transfer_matrix_residual.max(transfer_matrix_residual);
        max_normalized_off_diagonal_residual =
            max_normalized_off_diagonal_residual.max(normalized_off_diagonal_residual);
        samples.push(PogoDistributedFeedlineBenchmarkSample {
            omega_rad_s: *omega_rad_s,
            cascaded_matrix,
            closed_form_matrix,
            determinant_residual,
            transfer_matrix_residual,
            normalized_off_diagonal_residual,
        });
    }

    Ok(PogoDistributedFeedlineBenchmarkReport {
        samples,
        max_determinant_residual,
        max_transfer_matrix_residual,
        max_normalized_off_diagonal_residual,
    })
}

/// Sampled Nyquist clearance verdict for the structural POGO loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PogoNyquistVerdict {
    /// Every sampled point stayed farther from `-1 + j0` than the configured
    /// clearance threshold.
    Clear,
    /// At least one sampled point approached `-1 + j0` at or below the
    /// configured clearance threshold.
    BelowClearance,
}

/// Evaluated open-loop POGO sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoOpenLoopSample {
    /// Angular frequency, rad/s.
    pub omega_rad_s: f64,
    /// Sum of structural modal receptances, m/N.
    pub structural_receptance_m_per_n: Complex64,
    /// Caller-provided feed transfer, N/m.
    pub feed_transfer_n_per_m: Complex64,
    /// Dimensionless open-loop value `L(j omega)`.
    pub open_loop: Complex64,
    /// Distance from the Nyquist critical point, `|1 + L(j omega)|`.
    pub distance_to_minus_one: f64,
    /// Open-loop magnitude, `|L(j omega)|`.
    pub gain: f64,
    /// Open-loop phase, degrees.
    pub phase_deg: f64,
}

/// Sampled structural POGO margin report.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoNyquistReport {
    samples: Vec<PogoOpenLoopSample>,
    /// Minimum sampled distance to `-1 + j0`.
    pub min_distance_to_minus_one: f64,
    /// Frequency where [`Self::min_distance_to_minus_one`] occurs.
    pub min_distance_frequency_rad_s: f64,
    /// Maximum sampled open-loop magnitude.
    pub peak_gain: f64,
    /// Frequency where [`Self::peak_gain`] occurs.
    pub peak_gain_frequency_rad_s: f64,
    /// Verdict against the configured sampled-clearance threshold.
    pub verdict: PogoNyquistVerdict,
}

impl PogoNyquistReport {
    /// Evaluated samples in declaration order.
    #[must_use]
    pub fn samples(&self) -> &[PogoOpenLoopSample] {
        &self.samples
    }
}

/// Verdict from the closed-loop POGO characteristic roots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PogoClosedLoopVerdict {
    /// Every closed-loop root has negative real part.
    AsymptoticallyStable,
    /// At least one root is marginal or in the right half plane.
    RightHalfPlaneOrMarginal,
}

/// First-order feed-loop model used for a reduced closed-loop POGO eigen test.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFirstOrderFeed {
    /// Effective open-loop modal feedback gain after accumulator attenuation,
    /// in rad^2/s^2.
    pub effective_gain_rad2: f64,
    /// First-order feed response time constant, seconds.
    pub feed_time_constant_s: f64,
    /// Effective feed zero, seconds.
    pub mass_flow_gain_time_s: f64,
}

impl PogoFirstOrderFeed {
    /// Construct and validate one reduced feed-loop model.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field is non-finite, when the feed
    /// pole time constant is non-positive, or when gain/zero are negative.
    pub fn new(
        effective_gain_rad2: f64,
        feed_time_constant_s: f64,
        mass_flow_gain_time_s: f64,
    ) -> Result<Self, StructDynError> {
        let feed = Self {
            effective_gain_rad2,
            feed_time_constant_s,
            mass_flow_gain_time_s,
        };
        feed.require_valid()?;
        Ok(feed)
    }

    /// Validate feed-loop fields.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field violates the constructor
    /// contract.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_nonnegative_finite(
            "pogo.closed_loop.effective_gain_rad2",
            self.effective_gain_rad2,
        )?;
        require_positive_finite(
            "pogo.closed_loop.feed_time_constant_s",
            self.feed_time_constant_s,
        )?;
        require_nonnegative_finite(
            "pogo.closed_loop.mass_flow_gain_time_s",
            self.mass_flow_gain_time_s,
        )
    }
}

/// One manufactured transient sample for the reduced first-order feed POGO
/// loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFirstOrderMmsSample {
    /// Structural modal displacement, m.
    pub displacement_m: f64,
    /// Structural modal velocity, m/s.
    pub velocity_m_s: f64,
    /// Structural modal acceleration, m/s².
    pub acceleration_m_s2: f64,
    /// First-order feed state driven by structural displacement, m.
    pub feed_state_m: f64,
    /// First-order feed-state rate, m/s.
    pub feed_state_rate_m_s: f64,
    /// Manufactured structural source acceleration, m/s², added to close the
    /// structural equation for the declared sample.
    pub manufactured_source_m_s2: f64,
}

impl PogoFirstOrderMmsSample {
    /// Construct and validate one manufactured transient sample.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any sample field is non-finite.
    pub fn new(
        displacement_m: f64,
        velocity_m_s: f64,
        acceleration_m_s2: f64,
        feed_state_m: f64,
        feed_state_rate_m_s: f64,
        manufactured_source_m_s2: f64,
    ) -> Result<Self, StructDynError> {
        let sample = Self {
            displacement_m,
            velocity_m_s,
            acceleration_m_s2,
            feed_state_m,
            feed_state_rate_m_s,
            manufactured_source_m_s2,
        };
        sample.require_valid()?;
        Ok(sample)
    }

    /// Validate sample fields.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any sample field is non-finite.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_all_finite_array(
            "pogo.first_order_mms.sample",
            &[
                self.displacement_m,
                self.velocity_m_s,
                self.acceleration_m_s2,
                self.feed_state_m,
                self.feed_state_rate_m_s,
                self.manufactured_source_m_s2,
            ],
        )
    }
}

/// Residuals from the reduced first-order feed POGO manufactured transient.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoFirstOrderMmsResidual {
    /// Residual of `tau * z_dot + z - q`, m.
    pub feed_residual_m: f64,
    /// Residual of the forced structural equation, m/s².
    pub structural_residual_m_s2: f64,
    /// Maximum absolute residual across the feed and structural equations.
    pub max_abs_residual: f64,
}

/// One pump cavitation/mass-flow-gain dispersion point for reduced POGO checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoPumpDispersionPoint {
    /// Cavitation compliance, m^3/Pa.
    pub cavitation_compliance_m3_per_pa: f64,
    /// Effective mass-flow-gain zero time, s.
    pub mass_flow_gain_time_s: f64,
}

impl PogoPumpDispersionPoint {
    /// Construct and validate one pump dispersion point.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when cavitation compliance is non-positive,
    /// mass-flow-gain time is negative, or either field is non-finite.
    pub fn new(
        cavitation_compliance_m3_per_pa: f64,
        mass_flow_gain_time_s: f64,
    ) -> Result<Self, StructDynError> {
        let point = Self {
            cavitation_compliance_m3_per_pa,
            mass_flow_gain_time_s,
        };
        point.require_valid()?;
        Ok(point)
    }

    /// Validate this pump dispersion point.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field violates the constructor
    /// contract.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        require_positive_finite(
            "pogo.pump_dispersion.cavitation_compliance_m3_per_pa",
            self.cavitation_compliance_m3_per_pa,
        )?;
        require_nonnegative_finite(
            "pogo.pump_dispersion.mass_flow_gain_time_s",
            self.mass_flow_gain_time_s,
        )
    }
}

/// Finite min/nominal/max pump dispersion box for reduced POGO checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PogoPumpDispersionBox {
    /// Minimum declared pump point.
    pub min: PogoPumpDispersionPoint,
    /// Nominal declared pump point.
    pub nominal: PogoPumpDispersionPoint,
    /// Maximum declared pump point.
    pub max: PogoPumpDispersionPoint,
}

impl PogoPumpDispersionBox {
    /// Construct and validate one pump dispersion box.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any point is invalid or the nominal point
    /// is outside the inclusive min/max interval for either field.
    pub fn new(
        min: PogoPumpDispersionPoint,
        nominal: PogoPumpDispersionPoint,
        max: PogoPumpDispersionPoint,
    ) -> Result<Self, StructDynError> {
        let dispersion = Self { min, nominal, max };
        dispersion.require_valid()?;
        Ok(dispersion)
    }

    /// Validate this dispersion box.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any point is invalid or the nominal point
    /// is outside the inclusive min/max interval for either field.
    pub fn require_valid(self) -> Result<(), StructDynError> {
        self.min.require_valid()?;
        self.nominal.require_valid()?;
        self.max.require_valid()?;
        require_ordered_box_value(
            "pogo.pump_dispersion.cavitation_compliance_m3_per_pa",
            self.min.cavitation_compliance_m3_per_pa,
            self.nominal.cavitation_compliance_m3_per_pa,
            self.max.cavitation_compliance_m3_per_pa,
        )?;
        require_ordered_box_value(
            "pogo.pump_dispersion.mass_flow_gain_time_s",
            self.min.mass_flow_gain_time_s,
            self.nominal.mass_flow_gain_time_s,
            self.max.mass_flow_gain_time_s,
        )
    }

    /// Deterministic min/nominal/max corner points.
    #[must_use]
    pub fn points(self) -> [PogoPumpDispersionPoint; 3] {
        [self.min, self.nominal, self.max]
    }
}

/// One evaluated pump-dispersion point.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoPumpDispersionSample {
    /// Pump dispersion point used for this sample.
    pub point: PogoPumpDispersionPoint,
    /// Accumulator attenuation applied at this sample.
    pub accumulator_attenuation: f64,
    /// Effective closed-loop gain after accumulator attenuation.
    pub effective_gain_rad2: f64,
    /// Closed-loop root report for this sample.
    pub closed_loop: PogoClosedLoopEigenReport,
}

/// Reduced POGO report across declared pump-dispersion points.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoPumpDispersionReport {
    samples: Vec<PogoPumpDispersionSample>,
    /// Index of the sample with the largest closed-loop maximum real part.
    pub worst_sample_index: usize,
    /// Worst closed-loop maximum real part across all samples.
    pub worst_max_real_part: f64,
    /// Worst closed-loop verdict across all samples.
    pub worst_verdict: PogoClosedLoopVerdict,
}

impl PogoPumpDispersionReport {
    /// Evaluated samples in deterministic min/nominal/max order.
    #[must_use]
    pub fn samples(&self) -> &[PogoPumpDispersionSample] {
        &self.samples
    }
}

/// Finite-dimensional lumped feedline model for a POGO coupled eigen check.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoLumpedFeedlineModel {
    segments: Vec<PogoFeedlineSegment>,
    /// Pump dispersion point used by this model.
    pub pump: PogoPumpDispersionPoint,
    /// Accumulator compliance, m^3/Pa.
    pub accumulator_compliance_m3_per_pa: f64,
    /// Open-loop modal feedback gain before accumulator attenuation, rad^2/s^2.
    pub open_loop_gain_rad2: f64,
    /// First-order feed response time constant, seconds.
    pub feed_time_constant_s: f64,
    /// Lumped feedline damping ratio.
    pub feedline_damping_ratio: f64,
}

impl PogoLumpedFeedlineModel {
    /// Construct and validate a lumped feedline-coupled POGO model.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the segment list is empty, any segment or
    /// pump point is invalid, accumulator compliance or open-loop gain is
    /// negative/non-finite, the feed time constant is non-positive, or feedline
    /// damping is negative/non-finite.
    pub fn new(
        segments: Vec<PogoFeedlineSegment>,
        pump: PogoPumpDispersionPoint,
        accumulator_compliance_m3_per_pa: f64,
        open_loop_gain_rad2: f64,
        feed_time_constant_s: f64,
        feedline_damping_ratio: f64,
    ) -> Result<Self, StructDynError> {
        let model = Self {
            segments,
            pump,
            accumulator_compliance_m3_per_pa,
            open_loop_gain_rad2,
            feed_time_constant_s,
            feedline_damping_ratio,
        };
        model.require_valid()?;
        Ok(model)
    }

    /// Validate this lumped feedline-coupled POGO model.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any field violates the constructor
    /// contract.
    pub fn require_valid(&self) -> Result<(), StructDynError> {
        if self.segments.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "pogo.lumped_feedline.segments",
                expected: 1,
                actual: 0,
            });
        }
        for segment in &self.segments {
            segment.require_valid()?;
        }
        self.pump.require_valid()?;
        require_nonnegative_finite(
            "pogo.lumped_feedline.accumulator_compliance_m3_per_pa",
            self.accumulator_compliance_m3_per_pa,
        )?;
        require_nonnegative_finite(
            "pogo.lumped_feedline.open_loop_gain_rad2",
            self.open_loop_gain_rad2,
        )?;
        require_positive_finite(
            "pogo.lumped_feedline.feed_time_constant_s",
            self.feed_time_constant_s,
        )?;
        require_nonnegative_finite(
            "pogo.lumped_feedline.feedline_damping_ratio",
            self.feedline_damping_ratio,
        )
    }

    /// Feedline segments in declaration order.
    #[must_use]
    pub fn segments(&self) -> &[PogoFeedlineSegment] {
        &self.segments
    }

    fn total_inertance_pa_s2_per_m3(&self) -> f64 {
        self.segments
            .iter()
            .map(|segment| segment.inertance_pa_s2_per_m3())
            .sum()
    }

    fn total_acoustic_compliance_m3_per_pa(&self) -> f64 {
        self.segments
            .iter()
            .map(|segment| segment.compliance_m3_per_pa())
            .sum::<f64>()
            + self.pump.cavitation_compliance_m3_per_pa
            + self.accumulator_compliance_m3_per_pa
    }
}

/// Finite-dimensional lumped feedline-coupled POGO eigen report.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoFeedlineCoupledEigenReport {
    /// Characteristic coefficients in descending powers of `s`.
    pub characteristic_coefficients: Vec<f64>,
    eigenvalues: Vec<Complex64>,
    /// Total lumped feedline inertance, Pa*s^2/m^3.
    pub feedline_inertance_pa_s2_per_m3: f64,
    /// Total segment plus pump plus accumulator compliance, m^3/Pa.
    pub feedline_compliance_m3_per_pa: f64,
    /// Accumulator attenuation applied to the open-loop gain.
    pub accumulator_attenuation: f64,
    /// Effective closed-loop gain after accumulator attenuation.
    pub effective_gain_rad2: f64,
    /// Largest real part across all roots.
    pub max_real_part: f64,
    /// Stability verdict against the RHP/marginal root test.
    pub verdict: PogoClosedLoopVerdict,
}

impl PogoFeedlineCoupledEigenReport {
    /// Complex roots of the assembled lumped feedline-coupled characteristic.
    #[must_use]
    pub fn eigenvalues(&self) -> &[Complex64] {
        &self.eigenvalues
    }
}

/// Analyze a finite-dimensional lumped feedline-coupled POGO eigenproblem.
///
/// The characteristic is
///
/// ```text
/// (tau*s + 1)
///   * (L*C*s^2 + 2*zeta_line*sqrt(L*C)*s + 1)
///   * (s^2 + 2*zeta_struct*omega*s + omega^2)
/// - gain_eff * (1 + pump_zero*s) = 0
/// ```
///
/// where `L` is the sum of declared segment inertances, `C` is the sum of
/// segment acoustic compliance plus pump cavitation compliance plus accumulator
/// compliance, and `gain_eff` uses the same accumulator attenuation convention
/// as [`analyze_pump_dispersion_box`]. This is a finite-dimensional lumped
/// feedline eigen substrate; it is not a distributed transmission-line
/// benchmark closure.
///
/// # Errors
///
/// Returns [`StructDynError`] when inputs are invalid, coefficients are
/// non-finite, or the polynomial root solve does not converge.
pub fn analyze_lumped_feedline_coupled_eigenproblem(
    mode: LongitudinalPogoMode,
    model: &PogoLumpedFeedlineModel,
) -> Result<PogoFeedlineCoupledEigenReport, StructDynError> {
    mode.require_valid()?;
    model.require_valid()?;

    let feedline_inertance_pa_s2_per_m3 = model.total_inertance_pa_s2_per_m3();
    let feedline_compliance_m3_per_pa = model.total_acoustic_compliance_m3_per_pa();
    require_positive_finite(
        "pogo.lumped_feedline.inertance_pa_s2_per_m3",
        feedline_inertance_pa_s2_per_m3,
    )?;
    require_positive_finite(
        "pogo.lumped_feedline.compliance_m3_per_pa",
        feedline_compliance_m3_per_pa,
    )?;
    let line_lc = feedline_inertance_pa_s2_per_m3 * feedline_compliance_m3_per_pa;
    require_positive_finite("pogo.lumped_feedline.lc", line_lc)?;

    let accumulator_attenuation = model.pump.cavitation_compliance_m3_per_pa
        / (model.pump.cavitation_compliance_m3_per_pa + model.accumulator_compliance_m3_per_pa);
    require_finite(
        "pogo.lumped_feedline.accumulator_attenuation",
        accumulator_attenuation,
    )?;
    let effective_gain_rad2 = model.open_loop_gain_rad2 * accumulator_attenuation;
    require_finite(
        "pogo.lumped_feedline.effective_gain_rad2",
        effective_gain_rad2,
    )?;

    let omega = mode.natural_frequency_rad_s;
    let structural = vec![1.0, 2.0 * mode.damping_ratio * omega, omega * omega];
    let feed_pole = vec![model.feed_time_constant_s, 1.0];
    let feedline = vec![
        line_lc,
        2.0 * model.feedline_damping_ratio * line_lc.sqrt(),
        1.0,
    ];
    let characteristic = subtract_lower_order_polynomial(
        polynomial_multiply(&polynomial_multiply(&feed_pole, &feedline)?, &structural)?,
        &[
            effective_gain_rad2 * model.pump.mass_flow_gain_time_s,
            effective_gain_rad2,
        ],
    )?;
    require_all_finite_array("pogo.lumped_feedline.characteristic", &characteristic)?;

    let eigenvalues = real_polynomial_roots(&characteristic)?;
    let mut max_real_part = f64::NEG_INFINITY;
    for eigenvalue in &eigenvalues {
        require_finite_complex("pogo.lumped_feedline.eigenvalue", *eigenvalue)?;
        max_real_part = max_real_part.max(eigenvalue.re);
    }
    let verdict = if max_real_part < 0.0 {
        PogoClosedLoopVerdict::AsymptoticallyStable
    } else {
        PogoClosedLoopVerdict::RightHalfPlaneOrMarginal
    };
    Ok(PogoFeedlineCoupledEigenReport {
        characteristic_coefficients: characteristic,
        eigenvalues,
        feedline_inertance_pa_s2_per_m3,
        feedline_compliance_m3_per_pa,
        accumulator_attenuation,
        effective_gain_rad2,
        max_real_part,
        verdict,
    })
}

/// Evaluate reduced POGO roots over a pump Cb/Mb dispersion box.
///
/// # Errors
///
/// Returns [`StructDynError`] when any input is invalid or any closed-loop
/// sample produces non-finite values.
pub fn analyze_pump_dispersion_box(
    mode: LongitudinalPogoMode,
    open_loop_gain_rad2: f64,
    feed_time_constant_s: f64,
    accumulator_compliance_m3_per_pa: f64,
    dispersion: PogoPumpDispersionBox,
) -> Result<PogoPumpDispersionReport, StructDynError> {
    mode.require_valid()?;
    require_nonnegative_finite(
        "pogo.pump_dispersion.open_loop_gain_rad2",
        open_loop_gain_rad2,
    )?;
    require_positive_finite(
        "pogo.pump_dispersion.feed_time_constant_s",
        feed_time_constant_s,
    )?;
    require_nonnegative_finite(
        "pogo.pump_dispersion.accumulator_compliance_m3_per_pa",
        accumulator_compliance_m3_per_pa,
    )?;
    dispersion.require_valid()?;

    let mut samples = Vec::with_capacity(3);
    let mut worst_sample_index = 0;
    let mut worst_max_real_part = f64::NEG_INFINITY;
    let mut worst_verdict = PogoClosedLoopVerdict::AsymptoticallyStable;
    for point in dispersion.points() {
        let attenuation = point.cavitation_compliance_m3_per_pa
            / (point.cavitation_compliance_m3_per_pa + accumulator_compliance_m3_per_pa);
        require_finite("pogo.pump_dispersion.accumulator_attenuation", attenuation)?;
        let effective_gain_rad2 = open_loop_gain_rad2 * attenuation;
        require_finite(
            "pogo.pump_dispersion.effective_gain_rad2",
            effective_gain_rad2,
        )?;
        let closed_loop = analyze_first_order_closed_loop(
            mode,
            PogoFirstOrderFeed::new(
                effective_gain_rad2,
                feed_time_constant_s,
                point.mass_flow_gain_time_s,
            )?,
        )?;
        if closed_loop.max_real_part > worst_max_real_part {
            worst_sample_index = samples.len();
            worst_max_real_part = closed_loop.max_real_part;
            worst_verdict = closed_loop.verdict;
        }
        samples.push(PogoPumpDispersionSample {
            point,
            accumulator_attenuation: attenuation,
            effective_gain_rad2,
            closed_loop,
        });
    }
    Ok(PogoPumpDispersionReport {
        samples,
        worst_sample_index,
        worst_max_real_part,
        worst_verdict,
    })
}

/// Closed-loop POGO eigenvalue report for a reduced first-order feed loop.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoClosedLoopEigenReport {
    /// Characteristic coefficients `[a3, a2, a1, a0]` for
    /// `a3*s^3 + a2*s^2 + a1*s + a0`.
    pub characteristic_coefficients: [f64; 4],
    /// Complex closed-loop roots of the cubic characteristic.
    pub eigenvalues: [Complex64; 3],
    /// Largest real part across all roots.
    pub max_real_part: f64,
    /// Stability verdict against the RHP/marginal root test.
    pub verdict: PogoClosedLoopVerdict,
}

/// Evaluate the reduced first-order feed/structural POGO closed-loop roots.
///
/// The characteristic is
///
/// ```text
/// (tau*s + 1) * (s^2 + 2*zeta*omega*s + omega^2)
///     - gain * (1 + zero*s) = 0
/// ```
///
/// where `gain` is an effective modal feedback gain supplied by the caller
/// after any accumulator or feed dispersion scaling.
///
/// # Errors
///
/// Returns [`StructDynError`] when the mode/feed inputs are invalid or when the
/// computed cubic roots are non-finite.
pub fn analyze_first_order_closed_loop(
    mode: LongitudinalPogoMode,
    feed: PogoFirstOrderFeed,
) -> Result<PogoClosedLoopEigenReport, StructDynError> {
    mode.require_valid()?;
    feed.require_valid()?;

    let omega = mode.natural_frequency_rad_s;
    let damping = mode.damping_ratio;
    let tau = feed.feed_time_constant_s;
    let zero = feed.mass_flow_gain_time_s;
    let gain = feed.effective_gain_rad2;
    let omega_sq = omega * omega;
    let coefficients = [
        tau,
        1.0 + 2.0 * damping * omega * tau,
        2.0 * damping * omega + tau * omega_sq - gain * zero,
        omega_sq - gain,
    ];
    require_all_finite_array("pogo.closed_loop.characteristic", &coefficients)?;

    let eigenvalues = cubic_roots(coefficients)?;
    let mut max_real_part = f64::NEG_INFINITY;
    for eigenvalue in eigenvalues {
        require_finite_complex("pogo.closed_loop.eigenvalue", eigenvalue)?;
        max_real_part = max_real_part.max(eigenvalue.re);
    }
    let verdict = if max_real_part < 0.0 {
        PogoClosedLoopVerdict::AsymptoticallyStable
    } else {
        PogoClosedLoopVerdict::RightHalfPlaneOrMarginal
    };
    Ok(PogoClosedLoopEigenReport {
        characteristic_coefficients: coefficients,
        eigenvalues,
        max_real_part,
        verdict,
    })
}

/// Evaluate the manufactured-solution residual for the reduced first-order
/// feed/structure POGO transient.
///
/// The evaluated equations are
///
/// ```text
/// tau * z_dot + z - q = 0
/// q_ddot + 2*zeta*omega*q_dot + omega^2*q
///   - gain * (z + zero*z_dot) - source = 0
/// ```
///
/// where `q` is the structural modal coordinate, `z` is the first-order feed
/// state, and `source` is the manufactured structural source acceleration.
///
/// # Errors
///
/// Returns [`StructDynError`] when the mode, feed, sample, or residual values
/// are invalid.
pub fn evaluate_first_order_mms_residual(
    mode: LongitudinalPogoMode,
    feed: PogoFirstOrderFeed,
    sample: PogoFirstOrderMmsSample,
) -> Result<PogoFirstOrderMmsResidual, StructDynError> {
    mode.require_valid()?;
    feed.require_valid()?;
    sample.require_valid()?;

    let omega = mode.natural_frequency_rad_s;
    let damping_term = 2.0 * mode.damping_ratio * omega * sample.velocity_m_s;
    let stiffness_term = omega * omega * sample.displacement_m;
    let feed_feedback_m_s2 = feed.effective_gain_rad2
        * (sample.feed_state_m + feed.mass_flow_gain_time_s * sample.feed_state_rate_m_s);
    let feed_residual_m = feed.feed_time_constant_s * sample.feed_state_rate_m_s
        + sample.feed_state_m
        - sample.displacement_m;
    let structural_residual_m_s2 = sample.acceleration_m_s2 + damping_term + stiffness_term
        - feed_feedback_m_s2
        - sample.manufactured_source_m_s2;
    let max_abs_residual = feed_residual_m.abs().max(structural_residual_m_s2.abs());
    require_all_finite_array(
        "pogo.first_order_mms.residual",
        &[feed_residual_m, structural_residual_m_s2, max_abs_residual],
    )?;
    Ok(PogoFirstOrderMmsResidual {
        feed_residual_m,
        structural_residual_m_s2,
        max_abs_residual,
    })
}

/// Structural POGO sampled loop-margin evaluator.
#[derive(Clone, Debug, PartialEq)]
pub struct PogoLoopMargin {
    modes: Vec<LongitudinalPogoMode>,
    feed_samples: Vec<FeedTransferSample>,
    nyquist_clearance_min: f64,
}

impl PogoLoopMargin {
    /// Construct a sampled structural POGO loop evaluator.
    ///
    /// The feed samples must be in strictly increasing angular-frequency order
    /// and the clearance threshold must be positive.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when modes, feed samples, or the clearance
    /// threshold are invalid.
    pub fn new(
        modes: Vec<LongitudinalPogoMode>,
        feed_samples: Vec<FeedTransferSample>,
        nyquist_clearance_min: f64,
    ) -> Result<Self, StructDynError> {
        if modes.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "pogo.modes",
                expected: 1,
                actual: 0,
            });
        }
        let mut nonzero_coupling = false;
        for mode in &modes {
            mode.require_valid()?;
            nonzero_coupling |= mode.coupling_factor != 0.0;
        }
        if !nonzero_coupling {
            return Err(StructDynError::InvalidNumber {
                field: "pogo.mode.coupling_factor",
            });
        }
        validate_feed_grid(&feed_samples)?;
        require_positive_finite("pogo.nyquist_clearance_min", nyquist_clearance_min)?;
        Ok(Self {
            modes,
            feed_samples,
            nyquist_clearance_min,
        })
    }

    /// Structural modes in locked order.
    #[must_use]
    pub fn modes(&self) -> &[LongitudinalPogoMode] {
        &self.modes
    }

    /// Feed transfer samples in locked frequency order.
    #[must_use]
    pub fn feed_samples(&self) -> &[FeedTransferSample] {
        &self.feed_samples
    }

    /// Minimum sampled Nyquist clearance required for a clear verdict.
    #[must_use]
    pub const fn nyquist_clearance_min(&self) -> f64 {
        self.nyquist_clearance_min
    }

    /// Return a copy with every feed transfer scaled by `scale`.
    ///
    /// This is a structural-side hook for accumulator detuning or feed
    /// dispersion sweeps computed by the feed-system layer.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when `scale` is negative or non-finite.
    pub fn with_feed_transfer_scale(&self, scale: f64) -> Result<Self, StructDynError> {
        require_nonnegative_finite("pogo.feed.scale", scale)?;
        let feed_samples = self
            .feed_samples
            .iter()
            .map(|sample| FeedTransferSample {
                omega_rad_s: sample.omega_rad_s,
                transfer_n_per_m: sample.transfer_n_per_m * scale,
            })
            .collect();
        Self::new(self.modes.clone(), feed_samples, self.nyquist_clearance_min)
    }

    /// Evaluate the sampled structural POGO open-loop margin.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any receptance or open-loop sample is
    /// non-finite.
    pub fn analyze(&self) -> Result<PogoNyquistReport, StructDynError> {
        let mut samples = Vec::with_capacity(self.feed_samples.len());
        let mut min_distance_to_minus_one = f64::INFINITY;
        let mut min_distance_frequency_rad_s = 0.0;
        let mut peak_gain = 0.0;
        let mut peak_gain_frequency_rad_s = 0.0;

        for feed in &self.feed_samples {
            let mut structural_receptance_m_per_n = Complex64::new(0.0, 0.0);
            for mode in &self.modes {
                structural_receptance_m_per_n += mode.receptance_at(feed.omega_rad_s)?;
            }
            require_finite_complex("pogo.structural_receptance", structural_receptance_m_per_n)?;

            let open_loop = structural_receptance_m_per_n * feed.transfer_n_per_m;
            require_finite_complex("pogo.open_loop", open_loop)?;
            let distance_to_minus_one = complex_abs(Complex64::new(1.0, 0.0) + open_loop);
            let gain = complex_abs(open_loop);
            let phase_deg = open_loop.im.atan2(open_loop.re).to_degrees();
            for value in [distance_to_minus_one, gain, phase_deg] {
                if !value.is_finite() {
                    return Err(StructDynError::InvalidNumber {
                        field: "pogo.open_loop",
                    });
                }
            }

            if distance_to_minus_one < min_distance_to_minus_one {
                min_distance_to_minus_one = distance_to_minus_one;
                min_distance_frequency_rad_s = feed.omega_rad_s;
            }
            if gain > peak_gain {
                peak_gain = gain;
                peak_gain_frequency_rad_s = feed.omega_rad_s;
            }

            samples.push(PogoOpenLoopSample {
                omega_rad_s: feed.omega_rad_s,
                structural_receptance_m_per_n,
                feed_transfer_n_per_m: feed.transfer_n_per_m,
                open_loop,
                distance_to_minus_one,
                gain,
                phase_deg,
            });
        }

        let verdict = if min_distance_to_minus_one <= self.nyquist_clearance_min {
            PogoNyquistVerdict::BelowClearance
        } else {
            PogoNyquistVerdict::Clear
        };
        Ok(PogoNyquistReport {
            samples,
            min_distance_to_minus_one,
            min_distance_frequency_rad_s,
            peak_gain,
            peak_gain_frequency_rad_s,
            verdict,
        })
    }
}

fn polynomial_multiply(lhs: &[f64], rhs: &[f64]) -> Result<Vec<f64>, StructDynError> {
    if lhs.is_empty() || rhs.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.polynomial",
            expected: 1,
            actual: 0,
        });
    }
    require_all_finite_array("pogo.polynomial.lhs", lhs)?;
    require_all_finite_array("pogo.polynomial.rhs", rhs)?;
    let mut out = vec![0.0; lhs.len() + rhs.len() - 1];
    for (lhs_index, lhs_value) in lhs.iter().enumerate() {
        for (rhs_index, rhs_value) in rhs.iter().enumerate() {
            out[lhs_index + rhs_index] += lhs_value * rhs_value;
        }
    }
    require_all_finite_array("pogo.polynomial.product", &out)?;
    Ok(out)
}

fn subtract_lower_order_polynomial(
    mut lhs: Vec<f64>,
    rhs: &[f64],
) -> Result<Vec<f64>, StructDynError> {
    if lhs.len() < rhs.len() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.polynomial.feedback",
            expected: lhs.len(),
            actual: rhs.len(),
        });
    }
    require_all_finite_array("pogo.polynomial.lhs", &lhs)?;
    require_all_finite_array("pogo.polynomial.rhs", rhs)?;
    let offset = lhs.len() - rhs.len();
    for (index, rhs_value) in rhs.iter().enumerate() {
        lhs[offset + index] -= rhs_value;
    }
    require_all_finite_array("pogo.polynomial.difference", &lhs)?;
    Ok(lhs)
}

fn real_polynomial_roots(coefficients: &[f64]) -> Result<Vec<Complex64>, StructDynError> {
    if coefficients.len() < 2 {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.polynomial.coefficients",
            expected: 2,
            actual: coefficients.len(),
        });
    }
    require_all_finite_array("pogo.polynomial.coefficients", coefficients)?;
    if coefficients.len() == 4 {
        return cubic_roots([
            coefficients[0],
            coefficients[1],
            coefficients[2],
            coefficients[3],
        ])
        .map(|roots| roots.to_vec());
    }

    let degree = coefficients.len() - 1;
    require_positive_finite("pogo.polynomial.leading_coefficient", coefficients[0])?;
    let leading = coefficients[0];
    let scale = polynomial_root_scale(coefficients, leading)?;
    let mut monic_scaled = Vec::with_capacity(coefficients.len());
    for (index, coefficient) in coefficients.iter().enumerate() {
        let scaled = coefficient / leading / scale.powi(index as i32);
        require_finite("pogo.polynomial.scaled_coefficient", scaled)?;
        monic_scaled.push(scaled);
    }

    let mut roots = Vec::with_capacity(degree);
    for index in 0..degree {
        let angle = std::f64::consts::TAU * index as f64 / degree as f64
            + std::f64::consts::FRAC_PI_4 / degree as f64;
        let radius = 0.85 + 0.1 * index as f64 / degree as f64;
        roots.push(Complex64::new(radius * angle.cos(), radius * angle.sin()));
    }

    const MAX_ITERATIONS: usize = 512;
    const TOLERANCE: f64 = 1.0e-13;
    let mut converged = false;
    for _ in 0..MAX_ITERATIONS {
        let previous = roots.clone();
        let mut max_update = 0.0_f64;
        for root_index in 0..degree {
            let value = evaluate_real_polynomial_complex(&monic_scaled, previous[root_index]);
            let mut denominator = Complex64::new(1.0, 0.0);
            for other_index in 0..degree {
                if other_index != root_index {
                    denominator *= previous[root_index] - previous[other_index];
                }
            }
            if denominator.norm_sqr() == 0.0 {
                return Err(StructDynError::InvalidNumber {
                    field: "pogo.polynomial.root_denominator",
                });
            }
            let update = value / denominator;
            roots[root_index] = previous[root_index] - update;
            max_update = max_update.max(update.norm());
        }
        if max_update < TOLERANCE {
            converged = true;
            break;
        }
    }
    if !converged {
        return Err(StructDynError::InvalidNumber {
            field: "pogo.polynomial.roots",
        });
    }
    for root in &mut roots {
        *root *= scale;
        require_finite_complex("pogo.polynomial.root", *root)?;
    }
    roots.sort_by(|lhs, rhs| lhs.re.total_cmp(&rhs.re).then(lhs.im.total_cmp(&rhs.im)));
    Ok(roots)
}

fn polynomial_root_scale(coefficients: &[f64], leading: f64) -> Result<f64, StructDynError> {
    let mut scale = 1.0_f64;
    for (index, coefficient) in coefficients.iter().enumerate().skip(1) {
        let normalized = (coefficient / leading).abs();
        require_finite("pogo.polynomial.normalized_coefficient", normalized)?;
        if normalized > 0.0 {
            scale = scale.max(normalized.powf(1.0 / index as f64));
        }
    }
    let scale = 1.0 + scale;
    require_positive_finite("pogo.polynomial.root_scale", scale)?;
    Ok(scale)
}

fn evaluate_real_polynomial_complex(coefficients: &[f64], root: Complex64) -> Complex64 {
    let mut value = Complex64::new(0.0, 0.0);
    for coefficient in coefficients {
        value = value * root + Complex64::new(*coefficient, 0.0);
    }
    value
}

fn cubic_roots(coefficients: [f64; 4]) -> Result<[Complex64; 3], StructDynError> {
    let [a3, a2, a1, a0] = coefficients;
    require_positive_finite("pogo.closed_loop.a3", a3)?;
    let a = a2 / a3;
    let b = a1 / a3;
    let c = a0 / a3;
    let p = b - a * a / 3.0;
    let q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;
    require_finite("pogo.closed_loop.depressed_p", p)?;
    require_finite("pogo.closed_loop.depressed_q", q)?;

    let discriminant = q * q / 4.0 + p * p * p / 27.0;
    require_finite("pogo.closed_loop.discriminant", discriminant)?;
    let shift = -a / 3.0;
    let roots = if discriminant >= 0.0 {
        let sqrt_disc = discriminant.sqrt();
        let u = (-q / 2.0 + sqrt_disc).cbrt();
        let v = (-q / 2.0 - sqrt_disc).cbrt();
        let real = u + v;
        let pair_real = -0.5 * real + shift;
        let pair_imag = 0.5 * 3.0_f64.sqrt() * (u - v);
        [
            Complex64::new(real + shift, 0.0),
            Complex64::new(pair_real, pair_imag),
            Complex64::new(pair_real, -pair_imag),
        ]
    } else {
        if p >= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "pogo.closed_loop.depressed_p",
            });
        }
        let radius = 2.0 * (-p / 3.0).sqrt();
        let acos_arg = ((3.0 * q) / (2.0 * p)) * (-3.0 / p).sqrt();
        let angle = acos_arg.clamp(-1.0, 1.0).acos() / 3.0;
        [
            Complex64::new(radius * angle.cos() + shift, 0.0),
            Complex64::new(
                radius * (angle - std::f64::consts::TAU / 3.0).cos() + shift,
                0.0,
            ),
            Complex64::new(
                radius * (angle - 2.0 * std::f64::consts::TAU / 3.0).cos() + shift,
                0.0,
            ),
        ]
    };
    for root in &roots {
        require_finite_complex("pogo.closed_loop.root", *root)?;
    }
    Ok(roots)
}

fn validate_feed_grid(feed_samples: &[FeedTransferSample]) -> Result<(), StructDynError> {
    if feed_samples.len() < 2 {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.feed_samples",
            expected: 2,
            actual: feed_samples.len(),
        });
    }

    let mut previous_omega = None;
    for sample in feed_samples {
        sample.require_valid()?;
        if let Some(previous) = previous_omega
            && sample.omega_rad_s <= previous
        {
            return Err(StructDynError::InvalidNumber {
                field: "pogo.feed.omega_rad_s",
            });
        }
        previous_omega = Some(sample.omega_rad_s);
    }
    Ok(())
}

fn validate_frequency_grid(field: &'static str, values: &[f64]) -> Result<(), StructDynError> {
    if values.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: 1,
            actual: 0,
        });
    }
    let mut previous = None;
    for value in values {
        require_nonnegative_finite(field, *value)?;
        if let Some(previous_value) = previous
            && *value <= previous_value
        {
            return Err(StructDynError::InvalidNumber { field });
        }
        previous = Some(*value);
    }
    Ok(())
}

fn equivalent_uniform_feedline_segment(
    segments: &[PogoFeedlineSegment],
) -> Result<PogoFeedlineSegment, StructDynError> {
    if segments.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "pogo.feedline.uniform_segments",
            expected: 1,
            actual: 0,
        });
    }
    let first = segments[0];
    first.require_valid()?;
    let mut total_length_m = 0.0_f64;
    for segment in segments {
        segment.require_valid()?;
        if segment.wave_speed_m_s.to_bits() != first.wave_speed_m_s.to_bits()
            || segment.density_kg_m3.to_bits() != first.density_kg_m3.to_bits()
            || segment.cross_section_area_m2.to_bits() != first.cross_section_area_m2.to_bits()
        {
            return Err(StructDynError::InvalidMatrix {
                field: "pogo.feedline.uniform_medium",
                reason: "segments must share wave speed, density, and cross-sectional area",
            });
        }
        total_length_m += segment.length_m;
        require_finite("pogo.feedline.total_length_m", total_length_m)?;
    }
    PogoFeedlineSegment::new(
        total_length_m,
        first.wave_speed_m_s,
        first.density_kg_m3,
        first.cross_section_area_m2,
    )
}

fn feedline_matrix_relative_residual(
    lhs: PogoFeedlineTransferMatrix,
    rhs: PogoFeedlineTransferMatrix,
) -> Result<f64, StructDynError> {
    let mut max_residual = 0.0_f64;
    for (lhs_value, rhs_value) in [
        (lhs.a, rhs.a),
        (lhs.b, rhs.b),
        (lhs.c, rhs.c),
        (lhs.d, rhs.d),
    ] {
        let scale = lhs_value.norm().max(rhs_value.norm()).max(1.0);
        let residual = (lhs_value - rhs_value).norm() / scale;
        require_finite("pogo.feedline.benchmark.matrix_residual", residual)?;
        max_residual = max_residual.max(residual);
    }
    Ok(max_residual)
}

fn require_positive_finite(field: &'static str, value: f64) -> Result<(), StructDynError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(StructDynError::InvalidNumber { field });
    }
    Ok(())
}

fn require_nonnegative_finite(field: &'static str, value: f64) -> Result<(), StructDynError> {
    if !value.is_finite() || value < 0.0 {
        return Err(StructDynError::InvalidNumber { field });
    }
    Ok(())
}

fn require_finite(field: &'static str, value: f64) -> Result<(), StructDynError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn require_nonempty_str(field: &'static str, value: &str) -> Result<(), StructDynError> {
    if value.trim().is_empty() {
        return Err(StructDynError::InvalidString { field });
    }
    Ok(())
}

fn require_all_finite_array(field: &'static str, values: &[f64]) -> Result<(), StructDynError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn require_ordered_box_value(
    field: &'static str,
    min: f64,
    nominal: f64,
    max: f64,
) -> Result<(), StructDynError> {
    if min <= nominal && nominal <= max {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn require_finite_complex(field: &'static str, value: Complex64) -> Result<(), StructDynError> {
    if value.re.is_finite() && value.im.is_finite() {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn require_feedline_matrix_finite(
    matrix: PogoFeedlineTransferMatrix,
) -> Result<(), StructDynError> {
    for value in [matrix.a, matrix.b, matrix.c, matrix.d] {
        require_finite_complex("pogo.feedline.matrix", value)?;
    }
    Ok(())
}

fn complex_abs(value: Complex64) -> f64 {
    (value.re * value.re + value.im * value.im).sqrt()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn sample(omega_rad_s: f64, transfer_n_per_m: f64) -> FeedTransferSample {
        FeedTransferSample::new(omega_rad_s, Complex64::new(transfer_n_per_m, 0.0)).unwrap()
    }

    fn margin(transfer_n_per_m: f64) -> PogoLoopMargin {
        PogoLoopMargin::new(
            vec![LongitudinalPogoMode::new(60.0, 0.04, 1_000.0, 1.0).unwrap()],
            vec![
                sample(0.0, transfer_n_per_m),
                sample(20.0, transfer_n_per_m),
                sample(60.0, transfer_n_per_m),
                sample(100.0, transfer_n_per_m),
            ],
            0.05,
        )
        .unwrap()
    }

    fn feedline_segment(length_m: f64) -> PogoFeedlineSegment {
        PogoFeedlineSegment::new(length_m, 1_000.0, 1_000.0, 0.01).unwrap()
    }

    fn closed_loop_mode() -> LongitudinalPogoMode {
        LongitudinalPogoMode::new(60.0, 0.04, 1_000.0, 1.0).unwrap()
    }

    fn closed_loop_feed(effective_gain_rad2: f64) -> PogoFirstOrderFeed {
        PogoFirstOrderFeed::new(effective_gain_rad2, 0.02, 0.004).unwrap()
    }

    fn pump_dispersion_box() -> PogoPumpDispersionBox {
        PogoPumpDispersionBox::new(
            PogoPumpDispersionPoint::new(1.0e-9, 0.002).unwrap(),
            PogoPumpDispersionPoint::new(2.0e-9, 0.004).unwrap(),
            PogoPumpDispersionPoint::new(4.0e-9, 0.006).unwrap(),
        )
        .unwrap()
    }

    fn lumped_feedline_model(open_loop_gain_rad2: f64) -> PogoLumpedFeedlineModel {
        PogoLumpedFeedlineModel::new(
            vec![feedline_segment(20.0), feedline_segment(10.0)],
            PogoPumpDispersionPoint::new(2.0e-9, 0.004).unwrap(),
            1.0e-9,
            open_loop_gain_rad2,
            0.02,
            0.08,
        )
        .unwrap()
    }

    fn characteristic_at(coefficients: [f64; 4], root: Complex64) -> Complex64 {
        let [a3, a2, a1, a0] = coefficients;
        ((root * a3 + Complex64::new(a2, 0.0)) * root + Complex64::new(a1, 0.0)) * root
            + Complex64::new(a0, 0.0)
    }

    fn characteristic_slice_at(coefficients: &[f64], root: Complex64) -> Complex64 {
        coefficients
            .iter()
            .fold(Complex64::new(0.0, 0.0), |value, coefficient| {
                value * root + Complex64::new(*coefficient, 0.0)
            })
    }

    fn relative_characteristic_residual(coefficients: &[f64], root: Complex64) -> f64 {
        let residual = characteristic_slice_at(coefficients, root).norm();
        let degree = coefficients.len() - 1;
        let scale = coefficients
            .iter()
            .enumerate()
            .map(|(index, coefficient)| {
                coefficient.abs() * root.norm().powi((degree - index) as i32)
            })
            .sum::<f64>()
            .max(1.0);
        residual / scale
    }

    #[test]
    fn pogo_structural_receptance_matches_modal_closed_form() {
        let mode = LongitudinalPogoMode::new(60.0, 0.04, 1_000.0, 2.0).unwrap();
        let receptance = mode.receptance_at(0.0).unwrap();
        let expected = 4.0 / (1_000.0 * 60.0 * 60.0);

        assert!((receptance.re - expected).abs() < 1.0e-18);
        assert_eq!(receptance.im.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn feedline_transfer_matrix_matches_low_frequency_lumped_line() {
        let segment = feedline_segment(40.0);
        let omega = 0.1;
        let matrix = assemble_feedline_transfer_matrix(&[segment], omega).unwrap();

        assert!((matrix.a.re - 1.0).abs() < 1.0e-5);
        assert_eq!(matrix.a.im.to_bits(), 0.0_f64.to_bits());
        assert!((matrix.d.re - matrix.a.re).abs() < 1.0e-15);
        assert!((matrix.b.im / (omega * segment.inertance_pa_s2_per_m3()) - 1.0).abs() < 3.0e-6);
        assert!((matrix.c.im / (omega * segment.compliance_m3_per_pa()) - 1.0).abs() < 3.0e-6);
        assert!((matrix.determinant().re - 1.0).abs() < 1.0e-15);
        assert!(matrix.determinant().im.abs() < 1.0e-15);
    }

    #[test]
    fn feedline_transfer_matrix_cascades_identical_medium_segments() {
        let first = feedline_segment(10.0);
        let second = feedline_segment(15.0);
        let omega = 20.0;
        let cascaded = assemble_feedline_transfer_matrix(&[first, second], omega).unwrap();
        let single = assemble_feedline_transfer_matrix(&[feedline_segment(25.0)], omega).unwrap();

        assert!((cascaded.a - single.a).norm() < 1.0e-12);
        assert!((cascaded.b - single.b).norm() < 1.0e-7);
        assert!((cascaded.c - single.c).norm() < 1.0e-18);
        assert!((cascaded.d - single.d).norm() < 1.0e-12);
    }

    #[test]
    fn distributed_feedline_benchmark_reports_closed_form_residuals() {
        let report = evaluate_uniform_distributed_feedline_benchmark(
            &[
                feedline_segment(7.5),
                feedline_segment(12.5),
                feedline_segment(20.0),
            ],
            &[0.0, 5.0, 20.0, 75.0],
        )
        .unwrap();

        assert_eq!(report.samples().len(), 4);
        assert!(report.max_determinant_residual < 1.0e-12);
        assert!(report.max_transfer_matrix_residual < 1.0e-12);
        assert!(report.max_normalized_off_diagonal_residual < 1.0e-12);
        assert_eq!(report.samples()[0].omega_rad_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            report.samples()[0].cascaded_matrix,
            PogoFeedlineTransferMatrix::identity()
        );
        for sample in report.samples() {
            assert!(sample.determinant_residual <= report.max_determinant_residual);
            assert!(sample.transfer_matrix_residual <= report.max_transfer_matrix_residual);
            assert!(
                sample.normalized_off_diagonal_residual
                    <= report.max_normalized_off_diagonal_residual
            );
        }
    }

    #[test]
    fn distributed_feedline_benchmark_rejects_nonuniform_medium_and_bad_grid() {
        assert!(
            evaluate_uniform_distributed_feedline_benchmark(&[feedline_segment(10.0)], &[])
                .is_err()
        );
        assert!(
            evaluate_uniform_distributed_feedline_benchmark(
                &[feedline_segment(10.0)],
                &[1.0, 1.0],
            )
            .is_err()
        );
        assert!(
            evaluate_uniform_distributed_feedline_benchmark(
                &[
                    feedline_segment(10.0),
                    PogoFeedlineSegment::new(10.0, 1_100.0, 1_000.0, 0.01).unwrap(),
                ],
                &[1.0, 2.0],
            )
            .is_err()
        );
    }

    #[test]
    fn public_pogo_case_history_fixture_carries_titan_saturn_accumulator_trends() {
        let deck = include_str!("../../../data/feed_system/public-pogo-case-history-v1.toml");
        let report = parse_public_pogo_case_history_toml(deck).unwrap();

        assert_eq!(report.dataset_id, PUBLIC_POGO_CASE_HISTORY_DATASET_ID);
        assert_eq!(report.source_class, "public-case-history");
        assert_eq!(report.validation, "research-method-level");
        assert_eq!(report.sources.len(), 2);
        assert!(report.sources.iter().all(|source| source.public));
        assert!(report.sources.iter().any(|source| {
            source.document_id == "19710016604" && source.report_number == "NASA-SP-8055"
        }));
        assert!(report.sources.iter().any(|source| {
            source.document_id == "20080018689" && source.report_number == "RTO-MP-AVT-152"
        }));

        for (case_id, expected_band, expected_peak_g) in [
            ("gemini_titan_ii", [10.0, 13.0], 2.5),
            ("apollo_saturn_v_sic", [5.0, 5.5], 0.6),
            ("apollo_saturn_v_sii", [17.0, 18.0], 34.0),
        ] as [(&str, [f64; 2], f64); 3]
        {
            let case = report.cases.iter().find(|case| case.id == case_id).unwrap();
            assert!(case.observed_pogo);
            assert_eq!(
                case.public_frequency_band_hz[0].to_bits(),
                expected_band[0].to_bits()
            );
            assert_eq!(
                case.public_frequency_band_hz[1].to_bits(),
                expected_band[1].to_bits()
            );
            assert_eq!(
                case.public_peak_acceleration_g.unwrap().to_bits(),
                expected_peak_g.to_bits()
            );
            assert_eq!(case.accumulator_trend, "raises_margin");
            assert!(
                case.model_expectation
                    .contains("stability verdict or sampled margin")
            );
            assert!(!case.source_ids.is_empty());
        }
    }

    #[test]
    fn public_pogo_case_history_parser_rejects_bad_references_and_bands() {
        let deck = include_str!("../../../data/feed_system/public-pogo-case-history-v1.toml");
        let bad_source = deck.replace(
            "source_ids = [\"nasa_sp_8055\", \"larsen_2008\"]",
            "source_ids = [\"missing\"]",
        );
        assert!(matches!(
            parse_public_pogo_case_history_toml(&bad_source),
            Err(StructDynError::InvalidString {
                field: "pogo.case_history.case.source_ids"
            })
        ));

        let bad_band = deck.replace(
            "public_frequency_band_hz = [10.0, 13.0]",
            "public_frequency_band_hz = [13.0, 10.0]",
        );
        assert!(matches!(
            parse_public_pogo_case_history_toml(&bad_band),
            Err(StructDynError::InvalidNumber {
                field: "pogo.case_history.case.public_frequency_band_hz"
            })
        ));
    }

    #[test]
    fn pogo_first_order_closed_loop_reports_stable_eigenvalues() {
        let report =
            analyze_first_order_closed_loop(closed_loop_mode(), closed_loop_feed(500.0)).unwrap();

        assert_eq!(report.verdict, PogoClosedLoopVerdict::AsymptoticallyStable);
        assert!(report.max_real_part < 0.0);
        assert_eq!(
            report.characteristic_coefficients[0].to_bits(),
            0.02_f64.to_bits()
        );
        assert_eq!(
            report.characteristic_coefficients[3].to_bits(),
            3_100.0_f64.to_bits()
        );
        for eigenvalue in report.eigenvalues {
            assert!(eigenvalue.re < 0.0);
            assert!(
                characteristic_at(report.characteristic_coefficients, eigenvalue).norm() < 1.0e-8
            );
        }
    }

    #[test]
    fn pogo_first_order_closed_loop_flags_rhp_eigenvalue() {
        let report =
            analyze_first_order_closed_loop(closed_loop_mode(), closed_loop_feed(4_000.0)).unwrap();

        assert_eq!(
            report.verdict,
            PogoClosedLoopVerdict::RightHalfPlaneOrMarginal
        );
        assert!(report.max_real_part > 0.0);
        assert!(
            report
                .eigenvalues
                .iter()
                .any(|eigenvalue| eigenvalue.re > 0.0)
        );
        for eigenvalue in report.eigenvalues {
            assert!(
                characteristic_at(report.characteristic_coefficients, eigenvalue).norm() < 1.0e-8
            );
        }
    }

    #[test]
    fn pogo_first_order_mms_recovers_manufactured_transient_residual() {
        let mode = closed_loop_mode();
        let feed = closed_loop_feed(500.0);
        let manufactured_frequency_rad_s = 7.0_f64;
        let time_s = 0.37_f64;
        let phase: f64 = manufactured_frequency_rad_s * time_s;
        let feed_state_m = phase.sin();
        let feed_state_rate_m_s = manufactured_frequency_rad_s * phase.cos();
        let feed_state_acceleration_m_s2 =
            -manufactured_frequency_rad_s * manufactured_frequency_rad_s * phase.sin();
        let feed_state_jerk_m_s3 = -manufactured_frequency_rad_s
            * manufactured_frequency_rad_s
            * manufactured_frequency_rad_s
            * phase.cos();
        let displacement_m = feed.feed_time_constant_s * feed_state_rate_m_s + feed_state_m;
        let velocity_m_s =
            feed.feed_time_constant_s * feed_state_acceleration_m_s2 + feed_state_rate_m_s;
        let acceleration_m_s2 =
            feed.feed_time_constant_s * feed_state_jerk_m_s3 + feed_state_acceleration_m_s2;
        let manufactured_source_m_s2 = acceleration_m_s2
            + 2.0 * mode.damping_ratio * mode.natural_frequency_rad_s * velocity_m_s
            + mode.natural_frequency_rad_s * mode.natural_frequency_rad_s * displacement_m
            - feed.effective_gain_rad2
                * (feed_state_m + feed.mass_flow_gain_time_s * feed_state_rate_m_s);
        let sample = PogoFirstOrderMmsSample::new(
            displacement_m,
            velocity_m_s,
            acceleration_m_s2,
            feed_state_m,
            feed_state_rate_m_s,
            manufactured_source_m_s2,
        )
        .unwrap();

        let residual = evaluate_first_order_mms_residual(mode, feed, sample).unwrap();

        assert!(residual.feed_residual_m.abs() < 1.0e-12);
        assert!(residual.structural_residual_m_s2.abs() < 1.0e-10);
        assert!(residual.max_abs_residual < 1.0e-10);
    }

    #[test]
    fn pogo_first_order_mms_rejects_invalid_sample() {
        let sample = PogoFirstOrderMmsSample::new(0.0, 0.0, f64::NAN, 0.0, 0.0, 0.0).unwrap_err();

        assert!(matches!(
            sample,
            StructDynError::InvalidNumber {
                field: "pogo.first_order_mms.sample"
            }
        ));
    }

    #[test]
    fn pump_dispersion_box_reports_worst_closed_loop_corner() {
        let report = analyze_pump_dispersion_box(
            closed_loop_mode(),
            5_000.0,
            0.02,
            1.0e-9,
            pump_dispersion_box(),
        )
        .unwrap();

        assert_eq!(report.samples().len(), 3);
        assert_eq!(report.worst_sample_index, 2);
        assert_eq!(
            report.worst_verdict,
            PogoClosedLoopVerdict::RightHalfPlaneOrMarginal
        );
        assert!(report.worst_max_real_part > 0.0);
        assert!(report.samples()[2].effective_gain_rad2 > report.samples()[0].effective_gain_rad2);
        assert!(
            report.samples()[2].closed_loop.max_real_part
                > report.samples()[0].closed_loop.max_real_part
        );
    }

    #[test]
    fn pump_dispersion_box_rejects_invalid_order_and_inputs() {
        assert!(
            PogoPumpDispersionBox::new(
                PogoPumpDispersionPoint::new(2.0e-9, 0.002).unwrap(),
                PogoPumpDispersionPoint::new(1.0e-9, 0.004).unwrap(),
                PogoPumpDispersionPoint::new(4.0e-9, 0.006).unwrap(),
            )
            .is_err()
        );
        assert!(PogoPumpDispersionPoint::new(0.0, 0.004).is_err());
        assert!(
            analyze_pump_dispersion_box(
                closed_loop_mode(),
                500.0,
                0.0,
                1.0e-9,
                pump_dispersion_box()
            )
            .is_err()
        );
    }

    #[test]
    fn lumped_feedline_coupled_eigenproblem_reports_characteristic_roots() {
        let report = analyze_lumped_feedline_coupled_eigenproblem(
            closed_loop_mode(),
            &lumped_feedline_model(500.0),
        )
        .unwrap();

        assert_eq!(report.characteristic_coefficients.len(), 6);
        assert_eq!(report.eigenvalues().len(), 5);
        assert_eq!(report.verdict, PogoClosedLoopVerdict::AsymptoticallyStable);
        assert!(report.feedline_inertance_pa_s2_per_m3 > 0.0);
        assert!(report.feedline_compliance_m3_per_pa > 0.0);
        assert!(report.accumulator_attenuation > 0.0);
        assert!(report.accumulator_attenuation < 1.0);
        assert!(report.effective_gain_rad2 < 500.0);
        for eigenvalue in report.eigenvalues() {
            assert!(eigenvalue.re < 0.0);
            assert!(
                relative_characteristic_residual(&report.characteristic_coefficients, *eigenvalue)
                    < 1.0e-10
            );
        }
    }

    #[test]
    fn lumped_feedline_coupled_eigenproblem_rejects_invalid_inputs() {
        assert!(
            PogoLumpedFeedlineModel::new(
                Vec::new(),
                PogoPumpDispersionPoint::new(2.0e-9, 0.004).unwrap(),
                1.0e-9,
                500.0,
                0.02,
                0.08,
            )
            .is_err()
        );
        assert!(
            PogoLumpedFeedlineModel::new(
                vec![feedline_segment(20.0)],
                PogoPumpDispersionPoint::new(2.0e-9, 0.004).unwrap(),
                1.0e-9,
                500.0,
                0.02,
                -0.01,
            )
            .is_err()
        );
        let invalid_mode = LongitudinalPogoMode {
            natural_frequency_rad_s: 60.0,
            damping_ratio: -0.01,
            modal_mass_kg: 1_000.0,
            coupling_factor: 1.0,
        };
        assert!(
            analyze_lumped_feedline_coupled_eigenproblem(
                invalid_mode,
                &lumped_feedline_model(500.0)
            )
            .is_err()
        );
    }

    #[test]
    fn pogo_loop_margin_evaluates_sampled_nyquist_clearance() {
        let report = margin(-3.0e6).analyze().unwrap();
        let first = report.samples()[0];

        assert_eq!(report.verdict, PogoNyquistVerdict::Clear);
        assert!((first.open_loop.re + 0.833_333_333_333_333_4).abs() < 1.0e-15);
        assert_eq!(first.open_loop.im.to_bits(), 0.0_f64.to_bits());
        assert!((first.distance_to_minus_one - (1.0 / 6.0)).abs() < 1.0e-15);
        assert_eq!(
            report.min_distance_frequency_rad_s.to_bits(),
            20.0_f64.to_bits()
        );
        assert!(report.peak_gain > 10.0);
    }

    #[test]
    fn pogo_loop_margin_flags_samples_below_clearance() {
        let report = margin(-3.55e6).analyze().unwrap();

        assert_eq!(report.verdict, PogoNyquistVerdict::BelowClearance);
        assert!(report.min_distance_to_minus_one < 0.05);
        assert_eq!(
            report.min_distance_frequency_rad_s.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn pogo_feed_transfer_scaling_models_accumulator_detuning() {
        let baseline = margin(-3.55e6);
        let baseline_report = baseline.analyze().unwrap();
        let detuned_report = baseline
            .with_feed_transfer_scale(0.5)
            .unwrap()
            .analyze()
            .unwrap();

        assert_eq!(baseline_report.verdict, PogoNyquistVerdict::BelowClearance);
        assert_eq!(detuned_report.verdict, PogoNyquistVerdict::Clear);
        assert!(
            detuned_report.min_distance_to_minus_one > baseline_report.min_distance_to_minus_one
        );
        assert!(detuned_report.peak_gain < baseline_report.peak_gain);
    }

    #[test]
    fn pogo_loop_margin_rejects_invalid_inputs() {
        assert!(
            PogoLoopMargin::new(Vec::new(), vec![sample(0.0, -1.0), sample(1.0, -1.0)], 0.1)
                .is_err()
        );
        assert!(
            PogoLoopMargin::new(
                vec![LongitudinalPogoMode::new(60.0, 0.04, 1_000.0, 0.0).unwrap()],
                vec![sample(0.0, -1.0), sample(1.0, -1.0)],
                0.1,
            )
            .is_err()
        );
        assert!(
            PogoLoopMargin::new(
                vec![LongitudinalPogoMode::new(60.0, 0.04, 1_000.0, 1.0).unwrap()],
                vec![sample(1.0, -1.0), sample(1.0, -1.0)],
                0.1,
            )
            .is_err()
        );
        assert!(LongitudinalPogoMode::new(60.0, 0.04, 0.0, 1.0).is_err());
        assert!(FeedTransferSample::new(0.0, Complex64::new(f64::NAN, 0.0)).is_err());
        assert!(PogoFeedlineSegment::new(0.0, 1_000.0, 1_000.0, 0.01).is_err());
        assert!(assemble_feedline_transfer_matrix(&[], 1.0).is_err());
        assert!(assemble_feedline_transfer_matrix(&[feedline_segment(1.0)], -1.0).is_err());
        assert!(PogoFirstOrderFeed::new(1.0, 0.0, 0.0).is_err());
        assert!(
            analyze_first_order_closed_loop(
                closed_loop_mode(),
                PogoFirstOrderFeed {
                    effective_gain_rad2: f64::NAN,
                    feed_time_constant_s: 0.02,
                    mass_flow_gain_time_s: 0.0,
                },
            )
            .is_err()
        );
    }
}
