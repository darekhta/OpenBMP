//! Offline trajectory-optimization CLI commands.

use std::path::{Path, PathBuf};

use openbmp_trajopt::{DifferentialCorrector, TwoBodyApogeeTargeting};

use crate::CliError;

/// Summary returned by `openbmp trajopt correct-apogee`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeCliReport {
    /// Corrected tangential speed, in m/s.
    pub corrected_speed_m_s: f64,
    /// Final residual norm, in m.
    pub residual_norm_m: f64,
    /// Corrector iterations.
    pub iterations: usize,
    /// Number of reference-profile samples emitted into the I-load.
    pub reference_samples: usize,
    /// Serialized I-load size, in bytes.
    pub iload_bytes: usize,
    /// Written I-load path.
    pub output_iload: PathBuf,
}

/// Arguments for `openbmp trajopt correct-apogee`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeArgs {
    /// Initial inertial radius on the x-axis, in m.
    pub initial_radius_m: f64,
    /// Target apogee radius from the central body, in m.
    pub target_apogee_radius_m: f64,
    /// Optional initial tangential speed guess, in m/s.
    pub initial_speed_m_s: Option<f64>,
    /// Coast duration before residual evaluation, in s.
    pub coast_duration_s: f64,
    /// Fixed RK4 propagation step, in s.
    pub step_s: f64,
    /// Central-body gravitational parameter, in m^3/s^2.
    pub mu_m3_s2: f64,
    /// Residual convergence tolerance, in m.
    pub residual_tolerance_m: f64,
    /// Maximum Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Deterministic synthesis seed recorded in the I-load.
    pub synthesis_seed: u64,
    /// Opaque scenario or driver digest.
    pub scenario_digest: String,
    /// Source revision recorded in the I-load.
    pub source_revision: String,
    /// Producer name recorded in the I-load.
    pub producer: String,
}

/// Run the T0 two-body apogee corrector and write a postcard I-load.
///
/// # Errors
///
/// Returns [`CliError`] when the trajectory corrector rejects inputs,
/// reports non-convergence, or the I-load cannot be written.
pub fn run_correct_apogee(
    args: CorrectApogeeArgs,
    output_iload: &Path,
) -> Result<CorrectApogeeCliReport, CliError> {
    let initial_speed_m_s = args.initial_speed_m_s.unwrap_or_else(|| {
        if args.mu_m3_s2 > 0.0 && args.initial_radius_m > 0.0 {
            (args.mu_m3_s2 / args.initial_radius_m).sqrt()
        } else {
            f64::NAN
        }
    });
    let mut config = TwoBodyApogeeTargeting::wgs84(
        args.initial_radius_m,
        args.target_apogee_radius_m,
        initial_speed_m_s,
    );
    config.coast_duration_s = args.coast_duration_s;
    config.step_s = args.step_s;
    config.mu_m3_s2 = args.mu_m3_s2;
    config.corrector = DifferentialCorrector {
        residual_tolerance: args.residual_tolerance_m,
        max_iterations: args.max_iterations,
        ..DifferentialCorrector::default()
    };
    config.synthesis_seed = args.synthesis_seed;
    config.scenario_digest = args.scenario_digest;
    config.source_revision = args.source_revision;
    config.producer = args.producer;

    let report = openbmp_trajopt::correct_two_body_apogee(&config)?;
    if !report.correction.converged {
        return Err(CliError::Trajopt {
            summary: format!(
                "two-body apogee correction did not converge: residual_norm_m={:.12e}, iterations={}",
                report.correction.residual.norm, report.correction.iterations
            ),
        });
    }
    let Some(encoded) = report.encoded_iload else {
        return Err(CliError::Trajopt {
            summary: "two-body apogee correction converged without encoded I-load".to_owned(),
        });
    };
    std::fs::write(output_iload, &encoded).map_err(|source| CliError::Io {
        path: output_iload.to_path_buf(),
        source,
    })?;
    let Some(&corrected_speed_m_s) = report.correction.free_variables.first() else {
        return Err(CliError::Trajopt {
            summary: "two-body apogee correction returned no speed variable".to_owned(),
        });
    };
    Ok(CorrectApogeeCliReport {
        corrected_speed_m_s,
        residual_norm_m: report.correction.residual.norm,
        iterations: report.correction.iterations,
        reference_samples: report.reference_profile.len(),
        iload_bytes: encoded.len(),
        output_iload: output_iload.to_path_buf(),
    })
}
