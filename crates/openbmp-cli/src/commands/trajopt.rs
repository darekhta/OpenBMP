//! Offline trajectory-optimization CLI commands.

use std::path::{Path, PathBuf};

use openbmp_core::ChannelId;
use openbmp_physics::WGS84_MU_M3_S2;
use openbmp_physics::profile::{
    BallisticState, ForwardSimulationProvenance, ForwardSimulationSource, TerminalCondition,
};
use openbmp_runner::RunOutcome;
use openbmp_scenario::Scenario;
use openbmp_telemetry::{TelemetryRow, TelemetryTable, TelemetryValue};
use openbmp_trajopt::{
    DifferentialCorrector, ILoadHeader, ILoadPayload, ReferenceProfileSample, SynthesisMetadata,
    TrajoptError, TwoBodyApogeeTargeting, encode_iload_payload,
};
use sha2::{Digest, Sha256};

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

/// Arguments for `openbmp trajopt correct-apogee-scenario`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeScenarioArgs {
    /// Scenario TOML file used as the runner forward map.
    pub scenario: PathBuf,
    /// Target apogee radius from the central body, in m.
    pub target_apogee_radius_m: f64,
    /// Optional initial speed guess, in m/s.
    pub initial_speed_m_s: Option<f64>,
    /// Optional central-body gravitational parameter for terminal residuals.
    pub mu_m3_s2: Option<f64>,
    /// Residual convergence tolerance, in m.
    pub residual_tolerance_m: f64,
    /// Maximum Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Deterministic synthesis seed recorded in the I-load.
    pub synthesis_seed: u64,
    /// Optional scenario digest; omitted values are derived from the scenario file bytes.
    pub scenario_digest: Option<String>,
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

/// Run the scenario-backed T0 apogee corrector and write a postcard I-load.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario cannot run, the trajectory corrector
/// rejects inputs, reports non-convergence, or the I-load cannot be written.
pub fn run_correct_apogee_scenario(
    args: CorrectApogeeScenarioArgs,
    output_iload: &Path,
) -> Result<CorrectApogeeCliReport, CliError> {
    let base_scenario = Scenario::from_file(&args.scenario)?;
    let initial_velocity = base_scenario.document.vehicle.initial_velocity_eci_m_s;
    let initial_speed_from_scenario = norm3(initial_velocity);
    if initial_speed_from_scenario <= f64::EPSILON || !initial_speed_from_scenario.is_finite() {
        return Err(CliError::Trajopt {
            summary: "scenario initial velocity must be finite and non-zero".to_owned(),
        });
    }
    let velocity_direction = scale3(initial_velocity, 1.0 / initial_speed_from_scenario);
    let initial_speed_m_s = args
        .initial_speed_m_s
        .unwrap_or(initial_speed_from_scenario);
    let mu_m3_s2 = args
        .mu_m3_s2
        .or(base_scenario.document.environment.mu_m3_s2)
        .unwrap_or(WGS84_MU_M3_S2);
    if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
        return Err(CliError::Trajopt {
            summary: "terminal residual gravity parameter must be finite and positive".to_owned(),
        });
    }

    let condition = TerminalCondition::ApogeeRadius {
        radius_m: args.target_apogee_radius_m,
    };
    let corrector = DifferentialCorrector {
        residual_tolerance: args.residual_tolerance_m,
        max_iterations: args.max_iterations,
        ..DifferentialCorrector::default()
    };
    let mut forward_failure = None::<String>;
    let correction = corrector
        .solve(&condition, mu_m3_s2, &[initial_speed_m_s], |free| {
            let speed = first_speed_variable(free)?;
            let scenario = scenario_with_speed(&base_scenario, velocity_direction, speed);
            match runner_trajectory(&scenario) {
                Ok((terminal_state, _reference_profile)) => Ok(terminal_state),
                Err(summary) => {
                    forward_failure = Some(summary);
                    Err(TrajoptError::InvalidPayload {
                        reason: "scenario runner forward map failed",
                    })
                }
            }
        })
        .map_err(|source| {
            if let Some(summary) = forward_failure {
                CliError::Trajopt {
                    summary: format!("scenario runner forward map failed: {summary}"),
                }
            } else {
                CliError::TrajoptLib(source)
            }
        })?;

    if !correction.converged {
        return Err(CliError::Trajopt {
            summary: format!(
                "scenario apogee correction did not converge: residual_norm_m={:.12e}, iterations={}",
                correction.residual.norm, correction.iterations
            ),
        });
    }
    let Some(&corrected_speed_m_s) = correction.free_variables.first() else {
        return Err(CliError::Trajopt {
            summary: "scenario apogee correction returned no speed variable".to_owned(),
        });
    };
    let final_scenario =
        scenario_with_speed(&base_scenario, velocity_direction, corrected_speed_m_s);
    let (_terminal_state, reference_profile) =
        runner_trajectory(&final_scenario).map_err(|summary| CliError::Trajopt {
            summary: format!("scenario runner final propagation failed: {summary}"),
        })?;
    let payload = ILoadPayload {
        header: ILoadHeader::new(&condition, args.synthesis_seed, args.producer),
        metadata: SynthesisMetadata {
            scenario_digest: args
                .scenario_digest
                .map_or_else(|| digest_file(&args.scenario), Ok)?,
            source_revision: args.source_revision,
            method: "runner_scenario_apogee_single_shooting".to_owned(),
        },
        event_bindings_postcard: Vec::new(),
        gain_tables: Vec::new(),
        reference_profile,
    };
    let encoded = encode_iload_payload(&payload)?;
    std::fs::write(output_iload, &encoded).map_err(|source| CliError::Io {
        path: output_iload.to_path_buf(),
        source,
    })?;
    Ok(CorrectApogeeCliReport {
        corrected_speed_m_s,
        residual_norm_m: correction.residual.norm,
        iterations: correction.iterations,
        reference_samples: payload.reference_profile.len(),
        iload_bytes: encoded.len(),
        output_iload: output_iload.to_path_buf(),
    })
}

fn digest_file(path: &Path) -> Result<String, CliError> {
    let bytes = std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
}

fn scenario_with_speed(
    base_scenario: &Scenario,
    velocity_direction: [f64; 3],
    speed_m_s: f64,
) -> Scenario {
    let mut scenario = base_scenario.clone();
    scenario.document.vehicle.initial_velocity_eci_m_s = scale3(velocity_direction, speed_m_s);
    scenario
}

fn runner_trajectory(
    scenario: &Scenario,
) -> Result<(BallisticState, Vec<ReferenceProfileSample>), String> {
    let outcome = openbmp_runner::run(scenario).map_err(|source| source.to_string())?;
    trajectory_from_outcome(&outcome)
}

fn trajectory_from_outcome(
    outcome: &RunOutcome,
) -> Result<(BallisticState, Vec<ReferenceProfileSample>), String> {
    let channels = StateChannelIds::from_table(&outcome.table)?;
    let rows = outcome.table.rows();
    if rows.is_empty() {
        return Err("runner produced no telemetry rows".to_owned());
    }
    let mut reference_profile = Vec::with_capacity(rows.len());
    for row in rows {
        let sample = channels.state_sample(row)?;
        reference_profile.push(reference_sample(
            row.time.as_seconds(),
            sample.position,
            sample.velocity,
        )?);
    }
    let Some(last_row) = rows.last() else {
        return Err("runner produced no telemetry rows".to_owned());
    };
    let terminal_sample = channels.state_sample(last_row)?;
    let provenance = ForwardSimulationProvenance::for_state(
        terminal_sample.position,
        terminal_sample.velocity,
        0.0,
        last_row.time,
        ForwardSimulationSource::RunnerTelemetryState,
    );
    let terminal_state = BallisticState::from_forward_simulation(
        terminal_sample.position,
        terminal_sample.velocity,
        0.0,
        last_row.time,
        provenance,
    )
    .map_err(|source| source.to_string())?;
    Ok((terminal_state, reference_profile))
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StateChannelIds {
    position_x: ChannelId,
    position_y: ChannelId,
    position_z: ChannelId,
    velocity_x: ChannelId,
    velocity_y: ChannelId,
    velocity_z: ChannelId,
}

impl StateChannelIds {
    fn from_table(table: &TelemetryTable) -> Result<Self, String> {
        Ok(Self {
            position_x: channel_id(table, "position_x_m")?,
            position_y: channel_id(table, "position_y_m")?,
            position_z: channel_id(table, "position_z_m")?,
            velocity_x: channel_id(table, "velocity_x_m_s")?,
            velocity_y: channel_id(table, "velocity_y_m_s")?,
            velocity_z: channel_id(table, "velocity_z_m_s")?,
        })
    }

    fn state_sample(self, row: &TelemetryRow) -> Result<StateSample, String> {
        Ok(StateSample {
            position: [
                row_f64(row, self.position_x, "position_x_m")?,
                row_f64(row, self.position_y, "position_y_m")?,
                row_f64(row, self.position_z, "position_z_m")?,
            ],
            velocity: [
                row_f64(row, self.velocity_x, "velocity_x_m_s")?,
                row_f64(row, self.velocity_y, "velocity_y_m_s")?,
                row_f64(row, self.velocity_z, "velocity_z_m_s")?,
            ],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StateSample {
    position: [f64; 3],
    velocity: [f64; 3],
}

fn channel_id(table: &TelemetryTable, name: &str) -> Result<ChannelId, String> {
    table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .map(|channel| channel.id)
        .ok_or_else(|| format!("runner telemetry is missing channel `{name}`"))
}

fn row_f64(row: &TelemetryRow, channel: ChannelId, name: &str) -> Result<f64, String> {
    match row.get(channel) {
        Some(TelemetryValue::Float64(value)) => Ok(*value),
        other => Err(format!(
            "runner telemetry channel `{name}` has unexpected value {other:?}"
        )),
    }
}

fn reference_sample(
    time_s: f64,
    position: [f64; 3],
    velocity: [f64; 3],
) -> Result<ReferenceProfileSample, String> {
    let radius_m = norm3(position);
    let speed_m_s = norm3(velocity);
    if radius_m <= f64::EPSILON || speed_m_s <= f64::EPSILON {
        return Err("runner reference sample is degenerate".to_owned());
    }
    let radial_speed_m_s = dot3(position, velocity) / radius_m;
    Ok(ReferenceProfileSample {
        time_s,
        radius_m,
        speed_m_s,
        flight_path_angle_rad: (radial_speed_m_s / speed_m_s).clamp(-1.0, 1.0).asin(),
    })
}

fn first_speed_variable(free: &[f64]) -> Result<f64, TrajoptError> {
    let Some(&speed) = free.first() else {
        return Err(TrajoptError::InvalidPayload {
            reason: "scenario apogee correction requires one speed variable",
        });
    };
    if !speed.is_finite() || speed <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "scenario apogee speed variable must be finite and positive",
        });
    }
    Ok(speed)
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm3(value: [f64; 3]) -> f64 {
    dot3(value, value).sqrt()
}

fn scale3(value: [f64; 3], scale: f64) -> [f64; 3] {
    [value[0] * scale, value[1] * scale, value[2] * scale]
}
