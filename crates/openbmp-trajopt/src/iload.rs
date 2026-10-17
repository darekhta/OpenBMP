//! Versioned I-load payload producer.

use alloc::string::String;
use alloc::vec::Vec;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use openbmp_physics::profile::TerminalCondition;

/// Current I-load payload schema version.
pub const CURRENT_SCHEMA_VERSION: ILoadSchemaVersion = ILoadSchemaVersion {
    major: 1,
    minor: 3,
    patch: 0,
};

/// Semantic version of the postcard-serialized I-load payload schema.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ILoadSchemaVersion {
    /// Major schema version. Incompatible changes increment this.
    pub major: u16,
    /// Minor schema version. Backward-compatible fields increment this.
    pub minor: u16,
    /// Patch schema version. Documentation / producer-only fixes increment this.
    pub patch: u16,
}

/// Forward-only terminal-condition kind recorded in an I-load header.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum TerminalConditionKind {
    /// Orbital elements at cutoff.
    OrbitalElements,
    /// Apogee radius at cutoff.
    ApogeeRadius,
    /// Flight-path angle at burnout.
    FlightPathAngleAtBurnout,
    /// Inertial rendezvous state.
    RendezvousState,
    /// Vehicle-intrinsic payload objective.
    MaximizePayloadMass,
}

impl From<&TerminalCondition> for TerminalConditionKind {
    fn from(value: &TerminalCondition) -> Self {
        match value {
            TerminalCondition::OrbitalElements { .. } => Self::OrbitalElements,
            TerminalCondition::ApogeeRadius { .. } => Self::ApogeeRadius,
            TerminalCondition::FlightPathAngleAtBurnout { .. } => Self::FlightPathAngleAtBurnout,
            TerminalCondition::RendezvousState { .. } => Self::RendezvousState,
            TerminalCondition::MaximizePayloadMass => Self::MaximizePayloadMass,
        }
    }
}

/// High-level objective family recorded in an I-load header.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum ILoadObjectiveClass {
    /// Correct a terminal residual, such as a two-body apogee insertion error.
    TerminalResidualCorrection,
    /// Emit an optimized reference profile produced by an offline OCP.
    OptimizedReference,
    /// Minimize integrated control effort or another energy-like quadratic.
    MinimumEnergy,
    /// Minimize propellant use or maximize final mass.
    MinimumFuel,
    /// Minimize time of flight.
    MinimumTime,
    /// Recenter day-of-launch I-load gains against updated conditions.
    DayOfLaunchRecenter,
    /// Vehicle-intrinsic payload objective.
    MaximizePayloadMass,
}

impl ILoadObjectiveClass {
    fn default_for_terminal_condition(terminal_condition: &TerminalCondition) -> Self {
        match terminal_condition {
            TerminalCondition::MaximizePayloadMass => Self::MaximizePayloadMass,
            TerminalCondition::OrbitalElements { .. }
            | TerminalCondition::ApogeeRadius { .. }
            | TerminalCondition::FlightPathAngleAtBurnout { .. }
            | TerminalCondition::RendezvousState { .. } => Self::TerminalResidualCorrection,
        }
    }
}

/// I-load payload header.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ILoadHeader {
    /// Payload schema version.
    pub schema_version: ILoadSchemaVersion,
    /// Terminal-condition family used by the synthesis pass.
    pub terminal_condition_kind: TerminalConditionKind,
    /// High-level objective family used by the synthesis pass.
    pub objective_class: ILoadObjectiveClass,
    /// Deterministic synthesis seed used for any stochastic design
    /// substeps.
    pub synthesis_seed: u64,
    /// Producer name and version.
    pub producer: String,
}

impl ILoadHeader {
    /// Construct a header for `terminal_condition`.
    #[must_use]
    pub fn new(
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
    ) -> Self {
        Self::new_with_objective(
            terminal_condition,
            ILoadObjectiveClass::default_for_terminal_condition(terminal_condition),
            synthesis_seed,
            producer,
        )
    }

    /// Construct a header for `terminal_condition` and `objective_class`.
    #[must_use]
    pub fn new_with_objective(
        terminal_condition: &TerminalCondition,
        objective_class: ILoadObjectiveClass,
        synthesis_seed: u64,
        producer: String,
    ) -> Self {
        Self {
            schema_version: CURRENT_SCHEMA_VERSION,
            terminal_condition_kind: terminal_condition.into(),
            objective_class,
            synthesis_seed,
            producer,
        }
    }
}

/// Additional metadata attached by the offline synthesis pass.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct SynthesisMetadata {
    /// Opaque scenario or mission graph digest, normally SHA-256 hex.
    pub scenario_digest: String,
    /// Opaque source-revision identifier for traceability.
    pub source_revision: String,
    /// Human-readable synthesis method, such as `multiple_shooting`.
    pub method: String,
}

/// Source family for an optional I-load optimality certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum ILoadOptimalityEvidenceKind {
    /// Deterministic scalar PMP minimum-energy shooting witness.
    PmpScalarMinEnergyShooting,
    /// Hermite-Simpson direct-method/KKT PMP witness.
    HermiteSimpsonPmpKkt,
    /// LGR pseudospectral direct-method/KKT PMP witness.
    LgrPseudospectralPmpKkt,
    /// External offline oracle evidence recorded by the synthesis pass.
    ExternalOracle,
}

/// Optional optimality summary attached to a synthesized I-load.
///
/// This is an audit certificate, not a flight-control input. Validation is
/// fail-closed: any attached certificate must have finite tolerances and must
/// satisfy its declared Hamiltonian, optional control-stationarity, and optional
/// costate bounds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ILoadOptimalityCertificate {
    /// Evidence source family.
    pub evidence_kind: ILoadOptimalityEvidenceKind,
    /// State/costate dimension in the witness.
    pub state_dimension: usize,
    /// Number of witness samples.
    pub sample_count: usize,
    /// Maximum absolute Hamiltonian drift over the witness profile.
    pub max_abs_hamiltonian_drift: f64,
    /// Required upper bound for `max_abs_hamiltonian_drift`.
    pub hamiltonian_drift_tolerance: f64,
    /// Optional maximum Hamiltonian control-stationarity gradient norm.
    pub max_control_stationarity_gradient_norm: Option<f64>,
    /// Required upper bound for `max_control_stationarity_gradient_norm`, when present.
    pub control_stationarity_tolerance: Option<f64>,
    /// Optional maximum PMP-vs-KKT costate-vector norm difference.
    pub max_costate_norm_difference: Option<f64>,
    /// Required upper bound for `max_costate_norm_difference`, when present.
    pub costate_tolerance: Option<f64>,
}

impl ILoadOptimalityCertificate {
    /// Build a Hamiltonian-constancy certificate from a PMP witness report.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or tolerance is malformed, or
    /// the report does not satisfy the declared tolerance.
    pub fn pmp_hamiltonian_constancy(
        evidence_kind: ILoadOptimalityEvidenceKind,
        report: &crate::pmp::PmpHamiltonianReport,
        hamiltonian_drift_tolerance: f64,
    ) -> Result<Self, TrajoptError> {
        let certificate = Self {
            evidence_kind,
            state_dimension: report.state_dimension,
            sample_count: report.sample_count,
            max_abs_hamiltonian_drift: report.max_abs_hamiltonian_drift,
            hamiltonian_drift_tolerance,
            max_control_stationarity_gradient_norm: None,
            control_stationarity_tolerance: None,
            max_costate_norm_difference: None,
            costate_tolerance: None,
        };
        certificate.validate()?;
        Ok(certificate)
    }

    /// Add a PMP Hamiltonian control-stationarity summary.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity dimensions do not match
    /// the certificate or the gradient norm exceeds its declared tolerance.
    pub fn with_control_stationarity(
        mut self,
        report: &crate::pmp::PmpHamiltonianControlStationarityReport,
    ) -> Result<Self, TrajoptError> {
        if report.state_dimension != self.state_dimension
            || report.sample_count != self.sample_count
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load optimality certificate stationarity dimensions are inconsistent",
            });
        }
        self.max_control_stationarity_gradient_norm = Some(report.max_gradient_norm);
        self.control_stationarity_tolerance = Some(report.gradient_tolerance);
        self.validate()?;
        Ok(self)
    }

    /// Add a same-mesh PMP-vs-KKT costate comparison summary.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the comparison dimensions do not match the
    /// certificate or the comparison does not satisfy its declared tolerance.
    pub fn with_costate_comparison(
        mut self,
        comparison: &crate::pmp::PmpCostateComparisonReport,
    ) -> Result<Self, TrajoptError> {
        if comparison.state_dimension != self.state_dimension
            || comparison.sample_count != self.sample_count
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load optimality certificate costate dimensions are inconsistent",
            });
        }
        self.max_costate_norm_difference = Some(comparison.max_costate_norm_difference);
        self.costate_tolerance = Some(comparison.costate_tolerance);
        self.validate()?;
        Ok(self)
    }

    fn validate(&self) -> Result<(), TrajoptError> {
        if self.state_dimension == 0 || self.sample_count == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load optimality certificate dimensions must be nonzero",
            });
        }
        if !self.max_abs_hamiltonian_drift.is_finite()
            || self.max_abs_hamiltonian_drift < 0.0
            || !self.hamiltonian_drift_tolerance.is_finite()
            || self.hamiltonian_drift_tolerance < 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load optimality certificate Hamiltonian values must be finite",
            });
        }
        if self.max_abs_hamiltonian_drift > self.hamiltonian_drift_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load optimality certificate Hamiltonian drift exceeds tolerance",
            });
        }
        match (
            self.max_control_stationarity_gradient_norm,
            self.control_stationarity_tolerance,
        ) {
            (Some(gradient_norm), Some(tolerance)) => {
                if !gradient_norm.is_finite()
                    || gradient_norm < 0.0
                    || !tolerance.is_finite()
                    || tolerance < 0.0
                {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "I-load optimality certificate stationarity values must be finite",
                    });
                }
                if gradient_norm > tolerance {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "I-load optimality certificate stationarity gradient exceeds tolerance",
                    });
                }
            }
            (None, None) => {}
            _ => {
                return Err(TrajoptError::InvalidPayload {
                    reason: "I-load optimality certificate stationarity fields must be paired",
                });
            }
        }
        match (self.max_costate_norm_difference, self.costate_tolerance) {
            (Some(difference), Some(tolerance)) => {
                if !difference.is_finite()
                    || difference < 0.0
                    || !tolerance.is_finite()
                    || tolerance < 0.0
                {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "I-load optimality certificate costate values must be finite",
                    });
                }
                if difference > tolerance {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "I-load optimality certificate costate difference exceeds tolerance",
                    });
                }
            }
            (None, None) => {}
            _ => {
                return Err(TrajoptError::InvalidPayload {
                    reason: "I-load optimality certificate costate fields must be paired",
                });
            }
        }
        Ok(())
    }
}

/// One reference-profile sample emitted into the I-load.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReferenceProfileSample {
    /// Mission elapsed time (s).
    pub time_s: f64,
    /// Inertial reference radius (m).
    pub radius_m: f64,
    /// Inertial reference speed (m/s).
    pub speed_m_s: f64,
    /// Flight-path angle relative to the local horizon (rad).
    pub flight_path_angle_rad: f64,
}

/// Scheduling axis for a gain table.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum GainAxis {
    /// Time since scenario start.
    Time,
    /// Mach-number schedule.
    Mach,
    /// Dynamic-pressure schedule.
    DynamicPressure,
}

/// Scheduled gain table emitted as part of an I-load.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GainTable {
    /// Table name.
    pub name: String,
    /// Scheduling axis.
    pub axis: GainAxis,
    /// Monotonic schedule breakpoints in the axis units.
    pub breakpoints: Vec<f64>,
    /// Row-major gain values associated with `breakpoints`.
    pub values: Vec<f64>,
}

/// Complete postcard-serialized offline I-load payload.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ILoadPayload {
    /// Version and synthesis family.
    pub header: ILoadHeader,
    /// Traceability metadata for the synthesis run.
    pub metadata: SynthesisMetadata,
    /// Compact event-binding representation owned by the producer.
    ///
    /// The flight image validates the envelope version/CRC before it
    /// deserializes this opaque block into its own mission table
    /// schema.
    pub event_bindings_postcard: Vec<u8>,
    /// Gain schedules produced by the synthesis pass.
    pub gain_tables: Vec<GainTable>,
    /// Inertial reference profile.
    pub reference_profile: Vec<ReferenceProfileSample>,
    /// Optional optimality evidence attached by the synthesis pass.
    pub optimality_certificate: Option<ILoadOptimalityCertificate>,
}

/// Errors raised while validating or serializing an I-load payload.
#[derive(Debug, Error)]
pub enum TrajoptError {
    /// Payload validation failed.
    #[error("trajopt I-load validation failed: {reason}")]
    InvalidPayload {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Postcard serialization failed.
    #[error("trajopt I-load postcard serialization failed: {0}")]
    Serialize(#[from] postcard::Error),
    /// A differential-correction normal-equation solve was singular.
    #[error("trajopt differential correction linear system was singular: {reason}")]
    SingularSystem {
        /// Human-readable reason.
        reason: &'static str,
    },
}

impl ILoadPayload {
    /// Validate finite numeric fields and internal table dimensions.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when any numeric field is non-finite
    /// or a gain table has inconsistent lengths.
    pub fn validate(&self) -> Result<(), TrajoptError> {
        if self.header.producer.trim().is_empty()
            || self.metadata.scenario_digest.trim().is_empty()
            || self.metadata.source_revision.trim().is_empty()
            || self.metadata.method.trim().is_empty()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "I-load provenance strings must be non-empty",
            });
        }
        for table in &self.gain_tables {
            if table.name.trim().is_empty() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "gain table name must be non-empty",
                });
            }
            if table.breakpoints.len() != table.values.len() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "gain table breakpoints and values must have equal length",
                });
            }
            if table.breakpoints.is_empty() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "gain table must contain at least one breakpoint",
                });
            }
            for value in table.breakpoints.iter().chain(table.values.iter()) {
                if !value.is_finite() {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "gain table values must be finite",
                    });
                }
            }
            if !table
                .breakpoints
                .windows(2)
                .all(|window| window[0] < window[1])
            {
                return Err(TrajoptError::InvalidPayload {
                    reason: "gain table breakpoints must be strictly increasing",
                });
            }
        }
        for sample in &self.reference_profile {
            if !sample.time_s.is_finite()
                || !sample.radius_m.is_finite()
                || !sample.speed_m_s.is_finite()
                || !sample.flight_path_angle_rad.is_finite()
            {
                return Err(TrajoptError::InvalidPayload {
                    reason: "reference profile samples must be finite",
                });
            }
        }
        if let Some(certificate) = &self.optimality_certificate {
            certificate.validate()?;
        }
        Ok(())
    }
}

/// Serialize an I-load payload with the stable postcard wire format.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation or serialization fails.
pub fn encode_iload_payload(payload: &ILoadPayload) -> Result<Vec<u8>, TrajoptError> {
    payload.validate()?;
    postcard::to_allocvec(payload).map_err(TrajoptError::Serialize)
}

/// Deserialize and validate an I-load payload from the stable postcard wire
/// format.
///
/// # Errors
///
/// Returns [`TrajoptError`] when deserialization fails or the decoded payload is
/// malformed.
pub fn decode_iload_payload(bytes: &[u8]) -> Result<ILoadPayload, TrajoptError> {
    let payload: ILoadPayload = postcard::from_bytes(bytes).map_err(TrajoptError::Serialize)?;
    payload.validate()?;
    Ok(payload)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_physics::WGS84_A_M;

    fn sample_payload() -> ILoadPayload {
        let terminal = TerminalCondition::ApogeeRadius {
            radius_m: 7_000_000.0,
        };
        ILoadPayload {
            header: ILoadHeader::new(&terminal, 42, "openbmp-trajopt-test".to_owned()),
            metadata: SynthesisMetadata {
                scenario_digest: "abc123".to_owned(),
                source_revision: "test".to_owned(),
                method: "unit".to_owned(),
            },
            event_bindings_postcard: vec![1, 2, 3],
            gain_tables: vec![GainTable {
                name: "pitch".to_owned(),
                axis: GainAxis::Time,
                breakpoints: vec![0.0, 1.0],
                values: vec![0.1, 0.2],
            }],
            reference_profile: vec![ReferenceProfileSample {
                time_s: 0.0,
                radius_m: WGS84_A_M,
                speed_m_s: 7_800.0,
                flight_path_angle_rad: 0.0,
            }],
            optimality_certificate: None,
        }
    }

    #[test]
    fn iload_payload_encodes_as_postcard() {
        let payload = sample_payload();
        let encoded = encode_iload_payload(&payload).unwrap();
        let decoded = decode_iload_payload(&encoded).unwrap();
        assert_eq!(decoded, payload);
        assert_eq!(decoded.header.schema_version, CURRENT_SCHEMA_VERSION);
        assert_eq!(
            decoded.header.objective_class,
            ILoadObjectiveClass::TerminalResidualCorrection
        );
    }

    #[test]
    fn iload_optimality_certificate_validates_declared_tolerances() {
        let terminal = TerminalCondition::ApogeeRadius {
            radius_m: 7_000_000.0,
        };
        let hamiltonian = crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &[0.0, 1.0],
                states: &[0.0, 1.0],
                controls: &[1.0, 1.0],
                costates: &[-1.0, -1.0],
            },
            |_time_s, _state, control| Ok(vec![control[0]]),
            |_time_s, _state, control| Ok(0.5 * control[0] * control[0]),
        )
        .expect("PMP Hamiltonian report evaluates");
        let certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::PmpScalarMinEnergyShooting,
            &hamiltonian,
            1.0e-12,
        )
        .expect("certificate validates");
        let stationarity = crate::pmp::evaluate_pmp_hamiltonian_control_stationarity(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &[0.0, 1.0],
                states: &[0.0, 1.0],
                controls: &[1.0, 1.0],
                costates: &[-1.0, -1.0],
            },
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            |_time_s, _state, control| Ok(vec![control[0]]),
            |_time_s, _state, control| Ok(0.5 * control[0] * control[0]),
        )
        .expect("PMP stationarity report evaluates");
        let certificate = certificate
            .with_control_stationarity(&stationarity)
            .expect("stationarity certificate validates");
        assert_eq!(certificate.sample_count, 2);
        assert_eq!(certificate.max_abs_hamiltonian_drift, 0.0);
        assert!(
            certificate
                .max_control_stationarity_gradient_norm
                .expect("stationarity summary")
                <= 1.0e-9
        );

        let off_optimum_stationarity = crate::pmp::evaluate_pmp_hamiltonian_control_stationarity(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: 1,
                control_dimension: 1,
                times_s: &[0.0, 1.0],
                states: &[0.0, 1.0],
                controls: &[0.5, 0.5],
                costates: &[-1.0, -1.0],
            },
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-12,
                ..Default::default()
            },
            |_time_s, _state, control| Ok(vec![control[0]]),
            |_time_s, _state, control| Ok(0.5 * control[0] * control[0]),
        )
        .expect("off-optimum stationarity report evaluates");
        assert!(
            certificate
                .clone()
                .with_control_stationarity(&off_optimum_stationarity)
                .is_err()
        );

        let mut payload = ILoadPayload {
            header: ILoadHeader::new_with_objective(
                &terminal,
                ILoadObjectiveClass::OptimizedReference,
                7,
                "openbmp-trajopt-test".to_owned(),
            ),
            metadata: SynthesisMetadata {
                scenario_digest: "abc123".to_owned(),
                source_revision: "test".to_owned(),
                method: "pmp-scalar-min-energy-shooting".to_owned(),
            },
            event_bindings_postcard: Vec::new(),
            gain_tables: Vec::new(),
            reference_profile: vec![ReferenceProfileSample {
                time_s: 0.0,
                radius_m: WGS84_A_M,
                speed_m_s: 7_800.0,
                flight_path_angle_rad: 0.0,
            }],
            optimality_certificate: Some(certificate),
        };
        let decoded = decode_iload_payload(&encode_iload_payload(&payload).unwrap()).unwrap();
        assert_eq!(decoded, payload);

        payload
            .optimality_certificate
            .as_mut()
            .expect("certificate present")
            .hamiltonian_drift_tolerance = -1.0;
        assert!(payload.validate().is_err());
    }

    #[test]
    fn iload_header_records_explicit_objective_class() {
        let terminal = TerminalCondition::ApogeeRadius {
            radius_m: 7_000_000.0,
        };
        let header = ILoadHeader::new_with_objective(
            &terminal,
            ILoadObjectiveClass::OptimizedReference,
            7,
            "openbmp-trajopt-test".to_owned(),
        );
        assert_eq!(header.schema_version, CURRENT_SCHEMA_VERSION);
        assert_eq!(
            header.terminal_condition_kind,
            TerminalConditionKind::ApogeeRadius
        );
        assert_eq!(
            header.objective_class,
            ILoadObjectiveClass::OptimizedReference
        );
    }

    #[test]
    fn gain_tables_must_have_matching_lengths() {
        let mut payload = sample_payload();
        payload.gain_tables[0].values.pop();
        let err = payload.validate().unwrap_err();
        assert!(matches!(err, TrajoptError::InvalidPayload { .. }));
    }
}
