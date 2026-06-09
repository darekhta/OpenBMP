//! Versioned I-load payload producer.

use alloc::string::String;
use alloc::vec::Vec;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use openbmp_physics::profile::TerminalCondition;

/// Current I-load payload schema version.
pub const CURRENT_SCHEMA_VERSION: ILoadSchemaVersion = ILoadSchemaVersion {
    major: 1,
    minor: 0,
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

/// I-load payload header.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ILoadHeader {
    /// Payload schema version.
    pub schema_version: ILoadSchemaVersion,
    /// Terminal-condition family used by the synthesis pass.
    pub terminal_condition_kind: TerminalConditionKind,
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
        Self {
            schema_version: CURRENT_SCHEMA_VERSION,
            terminal_condition_kind: terminal_condition.into(),
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
        }
    }

    #[test]
    fn iload_payload_encodes_as_postcard() {
        let payload = sample_payload();
        let encoded = encode_iload_payload(&payload).unwrap();
        let decoded: ILoadPayload = postcard::from_bytes(&encoded).unwrap();
        assert_eq!(decoded, payload);
    }

    #[test]
    fn gain_tables_must_have_matching_lengths() {
        let mut payload = sample_payload();
        payload.gain_tables[0].values.pop();
        let err = payload.validate().unwrap_err();
        assert!(matches!(err, TrajoptError::InvalidPayload { .. }));
    }
}
