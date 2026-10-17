//! Declared CLA event-force helpers.
//!
//! This module contains pure force-time profiles, a closed CLA flight-event
//! vocabulary, and modal projection helpers. Scenario parsing, event
//! scheduling, and runner wiring live above this crate.

use crate::{DenseMatrix, StructDynError};
use serde::Deserialize;
use std::collections::BTreeSet;

const CLA_EVENT_LIBRARY_DATASET_ID: &str = "openbmp.structdyn.cla_event_library.v1";

/// Closed flight-event classes used by the CLA forcing library.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ClaFlightEventKind {
    /// Liftoff hold-down/twang preload release.
    LiftoffRelease,
    /// Maximum-dynamic-pressure gust loading.
    MaxQGust,
    /// Engine ignition transient.
    EngineIgnition,
    /// Stage or component separation transient.
    StageSeparation,
    /// Main-engine cutoff transient.
    MainEngineCutoff,
}

impl ClaFlightEventKind {
    /// Parse a stable event-kind string.
    ///
    /// Accepted values are `liftoff_release`, `max_q_gust`,
    /// `engine_ignition`, `stage_separation`, and `meco`.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the kind is unknown.
    pub fn from_kind(kind: &str) -> Result<Self, StructDynError> {
        match kind {
            "liftoff_release" => Ok(Self::LiftoffRelease),
            "max_q_gust" => Ok(Self::MaxQGust),
            "engine_ignition" => Ok(Self::EngineIgnition),
            "stage_separation" => Ok(Self::StageSeparation),
            "meco" | "main_engine_cutoff" => Ok(Self::MainEngineCutoff),
            _ => Err(StructDynError::InvalidString {
                field: "event.flight_event_kind",
            }),
        }
    }

    /// Stable snake-case event-kind string.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::LiftoffRelease => "liftoff_release",
            Self::MaxQGust => "max_q_gust",
            Self::EngineIgnition => "engine_ignition",
            Self::StageSeparation => "stage_separation",
            Self::MainEngineCutoff => "meco",
        }
    }

    fn accepts_profile(self, profile: EventForceProfile) -> bool {
        match self {
            Self::LiftoffRelease => matches!(profile, EventForceProfile::RampRelease { .. }),
            Self::MaxQGust => matches!(
                profile,
                EventForceProfile::DrydenGust { .. } | EventForceProfile::VonKarmanGust { .. }
            ),
            Self::EngineIgnition | Self::StageSeparation => matches!(
                profile,
                EventForceProfile::RectangularImpulse { .. }
                    | EventForceProfile::HalfSineImpulse { .. }
            ),
            Self::MainEngineCutoff => matches!(
                profile,
                EventForceProfile::Step { .. } | EventForceProfile::RampRelease { .. }
            ),
        }
    }

    fn accepts_stiffness_change(self) -> bool {
        matches!(
            self,
            Self::EngineIgnition | Self::StageSeparation | Self::MainEngineCutoff
        )
    }
}

/// Fixture-backed CLA event-library deck.
#[derive(Clone, Debug, PartialEq)]
pub struct ClaEventLibraryDeck {
    /// Dataset identifier from the deck metadata.
    pub dataset_id: String,
    /// Source class from the deck metadata.
    pub source_class: String,
    /// Validation label from the deck metadata.
    pub validation: String,
    /// Provenance pointer from the deck metadata.
    pub provenance: String,
    /// Absolute tolerance used for fixture multiplier checks.
    pub multiplier_abs_tolerance_1: f64,
    /// Absolute tolerance used for fixture stiffness-scale checks.
    pub stiffness_abs_tolerance_1: f64,
    /// Declared event templates.
    pub events: Vec<ClaEventLibraryEvent>,
}

/// One event template in a CLA event-library deck.
#[derive(Clone, Debug, PartialEq)]
pub struct ClaEventLibraryEvent {
    /// Stable event-template id.
    pub id: String,
    /// Closed flight-event class.
    pub flight_event_kind: ClaFlightEventKind,
    /// Time profile for the physical force vector.
    pub profile: EventForceProfile,
    /// Physical force or impulse vector in structural DOF order.
    pub physical_force_n: Vec<f64>,
    /// Optional modal stiffness scale applied with the event profile.
    pub modal_stiffness_scale_1: Vec<f64>,
    /// Fixture multiplier samples.
    pub multiplier_samples: Vec<ClaEventMultiplierSample>,
    /// Fixture modal-stiffness scale samples.
    pub stiffness_samples: Vec<ClaEventStiffnessSample>,
}

impl ClaEventLibraryEvent {
    /// Build an [`EventForcing`] from this template's profile and force vector.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] if the template contents are invalid.
    pub fn event_forcing(&self) -> Result<EventForcing, StructDynError> {
        EventForcing::new(self.id.clone(), self.profile, self.physical_force_n.clone())
    }

    /// Evaluate this template's modal stiffness scale at one time.
    ///
    /// The returned vector is empty when the template does not declare a
    /// stiffness change. Otherwise each component is blended as
    /// `1 + multiplier * (declared_scale - 1)`, matching the runner's live
    /// event-stiffness semantics.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the profile cannot be evaluated or a
    /// blended scale is non-finite or non-positive.
    pub fn modal_stiffness_scale_at(&self, time_s: f64) -> Result<Vec<f64>, StructDynError> {
        modal_stiffness_scale_at(
            self.profile,
            &self.modal_stiffness_scale_1,
            time_s,
            "event.library.event.modal_stiffness_scale_1",
        )
    }
}

/// One checked profile-multiplier sample.
#[derive(Clone, Debug, PartialEq)]
pub struct ClaEventMultiplierSample {
    /// Sample time, s.
    pub time_s: f64,
    /// Expected dimensionless multiplier.
    pub expected_multiplier_1: f64,
}

/// One checked event modal-stiffness scale sample.
#[derive(Clone, Debug, PartialEq)]
pub struct ClaEventStiffnessSample {
    /// Sample time, s.
    pub time_s: f64,
    /// Expected modal stiffness scale vector.
    pub expected_modal_stiffness_scale_1: Vec<f64>,
}

/// Parse and validate a CLA event-library TOML deck.
///
/// # Errors
///
/// Returns [`StructDynError`] when the deck cannot be parsed, has wrong
/// metadata, declares duplicate/unknown event classes, pairs an event class
/// with an incompatible profile, or has multiplier samples that drift from the
/// runtime profile evaluator.
pub fn parse_cla_event_library_toml(input: &str) -> Result<ClaEventLibraryDeck, StructDynError> {
    let raw: RawClaEventLibraryDeck =
        toml::from_str(input).map_err(|_| StructDynError::InvalidMatrix {
            field: "event.library",
            reason: "invalid TOML",
        })?;
    if raw.openbmp_structdyn_cla_event_library != 1 {
        return Err(StructDynError::InvalidNumber {
            field: "event.library.magic",
        });
    }
    require_nonempty_str("event.library.meta.dataset_id", &raw.meta.dataset_id)?;
    require_nonempty_str("event.library.meta.source_class", &raw.meta.source_class)?;
    require_nonempty_str("event.library.meta.validation", &raw.meta.validation)?;
    require_nonempty_str("event.library.meta.provenance", &raw.meta.provenance)?;
    if raw.meta.dataset_id != CLA_EVENT_LIBRARY_DATASET_ID {
        return Err(StructDynError::InvalidString {
            field: "event.library.meta.dataset_id",
        });
    }
    if raw.meta.source_class != "openbmp-local-event-library" {
        return Err(StructDynError::InvalidString {
            field: "event.library.meta.source_class",
        });
    }
    if !raw.meta.multiplier_abs_tolerance_1.is_finite() || raw.meta.multiplier_abs_tolerance_1 < 0.0
    {
        return Err(StructDynError::InvalidNumber {
            field: "event.library.meta.multiplier_abs_tolerance_1",
        });
    }
    if !raw.meta.stiffness_abs_tolerance_1.is_finite() || raw.meta.stiffness_abs_tolerance_1 < 0.0 {
        return Err(StructDynError::InvalidNumber {
            field: "event.library.meta.stiffness_abs_tolerance_1",
        });
    }
    if raw.event.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "event.library.event",
            expected: 1,
            actual: 0,
        });
    }

    let mut ids = BTreeSet::new();
    let mut events = Vec::with_capacity(raw.event.len());
    for event in raw.event {
        require_nonempty_str("event.library.event.id", &event.id)?;
        if !ids.insert(event.id.clone()) {
            return Err(StructDynError::InvalidString {
                field: "event.library.event.id",
            });
        }
        let flight_event_kind = ClaFlightEventKind::from_kind(&event.flight_event_kind)?;
        let profile = EventForceProfile::from_kind_with_gust(
            &event.profile_kind,
            event.start_s,
            event.duration_s,
            event.gust_length_scale_s,
        )?;
        if !flight_event_kind.accepts_profile(profile) {
            return Err(StructDynError::InvalidString {
                field: "event.library.event.profile_kind",
            });
        }
        if event.physical_force_n.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "event.library.event.physical_force_n",
                expected: 1,
                actual: 0,
            });
        }
        if !event.physical_force_n.iter().all(|value| value.is_finite()) {
            return Err(StructDynError::InvalidNumber {
                field: "event.library.event.physical_force_n",
            });
        }
        if !event.modal_stiffness_scale_1.is_empty() {
            if !flight_event_kind.accepts_stiffness_change() {
                return Err(StructDynError::InvalidString {
                    field: "event.library.event.modal_stiffness_scale_1",
                });
            }
            validate_modal_stiffness_scale(
                &event.modal_stiffness_scale_1,
                "event.library.event.modal_stiffness_scale_1",
            )?;
        }
        if event.sample_times_s.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "event.library.event.sample_times_s",
                expected: 1,
                actual: 0,
            });
        }
        if event.sample_times_s.len() != event.expected_multiplier_1.len() {
            return Err(StructDynError::DimensionMismatch {
                field: "event.library.event.expected_multiplier_1",
                expected: event.sample_times_s.len(),
                actual: event.expected_multiplier_1.len(),
            });
        }
        if !event.sample_times_s.iter().all(|value| value.is_finite()) {
            return Err(StructDynError::InvalidNumber {
                field: "event.library.event.sample_times_s",
            });
        }
        if !event
            .expected_multiplier_1
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(StructDynError::InvalidNumber {
                field: "event.library.event.expected_multiplier_1",
            });
        }

        let mut multiplier_samples = Vec::with_capacity(event.sample_times_s.len());
        for (time_s, expected_multiplier_1) in event
            .sample_times_s
            .iter()
            .copied()
            .zip(event.expected_multiplier_1.iter().copied())
        {
            let actual = profile.multiplier_at(time_s)?;
            if (actual - expected_multiplier_1).abs() > raw.meta.multiplier_abs_tolerance_1 {
                return Err(StructDynError::InvalidMatrix {
                    field: "event.library.event.expected_multiplier_1",
                    reason: "expected multiplier mismatch",
                });
            }
            multiplier_samples.push(ClaEventMultiplierSample {
                time_s,
                expected_multiplier_1,
            });
        }

        let mut stiffness_samples = Vec::new();
        if !event.modal_stiffness_scale_1.is_empty()
            || !event.stiffness_sample_times_s.is_empty()
            || !event.expected_modal_stiffness_scale_1.is_empty()
        {
            if event.modal_stiffness_scale_1.is_empty() {
                return Err(StructDynError::DimensionMismatch {
                    field: "event.library.event.modal_stiffness_scale_1",
                    expected: 1,
                    actual: 0,
                });
            }
            if event.stiffness_sample_times_s.is_empty() {
                return Err(StructDynError::DimensionMismatch {
                    field: "event.library.event.stiffness_sample_times_s",
                    expected: 1,
                    actual: 0,
                });
            }
            if event.stiffness_sample_times_s.len() != event.expected_modal_stiffness_scale_1.len()
            {
                return Err(StructDynError::DimensionMismatch {
                    field: "event.library.event.expected_modal_stiffness_scale_1",
                    expected: event.stiffness_sample_times_s.len(),
                    actual: event.expected_modal_stiffness_scale_1.len(),
                });
            }
            if !event
                .stiffness_sample_times_s
                .iter()
                .all(|value| value.is_finite())
            {
                return Err(StructDynError::InvalidNumber {
                    field: "event.library.event.stiffness_sample_times_s",
                });
            }
            stiffness_samples.reserve(event.stiffness_sample_times_s.len());
            for (time_s, expected_row) in event
                .stiffness_sample_times_s
                .iter()
                .copied()
                .zip(event.expected_modal_stiffness_scale_1.iter())
            {
                if expected_row.len() != event.modal_stiffness_scale_1.len() {
                    return Err(StructDynError::DimensionMismatch {
                        field: "event.library.event.expected_modal_stiffness_scale_1",
                        expected: event.modal_stiffness_scale_1.len(),
                        actual: expected_row.len(),
                    });
                }
                validate_modal_stiffness_scale(
                    expected_row,
                    "event.library.event.expected_modal_stiffness_scale_1",
                )?;
                let actual = modal_stiffness_scale_at(
                    profile,
                    &event.modal_stiffness_scale_1,
                    time_s,
                    "event.library.event.modal_stiffness_scale_1",
                )?;
                for (actual_value, expected_value) in actual.iter().zip(expected_row) {
                    if (actual_value - expected_value).abs() > raw.meta.stiffness_abs_tolerance_1 {
                        return Err(StructDynError::InvalidMatrix {
                            field: "event.library.event.expected_modal_stiffness_scale_1",
                            reason: "expected stiffness scale mismatch",
                        });
                    }
                }
                stiffness_samples.push(ClaEventStiffnessSample {
                    time_s,
                    expected_modal_stiffness_scale_1: expected_row.clone(),
                });
            }
        }

        events.push(ClaEventLibraryEvent {
            id: event.id,
            flight_event_kind,
            profile,
            physical_force_n: event.physical_force_n,
            modal_stiffness_scale_1: event.modal_stiffness_scale_1,
            multiplier_samples,
            stiffness_samples,
        });
    }

    Ok(ClaEventLibraryDeck {
        dataset_id: raw.meta.dataset_id,
        source_class: raw.meta.source_class,
        validation: raw.meta.validation,
        provenance: raw.meta.provenance,
        multiplier_abs_tolerance_1: raw.meta.multiplier_abs_tolerance_1,
        stiffness_abs_tolerance_1: raw.meta.stiffness_abs_tolerance_1,
        events,
    })
}

fn modal_stiffness_scale_at(
    profile: EventForceProfile,
    modal_stiffness_scale_1: &[f64],
    time_s: f64,
    field: &'static str,
) -> Result<Vec<f64>, StructDynError> {
    if modal_stiffness_scale_1.is_empty() {
        profile.multiplier_at(time_s)?;
        return Ok(Vec::new());
    }
    validate_modal_stiffness_scale(modal_stiffness_scale_1, field)?;
    let multiplier = profile.multiplier_at(time_s)?;
    let mut scale = Vec::with_capacity(modal_stiffness_scale_1.len());
    for value in modal_stiffness_scale_1 {
        let blended = 1.0 + (multiplier * (*value - 1.0));
        if !blended.is_finite() || blended <= 0.0 {
            return Err(StructDynError::InvalidNumber { field });
        }
        scale.push(blended);
    }
    Ok(scale)
}

fn validate_modal_stiffness_scale(
    values: &[f64],
    field: &'static str,
) -> Result<(), StructDynError> {
    if !values.iter().all(|value| value.is_finite() && *value > 0.0) {
        return Err(StructDynError::InvalidNumber { field });
    }
    Ok(())
}

/// Time profile for one declared physical force vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EventForceProfile {
    /// Zero before `start_s`, full force at and after `start_s`.
    Step {
        /// Event start time, s.
        start_s: f64,
    },
    /// Hold-down style preload release: full force before `start_s`, linearly
    /// decreasing to zero over `duration_s`.
    RampRelease {
        /// Event start time, s.
        start_s: f64,
        /// Release duration, s.
        duration_s: f64,
    },
    /// Unit-area rectangular impulse. The stored force vector is interpreted as
    /// impulse, N*s, and is divided by `duration_s` during the pulse.
    RectangularImpulse {
        /// Event start time, s.
        start_s: f64,
        /// Pulse duration, s.
        duration_s: f64,
    },
    /// Unit-area half-sine impulse. The stored force vector is interpreted as
    /// impulse, N*s, and is multiplied by `pi/(2*duration_s) *
    /// sin(pi*elapsed/duration_s)` during the pulse.
    HalfSineImpulse {
        /// Event start time, s.
        start_s: f64,
        /// Pulse duration, s.
        duration_s: f64,
    },
    /// Deterministic max-Q Dryden gust envelope. The stored force vector is the
    /// peak physical force and is multiplied by `exp(-elapsed/length_scale_s)`
    /// from `start_s` through `duration_s`.
    DrydenGust {
        /// Event start time, s.
        start_s: f64,
        /// Gust truncation duration, s.
        duration_s: f64,
        /// Temporal Dryden length scale, s.
        length_scale_s: f64,
    },
    /// Deterministic max-Q von Karman gust envelope. The stored force vector is
    /// the peak physical force and is multiplied by the normalized
    /// `(1 + (elapsed/L)^2)^(-5/6)` roll-off from `start_s` through
    /// `duration_s`.
    VonKarmanGust {
        /// Event start time, s.
        start_s: f64,
        /// Gust truncation duration, s.
        duration_s: f64,
        /// Temporal von Karman length scale, s.
        length_scale_s: f64,
    },
}

impl EventForceProfile {
    /// Parse a stable profile kind string.
    ///
    /// Accepted values are `step`, `ramp_release`, `rectangular_impulse`, and
    /// `half_sine_impulse`. `duration_s` is required for every non-step
    /// profile.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the kind is unknown or required numeric
    /// fields are invalid.
    pub fn from_kind(
        kind: &str,
        start_s: f64,
        duration_s: Option<f64>,
    ) -> Result<Self, StructDynError> {
        Self::from_kind_with_gust(kind, start_s, duration_s, None)
    }

    /// Parse a stable profile kind string with optional gust metadata.
    ///
    /// Accepted values are `step`, `ramp_release`, `rectangular_impulse`,
    /// `half_sine_impulse`, `dryden_gust`, and `von_karman_gust`.
    /// `duration_s` is required for every non-step profile, and
    /// `gust_length_scale_s` is required for gust profiles.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the kind is unknown or required numeric
    /// fields are invalid.
    pub fn from_kind_with_gust(
        kind: &str,
        start_s: f64,
        duration_s: Option<f64>,
        gust_length_scale_s: Option<f64>,
    ) -> Result<Self, StructDynError> {
        match kind {
            "step" => {
                validate_start(start_s)?;
                Ok(Self::Step { start_s })
            }
            "ramp_release" => {
                let duration_s = duration_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.duration_s",
                })?;
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                Ok(Self::RampRelease {
                    start_s,
                    duration_s,
                })
            }
            "rectangular_impulse" => {
                let duration_s = duration_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.duration_s",
                })?;
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                Ok(Self::RectangularImpulse {
                    start_s,
                    duration_s,
                })
            }
            "half_sine_impulse" => {
                let duration_s = duration_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.duration_s",
                })?;
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                Ok(Self::HalfSineImpulse {
                    start_s,
                    duration_s,
                })
            }
            "dryden_gust" => {
                let duration_s = duration_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.duration_s",
                })?;
                let length_scale_s = gust_length_scale_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.gust_length_scale_s",
                })?;
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                validate_gust_length_scale(length_scale_s)?;
                Ok(Self::DrydenGust {
                    start_s,
                    duration_s,
                    length_scale_s,
                })
            }
            "von_karman_gust" => {
                let duration_s = duration_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.duration_s",
                })?;
                let length_scale_s = gust_length_scale_s.ok_or(StructDynError::InvalidNumber {
                    field: "event.gust_length_scale_s",
                })?;
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                validate_gust_length_scale(length_scale_s)?;
                Ok(Self::VonKarmanGust {
                    start_s,
                    duration_s,
                    length_scale_s,
                })
            }
            _ => Err(StructDynError::InvalidString {
                field: "event.kind",
            }),
        }
    }

    /// Evaluate the force multiplier at one time.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when time or stored profile fields are invalid.
    pub fn multiplier_at(self, time_s: f64) -> Result<f64, StructDynError> {
        if !time_s.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "event.time_s",
            });
        }
        match self {
            Self::Step { start_s } => {
                validate_start(start_s)?;
                Ok(if time_s >= start_s { 1.0 } else { 0.0 })
            }
            Self::RampRelease {
                start_s,
                duration_s,
            } => {
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                if time_s <= start_s {
                    Ok(1.0)
                } else if time_s >= start_s + duration_s {
                    Ok(0.0)
                } else {
                    Ok(1.0 - ((time_s - start_s) / duration_s))
                }
            }
            Self::RectangularImpulse {
                start_s,
                duration_s,
            } => {
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                if time_s >= start_s && time_s < start_s + duration_s {
                    Ok(1.0 / duration_s)
                } else {
                    Ok(0.0)
                }
            }
            Self::HalfSineImpulse {
                start_s,
                duration_s,
            } => {
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                let elapsed_s = time_s - start_s;
                if elapsed_s <= 0.0 || elapsed_s >= duration_s {
                    Ok(0.0)
                } else {
                    Ok((std::f64::consts::PI / (2.0 * duration_s))
                        * ((std::f64::consts::PI * elapsed_s) / duration_s).sin())
                }
            }
            Self::DrydenGust {
                start_s,
                duration_s,
                length_scale_s,
            } => {
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                validate_gust_length_scale(length_scale_s)?;
                if time_s < start_s || time_s > start_s + duration_s {
                    Ok(0.0)
                } else {
                    Ok((-(time_s - start_s) / length_scale_s).exp())
                }
            }
            Self::VonKarmanGust {
                start_s,
                duration_s,
                length_scale_s,
            } => {
                validate_start(start_s)?;
                validate_duration(duration_s)?;
                validate_gust_length_scale(length_scale_s)?;
                if time_s < start_s || time_s > start_s + duration_s {
                    Ok(0.0)
                } else {
                    let normalized_time = (time_s - start_s) / length_scale_s;
                    Ok((1.0 + (normalized_time * normalized_time)).powf(-5.0 / 6.0))
                }
            }
        }
    }
}

/// One declared event force in physical structural DOFs.
#[derive(Clone, Debug, PartialEq)]
pub struct EventForcing {
    id: String,
    profile: EventForceProfile,
    physical_force: Vec<f64>,
}

impl EventForcing {
    /// Construct a force event.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the id is empty, the force vector is
    /// empty or non-finite, or the profile is invalid.
    pub fn new(
        id: impl Into<String>,
        profile: EventForceProfile,
        physical_force: Vec<f64>,
    ) -> Result<Self, StructDynError> {
        let id = id.into();
        if id.trim().is_empty() {
            return Err(StructDynError::InvalidString { field: "event.id" });
        }
        if physical_force.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "event.physical_force",
                expected: 1,
                actual: 0,
            });
        }
        if !physical_force.iter().all(|value| value.is_finite()) {
            return Err(StructDynError::InvalidNumber {
                field: "event.physical_force",
            });
        }
        profile.multiplier_at(profile_start_time(profile))?;
        Ok(Self {
            id,
            profile,
            physical_force,
        })
    }

    /// Stable event id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Event time profile.
    #[must_use]
    pub const fn profile(&self) -> EventForceProfile {
        self.profile
    }

    /// Physical force vector in structural DOF order.
    #[must_use]
    pub fn physical_force(&self) -> &[f64] {
        &self.physical_force
    }

    /// Physical force at one time.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the profile cannot be evaluated.
    pub fn physical_force_at(&self, time_s: f64) -> Result<Vec<f64>, StructDynError> {
        let multiplier = self.profile.multiplier_at(time_s)?;
        Ok(self
            .physical_force
            .iter()
            .map(|value| multiplier * *value)
            .collect())
    }

    /// Modal generalized force `Phi^T F(t)` at one time.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not conform.
    pub fn modal_force_at(
        &self,
        time_s: f64,
        mode_shapes: &DenseMatrix,
    ) -> Result<Vec<f64>, StructDynError> {
        let force = self.physical_force_at(time_s)?;
        project_force_to_modes(mode_shapes, &force)
    }
}

/// Max-absolute modal force envelope for one event over a declared sample grid.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalForceEnvelope {
    event_id: String,
    max_abs_modal_force: Vec<f64>,
    sample_count: usize,
}

impl ModalForceEnvelope {
    /// Event id.
    #[must_use]
    pub fn event_id(&self) -> &str {
        &self.event_id
    }

    /// Max absolute modal force per retained mode.
    #[must_use]
    pub fn max_abs_modal_force(&self) -> &[f64] {
        &self.max_abs_modal_force
    }

    /// Number of sampled times used to build the envelope.
    #[must_use]
    pub const fn sample_count(&self) -> usize {
        self.sample_count
    }
}

/// Project a physical force vector into modal generalized coordinates.
///
/// The mode-shape matrix stores one physical DOF per row and one retained mode
/// per column, so this evaluates `Phi^T F` in locked column order.
///
/// # Errors
///
/// Returns [`StructDynError`] when dimensions mismatch, no modes are present, or
/// the force vector contains non-finite values.
pub fn project_force_to_modes(
    mode_shapes: &DenseMatrix,
    physical_force: &[f64],
) -> Result<Vec<f64>, StructDynError> {
    if mode_shapes.rows() != physical_force.len() {
        return Err(StructDynError::DimensionMismatch {
            field: "event.mode_shape.rows",
            expected: mode_shapes.rows(),
            actual: physical_force.len(),
        });
    }
    if mode_shapes.cols() == 0 {
        return Err(StructDynError::DimensionMismatch {
            field: "event.mode_shape.cols",
            expected: 1,
            actual: 0,
        });
    }
    if !physical_force.iter().all(|value| value.is_finite()) {
        return Err(StructDynError::InvalidNumber {
            field: "event.physical_force",
        });
    }
    let mut modal_force = vec![0.0; mode_shapes.cols()];
    for (mode, out) in modal_force.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (row, force) in physical_force.iter().enumerate() {
            sum += mode_shapes.get(row, mode) * *force;
        }
        *out = sum;
    }
    Ok(modal_force)
}

/// Sample one event and return a max-absolute modal force envelope.
///
/// # Errors
///
/// Returns [`StructDynError`] when the sample grid is empty, contains non-finite
/// values, or modal projection fails.
pub fn modal_force_envelope(
    event: &EventForcing,
    sample_times_s: &[f64],
    mode_shapes: &DenseMatrix,
) -> Result<ModalForceEnvelope, StructDynError> {
    if sample_times_s.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field: "event.sample_times_s",
            expected: 1,
            actual: 0,
        });
    }
    if !sample_times_s.iter().all(|value| value.is_finite()) {
        return Err(StructDynError::InvalidNumber {
            field: "event.sample_times_s",
        });
    }
    let mut max_abs_modal_force = vec![0.0_f64; mode_shapes.cols()];
    for time_s in sample_times_s {
        let modal_force = event.modal_force_at(*time_s, mode_shapes)?;
        for (out, value) in max_abs_modal_force.iter_mut().zip(modal_force) {
            *out = out.max(value.abs());
        }
    }
    Ok(ModalForceEnvelope {
        event_id: event.id.clone(),
        max_abs_modal_force,
        sample_count: sample_times_s.len(),
    })
}

fn validate_start(start_s: f64) -> Result<(), StructDynError> {
    if start_s.is_finite() && start_s >= 0.0 {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber {
            field: "event.start_s",
        })
    }
}

fn validate_duration(duration_s: f64) -> Result<(), StructDynError> {
    if duration_s.is_finite() && duration_s > 0.0 {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber {
            field: "event.duration_s",
        })
    }
}

fn validate_gust_length_scale(length_scale_s: f64) -> Result<(), StructDynError> {
    if length_scale_s.is_finite() && length_scale_s > 0.0 {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber {
            field: "event.gust_length_scale_s",
        })
    }
}

fn profile_start_time(profile: EventForceProfile) -> f64 {
    match profile {
        EventForceProfile::Step { start_s }
        | EventForceProfile::RampRelease { start_s, .. }
        | EventForceProfile::RectangularImpulse { start_s, .. }
        | EventForceProfile::HalfSineImpulse { start_s, .. }
        | EventForceProfile::DrydenGust { start_s, .. }
        | EventForceProfile::VonKarmanGust { start_s, .. } => start_s,
    }
}

fn require_nonempty_str(field: &'static str, value: &str) -> Result<(), StructDynError> {
    if value.trim().is_empty() {
        Err(StructDynError::InvalidString { field })
    } else {
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawClaEventLibraryDeck {
    openbmp_structdyn_cla_event_library: u32,
    meta: RawClaEventLibraryMeta,
    event: Vec<RawClaEventLibraryEvent>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawClaEventLibraryMeta {
    dataset_id: String,
    source_class: String,
    validation: String,
    provenance: String,
    multiplier_abs_tolerance_1: f64,
    stiffness_abs_tolerance_1: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawClaEventLibraryEvent {
    id: String,
    flight_event_kind: String,
    profile_kind: String,
    start_s: f64,
    duration_s: Option<f64>,
    gust_length_scale_s: Option<f64>,
    physical_force_n: Vec<f64>,
    #[serde(default)]
    modal_stiffness_scale_1: Vec<f64>,
    sample_times_s: Vec<f64>,
    expected_multiplier_1: Vec<f64>,
    #[serde(default)]
    stiffness_sample_times_s: Vec<f64>,
    #[serde(default)]
    expected_modal_stiffness_scale_1: Vec<Vec<f64>>,
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn cla_event_library_fixture_covers_named_flight_events() {
        let deck = parse_cla_event_library_toml(include_str!(
            "../../../data/structdyn/cla-event-library-v1.toml"
        ))
        .unwrap();

        assert_eq!(deck.dataset_id, "openbmp.structdyn.cla_event_library.v1");
        assert_eq!(deck.source_class, "openbmp-local-event-library");
        assert_eq!(deck.events.len(), 5);
        assert!((deck.stiffness_abs_tolerance_1 - 1.0e-12).abs() <= f64::EPSILON);

        let mut kinds = BTreeSet::new();
        let mut stiffness_event_count = 0;
        for event in &deck.events {
            kinds.insert(event.flight_event_kind);
            assert_eq!(
                ClaFlightEventKind::from_kind(event.flight_event_kind.as_str()).unwrap(),
                event.flight_event_kind
            );
            let forcing = event.event_forcing().unwrap();
            assert_eq!(forcing.id(), event.id);

            let times: Vec<f64> = event
                .multiplier_samples
                .iter()
                .map(|sample| sample.time_s)
                .collect();
            let mode_shapes = DenseMatrix::from_row_major(
                event.physical_force_n.len(),
                1,
                vec![1.0; event.physical_force_n.len()],
            )
            .unwrap();
            let envelope = modal_force_envelope(&forcing, &times, &mode_shapes).unwrap();
            assert_eq!(envelope.sample_count(), times.len());
            assert_eq!(envelope.event_id(), event.id);

            for sample in &event.multiplier_samples {
                let actual = event.profile.multiplier_at(sample.time_s).unwrap();
                assert!(
                    (actual - sample.expected_multiplier_1).abs()
                        <= deck.multiplier_abs_tolerance_1
                );
            }
            if !event.modal_stiffness_scale_1.is_empty() {
                stiffness_event_count += 1;
                assert!(event.flight_event_kind.accepts_stiffness_change());
                for sample in &event.stiffness_samples {
                    let actual = event.modal_stiffness_scale_at(sample.time_s).unwrap();
                    assert_eq!(actual.len(), sample.expected_modal_stiffness_scale_1.len());
                    for (actual_value, expected_value) in
                        actual.iter().zip(&sample.expected_modal_stiffness_scale_1)
                    {
                        assert!(
                            (actual_value - expected_value).abs() <= deck.stiffness_abs_tolerance_1
                        );
                    }
                }
            } else {
                assert!(event.stiffness_samples.is_empty());
            }
        }

        assert!(kinds.contains(&ClaFlightEventKind::LiftoffRelease));
        assert!(kinds.contains(&ClaFlightEventKind::MaxQGust));
        assert!(kinds.contains(&ClaFlightEventKind::EngineIgnition));
        assert!(kinds.contains(&ClaFlightEventKind::StageSeparation));
        assert!(kinds.contains(&ClaFlightEventKind::MainEngineCutoff));
        assert_eq!(stiffness_event_count, 3);
    }

    #[test]
    fn cla_event_library_rejects_unknown_kind_profile_mismatch_and_drift() {
        let deck = include_str!("../../../data/structdyn/cla-event-library-v1.toml");

        let unknown_kind = deck.replace(
            "flight_event_kind = \"meco\"",
            "flight_event_kind = \"unknown_event\"",
        );
        assert!(matches!(
            parse_cla_event_library_toml(&unknown_kind),
            Err(StructDynError::InvalidString { field })
                if field == "event.flight_event_kind"
        ));

        let profile_mismatch = deck.replacen(
            "profile_kind = \"ramp_release\"",
            "profile_kind = \"step\"",
            1,
        );
        assert!(matches!(
            parse_cla_event_library_toml(&profile_mismatch),
            Err(StructDynError::InvalidString { field })
                if field == "event.library.event.profile_kind"
        ));

        let multiplier_drift = deck.replace(
            "expected_multiplier_1 = [1.0, 0.5, 0.0]",
            "expected_multiplier_1 = [1.0, 0.4, 0.0]",
        );
        assert!(matches!(
            parse_cla_event_library_toml(&multiplier_drift),
            Err(StructDynError::InvalidMatrix { field, .. })
                if field == "event.library.event.expected_multiplier_1"
        ));

        let liftoff_stiffness = deck.replacen(
            "physical_force_n = [120000.0, -8000.0]",
            "physical_force_n = [120000.0, -8000.0]\nmodal_stiffness_scale_1 = [0.95, 1.05]",
            1,
        );
        assert!(matches!(
            parse_cla_event_library_toml(&liftoff_stiffness),
            Err(StructDynError::InvalidString { field })
                if field == "event.library.event.modal_stiffness_scale_1"
        ));

        let stiffness_drift = deck.replace(
            "[2.178097245096172, 0.21460183660255172]",
            "[2.178097245096172, 0.31460183660255172]",
        );
        assert!(matches!(
            parse_cla_event_library_toml(&stiffness_drift),
            Err(StructDynError::InvalidMatrix { field, .. })
                if field == "event.library.event.expected_modal_stiffness_scale_1"
        ));
    }

    #[test]
    fn event_forcing_projects_physical_force_into_modal_coordinates() {
        let mode_shapes = DenseMatrix::from_row_major(
            3,
            2,
            vec![
                1.0, 0.0, //
                0.5, 2.0, //
                -1.0, 1.0,
            ],
        )
        .unwrap();
        let event = EventForcing::new(
            "max_q_gust",
            EventForceProfile::from_kind("step", 2.0, None).unwrap(),
            vec![4.0, 6.0, -2.0],
        )
        .unwrap();

        let inactive_force = event.modal_force_at(1.0, &mode_shapes).unwrap();
        assert_eq!(inactive_force[0].to_bits(), 0.0_f64.to_bits());
        assert_eq!(inactive_force[1].to_bits(), 0.0_f64.to_bits());
        let modal_force = event.modal_force_at(2.0, &mode_shapes).unwrap();
        assert!((modal_force[0] - 9.0).abs() < 1.0e-14);
        assert!((modal_force[1] - 10.0).abs() < 1.0e-14);
    }

    #[test]
    fn event_profiles_and_envelopes_are_fail_closed() {
        assert!(EventForceProfile::from_kind("unknown", 0.0, None).is_err());
        assert!(EventForceProfile::from_kind("ramp_release", 0.0, Some(0.0)).is_err());
        assert!(
            EventForceProfile::Step { start_s: -1.0 }
                .multiplier_at(0.0)
                .is_err()
        );

        let release = EventForceProfile::from_kind("ramp_release", 10.0, Some(2.0)).unwrap();
        assert_eq!(
            release.multiplier_at(9.0).unwrap().to_bits(),
            1.0_f64.to_bits()
        );
        assert!((release.multiplier_at(11.0).unwrap() - 0.5).abs() < 1.0e-14);
        assert_eq!(
            release.multiplier_at(12.0).unwrap().to_bits(),
            0.0_f64.to_bits()
        );

        let impulse = EventForceProfile::from_kind("rectangular_impulse", 3.0, Some(0.25)).unwrap();
        assert_eq!(
            impulse.multiplier_at(2.9).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert!((impulse.multiplier_at(3.1).unwrap() - 4.0).abs() < 1.0e-14);
        assert_eq!(
            impulse.multiplier_at(3.25).unwrap().to_bits(),
            0.0_f64.to_bits()
        );

        let half_sine = EventForceProfile::from_kind("half_sine_impulse", 4.0, Some(0.5)).unwrap();
        assert_eq!(
            half_sine.multiplier_at(4.0).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert!((half_sine.multiplier_at(4.25).unwrap() - std::f64::consts::PI).abs() < 1.0e-14);
        assert_eq!(
            half_sine.multiplier_at(4.5).unwrap().to_bits(),
            0.0_f64.to_bits()
        );

        let gust =
            EventForceProfile::from_kind_with_gust("dryden_gust", 20.0, Some(4.0), Some(2.0))
                .unwrap();
        assert_eq!(
            gust.multiplier_at(19.9).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            gust.multiplier_at(20.0).unwrap().to_bits(),
            1.0_f64.to_bits()
        );
        assert!((gust.multiplier_at(22.0).unwrap() - std::f64::consts::E.recip()).abs() < 1.0e-14);
        assert_eq!(
            gust.multiplier_at(24.1).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert!(
            EventForceProfile::from_kind_with_gust("dryden_gust", 0.0, Some(1.0), None).is_err()
        );
        let von_karman =
            EventForceProfile::from_kind_with_gust("von_karman_gust", 30.0, Some(4.0), Some(2.0))
                .unwrap();
        assert_eq!(
            von_karman.multiplier_at(29.9).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            von_karman.multiplier_at(30.0).unwrap().to_bits(),
            1.0_f64.to_bits()
        );
        assert!(
            (von_karman.multiplier_at(32.0).unwrap() - 2.0_f64.powf(-5.0 / 6.0)).abs() < 1.0e-14
        );
        assert_eq!(
            von_karman.multiplier_at(34.1).unwrap().to_bits(),
            0.0_f64.to_bits()
        );
        assert!(
            EventForceProfile::from_kind_with_gust("von_karman_gust", 0.0, Some(1.0), None)
                .is_err()
        );
    }

    #[test]
    fn modal_force_envelope_samples_event_in_declaration_order() {
        let mode_shapes = DenseMatrix::from_row_major(2, 2, vec![1.0, -1.0, 0.25, 2.0]).unwrap();
        let event = EventForcing::new(
            "liftoff_release",
            EventForceProfile::from_kind("ramp_release", 1.0, Some(1.0)).unwrap(),
            vec![8.0, -4.0],
        )
        .unwrap();

        let envelope = modal_force_envelope(&event, &[0.0, 1.5, 2.0], &mode_shapes).unwrap();

        assert_eq!(envelope.event_id(), "liftoff_release");
        assert_eq!(envelope.sample_count(), 3);
        assert!((envelope.max_abs_modal_force()[0] - 7.0).abs() < 1.0e-14);
        assert!((envelope.max_abs_modal_force()[1] - 16.0).abs() < 1.0e-14);
        assert!(modal_force_envelope(&event, &[], &mode_shapes).is_err());
        assert!(
            event
                .modal_force_at(0.0, &DenseMatrix::zeros(2, 0).unwrap())
                .is_err()
        );
    }
}
