//! `openbmp-physics` — HAL-portable physics for OpenBMP.
//!
//! Owns every physics formula and constant the workspace shares
//! between the simulator-side environment and the flight controller:
//!
//! * [`frames`] — `FrameProfile`, `LocalGeodeticOrigin`, `FrameContext`,
//!   the time-aware [`FrameTransform`] impls,
//!   and the WGS84 ellipsoid constants
//!   (`WGS84_A_M`, `WGS84_INV_FLATTENING`, `WGS84_FLATTENING`,
//!   `WGS84_ECCENTRICITY_SQUARED`, `WGS84_MU_M3_S2`,
//!   `WGS84_OMEGA_RAD_S`).
//! * [`gravity`] — `GravityModel` trait + `ConstantGravity`,
//!   `PointMassGravity`, `J2Gravity`, `TesseralGravity`,
//!   `FiniteDifferencePinesGravity`, `Egm2008ZonalGravity`,
//!   `RelativisticCorrection`, plus standard-gravity, J2, SRP, and EGM2008
//!   zonal-harmonic constants.
//! * [`atmosphere`] — `AtmosphereModel` trait + `IsothermalAtmosphere`,
//!   `UsStandard1976` (full 7-layer, 0–86 km), plus USSA76 constants
//!   and closed-form helpers (`pressure_altitude_troposphere_m`).
//! * [`ephemeris`] — deterministic celestial body position providers
//!   for Sun/Moon third-body perturbation studies.
//! * [`magnetic`] — `MagneticModel` and `MagneticFieldEci` traits,
//!   `EarthDipoleField` (degree-1 academic toy), and `Wmm2025`
//!   (NOAA / NCEI 2025 release, 12-degree spherical harmonic).
//! * [`wind`] — `WindModel` trait + `NoWind`, `ConstantWind`,
//!   `LayeredWind`, `Hwm14Wind`, `GustWind` (Dryden, gated by the
//!   `synthetic` feature).
//! * [`earth`] — Earth-radius constants used by low-order toy models.
//! * [`validity`] — small finite-range helpers for model envelopes.
//! * [`error::PhysicsError`] — uniform error type for runtime
//!   evaluation failures (out-of-envelope, non-finite, invalid
//!   parameter).
//!
//! # Layering
//!
//! `openbmp-physics` depends only on `openbmp-core` (foundation
//! types: `SimTime`, `Position3`, `Eci`, `Ned`, `Velocity3`,
//! `Quaternion`, `FrameError`, `Frame` trait + tag types,
//! `DeterministicRng`) and `nalgebra`. **Both `openbmp-fc`
//! (controller-side) and `openbmp-sim` / `openbmp-cli`
//! (simulator-side) consume this crate directly.** No simulator
//! trait surfaces or scenario parsing live here.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic with locked operand order on every model;
//! no FMA, no wall-clock time, no system RNG, no network, no file
//! I/O. The optional `GustWind` model uses
//! [`openbmp_core::DeterministicRng`], whose seed is derived
//! deterministically from `(scenario_seed, step, channel_id)`.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(missing_docs)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod atmosphere;
#[cfg(feature = "std")]
pub mod ephemeris;
pub mod error;
#[cfg(feature = "std")]
pub mod external_reference;
pub mod frames;
pub mod gravity;
pub mod kinematics;
pub mod magnetic;
#[cfg(feature = "std")]
pub mod profile;
#[cfg(feature = "std")]
pub mod realgas;
#[cfg(feature = "std")]
pub mod reentry;
pub mod statistics;
#[cfg(feature = "std")]
pub mod uq;
pub mod validity;
pub mod wind;

pub use atmosphere::{
    AtmosphereModel, AtmosphereSample, ExoatmosphericPolicy, ExponentialLayer,
    IsothermalAtmosphere, PIECEWISE_EXP_MAX_GEOMETRIC_M, PiecewiseExpExoatmosphericPolicy,
    PiecewiseExponentialAtmosphere, UsStandard1976, sutherland_viscosity,
};
#[cfg(feature = "std")]
pub use atmosphere::{
    Nrlmsis2Compat, Nrlmsis2CompatOutputs, Nrlmsise00Full, Nrlmsise00Inputs, Nrlmsise00Outputs,
    Nrlmsise00Static,
};
#[cfg(feature = "std")]
pub use ephemeris::{
    ASTRONOMICAL_UNIT_M, CelestialBody, EphemerisModel, EphemerisState, J2000_JULIAN_DATE,
    LowPrecisionSunMoonEphemeris, MOON_MU_M3_S2, SUN_MU_M3_S2, SpkEphemeris, SpkFixedFrame,
};
pub use error::PhysicsError;
#[cfg(feature = "std")]
pub use external_reference::{
    EnvelopeBounds, ExternalReferencePackage, ProvenanceBlock, ReferencePackageKind, ReferenceQuery,
};
pub use frames::{
    ARCSECOND_TO_RAD, EARTH_ROTATION_ANGLE_RATE_RAD_S, EarthOrientationSample,
    EarthOrientationTable, FrameContext, FrameProfile, FrameTransform, LocalGeodeticOrigin,
    WGS84_A_M, WGS84_ECCENTRICITY_SQUARED, WGS84_FLATTENING, WGS84_INV_FLATTENING, WGS84_MU_M3_S2,
    WGS84_OMEGA_RAD_S,
};
pub use gravity::{
    ConstantGravity, DegreeTwoTesseralCoefficients, EGM2008_J3, EGM2008_J4, EGM2008_J5, EGM2008_J6,
    EGM2008_MAX_DEGREE, Egm2008ZonalGravity, FiniteDifferencePinesGravity, GottliebPotentialSum,
    GravityModel, HARMONIC_LONGITUDE_MAX_ORDER, HARMONIC_SYNTHESIS_EGM2008_DEGREE_70,
    HARMONIC_SYNTHESIS_EGM2008_DEGREE_120, HARMONIC_SYNTHESIS_EGM2008_DEGREE_360,
    HarmonicLongitudeTrigonometry, HarmonicSynthesisPlan, HarmonicSynthesisTier,
    HarmonicTruncation, J2Gravity, NormalizedHarmonicCoefficient, NormalizedHarmonicField,
    NormalizedHarmonicFieldIter, PINES_LEGENDRE_MAX_DEGREE, PinesLegendreTable,
    PinesLongitudePolynomials, PinesPotentialSum, PinesSynthesisPoint, PointMassGravity,
    RelativisticCorrection, SOLAR_RADIATION_PRESSURE_1_AU_N_M2, SPEED_OF_LIGHT_M_S,
    STANDARD_GRAVITY_M_S2, TESSERAL_GRAVITY_MAX_DEGREE, TESSERAL_GRAVITY_MAX_ORDER,
    TesseralGravity, TideSystem, WGS84_J2, fully_normalized_to_unnormalized_scale,
    standard_down_z_eci_m_s2,
};
#[cfg(feature = "std")]
pub use gravity::{
    IAU_NOMINAL_SOLAR_RADIUS_M, IcgemGfcNormalizedField, SolarRadiationPressure, ThirdBody,
    ThirdBodyGravity,
};
pub use kinematics::{
    quaternion_error_small_angle, quaternion_from_axis_angle, quaternion_from_omega,
    renormalize_quaternion, skew_symmetric,
};
#[cfg(feature = "std")]
pub use magnetic::Wmm2025;
pub use magnetic::{
    EARTH_DIPOLE_EQUATORIAL_FIELD_NT, EarthDipoleField, MagneticFieldEci, MagneticModel,
};
#[cfg(feature = "std")]
pub use profile::{
    IdealStagingBudgetAnalysis, StageMassProperties, StagingBudgetAnalysis, StagingBudgetInput,
    StagingBudgetMode, StagingBudgetReport, StagingStageReport,
};
#[cfg(feature = "std")]
pub use realgas::{
    AirComposition, ArrheniusForwardCoefficient, EquilibriumAir, EquilibriumAirState, FlowContext,
    MillikanWhitePairCoefficient, MugalevEquilibriumAir, NonequilibriumAir,
    PARK_2T_REFERENCE_PAYLOAD_SHA256_HEX, PARK_2T_REFERENCE_PAYLOAD_V1,
    PARK87_FORWARD_REACTIONS_AS_PUBLISHED, PARK93_FORWARD_REACTIONS_AS_PUBLISHED, ParkAirSpecies,
    ParkForwardReaction, ParkReactionSet, ParkTwoTemperatureModel, ReactionRates,
    TannehillEquilibriumAir, VibrationalEnergyDerivative, VibrationalRelaxationModel,
    park_2t_reference_package, park87_millikan_white_pair_coefficients,
    validate_park_2t_reference_package,
};
#[cfg(feature = "std")]
pub use reentry::{
    APOLLO_CM_TABLE13_HEATING, APOLLO4_ENTRY_INTERFACE, APOLLO4_ENTRY_TIMELINE_EVENTS, AllenEggers,
    EntryInterfaceBuilder, PUBLIC_ENTRY_BENCHMARK_PAYLOAD_SHA256_HEX,
    PUBLIC_ENTRY_BENCHMARK_PAYLOAD_V1, PublicEntryHeatingBenchmark, PublicEntryInterfaceBenchmark,
    PublicEntryTimelineEvent, PublicStardustPeakBenchmark, PublicStardustTrajectoryInputBenchmark,
    PublicStardustTrajectoryOutputBenchmark, STARDUST_SRC_TABLE13_ENTRY_INTERFACE,
    STARDUST_SRC_TABLE13_HEATING, STARDUST_SRC_TABLE19_TRAJ_INPUT,
    STARDUST_SRC_TABLE20_TRAJ_OUTPUT, Vinh, VinhState, VinhStateDerivative,
    public_entry_benchmark_reference_package, validate_public_entry_benchmark_reference_package,
    validate_public_entry_timeline,
};
pub use statistics::{chi_square_inverse_cdf_wilson_hilferty, inverse_standard_normal_cdf};
#[cfg(feature = "std")]
pub use uq::{ErrorBudget, UncertaintyContribution, ValidationStatus};
pub use validity::HalfOpenRange;
pub use wind::{ConstantWind, NoWind, WindModel};
#[cfg(feature = "synthetic")]
pub use wind::{GustWind, GustWindParams};
#[cfg(feature = "std")]
pub use wind::{
    HWM14_REFERENCE_MAX_ALTITUDE_M, Hwm14Inputs, Hwm14ReferenceRow, Hwm14Wind, LayerEntry,
    LayeredWind,
};

/// Earth constants used by low-order academic reference models.
pub mod earth {
    /// Mean spherical Earth radius (m) used by degree-1 toy models
    /// (e.g. the `EarthDipoleField` magnetic placeholder).
    ///
    /// Distinct from the WGS84 semi-major axis used by geodetic and
    /// J2 models — see [`crate::frames::WGS84_A_M`].
    pub const MEAN_RADIUS_M: f64 = 6_371_000.0;
}
