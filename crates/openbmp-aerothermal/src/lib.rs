//! `openbmp-aerothermal` — OpenBMP aerothermal heat transfer and
//! surface state for hypersonic re-entry studies.
//!
//! Modules:
//!
//! * [`stagnation`]: stagnation-point heating. Ships
//!   [`stagnation::FayRiddell`] as a cold-gas engineering scaffold
//!   plus caller-supplied [`stagnation::FayRiddellEdgeState`] assembly,
//!   with a neutral-composition helper for the dissociation enthalpy term,
//!   [`stagnation::SuttonGraves`] (engineering
//!   `K · √(ρ/R) · V³` simplification, NASA TR R-376
//!   [`stagnation::SuttonGravesGas`] coefficient presets, plus
//!   Allen-Eggers trajectory heat-load diagnostics), and
//!   opt-in [`stagnation::TauberSuttonRadiative`] table-driven
//!   radiative heating.
//! * [`boundary_layer`]: BL state, transition models, and
//!   distributed surface heating via reference-enthalpy / Spalding-Chi /
//!   Van Driest correlations.
//! * [`thermal_toy`]: 1-D explicit-FD surface-temperature
//!   evolution under prescribed heat flux.
//! * [`ablation`]: generic textbook ablators with blowing
//!   correction, surface recession, and an energy-limited 1-D
//!   pyrolysis-front toy.
//!
//! The crate exposes [`stagnation::HeatTransferModel`] as the unified
//! trait surface that downstream consumers (telemetry exporter,
//! validation harness) depend on.

#![deny(missing_docs)]

pub mod ablation;
pub mod boundary_layer;
pub mod error;
pub mod stagnation;
pub mod thermal_toy;

pub use ablation::{
    AblationModel, BlowingCorrelation, CharringAblator, DepthResolvedCharringAblator,
    PyrolysisFrontUpdate, RecessionRate, SteadyStateAblator, SurfaceState, ToyAblator,
};
pub use boundary_layer::{
    BoundaryLayer, BoundaryLayerState, EmpiricalTransition, EnTransition,
    PANT_REDA_ROUGHNESS_CRITICAL_INDICATOR, PantRedaRoughnessTransition, ReThetaTransition,
    ReferenceEnthalpyHeating, TransitionModel,
};
pub use error::AerothermalError;
pub use stagnation::{
    AerothermalContext, BallisticEntryHeating, BodyStation,
    DETRA_KEMP_RIDDELL_REFERENCE_HEAT_FLUX_BTU_FT2_S, DETRA_KEMP_RIDDELL_REFERENCE_VELOCITY_FT_S,
    DETRA_KEMP_RIDDELL_REFERENCE_WALL_TEMPERATURE_K, DETRA_KEMP_RIDDELL_SEA_LEVEL_DENSITY_KG_M3,
    DETRA_KEMP_RIDDELL_VELOCITY_EXPONENT, DetraKempRiddell, FayRiddell, FayRiddellEdgeState,
    HeatTransferModel, SUTTON_GRAVES_K_EARTH_SI, SUTTON_GRAVES_TABLE_K_EARTH_AIR,
    StagnationHeating, SurfaceHeating, SuttonGraves, SuttonGravesGas,
    TAUBER_SUTTON_EARTH_AIR_COEFFICIENT_W_CM2, TAUBER_SUTTON_EARTH_AIR_DENSITY_EXPONENT,
    TAUBER_SUTTON_EARTH_AIR_PAYLOAD_SHA256_HEX,
    TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_COEFFICIENT,
    TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_DENSITY_POWER,
    TAUBER_SUTTON_EARTH_AIR_RADIUS_EXPONENT_VELOCITY_POWER,
    TAUBER_SUTTON_EARTH_AIR_VELOCITY_FUNCTION, TauberSuttonPackage, TauberSuttonRadiative,
    TauberSuttonVelocityFunctionPoint, TotalStagnationHeating, WallCatalysis,
    tauber_sutton_earth_air_package,
};
pub use thermal_toy::{BackwallCondition, OneDThermalToy, ToyMaterial};
