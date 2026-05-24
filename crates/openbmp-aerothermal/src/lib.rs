//! `openbmp-aerothermal` — OpenBMP aerothermal heat transfer and
//! surface state for hypersonic re-entry studies.
//!
//! Phase 6 sub-phases:
//!
//! * **6.4** — [`stagnation`]: stagnation-point heating. Ships
//!   [`stagnation::FayRiddell`] (1958 equilibrium-air catalytic-wall
//!   axisymmetric), [`stagnation::SuttonGraves`] (engineering
//!   `K · √(ρ/R) · V³` simplification), and
//!   [`stagnation::TauberSuttonRadiative`] (radiative heating
//!   engineering estimate).
//! * **6.5** — [`boundary_layer`]: BL state, transition models, and
//!   distributed surface heating via reference-enthalpy / Spalding-Chi /
//!   Van Driest correlations.
//! * **6.9** — [`thermal_toy`]: 1-D explicit-FD surface-temperature
//!   evolution under prescribed heat flux.
//! * **6.11** — [`ablation`]: generic textbook ablators with blowing
//!   correction and surface recession.
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
    AblationModel, BlowingCorrelation, CharringAblator, RecessionRate, SteadyStateAblator,
    SurfaceState, ToyAblator,
};
pub use boundary_layer::{
    BoundaryLayer, BoundaryLayerState, EmpiricalTransition, EnTransition,
    ReferenceEnthalpyHeating, ReThetaTransition, TransitionModel,
};
pub use error::AerothermalError;
pub use stagnation::{
    AerothermalContext, BodyStation, FayRiddell, HeatTransferModel, StagnationHeating,
    SUTTON_GRAVES_K_EARTH_SI, SurfaceHeating, SuttonGraves, TauberSuttonRadiative, WallCatalysis,
};
pub use thermal_toy::{BackwallCondition, OneDThermalToy, ToyMaterial};
