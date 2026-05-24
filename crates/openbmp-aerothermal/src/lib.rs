//! `openbmp-aerothermal` — OpenBMP aerothermal heat transfer and
//! surface state for hypersonic re-entry studies.
//!
//! Phase 6 sub-phases:
//!
//! * **6.4** — [`stagnation`]: stagnation-point heating. Ships
//!   [`stagnation::FayRiddell`] as a cold-gas engineering scaffold
//!   plus caller-supplied [`stagnation::FayRiddellEdgeState`] assembly,
//!   [`stagnation::SuttonGraves`] (engineering
//!   `K · √(ρ/R) · V³` simplification), and
//!   [`stagnation::TauberSuttonRadiative`] as a typed-reserved model
//!   pending published coefficients.
//! * **6.5** — [`boundary_layer`]: BL state, transition models, and
//!   distributed surface heating via reference-enthalpy / Spalding-Chi /
//!   Van Driest correlations.
//! * **6.9** — [`thermal_toy`]: 1-D explicit-FD surface-temperature
//!   evolution under prescribed heat flux.
//! * **6.11** — [`ablation`]: generic textbook ablators with blowing
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
    BoundaryLayer, BoundaryLayerState, EmpiricalTransition, EnTransition, ReThetaTransition,
    ReferenceEnthalpyHeating, TransitionModel,
};
pub use error::AerothermalError;
pub use stagnation::{
    AerothermalContext, BodyStation, FayRiddell, FayRiddellEdgeState, HeatTransferModel,
    SUTTON_GRAVES_K_EARTH_SI, StagnationHeating, SurfaceHeating, SuttonGraves,
    TauberSuttonRadiative, WallCatalysis,
};
pub use thermal_toy::{BackwallCondition, OneDThermalToy, ToyMaterial};
