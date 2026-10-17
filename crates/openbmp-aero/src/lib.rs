//! `openbmp-aero` — OpenBMP aerodynamics.
//!
//! Provides:
//!
//! * [`buildup`] — Semi-empirical continuum drag buildup that turns
//!   declared launch-vehicle geometry into an [`deck::AeroDeck`].
//! * [`deck`] — Tabulated aerodynamic deck. **Schema 1** is the
//!   axisymmetric reduced sounding-rocket form indexed by
//!   `(mach, alpha_deg, beta_deg)` returning the three reduced
//!   coefficients `(CN, CD, CM)`. **Schema 2** extends
//!   the same struct with optional effector-axis dimensions
//!   (e.g. `delta_e_deg`); the lookup signature gains a name-keyed
//!   `BTreeMap<&str, f64>` for deflections. The internal
//!   representation is N-D (3 ≤ N ≤ 6); at N = 3 the multilinear
//!   reduction is bit-identical to the Schema-1 trilinear path,
//!   per a 1024-case property test. Decks may optionally carry
//!   per-node one-sigma `(CN, CD, CM)` uncertainty tables and optional
//!   explicit body-axis six-coefficient `(CX, CY, CZ, Cl, Cm, Cn)` tables.
//!   Locked-order operand reduction per Demmel & Nguyen 2020; FMA disabled.
//! * [`parser`] — TOML deck-file parser. Coefficient decks
//!   auto-detect schema 1 vs. schema 2 from the `openbmp.aero_deck`
//!   integer marker and dispatch to strict per-schema parsers. A
//!   separate strict `openbmp.panel_mesh_aero = 1` parser handles
//!   [`hypersonic::LocalInclinationPanels`]. The parser feature also
//!   exposes strict ASCII STL string/file and binary STL byte/file ingestion for
//!   [`hypersonic::PanelMesh`]. All TOML parsers use
//!   `serde(deny_unknown_fields)`.
//! * [`validation`] — parser-gated offline tolerance-fixture helpers for
//!   aerodynamic method validation cases.
//! * [`vlm`] — First attached-flow fin lifting-surface, grid-fin authority, and
//!   horseshoe-vortex substrates for lift/normal force, drag, pitching moment,
//!   and hinge-moment checks.
//! * [`method`] — [`method::AeroMethod`] trait, [`method::AeroContext`]
//!   input, [`method::AeroForceMomentBody`] output, and the
//!   [`method::DeckLookup`] implementation that composes
//!   `(CN, CD, CM)` into body-frame `(force, moment)`.
//! * [`error`] — [`error::AeroError`], the crate's typed error
//!   surface (out-of-envelope, non-finite, invalid parameter,
//!   malformed deck, deck I/O).
//!
//! Hypersonic methods (`ModifiedNewtonian`, `TangentCone`,
//! `LocalInclinationPanels`, `FreeMolecular`) and the `HybridAeroMethod`
//! dispatch are documented in `docs/hypersonic-extensions.md`.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the multilinear
//! reduction (bit-identical to the schema-1 trilinear at N = 3); no
//! FMA, no wall-clock, no system RNG, no network, no file I/O on the
//! hot path. The TOML parser performs file I/O at deck-load time only.
//!
//! # Crate layering
//!
//! `openbmp-aero` is an L2 crate. It depends only on third-party
//! math/serde primitives and **not** on `openbmp-sim` (L1) — the
//! kernel adapts the [`deck::AeroDeck`] / `AeroMethod` surfaces into
//! its `ForceModel` / `MomentModel` chain at a higher layer. See
//! `docs/software-architecture.md#aerodynamics`.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod buildup;
pub mod deck;
pub mod error;
pub mod hypersonic;
pub mod knudsen;
pub mod method;
#[cfg(feature = "parser")]
pub mod parser;
#[cfg(feature = "parser")]
pub mod validation;
pub mod vlm;

pub use buildup::{
    Afterbody, BodyGeometry, BuildupGrid, ComponentBuildup, DatcomComponentBuildup,
    DragBuildupModel, FinSet, FlowCondition, NoseShape, ProtuberanceSet,
};
pub use deck::{
    AeroCoefficientSigmas, AeroCoefficients, AeroDeck, AeroDynamicDerivativeTableSet,
    AeroDynamicDerivatives, AeroSixCoefficientSigmas, AeroSixCoefficients,
    AeroUncertaintyCorrelation, ExtrapolationPolicy,
};
pub use error::AeroError;
pub use hypersonic::{
    AxialLineLoad, LocalInclinationPanels, ModifiedNewtonian, PanelInclination, PanelMesh,
    PanelMeshBvh, PanelMethod, PanelSkinFriction, PanelSolution, TangentCone, TangentWedge,
    hypersonic_similarity_parameter,
};
pub use knudsen::{
    AccommodationCoeffs, BridgeFunction, ChengBridge, ErfcBridge, FreeMolecularAero, GasRegime,
    HybridAeroMethod, LinearKnudsenBridge, LinearMachBridge, erfc_approx, knudsen_number,
    mean_free_path_m,
};
pub use method::{AeroContext, AeroForceMomentBody, AeroMethod, DeckLookup};
#[cfg(feature = "parser")]
pub use validation::{
    DatcomBuildupValidationDeck, DatcomBuildupValidationReport,
    DatcomBuildupValidationSampleReport, DatcomFinBodyCarryoverValidationDeck,
    DatcomFinBodyCarryoverValidationReport, DatcomProtuberanceDragValidationDeck,
    DatcomProtuberanceDragValidationReport, NacaReport1048CrossflowDragRatioValidationDeck,
    NacaReport1048CrossflowDragRatioValidationReport, NacaReport1048CrossflowValidationDeck,
    NacaReport1048CrossflowValidationReport, NacaReport1048IncrementalForedragValidationDeck,
    NacaReport1048IncrementalForedragValidationReport, SearsHaackWaveDragValidationDeck,
    SearsHaackWaveDragValidationReport,
};
pub use vlm::{
    VlmFinFlow, VlmFinGeometry, VlmFinSolution, VlmFinSurface, VlmGridFinFlow, VlmGridFinGeometry,
    VlmGridFinSolution, VlmGridFinSurface, VlmHorseshoeDerivatives, VlmHorseshoeLattice,
    VlmHorseshoePanel, VlmHorseshoeSolution,
};
#[cfg(feature = "parser")]
pub use vlm::{
    VlmHorseshoeDerivativeComparisonDeck, VlmHorseshoeDerivativeComparisonReport,
    VlmHorseshoeDerivativeComparisonSampleReport,
};
