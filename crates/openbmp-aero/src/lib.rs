//! `openbmp-aero` — OpenBMP aerodynamics.
//!
//! Phase 2.5 ships:
//!
//! * [`deck`] — Schema-1 axisymmetric reduced sounding-rocket deck
//!   indexed by `(mach, alpha_deg, beta_deg)` returning the three
//!   reduced coefficients `(CN, CD, CM)`. Locked-order trilinear
//!   interpolation per Demmel & Nguyen 2020; FMA disabled.
//! * [`parser`] — TOML deck-file parser for the architecture-locked
//!   Schema-1 schema with `serde(deny_unknown_fields)`.
//! * [`method`] — [`method::AeroMethod`] trait, [`method::AeroContext`]
//!   input, [`method::AeroForceMomentBody`] output, and the
//!   [`method::DeckLookup`] implementation that composes
//!   `(CN, CD, CM)` into body-frame `(force, moment)`.
//! * [`error`] — [`error::AeroError`], the crate's typed error
//!   surface (out-of-envelope, non-finite, invalid parameter,
//!   malformed deck, deck I/O).
//!
//! The full six-coefficient `CX/CY/CZ/Cl/Cm/Cn` deck and
//! control-effector axes are deferred to Phase 3. Hypersonic methods
//! (`ModifiedNewtonian`, `TangentCone`, `LocalInclinationPanels`,
//! `FreeMolecular`) and the `HybridAeroMethod` dispatch ship in
//! Phase 6 — see `docs/hypersonic-extensions.md`.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the trilinear
//! reduction; no FMA, no wall-clock, no system RNG, no network, no
//! file I/O on the hot path. The TOML parser performs file I/O at
//! deck-load time only.
//!
//! # Crate layering
//!
//! `openbmp-aero` is an L2 crate. It depends only on third-party
//! math/serde primitives and **not** on `openbmp-sim` (L1) — the
//! kernel adapts the [`deck::AeroDeck`] / `AeroMethod` surfaces into
//! its `ForceModel` / `MomentModel` chain at a higher layer. See
//! `docs/phase-2-plan.md § Implementation Seams`.

pub mod deck;
pub mod error;
pub mod method;
pub mod parser;

pub use deck::{AeroCoefficients, AeroDeck, ExtrapolationPolicy};
pub use error::AeroError;
pub use method::{AeroContext, AeroForceMomentBody, AeroMethod, DeckLookup};
