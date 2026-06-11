//! `openbmp-thermochem` — thermochemistry deck ingestion and lookup.
//!
//! This crate is the L2 propulsion/aerothermal thermochemistry boundary. It
//! consumes offline CEA/Cantera-style tables as deterministic OpenBMP decks:
//! `c*`, chamber temperature, specific-heat ratio, and molecular weight are
//! stored on a `(chamber pressure, mixture ratio)` grid and interpolated in
//! locked order over `log(pc)` and `MR`.
//!
//! # Crate Layering
//!
//! `openbmp-thermochem` depends only on `openbmp-core` plus parser/error
//! primitives. It has no dependency on `openbmp-sim`, `openbmp-runner`, or
//! `openbmp-fc`; higher layers decide whether and how a deck is connected to
//! a motor, engine, or aerothermal model.

pub mod deck;
pub mod error;
#[cfg(feature = "parser")]
pub mod parser;

pub use deck::{
    ThermochemDeck, ThermochemQuery, ThermochemState, ThermochemTable,
    UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K, characteristic_velocity_m_s, choked_mass_flux_gamma,
};
pub use error::ThermochemError;
