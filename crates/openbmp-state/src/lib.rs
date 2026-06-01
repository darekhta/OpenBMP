//! `openbmp-state` — state types for OpenBMP scenarios.
//!
//! Pure data types for [`PointMassState`], [`RigidBodyState`], and
//! [`MassProperties`], plus literal finiteness checks, structural
//! validators (`is_valid`, `is_normalised`), and Result-returning
//! `require_valid` helpers that compose with [`openbmp_core`]'s error
//! types via `?`.
//!
//! # Frames
//!
//! - Position and velocity live in `Eci` (inertial propagation frame).
//! - Orientation is encoded as `Quaternion<Body, Eci>` — the rotation
//!   that maps a vector expressed in `Body` components to its
//!   expression in `Eci` components.
//! - Angular velocity is in `Body` (the rotating frame's own basis).
//! - Mass-property center of mass and inertia tensor are in `Body`.
//!
//! # Determinism
//!
//! State types are pure data with no IO and no system-clock or RNG
//! access. They inherit the determinism contract of `openbmp-core`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

pub mod error;
pub mod mass_properties;
pub mod point_mass;
pub mod rigid_body;

pub use error::StateError;
pub use mass_properties::MassProperties;
pub use point_mass::PointMassState;
pub use rigid_body::RigidBodyState;
