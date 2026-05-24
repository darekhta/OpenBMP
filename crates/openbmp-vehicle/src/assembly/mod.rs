//! Declarative vehicle composition.
//!
//! `VehicleAssembly` is the L1-side trait the scenario layer
//! resolves into. [`Assembly`] is a flat-tree
//! struct holding `Vec<Body>` plus empty placeholders for the
//! effector / engine / tank / sensor sub-trees.
//!
//! # Resolution
//!
//! The runner-side resolver takes a parsed scenario document and
//! produces a [`Assembly`]. Runners consume the
//! assembly's dry mass properties during kernel mass construction while
//! force / moment plumbing remains on the existing per-runner paths.
//! [`KernelModelBundle`] and [`KernelModelBundleRigid`] are forward
//! scaffolding for the later resolver that will flatten propulsion,
//! effectors, tanks, and sensors into kernel model lists.
//!
//! # Determinism
//!
//! - Body ids are FNV-1a-64 of the canonical scenario body path.
//!   Reordering `[[vehicle.assembly.bodies]]` does not shift any id.
//! - Multi-body mass-property summation is in scenario-declared
//!   order, matching the existing `[forces].models` convention.
//!   Reordering `[[vehicle.assembly.bodies]]` *does* change byte
//!   output — same contract as `[forces].models`.

pub mod basic;
pub mod body;
pub mod bundle;

pub use basic::{Assembly, AssemblyBuilder};
pub use body::{Body, BodyGeometry};
pub use bundle::{KernelModelBundle, KernelModelBundleRigid};

use std::borrow::Cow;

use openbmp_core::{BodyId, SimTime, VehicleId};
use openbmp_state::MassProperties;
use thiserror::Error;

use crate::error::VehicleError;

/// Mission-side declarative composition surface.
///
/// `Assembly` is the concrete impl OpenBMP ships;
/// downstream-user vehicle composition shapes (e.g. an
/// `ArvAssembly` for a particular reference vehicle) may implement
/// the trait directly and route through the same resolver.
///
/// The trait surface is intentionally small:
/// `id`, `bodies`, `mass_properties`. Extensions can add
/// `effectors`, `engines`, `tanks`, and `sensors`
/// surfaces. Today the resolver gates concrete sub-tree access through
/// the placeholder types in this module so trait implementations
/// stay forward-compatible.
pub trait VehicleAssembly {
    /// Stable assembly identifier (FNV-1a-64 of the assembly path
    /// or scenario `meta.name`).
    fn id(&self) -> VehicleId;

    /// Bodies in scenario-declared order.
    fn bodies(&self) -> &[Body];

    /// Aggregate mass properties at simulation time `t`. For the
    /// single-body case this is the body's dry mass-
    /// properties; the multi-body case sums per-body contributions
    /// in scenario-declared order using a parallel-axis transport
    /// for the inertia tensor.
    ///
    /// # Errors
    ///
    /// Returns an error when the underlying mass-property fold
    /// produces non-finite or non-positive components (typically a
    /// programmer error caught earlier by [`Body::new`]).
    fn mass_properties(&self, t: SimTime) -> Result<MassProperties, VehicleError>;
}

// Note: effectors live on the runner-side `EffectorRack`
// (see `crates/openbmp-runner/src/effectors.rs`), not on the
// assembly. Keeping `Box<dyn ControlEffector>` off the trait surface
// keeps `Assembly: Clone` and avoids interior-mutability
// complications. Other vehicle subsystems have their own runner-side
// racks and adapters.

/// Error type for assembly construction and resolution.
///
/// Surfaces through [`crate::VehicleError`] / `CliError` boundaries
/// without losing type information.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum AssemblyError {
    /// A required identifier is empty.
    #[error("{field} must not be empty")]
    EmptyId {
        /// Field path (e.g. `"vehicle.assembly.bodies[0].id"`).
        field: Cow<'static, str>,
    },
    /// A numeric input is non-finite or out of range.
    #[error("{field}={value} violates rule: {rule}")]
    InvalidNumber {
        /// Field path.
        field: &'static str,
        /// Invalid value.
        value: f64,
        /// Human-readable rule.
        rule: &'static str,
    },
    /// A body geometry component is invalid.
    #[error("body geometry invalid: {reason}")]
    InvalidBodyGeometry {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// An inertia tensor component is invalid.
    #[error("inertia tensor invalid: {reason}")]
    InvalidInertia {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Two bodies share the same [`BodyId`].
    #[error("duplicate body id {id:?} in assembly")]
    DuplicateBody {
        /// Duplicated id.
        id: BodyId,
    },
    /// Assembly has no bodies — every assembly must declare at least
    /// one rigid member.
    #[error("assembly must contain at least one body")]
    EmptyBodies,
}
