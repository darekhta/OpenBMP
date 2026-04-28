//! Wind models.
//!
//! Phase 2.4 ships the two toy models the Phase-2 plan locks in:
//!
//! * [`NoWind`] — returns the zero wind vector at every query. The
//!   default for analytic-toy scenarios where atmospheric quiescence
//!   is the assumed condition.
//! * [`ConstantWind`] — returns a caller-supplied constant wind in
//!   the local-NED frame. Useful for steady-crosswind regression
//!   tests and for sounding-rocket validation cases that pin a fixed
//!   surface wind.
//!
//! Phase 3.8 adds:
//!
//! * [`LayeredWind`] — per-altitude NED wind table with linear
//!   interpolation between layers and constant clamping outside the
//!   table envelope.
//! * `GustWind` (Phase 3.8.C) — Dryden rational-spectrum shaping
//!   filter per MIL-STD-1797A.
//!
//! # Frame convention
//!
//! Wind is reported as a [`Velocity3<Ned>`] with `(north, east, down)`
//! components in m/s; the third component is positive downward by the
//! NED convention. The vector is anchored at the active
//! [`FrameContext`]'s `local_origin`. `NoWind` and `ConstantWind`
//! themselves do not require a local origin, but consumers that
//! transform nonzero NED wind into body or ECI must require one
//! through the frame context. The trait surface carries position,
//! frame, and time so future altitude-dependent and location-dependent
//! wind models can use them without a trait-method break.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; no wall-clock, no system RNG, no
//! network, no file I/O for the time-only models. The Phase-3.8.C
//! `GustWind` adds a Dryden filter whose noise comes from
//! [`openbmp_core::DeterministicRng::for_wind_component`] keyed
//! `(scenario_seed, step, axis)`, with the per-step state advanced
//! once per kernel base tick by the runner-side `WindRack`.

pub mod constant;
pub mod gust;
pub mod layered;

pub use constant::{ConstantWind, NoWind};
pub use gust::{GustWind, GustWindParams};
pub use layered::{LayerEntry, LayeredWind};

use openbmp_core::{Eci, FrameContext, Ned, Position3, SimTime, Velocity3};

use crate::error::EnvError;

/// Trait implemented by wind-providing environment models.
///
/// Returns wind velocity in local-NED `(north, east, down)` components,
/// in m/s. The NED `down` component is positive downward. The vector is
/// anchored at the active [`FrameContext`]'s `local_origin`, but this
/// method does not itself require an origin unless a specific model
/// needs one. Consumers transforming nonzero NED wind into body or ECI
/// are responsible for requiring a local origin from the frame context.
/// The position and frame are passed for forward compatibility with
/// future altitude- or location-dependent models; the Phase-2.4
/// implementations ([`NoWind`], [`ConstantWind`]) ignore them.
pub trait WindModel {
    /// Wind velocity in local-NED, in m/s.
    ///
    /// # Errors
    ///
    /// Returns an [`EnvError`] when the model produces a non-finite
    /// output or the position is outside the model's declared
    /// validity envelope. The Phase-2.4 toy models never fail.
    fn wind_ned_m_s(
        &self,
        position_eci: Position3<Eci>,
        frame: &FrameContext,
        time: SimTime,
    ) -> Result<Velocity3<Ned>, EnvError>;
}
