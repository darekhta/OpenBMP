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
//! Layered profiles, gust spectra, and altitude-shear winds are
//! deferred to Phase 3 alongside the multi-rate scheduler that makes
//! time-varying gusts cheap to evaluate.
//!
//! # Frame convention
//!
//! Wind is reported as a [`Velocity3<Ned>`] anchored at the active
//! [`FrameContext`]'s `local_origin`. Consumers that need wind in
//! body or ECI must transform through the frame context — `NoWind`
//! and `ConstantWind` themselves never touch the position or frame
//! arguments, but the trait surface carries them so future altitude-
//! dependent and location-dependent wind models can use them without
//! a trait-method break.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; no wall-clock, no system RNG, no
//! network, no file I/O. `ConstantWind::new` validates finiteness at
//! construction so the hot path returns the cached vector without
//! re-checking.

use openbmp_core::{Eci, FrameContext, Ned, Position3, SimTime, Velocity3};

use crate::error::EnvError;

/// Trait implemented by wind-providing environment models.
///
/// Returns the wind velocity in the local-NED frame anchored at the
/// active [`FrameContext`]'s `local_origin`. The position and frame
/// are passed for forward compatibility with future altitude- or
/// location-dependent models; the Phase-2.4 implementations
/// ([`NoWind`], [`ConstantWind`]) ignore them.
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

// ---------------------------------------------------------------------
// NoWind
// ---------------------------------------------------------------------

/// Zero-everywhere wind model. Default for analytic-toy scenarios.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct NoWind;

impl NoWind {
    /// Construct the zero-wind model.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl WindModel for NoWind {
    fn wind_ned_m_s(
        &self,
        _position_eci: Position3<Eci>,
        _frame: &FrameContext,
        _time: SimTime,
    ) -> Result<Velocity3<Ned>, EnvError> {
        Ok(Velocity3::new(0.0, 0.0, 0.0))
    }
}

// ---------------------------------------------------------------------
// ConstantWind
// ---------------------------------------------------------------------

/// Constant-everywhere wind in the local-NED frame.
///
/// The three components are validated for finiteness through
/// [`ConstantWind::new`] so a caller can't construct a `ConstantWind`
/// whose vector contains `NaN` or `Inf`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConstantWind {
    wind_ned_m_s: Velocity3<Ned>,
}

impl ConstantWind {
    /// Construct from explicit `(north, east, down)` components, in m/s.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::NonFinite`] if any component is `NaN` or
    /// `Inf`. Negative components are valid (wind from any direction).
    pub fn new(north_m_s: f64, east_m_s: f64, down_m_s: f64) -> Result<Self, EnvError> {
        for v in [north_m_s, east_m_s, down_m_s] {
            if !v.is_finite() {
                return Err(EnvError::NonFinite {
                    reason: "constant-wind component is NaN or infinite",
                });
            }
        }
        Ok(Self {
            wind_ned_m_s: Velocity3::new(north_m_s, east_m_s, down_m_s),
        })
    }

    /// The configured wind vector (returned at every query).
    #[must_use]
    pub const fn wind_value(&self) -> Velocity3<Ned> {
        self.wind_ned_m_s
    }
}

impl WindModel for ConstantWind {
    fn wind_ned_m_s(
        &self,
        _position_eci: Position3<Eci>,
        _frame: &FrameContext,
        _time: SimTime,
    ) -> Result<Velocity3<Ned>, EnvError> {
        Ok(self.wind_ned_m_s)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn frame() -> FrameContext {
        FrameContext::toy_fixed_earth()
    }

    fn pos(x: f64, y: f64, z: f64) -> Position3<Eci> {
        Position3::new(x, y, z)
    }

    // -----------------------------------------------------------------
    // NoWind
    // -----------------------------------------------------------------

    #[test]
    fn no_wind_returns_zero_everywhere_and_at_every_time() {
        let w = NoWind::new();
        let positions = [
            pos(0.0, 0.0, 0.0),
            pos(7_000_000.0, 0.0, 0.0),
            pos(0.0, 0.0, 6_400_000.0),
            pos(7_000_000.0, 1_500_000.0, 800_000.0),
        ];
        let times = [
            SimTime::ZERO,
            SimTime::from_seconds(60.0),
            SimTime::from_seconds(86_400.0),
        ];
        for p in positions {
            for t in times {
                let v = w.wind_ned_m_s(p, &frame(), t).unwrap();
                assert_eq!(v.vector.x.to_bits(), 0.0_f64.to_bits());
                assert_eq!(v.vector.y.to_bits(), 0.0_f64.to_bits());
                assert_eq!(v.vector.z.to_bits(), 0.0_f64.to_bits());
            }
        }
    }

    // -----------------------------------------------------------------
    // ConstantWind
    // -----------------------------------------------------------------

    #[test]
    fn constant_wind_returns_configured_vector() {
        let w = ConstantWind::new(5.0, -3.0, 0.5).unwrap();
        let v = w
            .wind_ned_m_s(pos(7_000_000.0, 0.0, 0.0), &frame(), SimTime::ZERO)
            .unwrap();
        assert_eq!(v.vector.x.to_bits(), 5.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), (-3.0_f64).to_bits());
        assert_eq!(v.vector.z.to_bits(), 0.5_f64.to_bits());
    }

    #[test]
    fn constant_wind_independent_of_position_and_time() {
        let w = ConstantWind::new(5.0, -3.0, 0.5).unwrap();
        let baseline = w
            .wind_ned_m_s(pos(0.0, 0.0, 0.0), &frame(), SimTime::ZERO)
            .unwrap();
        let positions = [
            pos(0.0, 0.0, 0.0),
            pos(7_000_000.0, 0.0, 0.0),
            pos(0.0, 0.0, 6_400_000.0),
            pos(7_000_000.0, 1_500_000.0, 800_000.0),
        ];
        let times = [
            SimTime::ZERO,
            SimTime::from_seconds(60.0),
            SimTime::from_seconds(86_400.0),
        ];
        for p in positions {
            for t in times {
                let v = w.wind_ned_m_s(p, &frame(), t).unwrap();
                assert_eq!(v.vector.x.to_bits(), baseline.vector.x.to_bits());
                assert_eq!(v.vector.y.to_bits(), baseline.vector.y.to_bits());
                assert_eq!(v.vector.z.to_bits(), baseline.vector.z.to_bits());
            }
        }
    }

    #[test]
    fn constant_wind_value_returns_configured_vector() {
        let w = ConstantWind::new(5.0, -3.0, 0.5).unwrap();
        let v = w.wind_value();
        assert_eq!(v.vector.x.to_bits(), 5.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), (-3.0_f64).to_bits());
        assert_eq!(v.vector.z.to_bits(), 0.5_f64.to_bits());
    }

    #[test]
    fn constant_wind_rejects_non_finite_inputs() {
        assert!(matches!(
            ConstantWind::new(f64::NAN, 0.0, 0.0),
            Err(EnvError::NonFinite { .. })
        ));
        assert!(matches!(
            ConstantWind::new(0.0, f64::INFINITY, 0.0),
            Err(EnvError::NonFinite { .. })
        ));
        assert!(matches!(
            ConstantWind::new(0.0, 0.0, f64::NEG_INFINITY),
            Err(EnvError::NonFinite { .. })
        ));
    }

    #[test]
    fn constant_wind_accepts_negative_components() {
        let w = ConstantWind::new(-10.0, -20.0, -1.5).unwrap();
        let v = w.wind_value();
        assert_eq!(v.vector.x.to_bits(), (-10.0_f64).to_bits());
        assert_eq!(v.vector.y.to_bits(), (-20.0_f64).to_bits());
        assert_eq!(v.vector.z.to_bits(), (-1.5_f64).to_bits());
    }

    #[test]
    fn constant_wind_accepts_zero_vector() {
        let w = ConstantWind::new(0.0, 0.0, 0.0).unwrap();
        let v = w
            .wind_ned_m_s(pos(0.0, 0.0, 0.0), &frame(), SimTime::ZERO)
            .unwrap();
        assert_eq!(v.vector.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(v.vector.z.to_bits(), 0.0_f64.to_bits());
    }
}
