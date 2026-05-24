//! [`LayeredWind`] — per-altitude NED wind table.
//!
//! Linearly interpolates between scenario-declared `(altitude_m,
//! wind_ned_m_s)` layers. Below the first layer's altitude, returns
//! the first layer's wind verbatim; above the last layer's altitude,
//! returns the last layer's wind verbatim. Inside the table envelope,
//! linearly interpolates between the two bracketing layers using a
//! locked operand order (`t = (alt - lo) / (hi - lo); v = lo + t *
//! (hi - lo)`).
//!
//! # Altitude proxy
//!
//! Altitude is read as `position_eci.vector.z`, matching the
//! axial-drag adapter convention (`adapters.rs:466`). For
//! the toy-fixed-earth and WGS84-uniform-rotation frame profiles
//! shipped today, this is a vertical-launch simplification: the
//! +z body axis aligns with the launch-pad up direction. A future
//! "geodetic altitude resolver" would replace this in
//! lockstep for both atmosphere and wind models. Documented in the
//! model's docstring; downstream callers should be aware.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic; no FMA, no transcendentals. Locked operand
//! order on the interpolation. Layer table validated at construction
//! (strictly ascending altitudes, all components finite) so the hot
//! path returns the bracketed value without re-checking.

use openbmp_core::{Eci, Ned, Position3, SimTime, Velocity3};

use crate::error::PhysicsError;
use crate::frames::FrameContext;

use super::WindModel;

/// One row of a [`LayeredWind`] table.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LayerEntry {
    /// Altitude in metres at which this layer's wind applies.
    pub altitude_m: f64,
    /// Wind vector in local-NED `(north, east, down)`, m/s.
    pub wind_ned_m_s: Velocity3<Ned>,
}

impl LayerEntry {
    /// Construct a layer entry from explicit components.
    #[must_use]
    pub fn new(altitude_m: f64, north_m_s: f64, east_m_s: f64, down_m_s: f64) -> Self {
        Self {
            altitude_m,
            wind_ned_m_s: Velocity3::new(north_m_s, east_m_s, down_m_s),
        }
    }
}

/// Per-altitude NED wind table with linear interpolation.
#[derive(Clone, Debug, PartialEq)]
pub struct LayeredWind {
    layers: Vec<LayerEntry>,
}

impl LayeredWind {
    /// Construct from a layer table. Layers must be strictly ascending
    /// in altitude and all components must be finite.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] when the table is empty
    /// or altitudes are not strictly ascending. Returns
    /// [`PhysicsError::NonFinite`] when any altitude or wind component is
    /// non-finite.
    pub fn new(layers: Vec<LayerEntry>) -> Result<Self, PhysicsError> {
        if layers.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "LayeredWind requires at least one layer entry",
            });
        }
        for entry in &layers {
            if !entry.altitude_m.is_finite() {
                return Err(PhysicsError::NonFinite {
                    reason: "LayeredWind layer altitude is NaN or infinite",
                });
            }
            if !entry.wind_ned_m_s.vector.x.is_finite()
                || !entry.wind_ned_m_s.vector.y.is_finite()
                || !entry.wind_ned_m_s.vector.z.is_finite()
            {
                return Err(PhysicsError::NonFinite {
                    reason: "LayeredWind layer wind component is NaN or infinite",
                });
            }
        }
        for window in layers.windows(2) {
            if window[1].altitude_m <= window[0].altitude_m {
                return Err(PhysicsError::InvalidParameter {
                    reason: "LayeredWind layer altitudes must be strictly ascending",
                });
            }
        }
        Ok(Self { layers })
    }

    /// Number of layers in the table. Always ≥ 1 — the constructor
    /// rejects empty tables, so this never returns 0 and there is no
    /// `is_empty()` companion.
    #[must_use]
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.layers.len()
    }

    /// Scenario-declared layer table (in original order).
    #[must_use]
    pub fn layers(&self) -> &[LayerEntry] {
        &self.layers
    }

    /// Lookup wind at a given altitude. Clamps below the
    /// first layer and above the last layer; inside the envelope,
    /// linearly interpolates between the two bracketing layers.
    #[must_use]
    pub fn lookup(&self, altitude_m: f64) -> Velocity3<Ned> {
        // Below the first layer: clamp.
        let first = self.layers[0];
        if altitude_m <= first.altitude_m {
            return first.wind_ned_m_s;
        }
        // Above the last layer: clamp.
        let last = self.layers[self.layers.len() - 1];
        if altitude_m >= last.altitude_m {
            return last.wind_ned_m_s;
        }
        // Inside: locate the bracketing pair (lo, hi).
        for window in self.layers.windows(2) {
            let lo = window[0];
            let hi = window[1];
            if altitude_m >= lo.altitude_m && altitude_m <= hi.altitude_m {
                // Locked operand order: compute fraction first, then
                // (lo + t * (hi - lo)) per axis. No FMA.
                let span = hi.altitude_m - lo.altitude_m;
                let t = (altitude_m - lo.altitude_m) / span;
                let dx = hi.wind_ned_m_s.vector.x - lo.wind_ned_m_s.vector.x;
                let dy = hi.wind_ned_m_s.vector.y - lo.wind_ned_m_s.vector.y;
                let dz = hi.wind_ned_m_s.vector.z - lo.wind_ned_m_s.vector.z;
                let nx = lo.wind_ned_m_s.vector.x + t * dx;
                let ny = lo.wind_ned_m_s.vector.y + t * dy;
                let nz = lo.wind_ned_m_s.vector.z + t * dz;
                return Velocity3::new(nx, ny, nz);
            }
        }
        // Defensive fallback — unreachable given the strictly-ascending
        // invariant and the clamp branches above. Returns the last
        // layer's wind so the function is total.
        last.wind_ned_m_s
    }
}

impl WindModel for LayeredWind {
    fn wind_ned_m_s(
        &self,
        position_eci: Position3<Eci>,
        _frame: &FrameContext,
        _time: SimTime,
    ) -> Result<Velocity3<Ned>, PhysicsError> {
        // Altitude proxy: position_eci.vector.z. See module
        // docstring — matches the axial-drag adapter
        // convention. A future geodetic-altitude resolver replaces
        // this for atmosphere and wind in lockstep.
        let altitude_m = position_eci.vector.z;
        if !altitude_m.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "LayeredWind altitude proxy (position_eci.z) is non-finite",
            });
        }
        Ok(self.lookup(altitude_m))
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn frame() -> FrameContext {
        FrameContext::toy_fixed_earth()
    }

    fn pos(z: f64) -> Position3<Eci> {
        Position3::new(0.0, 0.0, z)
    }

    fn three_layer_table() -> LayeredWind {
        LayeredWind::new(vec![
            LayerEntry::new(0.0, 5.0, 0.0, 0.0),
            LayerEntry::new(3000.0, 12.0, 2.0, 0.0),
            LayerEntry::new(10000.0, 25.0, 8.0, 0.0),
        ])
        .unwrap()
    }

    // -----------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------

    #[test]
    fn rejects_empty_layer_list() {
        let result = LayeredWind::new(vec![]);
        assert!(matches!(result, Err(PhysicsError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_descending_altitudes() {
        let result = LayeredWind::new(vec![
            LayerEntry::new(1000.0, 0.0, 0.0, 0.0),
            LayerEntry::new(500.0, 0.0, 0.0, 0.0),
        ]);
        assert!(matches!(result, Err(PhysicsError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_equal_altitudes() {
        let result = LayeredWind::new(vec![
            LayerEntry::new(1000.0, 0.0, 0.0, 0.0),
            LayerEntry::new(1000.0, 5.0, 0.0, 0.0),
        ]);
        assert!(matches!(result, Err(PhysicsError::InvalidParameter { .. })));
    }

    #[test]
    fn rejects_non_finite_altitude() {
        let result = LayeredWind::new(vec![LayerEntry::new(f64::NAN, 0.0, 0.0, 0.0)]);
        assert!(matches!(result, Err(PhysicsError::NonFinite { .. })));
    }

    #[test]
    fn rejects_non_finite_wind_component() {
        let result = LayeredWind::new(vec![LayerEntry::new(0.0, f64::INFINITY, 0.0, 0.0)]);
        assert!(matches!(result, Err(PhysicsError::NonFinite { .. })));
    }

    #[test]
    fn accepts_single_layer() {
        let lw = LayeredWind::new(vec![LayerEntry::new(0.0, 10.0, 0.0, 0.0)]).unwrap();
        assert_eq!(lw.len(), 1);
        let v = lw.lookup(5000.0);
        assert_eq!(v.vector.x.to_bits(), 10.0_f64.to_bits());
    }

    // -----------------------------------------------------------------
    // Clamp policy
    // -----------------------------------------------------------------

    #[test]
    fn below_bottom_clamps_to_first_layer() {
        let lw = three_layer_table();
        let v = lw.lookup(-1000.0);
        assert_eq!(v.vector.x.to_bits(), 5.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(v.vector.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn above_top_clamps_to_last_layer() {
        let lw = three_layer_table();
        let v = lw.lookup(50_000.0);
        assert_eq!(v.vector.x.to_bits(), 25.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), 8.0_f64.to_bits());
        assert_eq!(v.vector.z.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn at_first_layer_returns_first_layer() {
        let lw = three_layer_table();
        let v = lw.lookup(0.0);
        assert_eq!(v.vector.x.to_bits(), 5.0_f64.to_bits());
    }

    #[test]
    fn at_last_layer_returns_last_layer() {
        let lw = three_layer_table();
        let v = lw.lookup(10_000.0);
        assert_eq!(v.vector.x.to_bits(), 25.0_f64.to_bits());
        assert_eq!(v.vector.y.to_bits(), 8.0_f64.to_bits());
    }

    // -----------------------------------------------------------------
    // Interpolation
    // -----------------------------------------------------------------

    #[test]
    fn midpoint_interpolation_matches_closed_form() {
        // Layer A at z = 0:    (5, 0, 0)
        // Layer B at z = 3000: (12, 2, 0)
        // Midpoint z = 1500:   (8.5, 1.0, 0.0)
        let lw = three_layer_table();
        let v = lw.lookup(1500.0);
        // Locked-order recomputation matching the impl's arithmetic.
        let span: f64 = 3000.0 - 0.0;
        let t: f64 = (1500.0 - 0.0) / span;
        let expected_x: f64 = 5.0 + t * (12.0 - 5.0);
        let expected_y: f64 = 0.0 + t * (2.0 - 0.0);
        let expected_z: f64 = 0.0 + t * (0.0 - 0.0);
        assert_eq!(v.vector.x.to_bits(), expected_x.to_bits());
        assert_eq!(v.vector.y.to_bits(), expected_y.to_bits());
        assert_eq!(v.vector.z.to_bits(), expected_z.to_bits());
    }

    #[test]
    fn quarter_point_interpolation_matches_closed_form() {
        let lw = three_layer_table();
        let v = lw.lookup(750.0);
        let span: f64 = 3000.0;
        let t: f64 = 750.0 / span;
        let expected_x: f64 = 5.0 + t * (12.0 - 5.0);
        assert_eq!(v.vector.x.to_bits(), expected_x.to_bits());
    }

    #[test]
    fn interpolation_uses_correct_bracket_in_multilayer() {
        let lw = three_layer_table();
        // Between layers B and C (3000 and 10000): midpoint 6500.
        let v = lw.lookup(6500.0);
        let span: f64 = 10000.0 - 3000.0;
        let t: f64 = (6500.0 - 3000.0) / span;
        let expected_x: f64 = 12.0 + t * (25.0 - 12.0);
        let expected_y: f64 = 2.0 + t * (8.0 - 2.0);
        assert_eq!(v.vector.x.to_bits(), expected_x.to_bits());
        assert_eq!(v.vector.y.to_bits(), expected_y.to_bits());
    }

    // -----------------------------------------------------------------
    // WindModel trait integration
    // -----------------------------------------------------------------

    #[test]
    fn wind_model_uses_position_z_as_altitude() {
        let lw = three_layer_table();
        let v = lw
            .wind_ned_m_s(pos(1500.0), &frame(), SimTime::ZERO)
            .unwrap();
        let expected = lw.lookup(1500.0);
        assert_eq!(v.vector.x.to_bits(), expected.vector.x.to_bits());
        assert_eq!(v.vector.y.to_bits(), expected.vector.y.to_bits());
        assert_eq!(v.vector.z.to_bits(), expected.vector.z.to_bits());
    }

    #[test]
    fn wind_model_rejects_non_finite_altitude() {
        let lw = three_layer_table();
        let result = lw.wind_ned_m_s(pos(f64::NAN), &frame(), SimTime::ZERO);
        assert!(matches!(result, Err(PhysicsError::NonFinite { .. })));
    }

    // -----------------------------------------------------------------
    // Property test: bracketing
    // -----------------------------------------------------------------

    proptest! {
        #[test]
        fn property_interpolated_value_is_bracketed_componentwise(
            altitude in 0.0_f64..10_000.0_f64,
        ) {
            let lw = three_layer_table();
            let v = lw.lookup(altitude);
            // For our monotonic increasing table (north/east components
            // increasing with altitude), the result must lie within the
            // [first, last] envelope.
            prop_assert!(v.vector.x >= 5.0 - 1e-9);
            prop_assert!(v.vector.x <= 25.0 + 1e-9);
            prop_assert!(v.vector.y >= 0.0 - 1e-9);
            prop_assert!(v.vector.y <= 8.0 + 1e-9);
        }
    }

    // -----------------------------------------------------------------
    // Determinism
    // -----------------------------------------------------------------

    #[test]
    fn lookup_byte_stable_across_two_evaluations() {
        let lw = three_layer_table();
        let altitudes = [0.0, 1500.0, 3000.0, 6500.0, 10_000.0, 50_000.0];
        for altitude in altitudes {
            let a = lw.lookup(altitude);
            let b = lw.lookup(altitude);
            assert_eq!(a.vector.x.to_bits(), b.vector.x.to_bits());
            assert_eq!(a.vector.y.to_bits(), b.vector.y.to_bits());
            assert_eq!(a.vector.z.to_bits(), b.vector.z.to_bits());
        }
    }
}
