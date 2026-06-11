//! Monotone scalar latch primitive.

use crate::{
    ContactError,
    error::{require_finite, require_non_negative},
};

/// Capture window for a scalar monotone latch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatchWindow {
    center: f64,
    half_width: f64,
}

impl LatchWindow {
    /// Creates a latch capture window.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when the center is not
    /// finite or the half-width is negative/non-finite.
    pub fn new(center: f64, half_width: f64) -> Result<Self, ContactError> {
        Ok(Self {
            center: require_finite("center", center)?,
            half_width: require_non_negative("half_width", half_width)?,
        })
    }

    /// Returns whether `coordinate` is inside the inclusive capture window.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when `coordinate` is not
    /// finite.
    pub fn contains(self, coordinate: f64) -> Result<bool, ContactError> {
        let coordinate = require_finite("coordinate", coordinate)?;
        Ok((coordinate - self.center).abs() <= self.half_width)
    }

    /// Returns the capture-window center.
    #[must_use]
    pub const fn center(self) -> f64 {
        self.center
    }

    /// Returns the capture-window half-width.
    #[must_use]
    pub const fn half_width(self) -> f64 {
        self.half_width
    }
}

/// Runtime state for a monotone latch.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LatchState {
    engaged: bool,
}

impl LatchState {
    /// Returns a disengaged latch state.
    #[must_use]
    pub const fn disengaged() -> Self {
        Self { engaged: false }
    }

    /// Returns an already engaged latch state.
    #[must_use]
    pub const fn engaged() -> Self {
        Self { engaged: true }
    }

    /// Returns whether the latch is engaged.
    #[must_use]
    pub const fn is_engaged(self) -> bool {
        self.engaged
    }
}

/// Engage-once latch on a scalar coordinate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MonotoneLatch {
    window: LatchWindow,
}

impl MonotoneLatch {
    /// Creates a monotone latch around `window`.
    #[must_use]
    pub const fn new(window: LatchWindow) -> Self {
        Self { window }
    }

    /// Updates latch state at a step boundary.
    ///
    /// The latch engages when the coordinate enters the capture window and
    /// never releases afterward.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when `coordinate` is not
    /// finite.
    pub fn update(
        self,
        state: &mut LatchState,
        coordinate: f64,
    ) -> Result<LatchTransition, ContactError> {
        let was_engaged = state.engaged;
        if !state.engaged && self.window.contains(coordinate)? {
            state.engaged = true;
        }
        Ok(LatchTransition {
            was_engaged,
            is_engaged: state.engaged,
            just_engaged: !was_engaged && state.engaged,
        })
    }

    /// Returns the capture window.
    #[must_use]
    pub const fn window(self) -> LatchWindow {
        self.window
    }
}

/// Result of one monotone latch update.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LatchTransition {
    /// State before the update.
    pub was_engaged: bool,
    /// State after the update.
    pub is_engaged: bool,
    /// `true` only for the step that first captures the latch.
    pub just_engaged: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latch_capture_window_is_inclusive() {
        let window = LatchWindow::new(2.0, 0.25).unwrap();
        assert!(window.contains(1.75).unwrap());
        assert!(window.contains(2.25).unwrap());
        assert!(!window.contains(1.749_999).unwrap());
        assert!(!window.contains(2.250_001).unwrap());
    }

    #[test]
    fn latch_monotone_property_engages_once_and_never_releases() {
        let latch = MonotoneLatch::new(LatchWindow::new(0.0, 0.1).unwrap());
        let mut state = LatchState::disengaged();
        let mut engagement_events = 0_u32;

        for coordinate in [-1.0, -0.2, -0.05, 0.2, 1.0, 0.0, -0.05, 2.0] {
            let transition = latch.update(&mut state, coordinate).unwrap();
            if transition.just_engaged {
                engagement_events += 1;
            }
            if engagement_events > 0 {
                assert!(transition.is_engaged);
            }
        }

        assert_eq!(engagement_events, 1);
        assert!(state.is_engaged());
    }
}
