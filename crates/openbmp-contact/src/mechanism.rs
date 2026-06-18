//! Scalar mechanism constraint composition.

use crate::{
    BacklashFlank, BacklashGap, ContactError, LatchState, MonotoneLatch, ScalarStop,
    error::require_finite,
};

/// One scalar mechanism constraint element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScalarMechanismElement {
    /// One-sided scalar stop.
    Stop(ScalarStop),
    /// Two-sided backlash gap.
    Backlash(BacklashGap),
    /// Engage-once latch.
    Latch(MonotoneLatch),
}

impl ScalarMechanismElement {
    /// Creates a scalar stop element.
    #[must_use]
    pub const fn stop(stop: ScalarStop) -> Self {
        Self::Stop(stop)
    }

    /// Creates a backlash gap element.
    #[must_use]
    pub const fn backlash(backlash: BacklashGap) -> Self {
        Self::Backlash(backlash)
    }

    /// Creates a monotone latch element.
    #[must_use]
    pub const fn latch(latch: MonotoneLatch) -> Self {
        Self::Latch(latch)
    }

    /// Returns whether this element needs mutable latch state storage.
    #[must_use]
    pub const fn requires_latch_state(self) -> bool {
        matches!(self, Self::Latch(_))
    }
}

/// Aggregated response from evaluating scalar mechanism elements in declared order.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScalarMechanismResponse {
    /// Sum of generalized forces conjugate to the scalar coordinate.
    pub generalized_force: f64,
    /// Elastic energy stored in active stop/backlash springs.
    pub elastic_energy_j: f64,
    /// Instantaneous non-negative damping power from active elements.
    pub damping_power_w: f64,
    /// Number of active one-sided stops.
    pub active_stops: u32,
    /// Number of active backlash flanks.
    pub active_backlash_flanks: u32,
    /// Number of latches engaged after this evaluation.
    pub engaged_latches: u32,
    /// Number of latches that engaged during this evaluation.
    pub just_engaged_latches: u32,
}

impl ScalarMechanismResponse {
    fn add_stop(&mut self, response: crate::ScalarStopResponse) {
        self.generalized_force += response.generalized_force;
        self.elastic_energy_j += response.elastic_energy_j;
        self.damping_power_w += response.damping_power_w;
        if response.active {
            self.active_stops += 1;
        }
    }

    fn add_backlash(&mut self, response: crate::BacklashResponse) {
        self.generalized_force += response.generalized_force;
        self.elastic_energy_j += response.elastic_energy_j;
        self.damping_power_w += response.damping_power_w;
        if response.active && response.flank != BacklashFlank::Free {
            self.active_backlash_flanks += 1;
        }
    }

    fn add_latch(&mut self, transition: crate::LatchTransition) {
        if transition.is_engaged {
            self.engaged_latches += 1;
        }
        if transition.just_engaged {
            self.just_engaged_latches += 1;
        }
    }
}

/// Counts latch elements in a scalar mechanism declaration.
#[must_use]
pub fn scalar_mechanism_latch_count(elements: &[ScalarMechanismElement]) -> usize {
    elements
        .iter()
        .filter(|element| element.requires_latch_state())
        .count()
}

/// Evaluates scalar mechanism elements in declaration order.
///
/// Callers own latch state storage so the core path does not allocate. The
/// `latch_states` slice length must exactly match
/// [`scalar_mechanism_latch_count`] for `elements`.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] when the scalar state is
/// non-finite or latch storage does not match the declaration.
pub fn evaluate_scalar_mechanism(
    elements: &[ScalarMechanismElement],
    latch_states: &mut [LatchState],
    coordinate: f64,
    coordinate_rate: f64,
) -> Result<ScalarMechanismResponse, ContactError> {
    let coordinate = require_finite("coordinate", coordinate)?;
    let coordinate_rate = require_finite("coordinate_rate", coordinate_rate)?;
    let required_latches = scalar_mechanism_latch_count(elements);
    if latch_states.len() != required_latches {
        return Err(ContactError::InvalidParameter {
            field: "latch_states",
            reason: "must match scalar mechanism latch count",
        });
    }

    let mut latch_index = 0_usize;
    let mut aggregate = ScalarMechanismResponse::default();
    for element in elements {
        match *element {
            ScalarMechanismElement::Stop(stop) => {
                aggregate.add_stop(stop.evaluate(coordinate, coordinate_rate)?);
            }
            ScalarMechanismElement::Backlash(backlash) => {
                aggregate.add_backlash(backlash.evaluate(coordinate, coordinate_rate)?);
            }
            ScalarMechanismElement::Latch(latch) => {
                let transition = latch.update(&mut latch_states[latch_index], coordinate)?;
                latch_index += 1;
                aggregate.add_latch(transition);
            }
        }
    }
    Ok(aggregate)
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use crate::{BacklashGap, LatchWindow, StopSide};

    use super::*;

    #[test]
    fn scalar_mechanism_evaluator_sums_stop_and_backlash_forces() -> Result<(), ContactError> {
        let elements = [
            ScalarMechanismElement::stop(ScalarStop::new(0.0, StopSide::Lower, 100.0, 0.0)?),
            ScalarMechanismElement::backlash(BacklashGap::new(0.0, 0.2, 50.0, 0.0)?),
        ];
        let mut latch_states = [];

        let response = evaluate_scalar_mechanism(&elements, &mut latch_states, -0.3, 0.0)?;

        assert_eq!(response.active_stops, 1);
        assert_eq!(response.active_backlash_flanks, 1);
        assert_abs_diff_eq!(response.generalized_force, 40.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.elastic_energy_j, 5.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.damping_power_w, 0.0, epsilon = 1.0e-15);
        Ok(())
    }

    #[test]
    fn scalar_mechanism_evaluator_updates_latches_in_declaration_order() -> Result<(), ContactError>
    {
        let elements = [
            ScalarMechanismElement::latch(MonotoneLatch::new(LatchWindow::new(-1.0, 0.1)?)),
            ScalarMechanismElement::latch(MonotoneLatch::new(LatchWindow::new(1.0, 0.1)?)),
        ];
        let mut latch_states = [LatchState::disengaged(), LatchState::disengaged()];

        let first = evaluate_scalar_mechanism(&elements, &mut latch_states, -1.0, 0.0)?;
        assert_eq!(first.engaged_latches, 1);
        assert_eq!(first.just_engaged_latches, 1);
        assert!(latch_states[0].is_engaged());
        assert!(!latch_states[1].is_engaged());

        let second = evaluate_scalar_mechanism(&elements, &mut latch_states, 1.0, 0.0)?;
        assert_eq!(second.engaged_latches, 2);
        assert_eq!(second.just_engaged_latches, 1);
        assert!(latch_states[0].is_engaged());
        assert!(latch_states[1].is_engaged());
        Ok(())
    }

    #[test]
    fn scalar_mechanism_evaluator_fails_closed_on_latch_storage_mismatch()
    -> Result<(), ContactError> {
        let elements = [ScalarMechanismElement::latch(MonotoneLatch::new(
            LatchWindow::new(0.0, 0.1)?,
        ))];
        let mut latch_states = [];

        let err = match evaluate_scalar_mechanism(&elements, &mut latch_states, 0.0, 0.0) {
            Ok(response) => {
                return Err(ContactError::InvalidParameter {
                    field: "test",
                    reason: if response.engaged_latches == 0 {
                        "unexpected success"
                    } else {
                        "unexpected latch success"
                    },
                });
            }
            Err(err) => err,
        };

        assert_eq!(
            err,
            ContactError::InvalidParameter {
                field: "latch_states",
                reason: "must match scalar mechanism latch count"
            }
        );
        Ok(())
    }
}
