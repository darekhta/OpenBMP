//! Slack-taut recovery riser line element.
//!
//! This is the WP-16.3 substrate: a deterministic tension-only
//! Kelvin-Voigt line between two attachment points. The element is
//! purely algebraic for now; later deployment-sequence and two-body
//! recovery paths can reuse the same sample output when coupling the
//! line into runner force and moment adapters.

use nalgebra::Vector3;

use super::RecoveryError;

/// One endpoint of a recovery riser line.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiserEndpointState {
    /// Attachment-point position in a common frame, m.
    pub position_m: Vector3<f64>,
    /// Attachment-point velocity in the same common frame, m/s.
    pub velocity_m_s: Vector3<f64>,
}

impl RiserEndpointState {
    /// Construct an endpoint state.
    #[must_use]
    pub const fn new(position_m: Vector3<f64>, velocity_m_s: Vector3<f64>) -> Self {
        Self {
            position_m,
            velocity_m_s,
        }
    }

    /// Advance the endpoint under constant velocity for `dt_s`.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `dt_s`, the input
    /// endpoint, or the advanced endpoint is non-finite, or when `dt_s` is
    /// negative.
    pub fn advance_by(self, dt_s: f64) -> Result<Self, RecoveryError> {
        require_non_negative_time(dt_s)?;
        require_finite_vec3(self.position_m)?;
        require_finite_vec3(self.velocity_m_s)?;
        let advanced = advance_endpoint_unchecked(self, dt_s);
        require_finite_vec3(advanced.position_m)?;
        Ok(advanced)
    }
}

/// Tension-only Kelvin-Voigt riser line element.
///
/// `axial_rigidity_n` is the declared effective `EA`; the linear
/// stiffness used by the element is `EA / L0`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiserLineElement {
    unstretched_length_m: f64,
    axial_rigidity_n: f64,
    damping_n_s_m: f64,
}

impl RiserLineElement {
    /// Construct a validated line element.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `L0` or `EA`
    /// is non-finite or non-positive, or when damping is non-finite or
    /// negative.
    pub fn new(
        unstretched_length_m: f64,
        axial_rigidity_n: f64,
        damping_n_s_m: f64,
    ) -> Result<Self, RecoveryError> {
        if !unstretched_length_m.is_finite() || unstretched_length_m <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser unstretched length must be finite and strictly positive",
            });
        }
        if !axial_rigidity_n.is_finite() || axial_rigidity_n <= 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser axial rigidity must be finite and strictly positive",
            });
        }
        if !damping_n_s_m.is_finite() || damping_n_s_m < 0.0 {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser damping must be finite and non-negative",
            });
        }
        Ok(Self {
            unstretched_length_m,
            axial_rigidity_n,
            damping_n_s_m,
        })
    }

    /// Unstretched length `L0`, m.
    #[must_use]
    pub const fn unstretched_length_m(self) -> f64 {
        self.unstretched_length_m
    }

    /// Effective axial rigidity `EA`, N.
    #[must_use]
    pub const fn axial_rigidity_n(self) -> f64 {
        self.axial_rigidity_n
    }

    /// Linear stiffness `EA/L0`, N/m.
    #[must_use]
    pub fn stiffness_n_m(self) -> f64 {
        self.axial_rigidity_n / self.unstretched_length_m
    }

    /// Damping coefficient, N s/m.
    #[must_use]
    pub const fn damping_n_s_m(self) -> f64 {
        self.damping_n_s_m
    }

    /// Evaluate the line at the supplied endpoint states.
    ///
    /// `anchor` and `canopy` positions/velocities must be expressed in
    /// the same frame. The returned force on the anchor points toward
    /// the canopy; the canopy force is equal and opposite. When the
    /// line is slack (`length < L0`) the tension and both forces are
    /// exactly zero. At `length == L0`, positive stretch rate can still
    /// produce damping tension.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when any endpoint
    /// component is non-finite.
    pub fn sample(
        self,
        anchor: RiserEndpointState,
        canopy: RiserEndpointState,
    ) -> Result<RiserLineSample, RecoveryError> {
        require_finite_vec3(anchor.position_m)?;
        require_finite_vec3(anchor.velocity_m_s)?;
        require_finite_vec3(canopy.position_m)?;
        require_finite_vec3(canopy.velocity_m_s)?;

        let separation_m = canopy.position_m - anchor.position_m;
        let length_m = separation_m.norm();
        if !length_m.is_finite() {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser length computation produced an invalid value",
            });
        }
        let stretch_m = length_m - self.unstretched_length_m;
        let stiffness_n_m = self.stiffness_n_m();
        let elastic_potential_energy_j = if stretch_m > 0.0 {
            0.5 * stiffness_n_m * stretch_m * stretch_m
        } else {
            0.0
        };
        if stretch_m < 0.0 || length_m == 0.0 {
            return Ok(RiserLineSample {
                length_m,
                stretch_m,
                stretch_rate_m_s: 0.0,
                tension_n: 0.0,
                unit_anchor_to_canopy: Vector3::zeros(),
                force_on_anchor_n: Vector3::zeros(),
                force_on_canopy_n: Vector3::zeros(),
                elastic_potential_energy_j,
            });
        }

        let unit_anchor_to_canopy = separation_m / length_m;
        let relative_velocity_m_s = canopy.velocity_m_s - anchor.velocity_m_s;
        let stretch_rate_m_s = relative_velocity_m_s.dot(&unit_anchor_to_canopy);
        if !stretch_rate_m_s.is_finite() {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser stretch-rate computation produced an invalid value",
            });
        }
        let raw_tension_n = stiffness_n_m * stretch_m + self.damping_n_s_m * stretch_rate_m_s;
        let tension_n = raw_tension_n.max(0.0);
        if !tension_n.is_finite() {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser tension computation produced an invalid value",
            });
        }
        let force_on_anchor_n = unit_anchor_to_canopy * tension_n;
        Ok(RiserLineSample {
            length_m,
            stretch_m,
            stretch_rate_m_s,
            tension_n,
            unit_anchor_to_canopy,
            force_on_anchor_n,
            force_on_canopy_n: -force_on_anchor_n,
            elastic_potential_energy_j,
        })
    }

    /// Locate the first slack-to-taut boundary crossing inside a fixed tick.
    ///
    /// Endpoint motion is assumed linear over `dt_s` using each endpoint's
    /// supplied velocity. The return value is:
    ///
    /// - `Some(event)` with `event.time_s == 0.0` when the line starts taut
    ///   (`|p_b - p_a| >= L0`).
    /// - `Some(event)` with `0 < event.time_s <= dt_s` when a slack line first
    ///   reaches `L0` during the tick.
    /// - `None` when the line remains slack for the full tick.
    ///
    /// This is a deterministic event locator for caller-owned fixed-step
    /// recovery state machines; it does not mutate line or endpoint state.
    ///
    /// # Errors
    ///
    /// Returns [`RecoveryError::InvalidParameter`] when `dt_s` is negative or
    /// non-finite, when any endpoint component is non-finite, or when the
    /// quadratic crossing calculation overflows to a non-finite value.
    pub fn slack_taut_event(
        self,
        anchor: RiserEndpointState,
        canopy: RiserEndpointState,
        dt_s: f64,
    ) -> Result<Option<RiserSlackTautEvent>, RecoveryError> {
        require_non_negative_time(dt_s)?;
        require_finite_vec3(anchor.position_m)?;
        require_finite_vec3(anchor.velocity_m_s)?;
        require_finite_vec3(canopy.position_m)?;
        require_finite_vec3(canopy.velocity_m_s)?;

        let separation_m = canopy.position_m - anchor.position_m;
        let length_sq_m2 = separation_m.dot(&separation_m);
        let l0_sq_m2 = self.unstretched_length_m * self.unstretched_length_m;
        if !length_sq_m2.is_finite() || !l0_sq_m2.is_finite() {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser slack-taut event calculation produced an invalid length",
            });
        }
        if length_sq_m2 >= l0_sq_m2 {
            return Ok(Some(RiserSlackTautEvent {
                time_s: 0.0,
                anchor,
                canopy,
            }));
        }
        if dt_s == 0.0 {
            return Ok(None);
        }

        let relative_velocity_m_s = canopy.velocity_m_s - anchor.velocity_m_s;
        let a = relative_velocity_m_s.dot(&relative_velocity_m_s);
        if !a.is_finite() {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser slack-taut event calculation produced an invalid velocity",
            });
        }
        if a <= 0.0 {
            return Ok(None);
        }
        let b = 2.0 * separation_m.dot(&relative_velocity_m_s);
        let c = length_sq_m2 - l0_sq_m2;
        let discriminant = b * b - 4.0 * a * c;
        if !(b.is_finite() && c.is_finite() && discriminant.is_finite()) {
            return Err(RecoveryError::InvalidParameter {
                reason: "riser slack-taut event calculation produced an invalid quadratic",
            });
        }
        if discriminant < 0.0 {
            return Ok(None);
        }

        let sqrt_discriminant = discriminant.sqrt();
        let inv_denom = 0.5 / a;
        let roots = [
            (-b - sqrt_discriminant) * inv_denom,
            (-b + sqrt_discriminant) * inv_denom,
        ];
        let Some(time_s) = roots
            .into_iter()
            .filter(|time_s| time_s.is_finite() && *time_s >= 0.0 && *time_s <= dt_s)
            .min_by(|a, b| a.total_cmp(b))
        else {
            return Ok(None);
        };

        let anchor = anchor.advance_by(time_s)?;
        let canopy = canopy.advance_by(time_s)?;
        Ok(Some(RiserSlackTautEvent {
            time_s,
            anchor,
            canopy,
        }))
    }
}

/// Output of one riser-line evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiserLineSample {
    /// Current endpoint distance, m.
    pub length_m: f64,
    /// `length_m - L0`, m.
    pub stretch_m: f64,
    /// Relative endpoint velocity projected along the line, m/s.
    pub stretch_rate_m_s: f64,
    /// Tension, N. Exactly zero while slack or when damping clamps the line.
    pub tension_n: f64,
    /// Unit vector from anchor endpoint to canopy endpoint.
    pub unit_anchor_to_canopy: Vector3<f64>,
    /// Force applied to the anchor endpoint, N.
    pub force_on_anchor_n: Vector3<f64>,
    /// Force applied to the canopy endpoint, N.
    pub force_on_canopy_n: Vector3<f64>,
    /// Elastic potential energy in the stretched line, J.
    pub elastic_potential_energy_j: f64,
}

/// First line-stretch event inside a fixed tick.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiserSlackTautEvent {
    /// Time from the interval start to the event, s.
    pub time_s: f64,
    /// Anchor endpoint state advanced to `time_s`.
    pub anchor: RiserEndpointState,
    /// Canopy/pack endpoint state advanced to `time_s`.
    pub canopy: RiserEndpointState,
}

/// Compute the moment about a body origin from an attachment force.
///
/// Both vectors must be expressed in the same body frame.
#[must_use]
pub fn riser_attach_moment_body_n_m(
    attach_point_body_m: Vector3<f64>,
    force_body_n: Vector3<f64>,
) -> Vector3<f64> {
    attach_point_body_m.cross(&force_body_n)
}

fn require_finite_vec3(value: Vector3<f64>) -> Result<(), RecoveryError> {
    if !value.x.is_finite() || !value.y.is_finite() || !value.z.is_finite() {
        return Err(RecoveryError::InvalidParameter {
            reason: "riser endpoint vectors must be finite",
        });
    }
    Ok(())
}

fn require_non_negative_time(dt_s: f64) -> Result<(), RecoveryError> {
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(RecoveryError::InvalidParameter {
            reason: "riser time increment must be finite and non-negative",
        });
    }
    Ok(())
}

fn advance_endpoint_unchecked(endpoint: RiserEndpointState, dt_s: f64) -> RiserEndpointState {
    RiserEndpointState {
        position_m: endpoint.position_m + endpoint.velocity_m_s * dt_s,
        velocity_m_s: endpoint.velocity_m_s,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn endpoint(position_m: [f64; 3], velocity_m_s: [f64; 3]) -> RiserEndpointState {
        RiserEndpointState::new(Vector3::from(position_m), Vector3::from(velocity_m_s))
    }

    #[test]
    fn rejects_invalid_line_parameters() {
        assert!(RiserLineElement::new(0.0, 1.0, 0.0).is_err());
        assert!(RiserLineElement::new(1.0, 0.0, 0.0).is_err());
        assert!(RiserLineElement::new(1.0, 1.0, -1.0).is_err());
        assert!(RiserLineElement::new(f64::NAN, 1.0, 0.0).is_err());
    }

    #[test]
    fn endpoint_advance_uses_constant_velocity() {
        let advanced = endpoint([1.0, -2.0, 3.0], [0.5, 2.0, -4.0])
            .advance_by(0.25)
            .unwrap();

        assert_eq!(advanced.position_m.x.to_bits(), 1.125_f64.to_bits());
        assert_eq!(advanced.position_m.y.to_bits(), (-1.5_f64).to_bits());
        assert_eq!(advanced.position_m.z.to_bits(), 2.0_f64.to_bits());
        assert_eq!(advanced.velocity_m_s, Vector3::new(0.5, 2.0, -4.0));
        assert!(
            endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                .advance_by(-1.0)
                .is_err()
        );
    }

    #[test]
    fn slack_line_transmits_exactly_zero_force() {
        let line = RiserLineElement::new(2.0, 2000.0, 50.0).unwrap();
        let sample = line
            .sample(
                endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                endpoint([1.5, 0.0, 0.0], [100.0, 0.0, 0.0]),
            )
            .unwrap();

        assert_eq!(sample.tension_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(sample.force_on_anchor_n, Vector3::zeros());
        assert_eq!(sample.force_on_canopy_n, Vector3::zeros());
        assert_eq!(
            sample.elastic_potential_energy_j.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn taut_line_matches_kelvin_voigt_equation() {
        let line = RiserLineElement::new(2.0, 2000.0, 50.0).unwrap();
        let sample = line
            .sample(
                endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                endpoint([2.1, 0.0, 0.0], [0.2, 0.0, 0.0]),
            )
            .unwrap();

        assert!((sample.length_m - 2.1).abs() < 1e-12);
        assert!((sample.stretch_m - 0.1).abs() < 1e-12);
        assert_eq!(sample.stretch_rate_m_s.to_bits(), 0.2_f64.to_bits());
        assert!((sample.tension_n - 110.0).abs() < 1e-12);
        assert!((sample.force_on_anchor_n - Vector3::new(110.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((sample.force_on_canopy_n - Vector3::new(-110.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((sample.elastic_potential_energy_j - 5.0).abs() < 1e-12);
    }

    #[test]
    fn damping_cannot_make_a_compressive_line_force() {
        let line = RiserLineElement::new(2.0, 2000.0, 1000.0).unwrap();
        let sample = line
            .sample(
                endpoint([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]),
                endpoint([2.1, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .unwrap();

        assert_eq!(sample.tension_n.to_bits(), 0.0_f64.to_bits());
        assert_eq!(sample.force_on_anchor_n, Vector3::zeros());
        assert!((sample.elastic_potential_energy_j - 5.0).abs() < 1e-12);
    }

    #[test]
    fn undamped_free_bounce_conserves_energy() {
        let line = RiserLineElement::new(2.0, 1000.0, 0.0).unwrap();
        let mass_kg = 5.0_f64;
        let stiffness_n_m = line.stiffness_n_m();
        let omega_rad_s = (stiffness_n_m / mass_kg).sqrt();
        let initial_taut_velocity_m_s = 3.0_f64;
        let expected_energy_j = 0.5 * mass_kg * initial_taut_velocity_m_s.powi(2);

        for i in 0..=64 {
            let phase_rad = core::f64::consts::PI * (i as f64) / 64.0;
            let stretch_m = (initial_taut_velocity_m_s / omega_rad_s) * phase_rad.sin();
            let stretch_rate_m_s = initial_taut_velocity_m_s * phase_rad.cos();
            let sample = line
                .sample(
                    endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                    endpoint(
                        [line.unstretched_length_m() + stretch_m, 0.0, 0.0],
                        [stretch_rate_m_s, 0.0, 0.0],
                    ),
                )
                .unwrap();

            let kinetic_energy_j = 0.5 * mass_kg * stretch_rate_m_s.powi(2);
            let total_energy_j = kinetic_energy_j + sample.elastic_potential_energy_j;
            let rel_error = ((total_energy_j - expected_energy_j) / expected_energy_j).abs();
            assert!(
                rel_error <= 1.0e-12,
                "phase {phase_rad}: total energy {total_energy_j}, expected {expected_energy_j}, rel {rel_error}"
            );
        }
    }

    #[test]
    fn slack_taut_event_finds_linear_endpoint_crossing() {
        let line = RiserLineElement::new(10.0, 1000.0, 0.0).unwrap();
        let anchor = endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let canopy = endpoint([8.0, 0.0, 0.0], [3.0, 0.0, 0.0]);

        let event = line
            .slack_taut_event(anchor, canopy, 1.0)
            .unwrap()
            .expect("crossing occurs inside the tick");

        assert!((event.time_s - (2.0 / 3.0)).abs() <= 1.0e-12);
        assert_eq!(event.anchor, anchor);
        assert!((event.canopy.position_m.x - 10.0).abs() <= 1.0e-12);
        assert_eq!(event.canopy.velocity_m_s, Vector3::new(3.0, 0.0, 0.0));
        let sample = line.sample(event.anchor, event.canopy).unwrap();
        assert!(sample.stretch_m.abs() <= 1.0e-12);
        assert_eq!(sample.tension_n.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn slack_taut_event_reports_none_when_interval_stays_slack() {
        let line = RiserLineElement::new(10.0, 1000.0, 0.0).unwrap();
        let event = line
            .slack_taut_event(
                endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                endpoint([8.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
                1.0,
            )
            .unwrap();

        assert_eq!(event, None);
    }

    #[test]
    fn slack_taut_event_returns_zero_when_initially_taut() {
        let line = RiserLineElement::new(10.0, 1000.0, 0.0).unwrap();
        let anchor = endpoint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let canopy = endpoint([10.5, 0.0, 0.0], [-100.0, 0.0, 0.0]);

        let event = line
            .slack_taut_event(anchor, canopy, 0.5)
            .unwrap()
            .expect("initially taut interval has a boundary event at t=0");

        assert_eq!(event.time_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(event.anchor, anchor);
        assert_eq!(event.canopy, canopy);
    }

    #[test]
    fn attach_point_moment_matches_cross_product() {
        let moment = riser_attach_moment_body_n_m(
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(110.0, 0.0, 0.0),
        );
        assert_eq!(moment, Vector3::new(0.0, 0.0, -110.0));
    }
}
