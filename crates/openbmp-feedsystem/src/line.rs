//! Fixed-grid Method-of-Characteristics line transient primitive.

use openbmp_core::ValidationStatus;

use crate::error::FeedSystemError;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Frictionless fixed-grid MOC line configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineConfig {
    /// Pipe length in m.
    pub length_m: f64,
    /// Acoustic wave speed in m/s.
    pub wave_speed_m_s: f64,
    /// Fluid density in kg/m^3.
    pub density_kg_m3: f64,
    /// Pipe cross-sectional flow area in m^2.
    pub cross_section_area_m2: f64,
    /// Number of equal pipe segments. The state has `segment_count + 1` nodes.
    pub segment_count: usize,
}

impl MocLineConfig {
    /// Validate line configuration.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for non-finite, non-positive, or unsupported
    /// line parameters.
    pub fn require_valid(&self) -> Result<(), FeedSystemError> {
        require_positive_finite(self.length_m, "MOC line length must be positive")?;
        require_positive_finite(self.wave_speed_m_s, "MOC line wave speed must be positive")?;
        require_positive_finite(self.density_kg_m3, "MOC line density must be positive")?;
        require_positive_finite(
            self.cross_section_area_m2,
            "MOC line cross-section area must be positive",
        )?;
        if self.segment_count == 0 {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line segment count must be positive",
            });
        }
        Ok(())
    }

    /// Number of pressure/velocity nodes.
    #[must_use]
    pub const fn node_count(self) -> usize {
        self.segment_count + 1
    }

    /// Courant-exact time step `dx/a` for this fixed grid.
    #[must_use]
    pub fn courant_dt_s(self) -> f64 {
        self.length_m / self.segment_count as f64 / self.wave_speed_m_s
    }

    /// Convert piezometric head in m to gauge pressure in Pa for this line.
    #[must_use]
    pub fn pressure_from_head_pa(self, head_m: f64) -> f64 {
        self.density_kg_m3 * STANDARD_GRAVITY_M_S2 * head_m
    }

    fn wave_over_gravity(self) -> f64 {
        self.wave_speed_m_s / STANDARD_GRAVITY_M_S2
    }
}

/// MOC line state: piezometric head and axial velocity at each node.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineState {
    /// Piezometric head in m at each node.
    pub head_m: Vec<f64>,
    /// Axial velocity in m/s at each node.
    pub velocity_m_s: Vec<f64>,
}

impl MocLineState {
    /// Build a uniform line state.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid config or non-finite state values.
    pub fn uniform(
        config: MocLineConfig,
        head_m: f64,
        velocity_m_s: f64,
    ) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        require_finite(head_m, "MOC line uniform head must be finite")?;
        require_finite(velocity_m_s, "MOC line uniform velocity must be finite")?;
        Ok(Self {
            head_m: vec![head_m; config.node_count()],
            velocity_m_s: vec![velocity_m_s; config.node_count()],
        })
    }

    fn require_valid(&self, config: MocLineConfig) -> Result<(), FeedSystemError> {
        let node_count = config.node_count();
        if self.head_m.len() != node_count || self.velocity_m_s.len() != node_count {
            return Err(FeedSystemError::InvalidParameter {
                reason: "MOC line state length must match segment_count + 1",
            });
        }
        for value in &self.head_m {
            require_finite(*value, "MOC line state head must be finite")?;
        }
        for value in &self.velocity_m_s {
            require_finite(*value, "MOC line state velocity must be finite")?;
        }
        Ok(())
    }
}

/// Boundary conditions for one MOC line step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLineBoundary {
    /// Fixed upstream reservoir/source head in m.
    pub upstream_head_m: f64,
    /// Downstream valve velocity in m/s.
    pub downstream_velocity_m_s: f64,
}

impl MocLineBoundary {
    fn require_valid(self) -> Result<(), FeedSystemError> {
        require_finite(
            self.upstream_head_m,
            "MOC line upstream boundary head must be finite",
        )?;
        require_finite(
            self.downstream_velocity_m_s,
            "MOC line downstream boundary velocity must be finite",
        )
    }
}

/// Snapshot from one MOC line step.
#[derive(Clone, Debug, PartialEq)]
pub struct MocLineSnapshot {
    /// Updated line state.
    pub state: MocLineState,
    /// Downstream valve-node head in m.
    pub downstream_head_m: f64,
    /// Downstream valve-node gauge pressure in Pa.
    pub downstream_pressure_pa: f64,
    /// Minimum head across the updated line in m.
    pub min_head_m: f64,
    /// Maximum head across the updated line in m.
    pub max_head_m: f64,
    /// Courant-exact step size in seconds for this update.
    pub dt_s: f64,
    /// Validation posture of the line model.
    pub validation: ValidationStatus,
}

/// Frictionless MOC line with fixed Courant number 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MocLine {
    config: MocLineConfig,
}

impl MocLine {
    /// Construct a validated MOC line.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] when the line configuration is invalid.
    pub fn new(config: MocLineConfig) -> Result<Self, FeedSystemError> {
        config.require_valid()?;
        Ok(Self { config })
    }

    /// Borrow the validated line configuration.
    #[must_use]
    pub const fn config(&self) -> MocLineConfig {
        self.config
    }

    /// Courant-exact time step `dx/a`.
    #[must_use]
    pub fn courant_dt_s(&self) -> f64 {
        self.config.courant_dt_s()
    }

    /// Advance the line by one Courant-exact MOC step.
    ///
    /// # Errors
    ///
    /// Returns [`FeedSystemError`] for invalid state/boundary values or
    /// non-finite characteristic updates.
    pub fn step(
        &self,
        state: &MocLineState,
        boundary: MocLineBoundary,
    ) -> Result<MocLineSnapshot, FeedSystemError> {
        state.require_valid(self.config)?;
        boundary.require_valid()?;

        let node_count = self.config.node_count();
        let mut next_head_m = vec![0.0; node_count];
        let mut next_velocity_m_s = vec![0.0; node_count];
        let wave_over_gravity = self.config.wave_over_gravity();
        let gravity_over_wave = STANDARD_GRAVITY_M_S2 / self.config.wave_speed_m_s;

        let upstream_compat_minus = state.head_m[1] - wave_over_gravity * state.velocity_m_s[1];
        next_head_m[0] = boundary.upstream_head_m;
        next_velocity_m_s[0] =
            (boundary.upstream_head_m - upstream_compat_minus) * gravity_over_wave;

        for index in 1..(node_count - 1) {
            let compat_plus =
                state.head_m[index - 1] + wave_over_gravity * state.velocity_m_s[index - 1];
            let compat_minus =
                state.head_m[index + 1] - wave_over_gravity * state.velocity_m_s[index + 1];
            next_head_m[index] = 0.5 * (compat_plus + compat_minus);
            next_velocity_m_s[index] = 0.5 * (compat_plus - compat_minus) * gravity_over_wave;
        }

        let downstream_index = node_count - 1;
        let downstream_compat_plus = state.head_m[downstream_index - 1]
            + wave_over_gravity * state.velocity_m_s[downstream_index - 1];
        next_velocity_m_s[downstream_index] = boundary.downstream_velocity_m_s;
        next_head_m[downstream_index] =
            downstream_compat_plus - wave_over_gravity * boundary.downstream_velocity_m_s;

        for value in next_head_m.iter().chain(next_velocity_m_s.iter()) {
            if !value.is_finite() {
                return Err(FeedSystemError::NonFinite {
                    reason: "MOC line update produced a non-finite value",
                });
            }
        }

        let downstream_head_m = next_head_m[downstream_index];
        let downstream_pressure_pa = self.config.pressure_from_head_pa(downstream_head_m);
        if !downstream_pressure_pa.is_finite() {
            return Err(FeedSystemError::NonFinite {
                reason: "MOC line downstream pressure is non-finite",
            });
        }

        let min_head_m = next_head_m
            .iter()
            .fold(f64::INFINITY, |acc, value| acc.min(*value));
        let max_head_m = next_head_m
            .iter()
            .fold(f64::NEG_INFINITY, |acc, value| acc.max(*value));

        Ok(MocLineSnapshot {
            state: MocLineState {
                head_m: next_head_m,
                velocity_m_s: next_velocity_m_s,
            },
            downstream_head_m,
            downstream_pressure_pa,
            min_head_m,
            max_head_m,
            dt_s: self.courant_dt_s(),
            validation: ValidationStatus::ValidatedToy,
        })
    }
}

fn require_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    if !value.is_finite() {
        return Err(FeedSystemError::NonFinite { reason });
    }
    Ok(())
}

fn require_positive_finite(value: f64, reason: &'static str) -> Result<(), FeedSystemError> {
    require_finite(value, reason)?;
    if value <= 0.0 {
        return Err(FeedSystemError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn config() -> MocLineConfig {
        MocLineConfig {
            length_m: 40.0,
            wave_speed_m_s: 1_000.0,
            density_kg_m3: 1_000.0,
            cross_section_area_m2: 0.01,
            segment_count: 4,
        }
    }

    #[test]
    fn moc_line_courant_dt_is_dx_over_wave_speed() {
        let line = MocLine::new(config()).unwrap();

        assert_abs_diff_eq!(line.courant_dt_s(), 0.01, epsilon = 1.0e-15);
    }

    #[test]
    fn moc_line_valve_closure_matches_joukowsky_bound() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState::uniform(config(), 100.0, 2.0).unwrap();
        let snapshot = line
            .step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 0.0,
                },
            )
            .unwrap();

        let expected_surge_head_m = config().wave_speed_m_s * 2.0 / STANDARD_GRAVITY_M_S2;
        assert_abs_diff_eq!(
            snapshot.downstream_head_m,
            100.0 + expected_surge_head_m,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            snapshot.downstream_pressure_pa - 1_000.0 * STANDARD_GRAVITY_M_S2 * 100.0,
            1_000.0 * config().wave_speed_m_s * 2.0,
            epsilon = 1.0e-6
        );
        assert_eq!(snapshot.validation, ValidationStatus::ValidatedToy);
    }

    #[test]
    fn moc_line_interior_node_uses_neighbor_characteristics() {
        let line = MocLine::new(config()).unwrap();
        let mut state = MocLineState::uniform(config(), 100.0, 1.0).unwrap();
        state.head_m[0] = 110.0;
        state.head_m[2] = 90.0;

        let snapshot = line
            .step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 1.0,
                },
            )
            .unwrap();

        let wave_over_gravity = config().wave_speed_m_s / STANDARD_GRAVITY_M_S2;
        let compat_plus = 110.0 + wave_over_gravity;
        let compat_minus = 90.0 - wave_over_gravity;
        assert_abs_diff_eq!(
            snapshot.state.head_m[1],
            0.5 * (compat_plus + compat_minus),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn moc_line_rejects_invalid_state_shape() {
        let line = MocLine::new(config()).unwrap();
        let state = MocLineState {
            head_m: vec![100.0],
            velocity_m_s: vec![1.0],
        };

        assert!(matches!(
            line.step(
                &state,
                MocLineBoundary {
                    upstream_head_m: 100.0,
                    downstream_velocity_m_s: 0.0,
                },
            ),
            Err(FeedSystemError::InvalidParameter { .. })
        ));
    }
}
