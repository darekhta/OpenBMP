//! Solid-motor grain regression and zero-dimensional internal ballistics.
//!
//! This module is a forward curve producer: a grain geometry and
//! synthetic/textbook propellant constants are regressed on a fixed web
//! grid to produce the [`crate::motor::ThrustCurve`] and
//! [`crate::motor::BurnSpec`] consumed by [`crate::motor::SolidMotor`].
//! It does not alter the hot motor trait surface.

use crate::error::MotorError;
use crate::motor::{
    AmbientPressureCorrection, BurnSpec, MotorGeometry, MotorMeta, SolidMotor, ThrustCurve,
    Validation,
};

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const MIN_POSITIVE: f64 = 1.0e-15;
const NOZZLE_MACH_BISECTION_ITERS: usize = 96;

/// Burning area as a function of regressed web distance.
pub trait GrainGeometry {
    /// Burning area at web distance `web_m`. Returns zero at or past
    /// burnout.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when `web_m` is non-finite or the
    /// geometry cannot produce a finite non-negative area.
    fn burn_area_m2(&self, web_m: f64) -> Result<f64, MotorError>;

    /// Total web distance available (m). Burnout occurs here.
    fn web_total_m(&self) -> f64;

    /// Initial propellant volume (m³).
    fn propellant_volume_m3(&self) -> f64;
}

/// Quasi-steady internal-ballistics solver: grain + propellant →
/// solid motor.
pub trait GrainRegressionModel {
    /// Solve the equilibrium burn and emit a validated [`SolidMotor`].
    ///
    /// # Errors
    ///
    /// Fails closed on invalid propellant/nozzle constants, unstable
    /// pressure exponent, non-finite intermediates, or malformed
    /// produced thrust curve.
    fn regress(&self, geom: &dyn GrainGeometry) -> Result<SolidMotor, MotorError>;
}

/// Propellant constants for Saint-Robert burn-rate regression.
#[derive(Clone, Debug, PartialEq)]
pub struct GrainPropellant {
    /// Display label and provenance tag.
    pub label: String,
    /// Bulk propellant density (kg/m³).
    pub density_kg_m3: f64,
    /// Saint-Robert coefficient in SI units for `r = a * Pc^n`.
    pub burn_rate_a: f64,
    /// Saint-Robert pressure exponent. Must lie in `(0, 1)`.
    pub burn_rate_n: f64,
    /// Characteristic velocity (m/s).
    pub c_star_m_s: f64,
    /// Specific heat ratio.
    pub gamma: f64,
}

impl GrainPropellant {
    /// Validate propellant constants.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when any constant is non-finite or out
    /// of the stable quasi-steady envelope.
    pub fn require_valid(&self) -> Result<(), MotorError> {
        if self.label.is_empty() {
            return Err(MotorError::InvalidParameter {
                reason: "grain propellant label must not be empty",
            });
        }
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain propellant density must be finite and positive",
            });
        }
        if !self.burn_rate_a.is_finite() || self.burn_rate_a <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain burn_rate_a must be finite and positive",
            });
        }
        if !self.burn_rate_n.is_finite() || !(0.0..1.0).contains(&self.burn_rate_n) {
            return Err(MotorError::InvalidParameter {
                reason: "grain burn_rate_n must be finite and lie in (0, 1)",
            });
        }
        if !self.c_star_m_s.is_finite() || self.c_star_m_s <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain c_star_m_s must be finite and positive",
            });
        }
        if !self.gamma.is_finite() || self.gamma <= 1.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain gamma must be finite and greater than 1",
            });
        }
        Ok(())
    }
}

/// End-burning cylindrical grain. The burning area is constant and
/// equal to the cross-section.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndBurnerGrain {
    /// Burning cross-section area (m²).
    pub cross_section_area_m2: f64,
    /// Grain length / total web (m).
    pub length_m: f64,
}

impl EndBurnerGrain {
    /// Construct a validated end-burner grain.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] for non-positive or non-finite
    /// dimensions.
    pub fn new(cross_section_area_m2: f64, length_m: f64) -> Result<Self, MotorError> {
        if !cross_section_area_m2.is_finite() || cross_section_area_m2 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "end-burner cross_section_area_m2 must be finite and positive",
            });
        }
        if !length_m.is_finite() || length_m <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "end-burner length_m must be finite and positive",
            });
        }
        Ok(Self {
            cross_section_area_m2,
            length_m,
        })
    }
}

impl GrainGeometry for EndBurnerGrain {
    fn burn_area_m2(&self, web_m: f64) -> Result<f64, MotorError> {
        if !web_m.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "grain web distance is NaN or infinite",
            });
        }
        if web_m >= self.length_m {
            Ok(0.0)
        } else {
            Ok(self.cross_section_area_m2)
        }
    }

    fn web_total_m(&self) -> f64 {
        self.length_m
    }

    fn propellant_volume_m3(&self) -> f64 {
        self.cross_section_area_m2 * self.length_m
    }
}

/// BATES/tubular grain with inhibited outer wall and uninhibited
/// segment ends.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatesGrain {
    /// Number of identical segments.
    pub segments: u32,
    /// Outer grain radius (m).
    pub outer_radius_m: f64,
    /// Initial core radius (m).
    pub core_radius_m: f64,
    /// Segment length (m).
    pub segment_length_m: f64,
}

impl BatesGrain {
    /// Construct a validated BATES grain.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] for invalid segment count or geometry.
    pub fn new(
        segments: u32,
        outer_radius_m: f64,
        core_radius_m: f64,
        segment_length_m: f64,
    ) -> Result<Self, MotorError> {
        if segments == 0 {
            return Err(MotorError::InvalidParameter {
                reason: "BATES grain segments must be positive",
            });
        }
        if !outer_radius_m.is_finite() || outer_radius_m <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "BATES outer_radius_m must be finite and positive",
            });
        }
        if !core_radius_m.is_finite() || core_radius_m <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "BATES core_radius_m must be finite and positive",
            });
        }
        if core_radius_m >= outer_radius_m {
            return Err(MotorError::InvalidParameter {
                reason: "BATES core_radius_m must be smaller than outer_radius_m",
            });
        }
        if !segment_length_m.is_finite() || segment_length_m <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "BATES segment_length_m must be finite and positive",
            });
        }
        Ok(Self {
            segments,
            outer_radius_m,
            core_radius_m,
            segment_length_m,
        })
    }
}

impl GrainGeometry for BatesGrain {
    fn burn_area_m2(&self, web_m: f64) -> Result<f64, MotorError> {
        if !web_m.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "grain web distance is NaN or infinite",
            });
        }
        if web_m >= self.web_total_m() {
            return Ok(0.0);
        }
        let core_radius = self.core_radius_m + web_m;
        let live_length = self.segment_length_m - 2.0 * web_m;
        if live_length <= 0.0 || core_radius >= self.outer_radius_m {
            return Ok(0.0);
        }
        let pi = std::f64::consts::PI;
        let inner = 2.0 * pi * core_radius * live_length;
        let ends =
            2.0 * pi * (self.outer_radius_m * self.outer_radius_m - core_radius * core_radius);
        let area = f64::from(self.segments) * (inner + ends);
        if !area.is_finite() || area < 0.0 {
            return Err(MotorError::NonFinite {
                reason: "BATES burn area is non-finite",
            });
        }
        Ok(area)
    }

    fn web_total_m(&self) -> f64 {
        (self.outer_radius_m - self.core_radius_m).min(0.5 * self.segment_length_m)
    }

    fn propellant_volume_m3(&self) -> f64 {
        let pi = std::f64::consts::PI;
        let annulus = pi
            * (self.outer_radius_m * self.outer_radius_m - self.core_radius_m * self.core_radius_m);
        f64::from(self.segments) * annulus * self.segment_length_m
    }
}

/// User-supplied burning-area table as a function of web distance.
#[derive(Clone, Debug, PartialEq)]
pub struct TabulatedGrain {
    points: Vec<[f64; 2]>,
    propellant_volume_m3: f64,
}

impl TabulatedGrain {
    /// Construct from strictly increasing `(web_m, burn_area_m2)`
    /// pairs.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when the table is too short,
    /// non-finite, non-monotone, or has negative areas.
    pub fn new(points: Vec<[f64; 2]>, propellant_volume_m3: f64) -> Result<Self, MotorError> {
        if points.len() < 2 {
            return Err(MotorError::MalformedMotor {
                reason: "tabulated grain must have at least two points",
            });
        }
        if !propellant_volume_m3.is_finite() || propellant_volume_m3 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "tabulated grain propellant_volume_m3 must be finite and positive",
            });
        }
        for point in &points {
            if !point[0].is_finite() || !point[1].is_finite() {
                return Err(MotorError::NonFinite {
                    reason: "tabulated grain point is NaN or infinite",
                });
            }
            if point[0] < 0.0 || point[1] < 0.0 {
                return Err(MotorError::InvalidParameter {
                    reason: "tabulated grain web and area must be non-negative",
                });
            }
        }
        if points[0][0] != 0.0 {
            return Err(MotorError::MalformedMotor {
                reason: "tabulated grain must start at web_m = 0",
            });
        }
        for pair in points.windows(2) {
            if pair[1][0] <= pair[0][0] {
                return Err(MotorError::MalformedMotor {
                    reason: "tabulated grain web grid must be strictly increasing",
                });
            }
        }
        Ok(Self {
            points,
            propellant_volume_m3,
        })
    }

    /// Read-only table access.
    #[must_use]
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
}

impl GrainGeometry for TabulatedGrain {
    fn burn_area_m2(&self, web_m: f64) -> Result<f64, MotorError> {
        if !web_m.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "grain web distance is NaN or infinite",
            });
        }
        if web_m >= self.web_total_m() {
            return Ok(0.0);
        }
        if web_m <= 0.0 {
            return Ok(self.points[0][1]);
        }
        let upper = self.points.partition_point(|p| p[0] <= web_m);
        let i = upper.saturating_sub(1).min(self.points.len() - 2);
        let p0 = self.points[i];
        let p1 = self.points[i + 1];
        let alpha = (web_m - p0[0]) / (p1[0] - p0[0]);
        Ok((1.0 - alpha) * p0[1] + alpha * p1[1])
    }

    fn web_total_m(&self) -> f64 {
        self.points[self.points.len() - 1][0]
    }

    fn propellant_volume_m3(&self) -> f64 {
        self.propellant_volume_m3
    }
}

/// Deterministic equilibrium internal-ballistics solver.
#[derive(Clone, Debug, PartialEq)]
pub struct EquilibriumInternalBallistics {
    /// Propellant constants.
    pub propellant: GrainPropellant,
    /// Nozzle throat area (m²).
    pub throat_area_m2: f64,
    /// Nozzle expansion ratio `Ae / At`.
    pub expansion_ratio: f64,
    /// Fixed web grid step count.
    pub web_steps: u32,
    /// Dry motor case/nozzle mass (kg). Scenario assembly dry mass is
    /// separate, so this may be zero for purely propellant-derived
    /// teaching examples.
    pub dry_mass_kg: f64,
    /// Display/provenance name.
    pub name: String,
    /// Provenance sentence.
    pub provenance: String,
    /// Validation label carried into [`SolidMotor`].
    pub validation: Validation,
}

impl EquilibriumInternalBallistics {
    /// Construct a validated solver.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] when constants are outside the
    /// quasi-steady envelope.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        propellant: GrainPropellant,
        throat_area_m2: f64,
        expansion_ratio: f64,
        web_steps: u32,
        dry_mass_kg: f64,
        name: String,
        provenance: String,
        validation: Validation,
    ) -> Result<Self, MotorError> {
        propellant.require_valid()?;
        if !throat_area_m2.is_finite() || throat_area_m2 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain throat_area_m2 must be finite and positive",
            });
        }
        if !expansion_ratio.is_finite() || expansion_ratio < 1.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain expansion_ratio must be finite and at least 1",
            });
        }
        if web_steps == 0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain web_steps must be positive",
            });
        }
        if !dry_mass_kg.is_finite() || dry_mass_kg < 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain dry_mass_kg must be finite and non-negative",
            });
        }
        if name.is_empty() || provenance.is_empty() {
            return Err(MotorError::InvalidParameter {
                reason: "grain regression name and provenance must not be empty",
            });
        }
        Ok(Self {
            propellant,
            throat_area_m2,
            expansion_ratio,
            web_steps,
            dry_mass_kg,
            name,
            provenance,
            validation,
        })
    }
}

impl GrainRegressionModel for EquilibriumInternalBallistics {
    fn regress(&self, geom: &dyn GrainGeometry) -> Result<SolidMotor, MotorError> {
        self.propellant.require_valid()?;
        validate_geometry_surface(geom)?;
        let cf = optimum_thrust_coefficient(self.propellant.gamma, self.expansion_ratio)?;
        let web_total_m = geom.web_total_m();
        let web_step_m = web_total_m / f64::from(self.web_steps);
        let mut times = Vec::with_capacity(self.web_steps as usize + 1);
        let mut thrusts = Vec::with_capacity(self.web_steps as usize + 1);
        times.push(0.0);
        thrusts.push(0.0);
        let mut t_s = 0.0;
        for k in 0..self.web_steps {
            let web_m = f64::from(k) * web_step_m;
            let area_m2 = geom.burn_area_m2(web_m)?;
            if area_m2 <= 0.0 {
                return Err(MotorError::OutOfEnvelope {
                    reason: "grain burn area reached zero before web_total_m",
                });
            }
            let kn = area_m2 / self.throat_area_m2;
            let pc_pa = equilibrium_chamber_pressure_pa(&self.propellant, kn)?;
            let burn_rate_m_s =
                self.propellant.burn_rate_a * pc_pa.powf(self.propellant.burn_rate_n);
            let thrust_n = cf * self.throat_area_m2 * pc_pa;
            if !burn_rate_m_s.is_finite()
                || burn_rate_m_s <= 0.0
                || !thrust_n.is_finite()
                || thrust_n < 0.0
            {
                return Err(MotorError::NonFinite {
                    reason: "grain regression produced non-finite burn rate or thrust",
                });
            }
            let dt_s = web_step_m / burn_rate_m_s;
            t_s += dt_s;
            if !t_s.is_finite() || t_s <= *times.last().unwrap_or(&0.0) {
                return Err(MotorError::NonFinite {
                    reason: "grain regression produced invalid time grid",
                });
            }
            times.push(t_s);
            thrusts.push(thrust_n);
        }
        thrusts[0] = 0.0;
        let last = thrusts.len() - 1;
        thrusts[last] = 0.0;
        let points: Vec<[f64; 2]> = times
            .iter()
            .zip(thrusts.iter())
            .map(|(t, f)| [*t, *f])
            .collect();
        let curve = ThrustCurve::new(points)?;
        let total_impulse_n_s = curve.integrated_impulse_n_s();
        let propellant_mass_kg = self.propellant.density_kg_m3 * geom.propellant_volume_m3();
        if !propellant_mass_kg.is_finite() || propellant_mass_kg <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain propellant mass must be finite and positive",
            });
        }
        if total_impulse_n_s <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "grain regression total impulse must be positive",
            });
        }
        let specific_impulse_s = total_impulse_n_s / (propellant_mass_kg * STANDARD_GRAVITY_M_S2);
        let burn = BurnSpec {
            duration_s: curve.last_time_s(),
            total_impulse_n_s,
            specific_impulse_s,
            propellant_mass_kg,
            dry_mass_kg: self.dry_mass_kg,
        };
        let meta = MotorMeta {
            name: self.name.clone(),
            provenance: self.provenance.clone(),
            validation: self.validation,
        };
        let geometry = MotorGeometry {
            exit_area_m2: self.throat_area_m2 * self.expansion_ratio,
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
        };
        SolidMotor::new(meta, burn, curve, geometry)
    }
}

fn validate_geometry_surface(geom: &dyn GrainGeometry) -> Result<(), MotorError> {
    let web_total_m = geom.web_total_m();
    if !web_total_m.is_finite() || web_total_m <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "grain web_total_m must be finite and positive",
        });
    }
    let propellant_volume_m3 = geom.propellant_volume_m3();
    if !propellant_volume_m3.is_finite() || propellant_volume_m3 <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "grain propellant volume must be finite and positive",
        });
    }
    let area0 = geom.burn_area_m2(0.0)?;
    if !area0.is_finite() || area0 <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "grain initial burn area must be finite and positive",
        });
    }
    Ok(())
}

fn equilibrium_chamber_pressure_pa(
    propellant: &GrainPropellant,
    kn: f64,
) -> Result<f64, MotorError> {
    if !kn.is_finite() || kn <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "grain Kn must be finite and positive",
        });
    }
    let base = propellant.density_kg_m3 * propellant.burn_rate_a * propellant.c_star_m_s * kn;
    let exponent = 1.0 / (1.0 - propellant.burn_rate_n);
    let pc_pa = base.powf(exponent);
    if !pc_pa.is_finite() || pc_pa <= 0.0 {
        return Err(MotorError::NonFinite {
            reason: "grain chamber pressure is non-finite",
        });
    }
    Ok(pc_pa)
}

fn optimum_thrust_coefficient(gamma: f64, expansion_ratio: f64) -> Result<f64, MotorError> {
    if !gamma.is_finite() || gamma <= 1.0 || !expansion_ratio.is_finite() || expansion_ratio < 1.0 {
        return Err(MotorError::InvalidParameter {
            reason: "nozzle gamma and expansion_ratio are outside envelope",
        });
    }
    let exit_mach = supersonic_mach_for_area_ratio(gamma, expansion_ratio)?;
    let pe_pc = (1.0 + 0.5 * (gamma - 1.0) * exit_mach * exit_mach).powf(-gamma / (gamma - 1.0));
    let term = 1.0 - pe_pc.powf((gamma - 1.0) / gamma);
    let coeff = ((2.0 * gamma * gamma / (gamma - 1.0))
        * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (gamma - 1.0))
        * term)
        .sqrt();
    if !coeff.is_finite() || coeff <= 0.0 {
        return Err(MotorError::NonFinite {
            reason: "nozzle thrust coefficient is non-finite",
        });
    }
    Ok(coeff)
}

fn supersonic_mach_for_area_ratio(gamma: f64, expansion_ratio: f64) -> Result<f64, MotorError> {
    if (expansion_ratio - 1.0).abs() <= MIN_POSITIVE {
        return Ok(1.0);
    }
    let mut lo = 1.0;
    let mut hi = 50.0;
    if nozzle_area_ratio(gamma, hi) < expansion_ratio {
        return Err(MotorError::OutOfEnvelope {
            reason: "nozzle expansion_ratio is outside the fixed Mach bracket",
        });
    }
    for _ in 0..NOZZLE_MACH_BISECTION_ITERS {
        let mid = 0.5 * (lo + hi);
        if nozzle_area_ratio(gamma, mid) < expansion_ratio {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(0.5 * (lo + hi))
}

fn nozzle_area_ratio(gamma: f64, mach: f64) -> f64 {
    let gm1 = gamma - 1.0;
    let gp1 = gamma + 1.0;
    let bracket = (2.0 / gp1) * (1.0 + 0.5 * gm1 * mach * mach);
    (1.0 / mach) * bracket.powf(gp1 / (2.0 * gm1))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::motor::Motor;
    use approx::assert_abs_diff_eq;

    fn propellant() -> GrainPropellant {
        GrainPropellant {
            label: "synthetic_apcp".to_owned(),
            density_kg_m3: 1700.0,
            burn_rate_a: 4.0e-5,
            burn_rate_n: 0.32,
            c_star_m_s: 1400.0,
            gamma: 1.2,
        }
    }

    fn solver(web_steps: u32) -> EquilibriumInternalBallistics {
        EquilibriumInternalBallistics::new(
            propellant(),
            std::f64::consts::PI * 0.003_f64 * 0.003_f64,
            8.0,
            web_steps,
            0.0,
            "synthetic-grain".to_owned(),
            "synthetic textbook unit test".to_owned(),
            Validation::Checked,
        )
        .unwrap()
    }

    #[test]
    fn end_burner_regression_is_nearly_constant_thrust() {
        let grain = EndBurnerGrain::new(0.002, 0.05).unwrap();
        let motor = solver(400).regress(&grain).unwrap();
        let f_a = motor.thrust_n_at(motor.burn_duration_s() * 0.25).unwrap();
        let f_b = motor.thrust_n_at(motor.burn_duration_s() * 0.75).unwrap();
        assert_abs_diff_eq!(f_a, f_b, epsilon = f_a.abs() * 1.0e-9);
        let analytic = f_a * motor.burn_duration_s();
        assert!((analytic - motor.total_impulse_n_s()).abs() / motor.total_impulse_n_s() < 3.0e-3);
    }

    #[test]
    fn bates_area_changes_with_web() {
        let grain = BatesGrain::new(4, 0.025, 0.010, 0.09).unwrap();
        let a0 = grain.burn_area_m2(0.0).unwrap();
        let amid = grain.burn_area_m2(0.5 * grain.web_total_m()).unwrap();
        assert!(a0 > 0.0);
        assert!(amid > 0.0);
        assert_ne!(a0.to_bits(), amid.to_bits());
    }

    #[test]
    fn tabulated_grain_interpolates_area() {
        let grain = TabulatedGrain::new(vec![[0.0, 2.0], [0.5, 4.0], [1.0, 0.0]], 1.0).unwrap();
        assert_abs_diff_eq!(grain.burn_area_m2(0.25).unwrap(), 3.0, epsilon = 1.0e-12);
    }

    #[test]
    fn regression_is_bit_deterministic() {
        let grain = BatesGrain::new(3, 0.03, 0.012, 0.08).unwrap();
        let a = solver(200).regress(&grain).unwrap();
        let b = solver(200).regress(&grain).unwrap();
        assert_eq!(a.thrust_curve().points(), b.thrust_curve().points());
        assert_eq!(
            a.total_impulse_n_s().to_bits(),
            b.total_impulse_n_s().to_bits()
        );
    }

    #[test]
    fn rejects_unstable_pressure_exponent() {
        let bad = GrainPropellant {
            burn_rate_n: 1.0,
            ..propellant()
        };
        assert!(matches!(
            EquilibriumInternalBallistics::new(
                bad,
                1.0e-5,
                4.0,
                16,
                0.0,
                "bad".to_owned(),
                "synthetic".to_owned(),
                Validation::Experimental,
            ),
            Err(MotorError::InvalidParameter { .. })
        ));
    }
}
