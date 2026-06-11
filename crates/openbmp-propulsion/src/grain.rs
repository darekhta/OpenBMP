//! Solid-motor grain regression and zero-dimensional internal ballistics.
//!
//! This module is a forward curve producer: a grain geometry and
//! synthetic/textbook propellant constants are regressed on a fixed web
//! grid to produce the [`crate::motor::ThrustCurve`] and
//! [`crate::motor::BurnSpec`] consumed by [`crate::motor::SolidMotor`].
//! It does not alter the hot motor trait surface.

#[cfg(not(feature = "std"))]
use alloc::{string::String, vec::Vec};
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::error::MotorError;
use crate::motor::{
    AmbientPressureCorrection, BurnSpec, MotorGeometry, MotorMeta, NozzleSeparationCriterion,
    SolidMotor, ThrustCurve, Validation, ideal_momentum_thrust_coefficient,
};

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const TRANSIENT_CHAMBER_VOLUME_FRACTION: f64 = 1.0;
const TRANSIENT_IGNITION_PRESSURE_PA: f64 = 101_325.0;
const TRANSIENT_EROSIVE_REFERENCE_MASS_FLUX_KG_M2_S: f64 = 1_000.0;
const TRANSIENT_PRESSURE_CFL: f64 = 0.2;
const TRANSIENT_WEB_CFL: f64 = 0.25;
const TRANSIENT_MAX_STEPS_PER_WEB_STEP: u32 = 2_048;
const MIN_POSITIVE: f64 = 1.0e-15;

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

/// Internal-ballistics regression mode for inline solid grains.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum GrainRegressionMode {
    /// Current equilibrium `pc(Kn)` march. This preserves legacy output.
    #[default]
    QuasiStatic,
    /// Explicit lumped-volume chamber-pressure integration.
    Transient,
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

/// Reduced lumped-volume chamber model used by transient grain regression.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransientChamber {
    /// Chamber free-volume scale relative to propellant volume.
    pub volume_fraction_of_propellant: f64,
    /// Initial pressure used to ignite the pressure ODE, in Pa.
    pub ignition_pressure_pa: f64,
    /// Lenoir-Robert-style erosive burn-rate coefficient.
    pub erosive_coefficient: f64,
    /// Reference nozzle mass flux for the erosive term, in kg/(m² s).
    pub erosive_reference_mass_flux_kg_m2_s: f64,
}

impl Default for TransientChamber {
    fn default() -> Self {
        Self {
            volume_fraction_of_propellant: TRANSIENT_CHAMBER_VOLUME_FRACTION,
            ignition_pressure_pa: TRANSIENT_IGNITION_PRESSURE_PA,
            erosive_coefficient: 0.0,
            erosive_reference_mass_flux_kg_m2_s: TRANSIENT_EROSIVE_REFERENCE_MASS_FLUX_KG_M2_S,
        }
    }
}

impl TransientChamber {
    /// Construct a transient chamber with the requested free-volume scale.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] if the volume scale is non-finite or not positive.
    pub fn new(volume_fraction_of_propellant: f64) -> Result<Self, MotorError> {
        Self {
            volume_fraction_of_propellant,
            ..Self::default()
        }
        .require_valid()
    }

    /// Return a copy with a nonzero erosive burn-rate term.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError`] if either coefficient is outside the
    /// deterministic reduced-model envelope.
    pub fn with_erosive_term(
        mut self,
        erosive_coefficient: f64,
        erosive_reference_mass_flux_kg_m2_s: f64,
    ) -> Result<Self, MotorError> {
        self.erosive_coefficient = erosive_coefficient;
        self.erosive_reference_mass_flux_kg_m2_s = erosive_reference_mass_flux_kg_m2_s;
        self.require_valid()
    }

    fn require_valid(self) -> Result<Self, MotorError> {
        if !self.volume_fraction_of_propellant.is_finite()
            || self.volume_fraction_of_propellant <= 0.0
        {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber volume fraction must be finite and positive",
            });
        }
        if !self.ignition_pressure_pa.is_finite() || self.ignition_pressure_pa <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber ignition pressure must be finite and positive",
            });
        }
        if !self.erosive_coefficient.is_finite() || self.erosive_coefficient < 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber erosive coefficient must be finite and non-negative",
            });
        }
        if !self.erosive_reference_mass_flux_kg_m2_s.is_finite()
            || self.erosive_reference_mass_flux_kg_m2_s <= 0.0
        {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber erosive reference mass flux must be finite and positive",
            });
        }
        Ok(self)
    }

    fn volume_m3(self, propellant_volume_m3: f64) -> Result<f64, MotorError> {
        let volume_m3 = self.volume_fraction_of_propellant * propellant_volume_m3;
        if !volume_m3.is_finite() || volume_m3 <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber volume must be finite and positive",
            });
        }
        Ok(volume_m3)
    }

    fn burn_rate_multiplier(self, nozzle_mass_flux_kg_m2_s: f64) -> Result<f64, MotorError> {
        if !nozzle_mass_flux_kg_m2_s.is_finite() || nozzle_mass_flux_kg_m2_s < 0.0 {
            return Err(MotorError::NonFinite {
                reason: "transient chamber mass flux is non-finite",
            });
        }
        let multiplier = if self.erosive_coefficient == 0.0 {
            1.0
        } else {
            1.0 + self.erosive_coefficient
                * (nozzle_mass_flux_kg_m2_s / self.erosive_reference_mass_flux_kg_m2_s).powf(0.8)
        };
        if !multiplier.is_finite() || multiplier < 1.0 {
            return Err(MotorError::NonFinite {
                reason: "transient chamber erosive multiplier is non-finite",
            });
        }
        Ok(multiplier)
    }

    fn pressure_derivative_pa_s(
        self,
        propellant: &GrainPropellant,
        chamber_volume_m3: f64,
        burn_area_m2: f64,
        throat_area_m2: f64,
        chamber_pressure_pa: f64,
    ) -> Result<f64, MotorError> {
        let nozzle_mass_flow_kg_s = chamber_pressure_pa * throat_area_m2 / propellant.c_star_m_s;
        let nozzle_mass_flux_kg_m2_s = nozzle_mass_flow_kg_s / throat_area_m2;
        let burn_rate_m_s = transient_burn_rate_m_s(
            self,
            propellant,
            chamber_pressure_pa,
            nozzle_mass_flux_kg_m2_s,
        )?;
        let generated_mass_flow_kg_s = propellant.density_kg_m3 * burn_area_m2 * burn_rate_m_s;
        let dpdt = propellant.c_star_m_s
            * propellant.c_star_m_s
            * (generated_mass_flow_kg_s - nozzle_mass_flow_kg_s)
            / chamber_volume_m3;
        if !dpdt.is_finite() {
            return Err(MotorError::NonFinite {
                reason: "transient chamber pressure derivative is non-finite",
            });
        }
        Ok(dpdt)
    }
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
        let pi = core::f64::consts::PI;
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
        let pi = core::f64::consts::PI;
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
    /// Internal-ballistics regression mode.
    pub mode: GrainRegressionMode,
    /// Reduced transient chamber configuration.
    pub transient_chamber: TransientChamber,
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
            mode: GrainRegressionMode::QuasiStatic,
            transient_chamber: TransientChamber::default(),
        })
    }

    /// Return a copy using a different grain-regression mode.
    #[must_use]
    pub const fn with_regression_mode(mut self, mode: GrainRegressionMode) -> Self {
        self.mode = mode;
        self
    }

    /// Return a copy with explicit transient chamber settings.
    #[must_use]
    pub const fn with_transient_chamber(mut self, chamber: TransientChamber) -> Self {
        self.transient_chamber = chamber;
        self
    }
}

impl GrainRegressionModel for EquilibriumInternalBallistics {
    fn regress(&self, geom: &dyn GrainGeometry) -> Result<SolidMotor, MotorError> {
        match self.mode {
            GrainRegressionMode::QuasiStatic => self.regress_quasi_static(geom),
            GrainRegressionMode::Transient => self.regress_transient(geom),
        }
    }
}

impl EquilibriumInternalBallistics {
    fn regress_quasi_static(&self, geom: &dyn GrainGeometry) -> Result<SolidMotor, MotorError> {
        self.propellant.require_valid()?;
        validate_geometry_surface(geom)?;
        let cf = ideal_momentum_thrust_coefficient(self.propellant.gamma, self.expansion_ratio)?;
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
        self.motor_from_curve(geom, curve)
    }

    fn regress_transient(&self, geom: &dyn GrainGeometry) -> Result<SolidMotor, MotorError> {
        self.propellant.require_valid()?;
        validate_geometry_surface(geom)?;
        let chamber = self.transient_chamber.require_valid()?;
        let chamber_volume_m3 = chamber.volume_m3(geom.propellant_volume_m3())?;
        let cf = ideal_momentum_thrust_coefficient(self.propellant.gamma, self.expansion_ratio)?;
        let web_total_m = geom.web_total_m();
        let web_step_m = web_total_m / f64::from(self.web_steps);
        let pressure_time_step_s = TRANSIENT_PRESSURE_CFL * chamber_volume_m3
            / (self.propellant.c_star_m_s * self.throat_area_m2);
        if !pressure_time_step_s.is_finite() || pressure_time_step_s <= 0.0 {
            return Err(MotorError::InvalidParameter {
                reason: "transient chamber pressure time step is outside envelope",
            });
        }
        let max_steps = self
            .web_steps
            .checked_mul(TRANSIENT_MAX_STEPS_PER_WEB_STEP)
            .ok_or(MotorError::OutOfEnvelope {
                reason: "transient chamber integration step cap overflowed",
            })?;

        let mut points = Vec::with_capacity(self.web_steps as usize + 2);
        points.push([0.0, 0.0]);
        let mut t_s = 0.0;
        let mut web_m = 0.0;
        let mut chamber_pressure_pa = chamber.ignition_pressure_pa;
        let mut steps = 0_u32;

        while web_m < web_total_m {
            if steps >= max_steps {
                return Err(MotorError::OutOfEnvelope {
                    reason: "transient chamber integration exceeded fixed step cap",
                });
            }
            let burn_area_m2 = geom.burn_area_m2(web_m)?;
            if burn_area_m2 <= 0.0 {
                break;
            }
            let nozzle_mass_flux_kg_m2_s = chamber_pressure_pa / self.propellant.c_star_m_s;
            let burn_rate_m_s = transient_burn_rate_m_s(
                chamber,
                &self.propellant,
                chamber_pressure_pa,
                nozzle_mass_flux_kg_m2_s,
            )?;
            let web_time_step_s = TRANSIENT_WEB_CFL * web_step_m / burn_rate_m_s;
            let burnout_time_step_s = (web_total_m - web_m) / burn_rate_m_s;
            let dt_s = pressure_time_step_s
                .min(web_time_step_s)
                .min(burnout_time_step_s);
            if !dt_s.is_finite() || dt_s <= 0.0 {
                return Err(MotorError::NonFinite {
                    reason: "transient chamber integration produced invalid time step",
                });
            }

            let dpdt = chamber.pressure_derivative_pa_s(
                &self.propellant,
                chamber_volume_m3,
                burn_area_m2,
                self.throat_area_m2,
                chamber_pressure_pa,
            )?;
            chamber_pressure_pa =
                explicit_pressure_euler_step(chamber_pressure_pa, dpdt, dt_s)?.max(MIN_POSITIVE);
            web_m += burn_rate_m_s * dt_s;
            t_s += dt_s;
            let thrust_n = cf * self.throat_area_m2 * chamber_pressure_pa;
            if !t_s.is_finite()
                || !web_m.is_finite()
                || !chamber_pressure_pa.is_finite()
                || !thrust_n.is_finite()
                || thrust_n < 0.0
            {
                return Err(MotorError::NonFinite {
                    reason: "transient chamber integration produced non-finite output",
                });
            }
            if points.last().is_some_and(|point| t_s <= point[0]) {
                return Err(MotorError::NonFinite {
                    reason: "transient chamber integration produced invalid time grid",
                });
            }
            points.push([t_s, thrust_n]);
            steps += 1;
        }
        if points.len() < 2 {
            return Err(MotorError::OutOfEnvelope {
                reason: "transient chamber integration produced no burn",
            });
        }
        let last = points.len() - 1;
        points[last][1] = 0.0;
        let curve = ThrustCurve::new(points)?;
        self.motor_from_curve(geom, curve)
    }

    fn motor_from_curve(
        &self,
        geom: &dyn GrainGeometry,
        curve: ThrustCurve,
    ) -> Result<SolidMotor, MotorError> {
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
            throat_area_m2: Some(self.throat_area_m2),
            gamma: Some(self.propellant.gamma),
            ambient_pressure_correction: AmbientPressureCorrection::Constant,
            separation: NozzleSeparationCriterion::Off,
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

fn transient_burn_rate_m_s(
    chamber: TransientChamber,
    propellant: &GrainPropellant,
    chamber_pressure_pa: f64,
    nozzle_mass_flux_kg_m2_s: f64,
) -> Result<f64, MotorError> {
    if !chamber_pressure_pa.is_finite() || chamber_pressure_pa <= 0.0 {
        return Err(MotorError::InvalidParameter {
            reason: "transient chamber pressure must be finite and positive",
        });
    }
    let base_burn_rate_m_s =
        propellant.burn_rate_a * chamber_pressure_pa.powf(propellant.burn_rate_n);
    let burn_rate_m_s =
        base_burn_rate_m_s * chamber.burn_rate_multiplier(nozzle_mass_flux_kg_m2_s)?;
    if !burn_rate_m_s.is_finite() || burn_rate_m_s <= 0.0 {
        return Err(MotorError::NonFinite {
            reason: "transient chamber burn rate is non-finite",
        });
    }
    Ok(burn_rate_m_s)
}

fn explicit_pressure_euler_step(
    pressure_pa: f64,
    pressure_derivative_pa_s: f64,
    dt_s: f64,
) -> Result<f64, MotorError> {
    if !pressure_pa.is_finite()
        || !pressure_derivative_pa_s.is_finite()
        || !dt_s.is_finite()
        || dt_s <= 0.0
    {
        return Err(MotorError::NonFinite {
            reason: "transient chamber pressure step is non-finite",
        });
    }
    let next = pressure_pa + pressure_derivative_pa_s * dt_s;
    if !next.is_finite() {
        return Err(MotorError::NonFinite {
            reason: "transient chamber pressure step produced non-finite output",
        });
    }
    Ok(next)
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
    fn quasi_static_mode_preserves_existing_curve() {
        let grain = BatesGrain::new(3, 0.03, 0.012, 0.08).unwrap();
        let baseline = solver(100).regress(&grain).unwrap();
        let explicit_quasi_static = solver(100)
            .with_regression_mode(GrainRegressionMode::QuasiStatic)
            .regress(&grain)
            .unwrap();

        assert_eq!(
            baseline.thrust_curve().points(),
            explicit_quasi_static.thrust_curve().points()
        );
        assert_eq!(
            baseline.total_impulse_n_s().to_bits(),
            explicit_quasi_static.total_impulse_n_s().to_bits()
        );
    }

    #[test]
    fn transient_end_burner_regression_relaxes_to_constant_thrust() {
        let grain = EndBurnerGrain::new(0.002, 0.05).unwrap();
        let chamber = TransientChamber::new(4.0).unwrap();
        let motor = solver(64)
            .with_regression_mode(GrainRegressionMode::Transient)
            .with_transient_chamber(chamber)
            .regress(&grain)
            .unwrap();

        let f_a = motor.thrust_n_at(motor.burn_duration_s() * 0.60).unwrap();
        let f_b = motor.thrust_n_at(motor.burn_duration_s() * 0.85).unwrap();
        assert!(motor.thrust_curve().points().len() > 64);
        assert!((f_a - f_b).abs() / f_b.abs() < 2.0e-2);
    }

    #[test]
    fn transient_pressure_mms_recovers_first_order_solution() {
        fn integrate(dt_s: f64) -> f64 {
            let mut pressure_pa = 5.0;
            let mut t_s = 0.0;
            while t_s < 1.0 {
                let step_s = dt_s.min(1.0 - t_s);
                let derivative_pa_s = 2.0 * t_s + 3.0;
                pressure_pa =
                    explicit_pressure_euler_step(pressure_pa, derivative_pa_s, step_s).unwrap();
                t_s += step_s;
            }
            pressure_pa
        }

        let exact = 9.0;
        let coarse_error = (integrate(0.1) - exact).abs();
        let fine_error = (integrate(0.05) - exact).abs();
        assert!(fine_error < 0.6 * coarse_error);
    }

    #[test]
    fn transient_chamber_erosive_term_increases_burn_rate() {
        let base = TransientChamber::default();
        let erosive = base.with_erosive_term(0.15, 1_000.0).unwrap();
        let pressure_pa = 2.0e6;
        let mass_flux_kg_m2_s = 1_500.0;

        let base_rate =
            transient_burn_rate_m_s(base, &propellant(), pressure_pa, mass_flux_kg_m2_s).unwrap();
        let erosive_rate =
            transient_burn_rate_m_s(erosive, &propellant(), pressure_pa, mass_flux_kg_m2_s)
                .unwrap();
        assert!(erosive_rate > base_rate);
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
