//! Landing-gear force primitives.
//!
//! This module is the vehicle-side substrate for contact WP-14.4. It keeps
//! gear mechanics independent from runner scenario plumbing: validated leg
//! geometry, a polytropic oleo stage, and an irreversible crush core. Runner
//! racks and per-leg telemetry are layered on top in later slices.

use openbmp_contact::ContactGeometry;
use thiserror::Error;

/// Errors raised by landing-gear primitive construction or evaluation.
#[derive(Clone, Debug, Eq, PartialEq, Error)]
pub enum LandingGearError {
    /// A scalar parameter failed validation.
    #[error("landing gear parameter {field} invalid: {reason}")]
    InvalidParameter {
        /// Parameter name.
        field: &'static str,
        /// Short validation reason.
        reason: &'static str,
    },
    /// A vector parameter failed validation.
    #[error("landing gear vector {field} invalid: {reason}")]
    InvalidVector {
        /// Vector name.
        field: &'static str,
        /// Short validation reason.
        reason: &'static str,
    },
}

/// Massless landing-gear leg definition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandingGearLeg {
    attach_body_m: [f64; 3],
    strut_axis_body: [f64; 3],
    oleo: Option<OleoStage>,
    crush: Option<CrushCore>,
    footpad: ContactGeometry,
}

impl LandingGearLeg {
    /// Creates a validated massless landing-gear leg.
    ///
    /// `strut_axis_body` is normalized during construction. At least one of
    /// `oleo` or `crush` must be present; a bare footpad is represented by the
    /// existing contact force path rather than by a landing-gear leg.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when vectors are non-finite/zero or when
    /// both compliant stages are absent.
    pub fn new(
        attach_body_m: [f64; 3],
        strut_axis_body: [f64; 3],
        oleo: Option<OleoStage>,
        crush: Option<CrushCore>,
        footpad: ContactGeometry,
    ) -> Result<Self, LandingGearError> {
        if oleo.is_none() && crush.is_none() {
            return Err(LandingGearError::InvalidParameter {
                field: "leg",
                reason: "at least one compliant stage is required",
            });
        }
        Ok(Self {
            attach_body_m: finite_vector("attach_body_m", attach_body_m)?,
            strut_axis_body: unit_vector("strut_axis_body", strut_axis_body)?,
            oleo,
            crush,
            footpad,
        })
    }

    /// Returns the body-frame hardpoint.
    #[must_use]
    pub const fn attach_body_m(self) -> [f64; 3] {
        self.attach_body_m
    }

    /// Returns the unit strut axis in body coordinates.
    #[must_use]
    pub const fn strut_axis_body(self) -> [f64; 3] {
        self.strut_axis_body
    }

    /// Returns the optional oleo stage.
    #[must_use]
    pub const fn oleo(self) -> Option<OleoStage> {
        self.oleo
    }

    /// Returns the optional crush-core stage.
    #[must_use]
    pub const fn crush(self) -> Option<CrushCore> {
        self.crush
    }

    /// Returns the footpad contact geometry.
    #[must_use]
    pub const fn footpad(self) -> ContactGeometry {
        self.footpad
    }
}

/// Polytropic oleo strut stage with quadratic orifice damping.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OleoStage {
    p0_pa: f64,
    v0_m3: f64,
    gamma: f64,
    orifice_c_n_s2_m2: f64,
    stroke_max_m: f64,
    piston_area_m2: f64,
}

impl OleoStage {
    /// Creates an oleo stage.
    ///
    /// Gas pressure follows `p = p0 * (v0 / v)^gamma`, where
    /// `v = v0 - piston_area * stroke`. Compression damping is
    /// `orifice_c_n_s2_m2 * max(stroke_rate, 0)^2`.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when parameters are non-finite, outside
    /// their physical ranges, or when the maximum stroke would exhaust the gas
    /// volume.
    pub fn new(
        p0_pa: f64,
        v0_m3: f64,
        gamma: f64,
        orifice_c_n_s2_m2: f64,
        stroke_max_m: f64,
        piston_area_m2: f64,
    ) -> Result<Self, LandingGearError> {
        let p0_pa = require_positive("p0_pa", p0_pa)?;
        let v0_m3 = require_positive("v0_m3", v0_m3)?;
        let gamma = require_positive("gamma", gamma)?;
        let orifice_c_n_s2_m2 = require_non_negative("orifice_c_n_s2_m2", orifice_c_n_s2_m2)?;
        let stroke_max_m = require_positive("stroke_max_m", stroke_max_m)?;
        let piston_area_m2 = require_positive("piston_area_m2", piston_area_m2)?;
        if piston_area_m2 * stroke_max_m >= v0_m3 {
            return Err(LandingGearError::InvalidParameter {
                field: "stroke_max_m",
                reason: "piston_area_m2 * stroke_max_m must be less than v0_m3",
            });
        }
        Ok(Self {
            p0_pa,
            v0_m3,
            gamma,
            orifice_c_n_s2_m2,
            stroke_max_m,
            piston_area_m2,
        })
    }

    /// Returns gas pressure at the supplied stroke.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when stroke is non-finite or outside
    /// `[0, stroke_max_m]`.
    pub fn gas_pressure_pa(self, stroke_m: f64) -> Result<f64, LandingGearError> {
        let stroke_m = require_bounded("stroke_m", stroke_m, 0.0, self.stroke_max_m)?;
        let volume_m3 = self.v0_m3 - self.piston_area_m2 * stroke_m;
        Ok(self.p0_pa * (self.v0_m3 / volume_m3).powf(self.gamma))
    }

    /// Returns the total compressive force at stroke and stroke rate.
    ///
    /// Negative stroke rates are treated as unloading and contribute no
    /// orifice damping.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] for invalid stroke or non-finite rate.
    pub fn force_n(
        self,
        stroke_m: f64,
        compression_rate_m_s: f64,
    ) -> Result<f64, LandingGearError> {
        let compression_rate_m_s = require_finite("compression_rate_m_s", compression_rate_m_s)?;
        let gas_force_n = self.gas_pressure_pa(stroke_m)? * self.piston_area_m2;
        let damping_force_n = self.orifice_c_n_s2_m2 * compression_rate_m_s.max(0.0).powi(2);
        Ok(gas_force_n + damping_force_n)
    }

    /// Returns polytropic gas energy stored between zero stroke and `stroke_m`.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when stroke is invalid.
    pub fn stored_energy_j(self, stroke_m: f64) -> Result<f64, LandingGearError> {
        let stroke_m = require_bounded("stroke_m", stroke_m, 0.0, self.stroke_max_m)?;
        let volume_m3 = self.v0_m3 - self.piston_area_m2 * stroke_m;
        if (self.gamma - 1.0).abs() <= f64::EPSILON {
            Ok(self.p0_pa * self.v0_m3 * (self.v0_m3 / volume_m3).ln())
        } else {
            Ok(self.p0_pa
                * self.v0_m3.powf(self.gamma)
                * (volume_m3.powf(1.0 - self.gamma) - self.v0_m3.powf(1.0 - self.gamma))
                / (self.gamma - 1.0))
        }
    }

    /// Returns maximum stroke.
    #[must_use]
    pub const fn stroke_max_m(self) -> f64 {
        self.stroke_max_m
    }
}

/// Irreversible crush-core stage.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrushCore {
    f_crush_n: f64,
    stroke_max_m: f64,
    k_elastic_n_m: f64,
    crushed_m: f64,
}

impl CrushCore {
    /// Creates an uncrushed elasto-plastic crush core.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when parameters are invalid or when the
    /// elastic yield stroke exceeds `stroke_max_m`.
    pub fn new(
        f_crush_n: f64,
        stroke_max_m: f64,
        k_elastic_n_m: f64,
    ) -> Result<Self, LandingGearError> {
        let f_crush_n = require_positive("f_crush_n", f_crush_n)?;
        let stroke_max_m = require_positive("stroke_max_m", stroke_max_m)?;
        let k_elastic_n_m = require_positive("k_elastic_n_m", k_elastic_n_m)?;
        if f_crush_n / k_elastic_n_m > stroke_max_m {
            return Err(LandingGearError::InvalidParameter {
                field: "k_elastic_n_m",
                reason: "elastic yield stroke must not exceed stroke_max_m",
            });
        }
        Ok(Self {
            f_crush_n,
            stroke_max_m,
            k_elastic_n_m,
            crushed_m: 0.0,
        })
    }

    /// Returns the irreversible plateau stroke already crushed.
    #[must_use]
    pub const fn crushed_m(self) -> f64 {
        self.crushed_m
    }

    /// Returns total plastic energy absorbed by the core.
    #[must_use]
    pub fn absorbed_energy_j(self) -> f64 {
        self.f_crush_n * self.crushed_m
    }

    /// Advances irreversible crushing by a supplied energy increment.
    ///
    /// The exact plateau identity is `delta_crushed = energy / f_crush`,
    /// capped by remaining stroke. This is the 1-D drop-test anchor for the
    /// crush stage.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when energy is negative or non-finite.
    pub fn absorb_energy_j(
        &mut self,
        energy_j: f64,
    ) -> Result<CrushCoreResponse, LandingGearError> {
        let energy_j = require_non_negative("energy_j", energy_j)?;
        let remaining_m = self.stroke_max_m - self.crushed_m;
        let requested_m = energy_j / self.f_crush_n;
        let delta_m = requested_m.min(remaining_m);
        self.crushed_m += delta_m;
        Ok(CrushCoreResponse {
            force_n: if delta_m > 0.0 { self.f_crush_n } else { 0.0 },
            crushed_m: self.crushed_m,
            delta_crushed_m: delta_m,
            absorbed_energy_j: delta_m * self.f_crush_n,
            stroke_limited: requested_m > remaining_m,
        })
    }

    /// Evaluates compressive force at total stroke without decreasing
    /// irreversible crushed stroke.
    ///
    /// # Errors
    ///
    /// Returns [`LandingGearError`] when stroke is invalid.
    pub fn force_at_stroke_n(&self, stroke_m: f64) -> Result<f64, LandingGearError> {
        let stroke_m = require_bounded("stroke_m", stroke_m, 0.0, self.stroke_max_m)?;
        let elastic_stroke_m = (stroke_m - self.crushed_m).max(0.0);
        Ok((self.k_elastic_n_m * elastic_stroke_m).min(self.f_crush_n))
    }
}

/// Output from [`CrushCore::absorb_energy_j`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrushCoreResponse {
    /// Plateau force applied while crushing.
    pub force_n: f64,
    /// Irreversible crushed coordinate after this update.
    pub crushed_m: f64,
    /// Incremental irreversible stroke from this update.
    pub delta_crushed_m: f64,
    /// Energy absorbed by this update.
    pub absorbed_energy_j: f64,
    /// `true` when the requested energy exceeded remaining crush stroke.
    pub stroke_limited: bool,
}

fn require_finite(field: &'static str, value: f64) -> Result<f64, LandingGearError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(LandingGearError::InvalidParameter {
            field,
            reason: "must be finite",
        })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<f64, LandingGearError> {
    let value = require_finite(field, value)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(LandingGearError::InvalidParameter {
            field,
            reason: "must be positive",
        })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<f64, LandingGearError> {
    let value = require_finite(field, value)?;
    if value >= 0.0 {
        Ok(value)
    } else {
        Err(LandingGearError::InvalidParameter {
            field,
            reason: "must be non-negative",
        })
    }
}

fn require_bounded(
    field: &'static str,
    value: f64,
    min: f64,
    max: f64,
) -> Result<f64, LandingGearError> {
    let value = require_finite(field, value)?;
    if (min..=max).contains(&value) {
        Ok(value)
    } else {
        Err(LandingGearError::InvalidParameter {
            field,
            reason: "must lie inside the configured stroke range",
        })
    }
}

fn finite_vector(field: &'static str, vector: [f64; 3]) -> Result<[f64; 3], LandingGearError> {
    if vector.iter().all(|component| component.is_finite()) {
        Ok(vector)
    } else {
        Err(LandingGearError::InvalidVector {
            field,
            reason: "all components must be finite",
        })
    }
}

fn unit_vector(field: &'static str, vector: [f64; 3]) -> Result<[f64; 3], LandingGearError> {
    let vector = finite_vector(field, vector)?;
    let norm = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
    if norm > 0.0 {
        Ok([vector[0] / norm, vector[1] / norm, vector[2] / norm])
    } else {
        Err(LandingGearError::InvalidVector {
            field,
            reason: "must have non-zero length",
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    use super::*;

    #[test]
    fn landing_gear_oleo_polytropic_curve_matches_closed_form() {
        let oleo = OleoStage::new(1.0e6, 0.02, 1.4, 250.0, 0.1, 0.05).unwrap();
        let stroke_m = 0.02;
        let volume_m3 = 0.02_f64 - 0.05_f64 * stroke_m;
        let expected_pressure_pa = 1.0e6_f64 * (0.02_f64 / volume_m3).powf(1.4);

        assert_relative_eq!(
            oleo.gas_pressure_pa(stroke_m).unwrap(),
            expected_pressure_pa,
            max_relative = 1.0e-15
        );
        assert_relative_eq!(
            oleo.force_n(stroke_m, 2.0).unwrap(),
            expected_pressure_pa * 0.05 + 250.0 * 4.0,
            max_relative = 1.0e-15
        );
    }

    #[test]
    fn landing_gear_oleo_stored_energy_matches_polytropic_integral() {
        let oleo = OleoStage::new(1.0e6, 0.02, 1.4, 0.0, 0.1, 0.05).unwrap();
        let stroke_m = 0.02;
        let volume_m3 = 0.02_f64 - 0.05_f64 * stroke_m;
        let expected_energy_j =
            1.0e6_f64 * 0.02_f64.powf(1.4) * (volume_m3.powf(-0.4) - 0.02_f64.powf(-0.4)) / 0.4;

        assert_relative_eq!(
            oleo.stored_energy_j(stroke_m).unwrap(),
            expected_energy_j,
            max_relative = 1.0e-12
        );
    }

    #[test]
    fn landing_gear_crush_energy_sets_plateau_stroke() {
        let mut crush = CrushCore::new(1000.0, 0.2, 10_000.0).unwrap();
        let response = crush.absorb_energy_j(100.0).unwrap();

        assert_abs_diff_eq!(response.force_n, 1000.0, epsilon = 0.0);
        assert_abs_diff_eq!(response.delta_crushed_m, 0.1, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.crushed_m, 0.1, epsilon = 1.0e-15);
        assert_abs_diff_eq!(response.absorbed_energy_j, 100.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(crush.absorbed_energy_j(), 100.0, epsilon = 1.0e-12);
        assert!(!response.stroke_limited);
    }

    #[test]
    fn landing_gear_crush_coordinate_is_monotone_and_stroke_limited() {
        let mut crush = CrushCore::new(1000.0, 0.12, 10_000.0).unwrap();
        let first = crush.absorb_energy_j(80.0).unwrap();
        let second = crush.absorb_energy_j(80.0).unwrap();
        let third = crush.absorb_energy_j(1.0).unwrap();

        assert!(second.crushed_m >= first.crushed_m);
        assert_abs_diff_eq!(second.crushed_m, 0.12, epsilon = 1.0e-15);
        assert_abs_diff_eq!(second.absorbed_energy_j, 40.0, epsilon = 1.0e-12);
        assert!(second.stroke_limited);
        assert_abs_diff_eq!(third.crushed_m, second.crushed_m, epsilon = 0.0);
        assert_abs_diff_eq!(third.absorbed_energy_j, 0.0, epsilon = 0.0);
        assert!(third.stroke_limited);
    }

    #[test]
    fn landing_gear_leg_normalizes_axis_and_requires_stage() {
        let crush = CrushCore::new(1000.0, 0.2, 10_000.0).unwrap();
        let leg = LandingGearLeg::new(
            [1.0, 2.0, 3.0],
            [0.0, 0.0, -2.0],
            None,
            Some(crush),
            ContactGeometry::Point,
        )
        .unwrap();

        assert_abs_diff_eq!(leg.attach_body_m()[0], 1.0, epsilon = 0.0);
        assert_abs_diff_eq!(leg.attach_body_m()[1], 2.0, epsilon = 0.0);
        assert_abs_diff_eq!(leg.attach_body_m()[2], 3.0, epsilon = 0.0);
        assert_abs_diff_eq!(leg.strut_axis_body()[0], 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(leg.strut_axis_body()[1], 0.0, epsilon = 0.0);
        assert_abs_diff_eq!(leg.strut_axis_body()[2], -1.0, epsilon = 0.0);
        assert_eq!(leg.footpad(), ContactGeometry::Point);
        assert!(
            LandingGearLeg::new(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                None,
                None,
                ContactGeometry::Point,
            )
            .is_err()
        );
    }
}
