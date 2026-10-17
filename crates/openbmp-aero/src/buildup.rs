//! Semi-empirical continuum drag buildup for slender launch vehicles.
//!
//! The model is a forward coefficient producer: declared vehicle
//! geometry plus a flow condition yields reduced `(CN, CD, CM)`
//! coefficients, or a fixed-grid [`AeroDeck`] suitable for the existing
//! tabulated hot path. It covers the subsonic, transonic, and low
//! supersonic continuum range and includes skin friction, forebody wave
//! drag, power-off base drag, boattail or flare afterbody drag, fin drag,
//! and the small-angle drag polar.
//!
//! The correlations are intentionally engineering-level and
//! provenance-pinned to open textbook relations.

#[cfg(not(feature = "std"))]
use alloc::{vec, vec::Vec};
use core::f64::consts::PI;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::deck::{AeroCoefficients, AeroDeck};
use crate::error::AeroError;

const RE_TRANSITION_DEFAULT: f64 = 5.0e5;
const MIN_REYNOLDS: f64 = 1.0e3;
const REYNOLDS_MACH_FLOOR: f64 = 1.0e-3;
const DEFAULT_BOATTAIL_SEPARATION_RAD: f64 = PI / 12.0;
const MAX_FLARE_ANGLE_RAD: f64 = PI / 9.0;
const PLANFORM_QUADRATURE_STEPS: usize = 64;
const NACA_1048_BODY_I_FINENESS: f64 = 21.1;
const NACA_1048_BODY_I_CROSSFLOW_DRAG_RATIO: f64 = 0.755;
const NACA_1048_BODY_II_FINENESS: f64 = 13.1;
const NACA_1048_BODY_II_CROSSFLOW_DRAG_RATIO: f64 = 0.692;

/// Nose / forebody shape used by pressure and wave-drag correlations.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum NoseShape {
    /// Conical forebody with the supplied half-angle.
    Conical {
        /// Cone half-angle, radians.
        half_angle_rad: f64,
    },
    /// Circular-ogive style forebody, parameterised by length/diameter.
    Ogive {
        /// Nose fineness ratio, `length / diameter`.
        fineness: f64,
    },
    /// Von Karman / Haack-like minimum-wave-drag forebody.
    VonKarman {
        /// Nose fineness ratio, `length / diameter`.
        fineness: f64,
    },
    /// Hemispherical blunt nose.
    Hemispherical,
}

/// Afterbody taper. `exit_diameter_m < body_diameter_m` is a boattail;
/// `exit_diameter_m > body_diameter_m` is a flare.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Afterbody {
    /// Exit or base diameter after the taper (m).
    pub exit_diameter_m: f64,
    /// Taper length (m).
    pub length_m: f64,
}

/// Single fin-set geometry for the component buildup.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FinSet {
    /// Number of identical fins.
    pub count: u32,
    /// Root chord (m).
    pub root_chord_m: f64,
    /// Tip chord (m).
    pub tip_chord_m: f64,
    /// Semispan from body surface to fin tip (m).
    pub span_m: f64,
    /// Maximum thickness divided by representative chord.
    pub thickness_ratio: f64,
    /// Leading-edge sweep angle (rad).
    pub sweep_rad: f64,
}

/// Equivalent axisymmetric protuberance drag declaration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ProtuberanceSet {
    /// Number of identical protuberances.
    pub count: u32,
    /// Frontal area of one protuberance (m^2).
    pub frontal_area_m2: f64,
    /// Isolated protuberance pressure-drag coefficient.
    pub drag_coefficient: f64,
    /// Optional installation/interference multiplier.
    pub interference_factor: f64,
}

/// Body-of-revolution plus optional fin-set geometry.
///
/// Every field is vehicle-intrinsic geometry or reference scaling. The
/// optional center of gravity is used only to form `CM`; when absent,
/// the body midpoint is used as a deterministic reference point.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BodyGeometry {
    /// Maximum body diameter (m).
    pub body_diameter_m: f64,
    /// Total body length including nose and afterbody (m).
    pub body_length_m: f64,
    /// Nose / forebody shape.
    pub nose: NoseShape,
    /// Optional afterbody taper.
    pub afterbody: Option<Afterbody>,
    /// Equivalent sand-grain roughness height (m).
    pub surface_roughness_m: f64,
    /// Optional fin set.
    pub fins: Option<FinSet>,
    /// Optional equivalent axisymmetric protuberance drag set.
    pub protuberances: Option<ProtuberanceSet>,
    /// Coefficient reference area (m^2), conventionally max cross-section.
    pub reference_area_m2: f64,
    /// Moment reference length (m), conventionally body diameter.
    pub reference_length_m: f64,
    /// Optional center of gravity from nose tip (m), for `CM`.
    pub center_of_gravity_from_nose_m: Option<f64>,
}

/// Flow condition for one buildup evaluation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FlowCondition {
    /// Freestream Mach number.
    pub mach: f64,
    /// Reynolds number based on body reference length.
    pub reynolds_length: f64,
    /// Angle of attack (rad).
    pub alpha_rad: f64,
}

/// Fixed grid used when baking a buildup into an [`AeroDeck`].
#[derive(Clone, Debug, PartialEq)]
pub struct BuildupGrid {
    /// Mach breakpoints.
    pub mach: Vec<f64>,
    /// Angle-of-attack breakpoints (deg).
    pub alpha_deg: Vec<f64>,
    /// Sideslip breakpoints (deg). The current reduced buildup
    /// supports only a zero-sideslip grid.
    pub beta_deg: Vec<f64>,
    /// Reference Reynolds number at Mach 1 for the declared atmosphere
    /// and reference length.
    pub reference_reynolds_at_mach1: f64,
}

impl BuildupGrid {
    /// Construct a grid from inclusive min/max ranges and step counts.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::InvalidParameter`] for non-finite bounds,
    /// fewer than one step, decreasing ranges, non-zero beta bounds, or
    /// non-positive reference Reynolds number.
    pub fn from_ranges(
        mach_min: f64,
        mach_max: f64,
        mach_steps: usize,
        alpha_min_deg: f64,
        alpha_max_deg: f64,
        alpha_steps: usize,
        reference_reynolds_at_mach1: f64,
    ) -> Result<Self, AeroError> {
        Ok(Self {
            mach: inclusive_grid(mach_min, mach_max, mach_steps, "mach grid")?,
            alpha_deg: inclusive_grid(alpha_min_deg, alpha_max_deg, alpha_steps, "alpha grid")?,
            beta_deg: vec![0.0],
            reference_reynolds_at_mach1,
        })
    }

    fn validate(&self) -> Result<(), AeroError> {
        validate_axis(&self.mach, "buildup mach grid")?;
        validate_axis(&self.alpha_deg, "buildup alpha grid")?;
        validate_axis(&self.beta_deg, "buildup beta grid")?;
        for &beta in &self.beta_deg {
            if beta.abs() > 1.0e-12 {
                return Err(AeroError::InvalidParameter {
                    reason: "buildup beta grid is reserved at zero",
                });
            }
        }
        if !self.reference_reynolds_at_mach1.is_finite() || self.reference_reynolds_at_mach1 <= 0.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "buildup reference Reynolds number must be positive",
            });
        }
        Ok(())
    }
}

/// Semi-empirical continuum drag buildup interface.
pub trait DragBuildupModel {
    /// Evaluate one flow condition.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry or flow is outside the
    /// correlation envelope, or when arithmetic produces a non-finite
    /// coefficient.
    fn coefficients(
        &self,
        geom: &BodyGeometry,
        flow: &FlowCondition,
    ) -> Result<AeroCoefficients, AeroError>;

    /// Bake an [`AeroDeck`] over a fixed Mach/alpha grid.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if any grid point fails the coefficient
    /// calculation or the resulting deck fails structural validation.
    fn bake_deck(&self, geom: &BodyGeometry, grid: &BuildupGrid) -> Result<AeroDeck, AeroError>;
}

/// Component drag buildup implementation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ComponentBuildup {
    /// Reynolds number separating laminar and turbulent flat-plate
    /// friction.
    pub transition_reynolds: f64,
    /// Maximum admitted boattail half-angle before separation is
    /// assumed outside the correlation envelope.
    pub boattail_separation_angle_rad: f64,
}

/// Opt-in DATCOM-style buildup extension.
///
/// The base [`ComponentBuildup`] remains the legacy linear normal-force
/// model. This wrapper preserves the legacy validation and deck-baking path,
/// replaces the forebody wave-drag term with a bounded DATCOM-style
/// area-rule term, replaces the legacy fin interference multiplier with
/// bounded DATCOM-style fin/body carryover factors, then adds a bounded
/// Allen-Perkins-style crossflow normal-force increment for slender bodies at
/// finite angle of attack.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DatcomComponentBuildup {
    base: ComponentBuildup,
}

impl Default for ComponentBuildup {
    fn default() -> Self {
        Self {
            transition_reynolds: RE_TRANSITION_DEFAULT,
            boattail_separation_angle_rad: DEFAULT_BOATTAIL_SEPARATION_RAD,
        }
    }
}

impl ComponentBuildup {
    /// Construct the opt-in DATCOM-style extension using this base model's
    /// drag and envelope settings.
    #[must_use]
    pub const fn datcom(self) -> DatcomComponentBuildup {
        DatcomComponentBuildup { base: self }
    }

    /// Power-off base-drag coefficient per base area.
    ///
    /// This is the Hoerner/Niskanen engineering correlation used by
    /// the buildup before scaling by `A_base / S_ref`.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::InvalidParameter`] for negative Mach and
    /// [`AeroError::NonFinite`] for non-finite Mach.
    pub fn base_drag_coefficient(mach: f64) -> Result<f64, AeroError> {
        if !mach.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "base-drag Mach is NaN or Inf",
            });
        }
        if mach < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "base-drag Mach must be non-negative",
            });
        }
        if mach < 1.0 {
            Ok(0.12 + 0.13 * mach * mach)
        } else {
            Ok(0.25 / mach)
        }
    }

    /// Smooth or rough-wall flat-plate friction coefficient before
    /// compressibility correction.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] if Reynolds number or roughness ratio is
    /// non-finite or outside the positive envelope.
    pub fn skin_friction_coefficient(
        reynolds_length: f64,
        roughness_over_length: f64,
        transition_reynolds: f64,
    ) -> Result<f64, AeroError> {
        if !(reynolds_length.is_finite()
            && roughness_over_length.is_finite()
            && transition_reynolds.is_finite())
        {
            return Err(AeroError::NonFinite {
                reason: "skin-friction input is NaN or Inf",
            });
        }
        if reynolds_length < MIN_REYNOLDS || transition_reynolds <= 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "skin-friction Reynolds number is outside envelope",
            });
        }
        if roughness_over_length < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "surface roughness must be non-negative",
            });
        }
        let smooth = if reynolds_length < transition_reynolds {
            1.328 / reynolds_length.sqrt()
        } else {
            0.455 / reynolds_length.log10().powf(2.58)
        };
        let rough = if roughness_over_length > 0.0 {
            0.032 * roughness_over_length.powf(0.2)
        } else {
            0.0
        };
        Ok(smooth.max(rough))
    }

    fn zero_lift_drag(&self, geom: &BodyGeometry, flow: &FlowCondition) -> Result<f64, AeroError> {
        let cf = Self::skin_friction_coefficient(
            flow.reynolds_length,
            geom.surface_roughness_m / geom.reference_length_m,
            self.transition_reynolds,
        )?;
        let cfc = compressibility_corrected_cf(cf, flow.mach);
        let wetted_body = body_wetted_area_m2(geom)?;
        let fin_wetted = geom.fins.map_or(0.0, fin_wetted_area_m2);
        let fineness = geom.body_length_m / geom.body_diameter_m;
        let body_friction =
            cfc * (1.0 + 1.0 / (2.0 * fineness)) * (wetted_body / geom.reference_area_m2);
        let fin_friction = geom.fins.map_or(0.0, |fins| {
            cfc * (1.0 + 2.0 * fins.thickness_ratio) * (fin_wetted / geom.reference_area_m2)
        });
        let forebody = forebody_pressure_wave_drag(geom, flow.mach)?;
        let afterbody = afterbody_pressure_drag(geom, flow.mach, self)?;
        let base = base_drag_ref(geom, flow.mach)?;
        let fin_pressure = geom
            .fins
            .map_or(Ok(0.0), |fins| fin_pressure_drag(geom, fins, flow.mach))?;
        let interference = geom.fins.map_or(0.0, |fins| {
            0.01 * f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2
        });
        let cd0 = body_friction
            + fin_friction
            + forebody
            + afterbody
            + base
            + fin_pressure
            + interference;
        if !cd0.is_finite() || cd0 <= 0.0 {
            return Err(AeroError::NonFinite {
                reason: "component buildup produced invalid zero-lift drag",
            });
        }
        Ok(cd0)
    }

    fn normal_force_slope_and_cp(
        &self,
        geom: &BodyGeometry,
        mach: f64,
    ) -> Result<(f64, f64), AeroError> {
        let nose_slope = 2.0;
        let nose_cp = nose_center_of_pressure_m(geom)?;
        let (fin_slope, fin_cp) = if let Some(fins) = geom.fins {
            (
                fin_normal_force_slope(geom, fins, mach)?,
                fin_cp_from_nose_m(geom, fins),
            )
        } else {
            (0.0, 0.0)
        };
        let total = nose_slope + fin_slope;
        if !total.is_finite() || total <= 0.0 {
            return Err(AeroError::NonFinite {
                reason: "normal-force slope is invalid",
            });
        }
        let moment_sum = nose_slope * nose_cp + fin_slope * fin_cp;
        let cp = moment_sum / total;
        if !cp.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "center of pressure is invalid",
            });
        }
        Ok((total, cp))
    }
}

impl Default for DatcomComponentBuildup {
    fn default() -> Self {
        ComponentBuildup::default().datcom()
    }
}

impl DatcomComponentBuildup {
    /// Construct the DATCOM-style wrapper from an existing base buildup model.
    #[must_use]
    pub const fn new(base: ComponentBuildup) -> Self {
        Self { base }
    }

    /// Return the wrapped legacy component model.
    #[must_use]
    pub const fn base(self) -> ComponentBuildup {
        self.base
    }

    /// Bounded DATCOM-style fin-on-body and body-on-fin carryover factors.
    ///
    /// The first value corresponds to a fin lifting surface mounted on a
    /// cylindrical body, and the second value corresponds to the body
    /// lift carried by the fin set. Both approach 1.0 as body radius becomes
    /// small compared with fin span.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when the geometry inputs are non-finite or
    /// outside the finite positive envelope.
    pub fn fin_body_carryover_factors(
        geom: &BodyGeometry,
        fins: FinSet,
    ) -> Result<(f64, f64), AeroError> {
        datcom_fin_body_carryover_factors(geom, fins)
    }

    /// DATCOM-style Allen-Perkins crossflow normal-force increment.
    ///
    /// Returns `(CN_crossflow, x_cp_m)` using the same bounded relation
    /// consumed by [`Self::coefficients`]. This public method exists so
    /// validation fixtures can check the nonlinear crossflow term without
    /// depending on the private static-normal-force and drag buildup pieces.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry or flow inputs are outside the
    /// finite model envelope.
    pub fn crossflow_normal_force_increment(
        geom: &BodyGeometry,
        flow: &FlowCondition,
    ) -> Result<(f64, f64), AeroError> {
        validate_geometry(geom, &ComponentBuildup::default())?;
        validate_flow(flow)?;
        allen_perkins_crossflow_normal_force(geom, flow)
    }

    /// DATCOM-style area-rule forebody wave-drag coefficient.
    ///
    /// Returns the same bounded Sears-Haack-style reduced coefficient used by
    /// the opt-in DATCOM buildup zero-lift drag path.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry or Mach inputs are outside the
    /// finite model envelope.
    pub fn area_rule_wave_drag_coefficient(
        geom: &BodyGeometry,
        mach: f64,
    ) -> Result<f64, AeroError> {
        validate_geometry(geom, &ComponentBuildup::default())?;
        if !mach.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "DATCOM area-rule wave-drag Mach is NaN or Inf",
            });
        }
        if mach < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM area-rule wave-drag Mach must be non-negative",
            });
        }
        datcom_forebody_area_rule_wave_drag(geom, mach)
    }

    /// DATCOM-style equivalent-axisymmetric protuberance drag coefficient.
    ///
    /// Returns the same bounded reduced coefficient used by the opt-in DATCOM
    /// buildup zero-lift drag path. A geometry with no protuberance declaration
    /// returns zero after the normal geometry envelope checks pass.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when geometry or Mach inputs are outside the
    /// finite model envelope.
    pub fn protuberance_drag_coefficient(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
        validate_geometry(geom, &ComponentBuildup::default())?;
        if !mach.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "DATCOM protuberance-drag Mach is NaN or Inf",
            });
        }
        if mach < 0.0 {
            return Err(AeroError::InvalidParameter {
                reason: "DATCOM protuberance-drag Mach must be non-negative",
            });
        }
        datcom_protuberance_drag(geom, mach)
    }

    fn zero_lift_drag(&self, geom: &BodyGeometry, flow: &FlowCondition) -> Result<f64, AeroError> {
        let cf = ComponentBuildup::skin_friction_coefficient(
            flow.reynolds_length,
            geom.surface_roughness_m / geom.reference_length_m,
            self.base.transition_reynolds,
        )?;
        let cfc = compressibility_corrected_cf(cf, flow.mach);
        let wetted_body = body_wetted_area_m2(geom)?;
        let fin_wetted = geom.fins.map_or(0.0, fin_wetted_area_m2);
        let fineness = geom.body_length_m / geom.body_diameter_m;
        let body_friction =
            cfc * (1.0 + 1.0 / (2.0 * fineness)) * (wetted_body / geom.reference_area_m2);
        let fin_friction = geom.fins.map_or(0.0, |fins| {
            cfc * (1.0 + 2.0 * fins.thickness_ratio) * (fin_wetted / geom.reference_area_m2)
        });
        let forebody = datcom_forebody_area_rule_wave_drag(geom, flow.mach)?;
        let afterbody = afterbody_pressure_drag(geom, flow.mach, &self.base)?;
        let base = base_drag_ref(geom, flow.mach)?;
        let fin_pressure = geom
            .fins
            .map_or(Ok(0.0), |fins| fin_pressure_drag(geom, fins, flow.mach))?;
        let protuberance = datcom_protuberance_drag(geom, flow.mach)?;
        let interference = geom.fins.map_or(0.0, |fins| {
            0.01 * f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2
        });
        let cd0 = body_friction
            + fin_friction
            + forebody
            + afterbody
            + base
            + fin_pressure
            + protuberance
            + interference;
        if !cd0.is_finite() || cd0 <= 0.0 {
            return Err(AeroError::NonFinite {
                reason: "DATCOM buildup produced invalid zero-lift drag",
            });
        }
        Ok(cd0)
    }

    fn normal_force_and_cp(
        &self,
        geom: &BodyGeometry,
        flow: &FlowCondition,
    ) -> Result<(f64, f64), AeroError> {
        let (cn_alpha, linear_cp_m) = datcom_linear_normal_force_slope_and_cp(geom, flow.mach)?;
        let linear_cn = cn_alpha * flow.alpha_rad;
        let (crossflow_cn, crossflow_cp_m) = allen_perkins_crossflow_normal_force(geom, flow)?;
        let cn = linear_cn + crossflow_cn;
        let cp = if cn.abs() > 1.0e-14 {
            (linear_cn * linear_cp_m + crossflow_cn * crossflow_cp_m) / cn
        } else {
            linear_cp_m
        };
        if !(cn.is_finite() && cp.is_finite()) {
            return Err(AeroError::NonFinite {
                reason: "DATCOM buildup normal force is invalid",
            });
        }
        Ok((cn, cp))
    }
}

impl DragBuildupModel for ComponentBuildup {
    fn coefficients(
        &self,
        geom: &BodyGeometry,
        flow: &FlowCondition,
    ) -> Result<AeroCoefficients, AeroError> {
        validate_geometry(geom, self)?;
        validate_flow(flow)?;
        let cd0 = self.zero_lift_drag(geom, flow)?;
        let (cn_alpha, x_cp_m) = self.normal_force_slope_and_cp(geom, flow.mach)?;
        let alpha = flow.alpha_rad;
        let cn = cn_alpha * alpha;
        let cd = cd0 * alpha.cos() + cn * alpha.sin();
        let x_cg_m = geom
            .center_of_gravity_from_nose_m
            .unwrap_or(0.5 * geom.body_length_m);
        let cm = -cn * (x_cp_m - x_cg_m) / geom.reference_length_m;
        if !(cn.is_finite() && cd.is_finite() && cm.is_finite()) || cd <= 0.0 {
            return Err(AeroError::NonFinite {
                reason: "component buildup coefficient is invalid",
            });
        }
        Ok(AeroCoefficients { cn, cd, cm })
    }

    fn bake_deck(&self, geom: &BodyGeometry, grid: &BuildupGrid) -> Result<AeroDeck, AeroError> {
        validate_geometry(geom, self)?;
        grid.validate()?;
        let expected = grid
            .mach
            .len()
            .checked_mul(grid.alpha_deg.len())
            .and_then(|n| n.checked_mul(grid.beta_deg.len()))
            .ok_or(AeroError::MalformedDeck {
                reason: "buildup grid size overflows usize",
            })?;
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        for &mach in &grid.mach {
            let reynolds_length = (grid.reference_reynolds_at_mach1
                * mach.max(REYNOLDS_MACH_FLOOR))
            .max(MIN_REYNOLDS);
            for &alpha_deg in &grid.alpha_deg {
                for &_beta_deg in &grid.beta_deg {
                    let coeffs = self.coefficients(
                        geom,
                        &FlowCondition {
                            mach,
                            reynolds_length,
                            alpha_rad: alpha_deg.to_radians(),
                        },
                    )?;
                    cn.push(coeffs.cn);
                    cd.push(coeffs.cd);
                    cm.push(coeffs.cm);
                }
            }
        }
        AeroDeck::new(
            grid.mach.clone(),
            grid.alpha_deg.clone(),
            grid.beta_deg.clone(),
            cn,
            cd,
            cm,
            geom.reference_area_m2,
            geom.reference_length_m,
        )
    }
}

impl DragBuildupModel for DatcomComponentBuildup {
    fn coefficients(
        &self,
        geom: &BodyGeometry,
        flow: &FlowCondition,
    ) -> Result<AeroCoefficients, AeroError> {
        validate_geometry(geom, &self.base)?;
        validate_flow(flow)?;
        let cd0 = self.zero_lift_drag(geom, flow)?;
        let (cn, x_cp_m) = self.normal_force_and_cp(geom, flow)?;
        let cd = cd0 + datcom_incremental_foredrag(geom, flow)?;
        let x_cg_m = geom
            .center_of_gravity_from_nose_m
            .unwrap_or(0.5 * geom.body_length_m);
        let cm = -cn * (x_cp_m - x_cg_m) / geom.reference_length_m;
        if !(cn.is_finite() && cd.is_finite() && cm.is_finite()) || cd <= 0.0 {
            return Err(AeroError::NonFinite {
                reason: "DATCOM buildup coefficient is invalid",
            });
        }
        Ok(AeroCoefficients { cn, cd, cm })
    }

    fn bake_deck(&self, geom: &BodyGeometry, grid: &BuildupGrid) -> Result<AeroDeck, AeroError> {
        validate_geometry(geom, &self.base)?;
        grid.validate()?;
        let expected = grid
            .mach
            .len()
            .checked_mul(grid.alpha_deg.len())
            .and_then(|n| n.checked_mul(grid.beta_deg.len()))
            .ok_or(AeroError::MalformedDeck {
                reason: "DATCOM buildup grid size overflows usize",
            })?;
        let mut cn = Vec::with_capacity(expected);
        let mut cd = Vec::with_capacity(expected);
        let mut cm = Vec::with_capacity(expected);
        let mut cm_q = Vec::with_capacity(expected);
        let mut cn_r = Vec::with_capacity(expected);
        let mut cl_p = Vec::with_capacity(expected);
        for &mach in &grid.mach {
            let reynolds_length = (grid.reference_reynolds_at_mach1
                * mach.max(REYNOLDS_MACH_FLOOR))
            .max(MIN_REYNOLDS);
            let derivatives = datcom_dynamic_derivatives(geom, mach)?;
            for &alpha_deg in &grid.alpha_deg {
                for &_beta_deg in &grid.beta_deg {
                    let coeffs = self.coefficients(
                        geom,
                        &FlowCondition {
                            mach,
                            reynolds_length,
                            alpha_rad: alpha_deg.to_radians(),
                        },
                    )?;
                    cn.push(coeffs.cn);
                    cd.push(coeffs.cd);
                    cm.push(coeffs.cm);
                    cm_q.push(derivatives.cm_q);
                    cn_r.push(derivatives.cn_r);
                    cl_p.push(derivatives.cl_p);
                }
            }
        }
        let deck = AeroDeck::new(
            grid.mach.clone(),
            grid.alpha_deg.clone(),
            grid.beta_deg.clone(),
            cn,
            cd,
            cm,
            geom.reference_area_m2,
            geom.reference_length_m,
        )?;
        deck.with_dynamic_derivative_tables(cm_q, cn_r, cl_p)
    }
}

fn validate_geometry(geom: &BodyGeometry, model: &ComponentBuildup) -> Result<(), AeroError> {
    for value in [
        geom.body_diameter_m,
        geom.body_length_m,
        geom.surface_roughness_m,
        geom.reference_area_m2,
        geom.reference_length_m,
    ] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "buildup geometry contains NaN or Inf",
            });
        }
    }
    if geom.body_diameter_m <= 0.0
        || geom.body_length_m <= geom.body_diameter_m
        || geom.reference_area_m2 <= 0.0
        || geom.reference_length_m <= 0.0
        || geom.surface_roughness_m < 0.0
    {
        return Err(AeroError::InvalidParameter {
            reason: "buildup body geometry is outside envelope",
        });
    }
    validate_nose(geom.nose)?;
    let nose_length = nose_length_m(geom)?;
    if nose_length >= geom.body_length_m {
        return Err(AeroError::InvalidParameter {
            reason: "buildup nose length exceeds body length",
        });
    }
    if let Some(afterbody) = geom.afterbody {
        validate_afterbody(geom, afterbody, model)?;
    }
    if let Some(fins) = geom.fins {
        validate_fins(geom, fins)?;
    }
    if let Some(protuberances) = geom.protuberances {
        validate_protuberances(geom, protuberances)?;
    }
    if let Some(x_cg) = geom.center_of_gravity_from_nose_m
        && (!x_cg.is_finite() || !(0.0..=geom.body_length_m).contains(&x_cg))
    {
        return Err(AeroError::InvalidParameter {
            reason: "buildup center of gravity is outside body length",
        });
    }
    if !(model.transition_reynolds.is_finite()
        && model.transition_reynolds > 0.0
        && model.boattail_separation_angle_rad.is_finite()
        && model.boattail_separation_angle_rad > 0.0)
    {
        return Err(AeroError::InvalidParameter {
            reason: "buildup model constants are outside envelope",
        });
    }
    Ok(())
}

fn validate_nose(nose: NoseShape) -> Result<(), AeroError> {
    match nose {
        NoseShape::Conical { half_angle_rad } => {
            if !half_angle_rad.is_finite()
                || half_angle_rad <= 1.0_f64.to_radians()
                || half_angle_rad >= 80.0_f64.to_radians()
            {
                return Err(AeroError::InvalidParameter {
                    reason: "conical nose half-angle is outside envelope",
                });
            }
        }
        NoseShape::Ogive { fineness } | NoseShape::VonKarman { fineness } => {
            if !fineness.is_finite() || !(0.5..=10.0).contains(&fineness) {
                return Err(AeroError::InvalidParameter {
                    reason: "nose fineness is outside envelope",
                });
            }
        }
        NoseShape::Hemispherical => {}
    }
    Ok(())
}

fn validate_afterbody(
    geom: &BodyGeometry,
    afterbody: Afterbody,
    model: &ComponentBuildup,
) -> Result<(), AeroError> {
    if !(afterbody.exit_diameter_m.is_finite() && afterbody.length_m.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "afterbody geometry contains NaN or Inf",
        });
    }
    if afterbody.exit_diameter_m <= 0.0 || afterbody.length_m <= 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "afterbody dimensions must be positive",
        });
    }
    let half_angle = afterbody_half_angle_rad(geom.body_diameter_m, afterbody);
    if afterbody.exit_diameter_m < geom.body_diameter_m
        && half_angle > model.boattail_separation_angle_rad
    {
        return Err(AeroError::OutOfEnvelope {
            reason: "boattail half-angle exceeds attached-flow envelope",
        });
    }
    if afterbody.exit_diameter_m > geom.body_diameter_m && half_angle > MAX_FLARE_ANGLE_RAD {
        return Err(AeroError::OutOfEnvelope {
            reason: "flare half-angle exceeds afterbody envelope",
        });
    }
    Ok(())
}

fn validate_fins(geom: &BodyGeometry, fins: FinSet) -> Result<(), AeroError> {
    for value in [
        fins.root_chord_m,
        fins.tip_chord_m,
        fins.span_m,
        fins.thickness_ratio,
        fins.sweep_rad,
    ] {
        if !value.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "fin geometry contains NaN or Inf",
            });
        }
    }
    if fins.count == 0
        || fins.root_chord_m <= 0.0
        || fins.tip_chord_m < 0.0
        || fins.span_m <= 0.0
        || !(0.001..=0.2).contains(&fins.thickness_ratio)
        || fins.root_chord_m > geom.body_length_m
        || fins.span_m > geom.body_length_m
    {
        return Err(AeroError::InvalidParameter {
            reason: "fin geometry is outside buildup envelope",
        });
    }
    Ok(())
}

fn validate_protuberances(
    geom: &BodyGeometry,
    protuberances: ProtuberanceSet,
) -> Result<(), AeroError> {
    if protuberances.count == 0
        || !(protuberances.frontal_area_m2.is_finite()
            && protuberances.drag_coefficient.is_finite()
            && protuberances.interference_factor.is_finite())
        || protuberances.frontal_area_m2 <= 0.0
        || protuberances.drag_coefficient <= 0.0
        || protuberances.interference_factor <= 0.0
    {
        return Err(AeroError::InvalidParameter {
            reason: "protuberance geometry is outside buildup envelope",
        });
    }
    let total_area = f64::from(protuberances.count) * protuberances.frontal_area_m2;
    if total_area >= geom.reference_area_m2 {
        return Err(AeroError::InvalidParameter {
            reason: "protuberance frontal area must be smaller than reference area",
        });
    }
    Ok(())
}

fn validate_flow(flow: &FlowCondition) -> Result<(), AeroError> {
    if !(flow.mach.is_finite() && flow.reynolds_length.is_finite() && flow.alpha_rad.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "buildup flow contains NaN or Inf",
        });
    }
    if flow.mach < 0.0 || flow.reynolds_length < MIN_REYNOLDS {
        return Err(AeroError::InvalidParameter {
            reason: "buildup flow is outside envelope",
        });
    }
    if flow.alpha_rad.abs() > 20.0_f64.to_radians() {
        return Err(AeroError::OutOfEnvelope {
            reason: "linear drag polar angle of attack exceeds envelope",
        });
    }
    Ok(())
}

fn inclusive_grid(
    min: f64,
    max: f64,
    steps: usize,
    reason: &'static str,
) -> Result<Vec<f64>, AeroError> {
    if !(min.is_finite() && max.is_finite()) {
        return Err(AeroError::NonFinite { reason });
    }
    if steps == 0 || max < min {
        return Err(AeroError::InvalidParameter { reason });
    }
    if steps == 1 {
        return Ok(vec![min]);
    }
    let denom = (steps - 1) as f64;
    let step = (max - min) / denom;
    let mut out = Vec::with_capacity(steps);
    for i in 0..steps {
        out.push(min + step * i as f64);
    }
    Ok(out)
}

fn validate_axis(axis: &[f64], reason: &'static str) -> Result<(), AeroError> {
    if axis.is_empty() {
        return Err(AeroError::MalformedDeck { reason });
    }
    for &value in axis {
        if !value.is_finite() {
            return Err(AeroError::NonFinite { reason });
        }
    }
    for pair in axis.windows(2) {
        if pair[1] <= pair[0] {
            return Err(AeroError::MalformedDeck { reason });
        }
    }
    Ok(())
}

fn compressibility_corrected_cf(cf: f64, mach: f64) -> f64 {
    if mach < 1.0 {
        cf * (1.0 - 0.1 * mach * mach).max(0.1)
    } else {
        cf / (1.0 + 0.15 * mach * mach).powf(0.58)
    }
}

fn body_wetted_area_m2(geom: &BodyGeometry) -> Result<f64, AeroError> {
    let d = geom.body_diameter_m;
    let r = 0.5 * d;
    let nose_len = nose_length_m(geom)?;
    let after_len = geom.afterbody.map_or(0.0, |a| a.length_m);
    let cylinder_len = (geom.body_length_m - nose_len - after_len).max(0.0);
    let cylinder = PI * d * cylinder_len;
    let nose = match geom.nose {
        NoseShape::Conical { .. } => PI * r * (r * r + nose_len * nose_len).sqrt(),
        NoseShape::Ogive { .. } | NoseShape::VonKarman { .. } => 0.80 * PI * d * nose_len,
        NoseShape::Hemispherical => 2.0 * PI * r * r,
    };
    let afterbody = geom.afterbody.map_or(0.0, |a| {
        let rb = 0.5 * a.exit_diameter_m;
        let slant = (a.length_m * a.length_m + (r - rb) * (r - rb)).sqrt();
        PI * (r + rb) * slant
    });
    Ok(cylinder + nose + afterbody)
}

fn nose_length_m(geom: &BodyGeometry) -> Result<f64, AeroError> {
    let d = geom.body_diameter_m;
    let r = 0.5 * d;
    match geom.nose {
        NoseShape::Conical { half_angle_rad } => Ok(r / half_angle_rad.tan()),
        NoseShape::Ogive { fineness } | NoseShape::VonKarman { fineness } => Ok(fineness * d),
        NoseShape::Hemispherical => Ok(r),
    }
}

fn nose_center_of_pressure_m(geom: &BodyGeometry) -> Result<f64, AeroError> {
    let length = nose_length_m(geom)?;
    Ok(match geom.nose {
        NoseShape::Conical { .. } => (2.0 / 3.0) * length,
        NoseShape::Ogive { .. } | NoseShape::VonKarman { .. } => 0.466 * length,
        NoseShape::Hemispherical => 0.5 * length,
    })
}

fn forebody_pressure_wave_drag(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let supersonic_at_12 = supersonic_nose_wave_drag(geom, 1.2)?;
    if mach < 0.8 {
        return Ok(blunt_subsonic_pressure_drag(geom, mach));
    }
    if mach < 1.2 {
        let t = (mach - 0.8) / 0.4;
        let s = t * t * (3.0 - 2.0 * t);
        return Ok(blunt_subsonic_pressure_drag(geom, mach) * (1.0 - s) + supersonic_at_12 * s);
    }
    supersonic_nose_wave_drag(geom, mach)
}

fn datcom_forebody_area_rule_wave_drag(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let supersonic_at_12 = sears_haack_area_rule_wave_drag(geom, 1.2)?;
    if mach < 0.8 {
        return Ok(blunt_subsonic_pressure_drag(geom, mach));
    }
    if mach < 1.2 {
        let t = (mach - 0.8) / 0.4;
        let s = t * t * (3.0 - 2.0 * t);
        return Ok(blunt_subsonic_pressure_drag(geom, mach) * (1.0 - s) + supersonic_at_12 * s);
    }
    sears_haack_area_rule_wave_drag(geom, mach)
}

fn blunt_subsonic_pressure_drag(geom: &BodyGeometry, mach: f64) -> f64 {
    if matches!(geom.nose, NoseShape::Hemispherical) {
        0.03 * (1.0 + mach * mach)
    } else {
        0.0
    }
}

fn supersonic_nose_wave_drag(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let beta = (mach * mach - 1.0).max(0.2).sqrt();
    let length = nose_length_m(geom)?;
    let fineness = (length / geom.body_diameter_m).max(0.5);
    let base = match geom.nose {
        NoseShape::Conical { half_angle_rad } => 2.1 * half_angle_rad.sin().powi(2),
        NoseShape::Ogive { .. } => 0.52 / (fineness * fineness),
        NoseShape::VonKarman { .. } => 0.38 / (fineness * fineness),
        NoseShape::Hemispherical => 0.18,
    };
    Ok((base / beta).max(0.0))
}

fn sears_haack_area_rule_wave_drag(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let beta = (mach * mach - 1.0).max(0.2).sqrt();
    let length = nose_length_m(geom)?;
    let fineness = (length / geom.body_diameter_m).max(0.5);
    let base = match geom.nose {
        NoseShape::Conical { half_angle_rad } => 1.75 * half_angle_rad.sin().powi(2),
        NoseShape::Ogive { .. } => 0.44 / (fineness * fineness),
        NoseShape::VonKarman { .. } => 0.30 / (fineness * fineness),
        NoseShape::Hemispherical => 0.18,
    };
    Ok((base / beta).max(0.0))
}

fn datcom_protuberance_drag(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let Some(protuberances) = geom.protuberances else {
        return Ok(0.0);
    };
    let area_ratio =
        f64::from(protuberances.count) * protuberances.frontal_area_m2 / geom.reference_area_m2;
    let drag = protuberances.drag_coefficient
        * protuberances.interference_factor
        * area_ratio
        * protuberance_mach_factor(mach);
    if !drag.is_finite() || drag < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "DATCOM protuberance drag is invalid",
        });
    }
    Ok(drag)
}

fn datcom_dynamic_derivatives(
    geom: &BodyGeometry,
    mach: f64,
) -> Result<crate::deck::AeroDynamicDerivatives, AeroError> {
    let (cn_alpha, x_cp_m) = datcom_linear_normal_force_slope_and_cp(geom, mach)?;
    let x_cg_m = geom
        .center_of_gravity_from_nose_m
        .unwrap_or(0.5 * geom.body_length_m);
    let moment_arm = (x_cp_m - x_cg_m) / geom.reference_length_m;
    let pitch_yaw_damping = -2.0 * cn_alpha * moment_arm * moment_arm;
    let cl_p = datcom_roll_damping_derivative(geom, mach)?;
    if !(pitch_yaw_damping.is_finite() && cl_p.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "DATCOM dynamic derivatives are invalid",
        });
    }
    Ok(crate::deck::AeroDynamicDerivatives {
        cl_p,
        cl_q: 0.0,
        cl_r: 0.0,
        cm_p: 0.0,
        cm_q: pitch_yaw_damping,
        cm_r: 0.0,
        cn_p: 0.0,
        cn_q: 0.0,
        cn_r: pitch_yaw_damping,
    })
}

fn datcom_roll_damping_derivative(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let Some(fins) = geom.fins else {
        return Ok(0.0);
    };
    let fin_slope = fin_lifting_surface_normal_force_slope(fins, mach)?;
    let area_ratio = f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2;
    let span_ratio = fins.span_m / geom.reference_length_m;
    let damping = -0.25 * fin_slope * area_ratio * span_ratio * span_ratio;
    if !damping.is_finite() || damping > 0.0 {
        return Err(AeroError::NonFinite {
            reason: "DATCOM roll damping derivative is invalid",
        });
    }
    Ok(damping)
}

fn protuberance_mach_factor(mach: f64) -> f64 {
    if mach < 0.8 {
        1.0 + 0.15 * mach * mach
    } else if mach < 1.2 {
        let lo = 1.0 + 0.15 * 0.8_f64 * 0.8;
        let hi = 1.2;
        let t = (mach - 0.8) / 0.4;
        let s = t * t * (3.0 - 2.0 * t);
        lo * (1.0 - s) + hi * s
    } else {
        1.2 + 0.05 * (mach - 1.2).min(6.8)
    }
}

fn base_drag_ref(geom: &BodyGeometry, mach: f64) -> Result<f64, AeroError> {
    let base_diameter = geom
        .afterbody
        .map_or(geom.body_diameter_m, |afterbody| afterbody.exit_diameter_m);
    let base_area = PI * base_diameter * base_diameter / 4.0;
    Ok(ComponentBuildup::base_drag_coefficient(mach)? * base_area / geom.reference_area_m2)
}

fn afterbody_pressure_drag(
    geom: &BodyGeometry,
    mach: f64,
    model: &ComponentBuildup,
) -> Result<f64, AeroError> {
    let Some(afterbody) = geom.afterbody else {
        return Ok(0.0);
    };
    validate_afterbody(geom, afterbody, model)?;
    let angle = afterbody_half_angle_rad(geom.body_diameter_m, afterbody);
    let area_change = ((geom.body_diameter_m * geom.body_diameter_m
        - afterbody.exit_diameter_m * afterbody.exit_diameter_m)
        .abs()
        * PI
        / 4.0)
        / geom.reference_area_m2;
    let compressibility = if mach < 1.0 {
        1.0 + 0.6 * mach * mach
    } else {
        1.4 + 0.15 * (mach - 1.0).min(4.0)
    };
    let angle_scale = (angle / 10.0_f64.to_radians()).powi(2);
    if afterbody.exit_diameter_m <= geom.body_diameter_m {
        Ok(0.018 * angle_scale * area_change * compressibility)
    } else {
        Ok(0.075 * angle_scale * area_change * compressibility)
    }
}

fn afterbody_half_angle_rad(body_diameter_m: f64, afterbody: Afterbody) -> f64 {
    ((body_diameter_m - afterbody.exit_diameter_m).abs() * 0.5 / afterbody.length_m).atan()
}

fn fin_planform_area_m2(fins: FinSet) -> f64 {
    0.5 * (fins.root_chord_m + fins.tip_chord_m) * fins.span_m
}

fn fin_wetted_area_m2(fins: FinSet) -> f64 {
    2.0 * f64::from(fins.count) * fin_planform_area_m2(fins)
}

fn fin_pressure_drag(geom: &BodyGeometry, fins: FinSet, mach: f64) -> Result<f64, AeroError> {
    let area_ratio = f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2;
    let sweep_factor = fins.sweep_rad.cos().abs().max(0.2);
    let compressibility = if mach < 1.0 {
        1.0 + 0.2 * mach * mach
    } else {
        1.0 + 0.1 * (mach - 1.0).min(4.0)
    };
    Ok(
        2.0 * fins.thickness_ratio * fins.thickness_ratio * area_ratio * compressibility
            / sweep_factor,
    )
}

fn fin_normal_force_slope(geom: &BodyGeometry, fins: FinSet, mach: f64) -> Result<f64, AeroError> {
    let finite_wing = fin_lifting_surface_normal_force_slope(fins, mach)?;
    let interference = legacy_fin_body_interference_factor(geom, fins);
    let slope = finite_wing
        * (f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2)
        * interference;
    if !slope.is_finite() || slope < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "fin normal-force slope is invalid",
        });
    }
    Ok(slope)
}

fn fin_lifting_surface_normal_force_slope(fins: FinSet, mach: f64) -> Result<f64, AeroError> {
    let area = fin_planform_area_m2(fins);
    let aspect_ratio = 2.0 * fins.span_m * fins.span_m / area.max(1.0e-12);
    let beta_comp = if mach < 0.8 {
        (1.0 - mach * mach).sqrt().max(0.25)
    } else if mach > 1.2 {
        (mach * mach - 1.0).sqrt().max(0.25)
    } else {
        0.25
    };
    let finite_wing = 2.0 * PI * aspect_ratio
        / (2.0 + (4.0 + aspect_ratio * aspect_ratio * beta_comp * beta_comp).sqrt());
    if !finite_wing.is_finite() || finite_wing < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "fin lifting-surface normal-force slope is invalid",
        });
    }
    Ok(finite_wing)
}

fn legacy_fin_body_interference_factor(geom: &BodyGeometry, fins: FinSet) -> f64 {
    1.0 + geom.body_diameter_m / (2.0 * fins.span_m + geom.body_diameter_m)
}

fn datcom_linear_normal_force_slope_and_cp(
    geom: &BodyGeometry,
    mach: f64,
) -> Result<(f64, f64), AeroError> {
    let nose_slope = 2.0;
    let nose_cp = nose_center_of_pressure_m(geom)?;
    let (fin_slope, fin_cp) = if let Some(fins) = geom.fins {
        (
            datcom_fin_normal_force_slope(geom, fins, mach)?,
            fin_cp_from_nose_m(geom, fins),
        )
    } else {
        (0.0, 0.0)
    };
    let total = nose_slope + fin_slope;
    if !total.is_finite() || total <= 0.0 {
        return Err(AeroError::NonFinite {
            reason: "DATCOM normal-force slope is invalid",
        });
    }
    let moment_sum = nose_slope * nose_cp + fin_slope * fin_cp;
    let cp = moment_sum / total;
    if !cp.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "DATCOM center of pressure is invalid",
        });
    }
    Ok((total, cp))
}

fn datcom_fin_normal_force_slope(
    geom: &BodyGeometry,
    fins: FinSet,
    mach: f64,
) -> Result<f64, AeroError> {
    let finite_wing = fin_lifting_surface_normal_force_slope(fins, mach)?;
    let (fin_on_body, body_on_fin) = datcom_fin_body_carryover_factors(geom, fins)?;
    let area_ratio = f64::from(fins.count) * fin_planform_area_m2(fins) / geom.reference_area_m2;
    let slope = finite_wing * area_ratio * fin_on_body * body_on_fin;
    if !slope.is_finite() || slope < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "DATCOM fin normal-force slope is invalid",
        });
    }
    Ok(slope)
}

fn datcom_fin_body_carryover_factors(
    geom: &BodyGeometry,
    fins: FinSet,
) -> Result<(f64, f64), AeroError> {
    if !(geom.body_diameter_m.is_finite() && fins.span_m.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "DATCOM fin/body carryover inputs are NaN or Inf",
        });
    }
    if geom.body_diameter_m <= 0.0 || fins.span_m <= 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "DATCOM fin/body carryover requires positive body diameter and fin span",
        });
    }
    let radius_to_span = (0.5 * geom.body_diameter_m / fins.span_m).clamp(0.0, 2.0);
    let fin_on_body = 1.0 + radius_to_span / (1.0 + radius_to_span);
    let body_on_fin = 1.0 + 0.5 * radius_to_span * radius_to_span / (1.0 + radius_to_span);
    if !(fin_on_body.is_finite() && body_on_fin.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "DATCOM fin/body carryover factors are non-finite",
        });
    }
    Ok((fin_on_body, body_on_fin))
}

fn fin_cp_from_nose_m(geom: &BodyGeometry, fins: FinSet) -> f64 {
    let root_le = geom.body_length_m - fins.root_chord_m;
    let mac = (2.0 / 3.0)
        * (fins.root_chord_m + fins.tip_chord_m
            - fins.root_chord_m * fins.tip_chord_m / (fins.root_chord_m + fins.tip_chord_m));
    let sweep_offset = 0.5 * fins.span_m * fins.sweep_rad.tan();
    (root_le + sweep_offset + 0.25 * mac).clamp(0.0, geom.body_length_m)
}

fn body_projected_planform_area_and_centroid_m(
    geom: &BodyGeometry,
) -> Result<(f64, f64), AeroError> {
    let nose_len = nose_length_m(geom)?;
    let after_len = geom.afterbody.map_or(0.0, |afterbody| afterbody.length_m);
    let cylinder_len = (geom.body_length_m - nose_len - after_len).max(0.0);

    let (nose_area, nose_moment) = nose_projected_planform_area_moment_m3(geom)?;
    let cylinder_area = geom.body_diameter_m * cylinder_len;
    let cylinder_moment = cylinder_area * (nose_len + 0.5 * cylinder_len);
    let (afterbody_area, afterbody_moment) =
        afterbody_projected_planform_area_moment_m3(geom, after_len);

    let area = nose_area + cylinder_area + afterbody_area;
    let moment = nose_moment + cylinder_moment + afterbody_moment;
    if !area.is_finite() || area <= 0.0 || !moment.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "DATCOM crossflow projected planform area is invalid",
        });
    }
    Ok((area, (moment / area).clamp(0.0, geom.body_length_m)))
}

fn nose_projected_planform_area_moment_m3(geom: &BodyGeometry) -> Result<(f64, f64), AeroError> {
    let length = nose_length_m(geom)?;
    let max_radius = 0.5 * geom.body_diameter_m;
    let step = length / PLANFORM_QUADRATURE_STEPS as f64;
    let mut area_sum = 0.0;
    let mut moment_sum = 0.0;
    for i in 0..=PLANFORM_QUADRATURE_STEPS {
        let x = step * i as f64;
        let weight = if i == 0 || i == PLANFORM_QUADRATURE_STEPS {
            1.0
        } else if i % 2 == 0 {
            2.0
        } else {
            4.0
        };
        let width = 2.0 * nose_radius_at_x_m(geom.nose, length, max_radius, x);
        area_sum += weight * width;
        moment_sum += weight * width * x;
    }
    Ok((step * area_sum / 3.0, step * moment_sum / 3.0))
}

fn nose_radius_at_x_m(nose: NoseShape, length: f64, max_radius: f64, x: f64) -> f64 {
    let x = x.clamp(0.0, length);
    match nose {
        NoseShape::Conical { .. } => max_radius * x / length,
        NoseShape::Ogive { .. } => {
            let generating_radius =
                (length * length + max_radius * max_radius) / (2.0 * max_radius);
            (generating_radius * generating_radius - (length - x) * (length - x))
                .max(0.0)
                .sqrt()
                + max_radius
                - generating_radius
        }
        NoseShape::VonKarman { .. } => {
            let theta = (1.0 - 2.0 * x / length).clamp(-1.0, 1.0).acos();
            let area_fraction = (theta - 0.5 * (2.0 * theta).sin()) / PI;
            max_radius * area_fraction.max(0.0).sqrt()
        }
        NoseShape::Hemispherical => (max_radius * max_radius - (x - max_radius) * (x - max_radius))
            .max(0.0)
            .sqrt(),
    }
}

fn afterbody_projected_planform_area_moment_m3(geom: &BodyGeometry, after_len: f64) -> (f64, f64) {
    let Some(afterbody) = geom.afterbody else {
        return (0.0, 0.0);
    };
    let start = geom.body_length_m - after_len;
    let root_width = geom.body_diameter_m;
    let exit_width = afterbody.exit_diameter_m;
    let area = 0.5 * (root_width + exit_width) * after_len;
    let local_moment = after_len * after_len * (0.5 * root_width + (exit_width - root_width) / 3.0);
    (area, area * start + local_moment)
}

fn finite_length_crossflow_drag_coefficient(geom: &BodyGeometry, flow: &FlowCondition) -> f64 {
    let crossflow_mach = (flow.mach * flow.alpha_rad.sin().abs()).clamp(0.0, 8.0);
    let cylinder_section_cd = 1.2 * (1.0 + 0.04 * crossflow_mach).min(1.24);
    cylinder_section_cd
        * finite_length_crossflow_drag_ratio(geom.body_length_m / geom.body_diameter_m)
}

fn finite_length_crossflow_drag_ratio(fineness_ratio: f64) -> f64 {
    if fineness_ratio <= NACA_1048_BODY_II_FINENESS {
        return NACA_1048_BODY_II_CROSSFLOW_DRAG_RATIO;
    }
    if fineness_ratio >= NACA_1048_BODY_I_FINENESS {
        return NACA_1048_BODY_I_CROSSFLOW_DRAG_RATIO;
    }
    let t = (fineness_ratio - NACA_1048_BODY_II_FINENESS)
        / (NACA_1048_BODY_I_FINENESS - NACA_1048_BODY_II_FINENESS);
    NACA_1048_BODY_II_CROSSFLOW_DRAG_RATIO * (1.0 - t) + NACA_1048_BODY_I_CROSSFLOW_DRAG_RATIO * t
}

fn datcom_incremental_foredrag(
    geom: &BodyGeometry,
    flow: &FlowCondition,
) -> Result<f64, AeroError> {
    let (projected_planform_area_m2, _) = body_projected_planform_area_and_centroid_m(geom)?;
    let base_diameter_m = geom
        .afterbody
        .map_or(geom.body_diameter_m, |afterbody| afterbody.exit_diameter_m);
    let base_area_ratio = (PI * base_diameter_m * base_diameter_m / 4.0) / geom.reference_area_m2;
    let sine_abs = flow.alpha_rad.sin().abs();
    let crossflow_drag = finite_length_crossflow_drag_coefficient(geom, flow);
    let increment = base_area_ratio * sine_abs * sine_abs
        + crossflow_drag * (projected_planform_area_m2 / geom.reference_area_m2) * sine_abs.powi(3);
    if !increment.is_finite() || increment < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "DATCOM incremental foredrag is invalid",
        });
    }
    Ok(increment)
}

fn allen_perkins_crossflow_normal_force(
    geom: &BodyGeometry,
    flow: &FlowCondition,
) -> Result<(f64, f64), AeroError> {
    let (projected_planform_area_m2, crossflow_cp_m) =
        body_projected_planform_area_and_centroid_m(geom)?;
    if flow.alpha_rad == 0.0 {
        return Ok((0.0, crossflow_cp_m));
    }
    let crossflow_drag = finite_length_crossflow_drag_coefficient(geom, flow);
    let side_area_ratio = projected_planform_area_m2 / geom.reference_area_m2;
    let sine = flow.alpha_rad.sin();
    let cosine = flow.alpha_rad.cos().abs();
    let cn = crossflow_drag * side_area_ratio * sine.abs() * sine * cosine;
    if !(cn.is_finite() && crossflow_cp_m.is_finite()) {
        return Err(AeroError::NonFinite {
            reason: "DATCOM crossflow normal-force increment is non-finite",
        });
    }
    Ok((cn, crossflow_cp_m))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use std::collections::BTreeMap;

    fn geom(exit_diameter_m: Option<f64>) -> BodyGeometry {
        BodyGeometry {
            body_diameter_m: 0.2,
            body_length_m: 2.4,
            nose: NoseShape::Ogive { fineness: 3.5 },
            afterbody: exit_diameter_m.map(|exit| Afterbody {
                exit_diameter_m: exit,
                length_m: 0.25,
            }),
            surface_roughness_m: 6.0e-5,
            fins: Some(FinSet {
                count: 4,
                root_chord_m: 0.30,
                tip_chord_m: 0.12,
                span_m: 0.16,
                thickness_ratio: 0.06,
                sweep_rad: 0.52,
            }),
            protuberances: None,
            reference_area_m2: PI * 0.2 * 0.2 / 4.0,
            reference_length_m: 0.2,
            center_of_gravity_from_nose_m: Some(1.25),
        }
    }

    #[test]
    fn crossflow_finite_length_drag_ratio_matches_naca_1048_body_limits() {
        assert_abs_diff_eq!(
            finite_length_crossflow_drag_ratio(NACA_1048_BODY_II_FINENESS),
            NACA_1048_BODY_II_CROSSFLOW_DRAG_RATIO,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            finite_length_crossflow_drag_ratio(NACA_1048_BODY_I_FINENESS),
            NACA_1048_BODY_I_CROSSFLOW_DRAG_RATIO,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            finite_length_crossflow_drag_ratio(
                0.5 * (NACA_1048_BODY_I_FINENESS + NACA_1048_BODY_II_FINENESS)
            ),
            0.5 * (NACA_1048_BODY_I_CROSSFLOW_DRAG_RATIO + NACA_1048_BODY_II_CROSSFLOW_DRAG_RATIO),
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn crossflow_planform_area_and_centroid_match_conical_body_geometry() {
        let geometry = BodyGeometry {
            body_diameter_m: 1.0,
            body_length_m: 3.0,
            nose: NoseShape::Conical {
                half_angle_rad: 0.5_f64.atan(),
            },
            afterbody: None,
            surface_roughness_m: 0.0,
            fins: None,
            protuberances: None,
            reference_area_m2: PI / 4.0,
            reference_length_m: 1.0,
            center_of_gravity_from_nose_m: None,
        };

        let (area, centroid) = body_projected_planform_area_and_centroid_m(&geometry).unwrap();
        assert_abs_diff_eq!(area, 2.5, epsilon = 1.0e-14);
        assert_abs_diff_eq!(centroid, 4.333333333333333 / 2.5, epsilon = 1.0e-14);

        let (_, zero_alpha_cp) = DatcomComponentBuildup::crossflow_normal_force_increment(
            &geometry,
            &FlowCondition {
                mach: 0.0,
                reynolds_length: 1.0e6,
                alpha_rad: 0.0,
            },
        )
        .unwrap();
        assert_abs_diff_eq!(zero_alpha_cp, centroid, epsilon = 1.0e-14);
    }

    #[test]
    fn base_drag_curve_matches_reference_formula() {
        assert_abs_diff_eq!(
            ComponentBuildup::base_drag_coefficient(0.5).unwrap(),
            0.12 + 0.13 * 0.25,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            ComponentBuildup::base_drag_coefficient(2.0).unwrap(),
            0.125,
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn skin_friction_matches_laminar_and_turbulent_formulas() {
        let laminar = ComponentBuildup::skin_friction_coefficient(1.0e5, 0.0, 5.0e5).unwrap();
        assert_relative_eq!(laminar, 1.328 / 1.0e5_f64.sqrt(), max_relative = 1.0e-14);
        let turbulent = ComponentBuildup::skin_friction_coefficient(1.0e7, 0.0, 5.0e5).unwrap();
        assert_relative_eq!(
            turbulent,
            0.455 / 1.0e7_f64.log10().powf(2.58),
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn boattail_reduces_total_zero_lift_drag_inside_envelope() {
        let model = ComponentBuildup::default();
        let flow = FlowCondition {
            mach: 0.9,
            reynolds_length: 3.0e6,
            alpha_rad: 0.0,
        };
        let blunt = model.coefficients(&geom(None), &flow).unwrap().cd;
        let boattailed = model.coefficients(&geom(Some(0.14)), &flow).unwrap().cd;
        assert!(
            boattailed < blunt,
            "boattail CD {boattailed} >= blunt CD {blunt}"
        );
    }

    #[test]
    fn drag_polar_adds_second_order_alpha_drag() {
        let model = ComponentBuildup::default();
        let flow0 = FlowCondition {
            mach: 0.7,
            reynolds_length: 3.0e6,
            alpha_rad: 0.0,
        };
        let alpha = 1.0_f64.to_radians();
        let flow1 = FlowCondition {
            alpha_rad: alpha,
            ..flow0
        };
        let c0 = model.coefficients(&geom(None), &flow0).unwrap();
        let c1 = model.coefficients(&geom(None), &flow1).unwrap();
        assert!(c1.cd > c0.cd);
        assert_relative_eq!(
            c1.cd - c0.cd,
            (c1.cn / alpha) * alpha * alpha,
            max_relative = 0.08
        );
    }

    #[test]
    fn datcom_crossflow_is_opt_in_and_odd_with_angle_of_attack() {
        let legacy = ComponentBuildup::default();
        let datcom = legacy.datcom();
        let flow = FlowCondition {
            mach: 2.0,
            reynolds_length: 4.0e6,
            alpha_rad: 15.0_f64.to_radians(),
        };
        let negative_flow = FlowCondition {
            alpha_rad: -flow.alpha_rad,
            ..flow
        };

        let legacy_positive = legacy.coefficients(&geom(None), &flow).unwrap();
        let legacy_negative = legacy.coefficients(&geom(None), &negative_flow).unwrap();
        let datcom_positive = datcom.coefficients(&geom(None), &flow).unwrap();
        let datcom_negative = datcom.coefficients(&geom(None), &negative_flow).unwrap();
        let datcom_zero = datcom
            .coefficients(
                &geom(None),
                &FlowCondition {
                    alpha_rad: 0.0,
                    ..flow
                },
            )
            .unwrap();

        assert!(datcom_positive.cn > legacy_positive.cn);
        assert!(datcom_positive.cd > datcom_zero.cd);
        assert_ne!(datcom_positive.cd.to_bits(), legacy_positive.cd.to_bits());
        assert_relative_eq!(
            legacy_positive.cn,
            -legacy_negative.cn,
            max_relative = 1.0e-14
        );
        assert_relative_eq!(
            datcom_positive.cn,
            -datcom_negative.cn,
            max_relative = 1.0e-14
        );
        assert_relative_eq!(
            datcom_positive.cd,
            datcom_negative.cd,
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn datcom_area_rule_wave_drag_reduces_von_karman_supersonic_cd_when_opted_in() {
        let mut geometry = geom(None);
        geometry.nose = NoseShape::VonKarman { fineness: 3.5 };
        geometry.fins = None;
        let legacy = ComponentBuildup::default();
        let datcom = legacy.datcom();
        let supersonic = FlowCondition {
            mach: 2.5,
            reynolds_length: 4.0e6,
            alpha_rad: 0.0,
        };
        let subsonic = FlowCondition {
            mach: 0.5,
            ..supersonic
        };

        let legacy_supersonic = legacy.coefficients(&geometry, &supersonic).unwrap();
        let datcom_supersonic = datcom.coefficients(&geometry, &supersonic).unwrap();
        assert!(legacy_supersonic.cd > 0.0);
        assert!(datcom_supersonic.cd > 0.0);
        assert!(datcom_supersonic.cd < legacy_supersonic.cd);

        let legacy_subsonic = legacy.coefficients(&geometry, &subsonic).unwrap();
        let datcom_subsonic = datcom.coefficients(&geometry, &subsonic).unwrap();
        assert_eq!(legacy_subsonic.cd.to_bits(), datcom_subsonic.cd.to_bits());
    }

    #[test]
    fn datcom_protuberance_drag_adds_cd_only_when_opted_in() {
        let mut geometry = geom(None);
        geometry.fins = None;
        geometry.protuberances = Some(ProtuberanceSet {
            count: 2,
            frontal_area_m2: 1.2e-4,
            drag_coefficient: 1.15,
            interference_factor: 1.1,
        });
        let flow = FlowCondition {
            mach: 0.5,
            reynolds_length: 4.0e6,
            alpha_rad: 0.0,
        };
        let legacy = ComponentBuildup::default();
        let datcom = legacy.datcom();

        let legacy_coeffs = legacy.coefficients(&geometry, &flow).unwrap();
        let datcom_coeffs = datcom.coefficients(&geometry, &flow).unwrap();
        let expected_increment = datcom_protuberance_drag(&geometry, flow.mach).unwrap();

        assert_eq!(legacy_coeffs.cn.to_bits(), datcom_coeffs.cn.to_bits());
        assert_eq!(legacy_coeffs.cm.to_bits(), datcom_coeffs.cm.to_bits());
        assert_relative_eq!(
            datcom_coeffs.cd - legacy_coeffs.cd,
            expected_increment,
            max_relative = 1.0e-14
        );
    }

    #[test]
    fn datcom_fin_body_carryover_replaces_legacy_interference_when_opted_in() {
        let geometry = geom(None);
        let fins = geometry.fins.unwrap();
        let mach = 1.6;
        let legacy_interference = legacy_fin_body_interference_factor(&geometry, fins);
        let (fin_on_body, body_on_fin) =
            DatcomComponentBuildup::fin_body_carryover_factors(&geometry, fins).unwrap();

        assert!(fin_on_body > 1.0);
        assert!(body_on_fin > 1.0);
        assert!(fin_on_body * body_on_fin > legacy_interference);

        let legacy_slope = fin_normal_force_slope(&geometry, fins, mach).unwrap();
        let datcom_slope = datcom_fin_normal_force_slope(&geometry, fins, mach).unwrap();
        assert!(datcom_slope > legacy_slope);

        let flow = FlowCondition {
            mach,
            reynolds_length: 4.0e6,
            alpha_rad: 2.0_f64.to_radians(),
        };
        let legacy = ComponentBuildup::default()
            .coefficients(&geometry, &flow)
            .unwrap();
        let datcom = DatcomComponentBuildup::default()
            .coefficients(&geometry, &flow)
            .unwrap();
        assert!(datcom.cn > legacy.cn);
    }

    #[test]
    fn datcom_bake_deck_is_bit_stable_and_differs_from_legacy_at_alpha() {
        let legacy = ComponentBuildup::default();
        let datcom = DatcomComponentBuildup::default();
        let grid = BuildupGrid::from_ranges(0.5, 2.5, 3, 0.0, 15.0, 4, 4.0e6).unwrap();
        let legacy_deck = legacy.bake_deck(&geom(Some(0.14)), &grid).unwrap();
        let first = datcom.bake_deck(&geom(Some(0.14)), &grid).unwrap();
        let second = datcom.bake_deck(&geom(Some(0.14)), &grid).unwrap();

        assert_eq!(first, second);
        assert!(!legacy_deck.has_dynamic_derivative_tables());
        assert!(first.has_dynamic_derivative_tables());
        let derivatives = first
            .lookup_dynamic_derivatives(2.5, 0.0, 0.0, &BTreeMap::new())
            .unwrap()
            .unwrap();
        assert!(derivatives.cm_q < 0.0);
        assert!(derivatives.cn_r < 0.0);
        assert!(derivatives.cl_p < 0.0);
        let legacy_high_alpha = legacy_deck
            .lookup(2.5, 15.0, 0.0, &BTreeMap::new())
            .unwrap();
        let datcom_high_alpha = first.lookup(2.5, 15.0, 0.0, &BTreeMap::new()).unwrap();
        assert!(datcom_high_alpha.cn > legacy_high_alpha.cn);
        assert_ne!(
            datcom_high_alpha.cd.to_bits(),
            legacy_high_alpha.cd.to_bits()
        );
        let legacy_zero_alpha = legacy_deck.lookup(2.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let datcom_zero_alpha = first.lookup(2.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert!(datcom_high_alpha.cd > datcom_zero_alpha.cd);
        assert_eq!(
            legacy_zero_alpha.cn.to_bits(),
            datcom_zero_alpha.cn.to_bits()
        );
        assert!(datcom_zero_alpha.cd < legacy_zero_alpha.cd);
        assert_eq!(
            legacy_zero_alpha.cm.to_bits(),
            datcom_zero_alpha.cm.to_bits()
        );
        let legacy_subsonic_zero_alpha =
            legacy_deck.lookup(0.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        let datcom_subsonic_zero_alpha = first.lookup(0.5, 0.0, 0.0, &BTreeMap::new()).unwrap();
        assert_eq!(
            legacy_subsonic_zero_alpha.cd.to_bits(),
            datcom_subsonic_zero_alpha.cd.to_bits()
        );
        assert_eq!(
            legacy_subsonic_zero_alpha.cm.to_bits(),
            datcom_subsonic_zero_alpha.cm.to_bits()
        );
    }

    #[test]
    fn bake_deck_is_bit_stable_for_fixed_grid() {
        let model = ComponentBuildup::default();
        let grid = BuildupGrid::from_ranges(0.0, 2.0, 5, 0.0, 4.0, 3, 3.0e6).unwrap();
        let first = model.bake_deck(&geom(Some(0.14)), &grid).unwrap();
        let second = model.bake_deck(&geom(Some(0.14)), &grid).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.mach_grid(), &[0.0, 0.5, 1.0, 1.5, 2.0]);
        assert_eq!(first.alpha_grid_deg(), &[0.0, 2.0, 4.0]);
        assert_eq!(first.beta_grid_deg(), &[0.0]);
    }
}
