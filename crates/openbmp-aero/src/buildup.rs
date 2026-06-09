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

impl Default for ComponentBuildup {
    fn default() -> Self {
        Self {
            transition_reynolds: RE_TRANSITION_DEFAULT,
            boattail_separation_angle_rad: DEFAULT_BOATTAIL_SEPARATION_RAD,
        }
    }
}

impl ComponentBuildup {
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
    let interference = 1.0 + geom.body_diameter_m / (2.0 * fins.span_m + geom.body_diameter_m);
    let slope =
        finite_wing * (f64::from(fins.count) * area / geom.reference_area_m2) * interference;
    if !slope.is_finite() || slope < 0.0 {
        return Err(AeroError::NonFinite {
            reason: "fin normal-force slope is invalid",
        });
    }
    Ok(slope)
}

fn fin_cp_from_nose_m(geom: &BodyGeometry, fins: FinSet) -> f64 {
    let root_le = geom.body_length_m - fins.root_chord_m;
    let mac = (2.0 / 3.0)
        * (fins.root_chord_m + fins.tip_chord_m
            - fins.root_chord_m * fins.tip_chord_m / (fins.root_chord_m + fins.tip_chord_m));
    let sweep_offset = 0.5 * fins.span_m * fins.sweep_rad.tan();
    (root_le + sweep_offset + 0.25 * mac).clamp(0.0, geom.body_length_m)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq};

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
            reference_area_m2: PI * 0.2 * 0.2 / 4.0,
            reference_length_m: 0.2,
            center_of_gravity_from_nose_m: Some(1.25),
        }
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
