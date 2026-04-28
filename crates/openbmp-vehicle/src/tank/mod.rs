//! Phase-3.7 tanks and slosh as moving-mass dynamics.
//!
//! Liquid propellant inside a tank is a moving mass: as the body
//! accelerates and rotates, the liquid sloshes, the CG shifts, and
//! a coupled-pendulum or equivalent moving-mass term loads the rigid
//! body. OpenBMP treats this as **generic moving-mass dynamics** so
//! the same machinery covers slosh, deployable masses, and shifting
//! payloads. Phase 3.7 ships:
//!
//! - [`MovingMassModel`] trait — `step(accel, omega, dt)`,
//!   `mass_contribution()`, `reaction_body()`, `drain(kg/s)`,
//!   `fluid_remaining_kg()`.
//! - [`Tank`] struct — outer container carrying [`TankId`],
//!   geometry, mount point, propellant, initial fill, optional
//!   [`BaffleModel`], and the boxed [`MovingMassModel`].
//! - [`MassContribution`] — `mass_kg` + mount-relative
//!   `cg_offset_body_m` + body-origin `inertia_delta_body_kg_m2`
//!   (parallel-axis already applied).
//! - [`ForceMomentBody`] — body-frame reaction force + moment.
//! - [`TankGeometry`] — closed-form textbook shapes (cylinder /
//!   sphere / ellipsoid).
//! - [`PropellantSpec`] — toy / textbook density only; no fielded
//!   data per [`safety-boundaries`].
//! - [`rigid_liquid::RigidLiquid`] — Phase-3.7.A toy: no slosh,
//!   point-mass at mount, zero reaction. The simplest baseline that
//!   exercises the full surface (drain, mass contribution, parallel-
//!   axis inertia delta).
//!
//! # Determinism
//!
//! - Pure `f64` arithmetic on [`MovingMassModel::step`]; no FMA, no
//!   wall-clock, no system RNG, no I/O.
//! - The trait does **not** take `&self` for `mass_contribution()` /
//!   `reaction_body()` — both must be pure observers of the model's
//!   internal state, so the runner can call them in any order
//!   without changing observable behaviour.
//! - One-step lag on `(accel_body, omega_body)`: the kernel can't
//!   supply current-step accelerations before force evaluation
//!   completes (circular dependency through tank reaction force /
//!   mass contribution). The runner caches `(accel, omega)` from
//!   step `n` and feeds them to `step()` at the start of step
//!   `n+1`. The first step uses zeros.
//! - Forward-Euler default (single sub-step inside the main RK4).
//!   Bit-stable across reruns at fixed sub-step count; per-tank
//!   `slosh_substeps` opt-in for higher fidelity changes the byte
//!   output and is documented as a per-scenario lock.
//!
//! # Crate layering
//!
//! Tanks live in `openbmp-vehicle` (L1) — they consume kernel-side
//! `Vector3<f64>` and `Matrix3<f64>` directly, but they don't depend
//! on `openbmp-sim`. The runner-side [`crate::tank`]-rack adapter
//! ships in Phase 3.7.D and is the kernel-side bridge mirroring the
//! Phase-3.6 [`crate::adapters::EngineClusterMassAdapter`] pattern.
//!
//! See `docs/phase-3-plan.md § 3.7` and
//! `docs/software-architecture.md § Tanks and Slosh as Moving-Mass
//! Dynamics` for the contract.
//!
//! [`safety-boundaries`]: ../../../../../docs/safety-boundaries.md

pub mod rigid_liquid;

pub use rigid_liquid::RigidLiquid;

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{BodyId, Duration, TankId};
use thiserror::Error;

// ---------------------------------------------------------------------
// Mass contribution + reaction
// ---------------------------------------------------------------------

/// Effective contribution of a tank's moving mass to the parent
/// vehicle's mass / CG / inertia at this instant.
///
/// `cg_offset_body_m` is the displacement of the moving-mass
/// centroid **relative to the tank mount point**, expressed in the
/// parent body frame. `inertia_delta_body_kg_m2` is **about the
/// parent body's origin** with the parallel-axis term already
/// applied; the runner adds it directly to the dry inertia tensor.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MassContribution {
    /// Current moving mass, in kilograms.
    pub mass_kg: f64,
    /// CG offset relative to tank mount point, in body frame, metres.
    pub cg_offset_body_m: Vector3<f64>,
    /// Inertia delta about the parent body's origin, in body frame,
    /// kg·m². Additive to the dry inertia tensor.
    pub inertia_delta_body_kg_m2: Matrix3<f64>,
}

/// Reaction force + moment that the tank's moving mass exerts back
/// on the parent body, expressed in the parent body frame.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct ForceMomentBody {
    /// Body-frame reaction force, Newtons.
    pub force_body_n: Vector3<f64>,
    /// Body-frame reaction moment about the parent body's origin,
    /// Newton-metres.
    pub moment_body_n_m: Vector3<f64>,
}

// ---------------------------------------------------------------------
// Geometry + propellant
// ---------------------------------------------------------------------

/// Phase-3 tank geometries. Closed-form textbook shapes only.
///
/// Real fielded tank geometry is rejected per
/// `docs/safety-boundaries.md`; downstream users may bind their own
/// shapes through their own real-data packages.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TankGeometry {
    /// Right circular cylinder.
    Cylinder {
        /// Internal radius, metres.
        radius_m: f64,
        /// Internal height, metres.
        height_m: f64,
    },
    /// Sphere.
    Sphere {
        /// Internal radius, metres.
        radius_m: f64,
    },
    /// Triaxial ellipsoid (a, b, c semi-axes).
    EllipsoidTextbook {
        /// Semi-axis along body x, metres.
        a_m: f64,
        /// Semi-axis along body y, metres.
        b_m: f64,
        /// Semi-axis along body z, metres.
        c_m: f64,
    },
}

impl TankGeometry {
    /// Internal volume in cubic metres.
    #[must_use]
    pub fn volume_m3(&self) -> f64 {
        match *self {
            Self::Cylinder { radius_m, height_m } => {
                std::f64::consts::PI * radius_m * radius_m * height_m
            }
            Self::Sphere { radius_m } => {
                (4.0 / 3.0) * std::f64::consts::PI * radius_m * radius_m * radius_m
            }
            Self::EllipsoidTextbook { a_m, b_m, c_m } => {
                (4.0 / 3.0) * std::f64::consts::PI * a_m * b_m * c_m
            }
        }
    }

    /// Validate that all dimensions are finite and strictly positive.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::InvalidGeometry`] for non-finite or
    /// non-positive dimensions.
    pub fn require_valid(&self) -> Result<(), TankError> {
        let positive_finite = |x: f64| x.is_finite() && x > 0.0;
        match *self {
            Self::Cylinder { radius_m, height_m } => {
                if !positive_finite(radius_m) || !positive_finite(height_m) {
                    return Err(TankError::InvalidGeometry {
                        reason: "cylinder radius_m and height_m must be finite and strictly positive",
                    });
                }
            }
            Self::Sphere { radius_m } => {
                if !positive_finite(radius_m) {
                    return Err(TankError::InvalidGeometry {
                        reason: "sphere radius_m must be finite and strictly positive",
                    });
                }
            }
            Self::EllipsoidTextbook { a_m, b_m, c_m } => {
                if !positive_finite(a_m) || !positive_finite(b_m) || !positive_finite(c_m) {
                    return Err(TankError::InvalidGeometry {
                        reason: "ellipsoid semi-axes must be finite and strictly positive",
                    });
                }
            }
        }
        Ok(())
    }
}

/// Phase-3 propellant spec. Textbook constants only.
///
/// `density_kg_m3` is the bulk liquid density at nominal storage
/// conditions; Phase-3 does not model temperature / pressure
/// variation. `label` is a short human-readable tag used in
/// telemetry and provenance records.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PropellantSpec {
    /// Liquid density, kg/m³. Strictly positive.
    pub density_kg_m3: f64,
    /// Display label (e.g. `"water_textbook"`).
    pub label: &'static str,
}

impl PropellantSpec {
    /// Validate the spec.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::InvalidPropellant`] when `density_kg_m3`
    /// is non-finite or non-positive.
    pub fn require_valid(&self) -> Result<(), TankError> {
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 <= 0.0 {
            return Err(TankError::InvalidPropellant {
                reason: "density_kg_m3 must be finite and strictly positive",
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Baffle model (Phase-3.7 minimum surface)
// ---------------------------------------------------------------------

/// Phase-3.7 baffle model.
///
/// Phase-3.7 ships the minimum surface: a scalar damping-ratio
/// increment that is added to the bare-tank pendulum's damping
/// term. Per Abramson SP-106 §7.4, baffle damping rises
/// approximately linearly with baffle-area-to-tank-cross-section
/// ratio; downstream users can curve-fit and ship a constant.
/// Baffle-area integration and full Eq. 7-46 evaluation are
/// deferred.
///
/// The increment is consumed by [`baffled_pendulum::BaffledPendulum`]
/// (Phase 3.7.C); other models ignore it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaffleModel {
    /// Additive damping-ratio increment applied to the pendulum's
    /// `2ζω_n θ̇` term. Non-negative; finite.
    pub damping_increment_zeta: f64,
}

impl BaffleModel {
    /// Validate.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::InvalidBaffle`] for non-finite or
    /// negative `damping_increment_zeta`.
    pub fn require_valid(&self) -> Result<(), TankError> {
        if !self.damping_increment_zeta.is_finite() || self.damping_increment_zeta < 0.0 {
            return Err(TankError::InvalidBaffle {
                reason: "damping_increment_zeta must be finite and non-negative",
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Tank container
// ---------------------------------------------------------------------

/// Phase-3.7 tank: id + metadata + boxed [`MovingMassModel`].
///
/// The `Tank` owns one moving-mass model. The runner-side rack
/// (Phase 3.7.D) holds a `BTreeMap<TankId, Tank>` and is the
/// authoritative store; the kernel reads per-step snapshots through
/// the [`crate::tank`]-rack adapter.
#[derive(Debug)]
pub struct Tank {
    /// Stable identifier for telemetry and event-action targeting.
    id: TankId,
    /// Closed-form geometry (cylinder / sphere / ellipsoid).
    geometry: TankGeometry,
    /// Parent body the tank is rigidly mounted to.
    mounted_to: BodyId,
    /// Mount point in the parent body frame, metres.
    mount_point_body_m: Vector3<f64>,
    /// Propellant constants.
    propellant: PropellantSpec,
    /// Initial fill fraction, ∈ `[0, 1]`. Used at scenario load to
    /// derive the initial fluid mass passed to the boxed model.
    initial_fill_fraction: f64,
    /// Optional baffle model (consumed only by `BaffledPendulum`).
    baffle_model: Option<BaffleModel>,
    /// Boxed moving-mass model (the slosh dynamics).
    moving_mass: Box<dyn MovingMassModel>,
}

impl Tank {
    /// Construct a new tank.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] when geometry, propellant, baffle model,
    /// fill fraction, or mount point is invalid.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: TankId,
        geometry: TankGeometry,
        mounted_to: BodyId,
        mount_point_body_m: Vector3<f64>,
        propellant: PropellantSpec,
        initial_fill_fraction: f64,
        baffle_model: Option<BaffleModel>,
        moving_mass: Box<dyn MovingMassModel>,
    ) -> Result<Self, TankError> {
        geometry.require_valid()?;
        propellant.require_valid()?;
        if let Some(baffle) = &baffle_model {
            baffle.require_valid()?;
        }
        if !initial_fill_fraction.is_finite()
            || !(0.0..=1.0).contains(&initial_fill_fraction)
        {
            return Err(TankError::InvalidFillFraction {
                value: initial_fill_fraction,
            });
        }
        if !mount_point_body_m.iter().all(|c| c.is_finite()) {
            return Err(TankError::InvalidMountPoint);
        }
        Ok(Self {
            id,
            geometry,
            mounted_to,
            mount_point_body_m,
            propellant,
            initial_fill_fraction,
            baffle_model,
            moving_mass,
        })
    }

    /// Tank identifier.
    #[must_use]
    pub fn id(&self) -> TankId {
        self.id
    }

    /// Closed-form geometry.
    #[must_use]
    pub fn geometry(&self) -> TankGeometry {
        self.geometry
    }

    /// Parent body id.
    #[must_use]
    pub fn mounted_to(&self) -> BodyId {
        self.mounted_to
    }

    /// Mount point in body frame.
    #[must_use]
    pub fn mount_point_body_m(&self) -> Vector3<f64> {
        self.mount_point_body_m
    }

    /// Propellant spec.
    #[must_use]
    pub fn propellant(&self) -> PropellantSpec {
        self.propellant
    }

    /// Initial fill fraction in `[0, 1]`.
    #[must_use]
    pub fn initial_fill_fraction(&self) -> f64 {
        self.initial_fill_fraction
    }

    /// Initial fluid mass derived from geometry × density × fill.
    #[must_use]
    pub fn initial_fluid_mass_kg(&self) -> f64 {
        self.geometry.volume_m3() * self.propellant.density_kg_m3 * self.initial_fill_fraction
    }

    /// Optional baffle model.
    #[must_use]
    pub fn baffle_model(&self) -> Option<&BaffleModel> {
        self.baffle_model.as_ref()
    }

    /// Set the drain rate the next [`Self::step`] will apply.
    ///
    /// # Errors
    ///
    /// Forwards any [`TankError`] from the inner moving-mass model
    /// (e.g. [`TankError::InvalidDrainRate`]).
    pub fn drain(&mut self, kg_per_s: f64) -> Result<(), TankError> {
        self.moving_mass.drain(kg_per_s)
    }

    /// Advance the moving-mass internal state by one kernel base
    /// tick. `accel_body_m_s2` and `omega_body_rad_s` come from the
    /// **prior** kernel step (one-step lag — see module docs).
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] when the inputs are non-finite.
    pub fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), TankError> {
        self.moving_mass.step(accel_body_m_s2, omega_body_rad_s, dt)
    }

    /// Snapshot of the moving-mass contribution to vehicle mass / CG
    /// / inertia at the current internal state.
    #[must_use]
    pub fn mass_contribution(&self) -> MassContribution {
        self.moving_mass.mass_contribution()
    }

    /// Body-frame reaction force + moment exerted on the parent
    /// body by the moving mass at the current internal state.
    #[must_use]
    pub fn reaction_body(&self) -> ForceMomentBody {
        self.moving_mass.reaction_body()
    }

    /// Remaining fluid mass.
    #[must_use]
    pub fn fluid_remaining_kg(&self) -> f64 {
        self.moving_mass.fluid_remaining_kg()
    }
}

// ---------------------------------------------------------------------
// MovingMassModel trait
// ---------------------------------------------------------------------

/// Phase-3.7 moving-mass trait.
///
/// Implementors hold their own internal state (fluid mass, slosh
/// angle, slosh rate, etc.) and expose four primitive operations:
/// `drain` to set the next step's drain rate, `step` to advance
/// internal state by one kernel base tick, `mass_contribution` to
/// publish the current effective mass/CG/inertia, and
/// `reaction_body` to publish the current body-frame reaction force
/// + moment.
///
/// Implementors must be deterministic: pure `f64` arithmetic, locked
/// operand order, no FMA, no system RNG. Higher-fidelity sub-steps
/// inside `step` are allowed but are bit-stable only at a fixed
/// sub-step count; the count is part of the scenario's
/// determinism contract.
pub trait MovingMassModel: Send + Sync + std::fmt::Debug {
    /// Set the drain rate (kg/s) the next [`Self::step`] will apply.
    /// The model clamps to zero internally if the fluid is depleted.
    ///
    /// # Errors
    ///
    /// Returns [`TankError::InvalidDrainRate`] if `kg_per_s` is
    /// non-finite or negative.
    fn drain(&mut self, kg_per_s: f64) -> Result<(), TankError>;

    /// Advance internal state by one main-step `dt`.
    ///
    /// `accel_body_m_s2` and `omega_body_rad_s` are the **prior
    /// step's** body-frame translational acceleration and angular
    /// rate at the parent body's reference point. The runner is
    /// responsible for the one-step lag — see module docs.
    ///
    /// # Errors
    ///
    /// Returns [`TankError`] for non-finite inputs.
    fn step(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
        dt: Duration,
    ) -> Result<(), TankError>;

    /// Effective contribution to vehicle mass / CG / inertia.
    ///
    /// Pure observer of the model's current internal state.
    fn mass_contribution(&self) -> MassContribution;

    /// Body-frame reaction force + moment about parent body origin.
    ///
    /// Pure observer of the model's current internal state.
    fn reaction_body(&self) -> ForceMomentBody;

    /// Remaining fluid mass, kg. Monotonically non-increasing across
    /// successive `drain`/`step` pairs.
    fn fluid_remaining_kg(&self) -> f64;
}

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Errors raised by tank construction or moving-mass evaluation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum TankError {
    /// Tank geometry has non-finite or non-positive dimensions.
    #[error("tank geometry invalid: {reason}")]
    InvalidGeometry {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Propellant spec has non-finite or non-positive density.
    #[error("tank propellant invalid: {reason}")]
    InvalidPropellant {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Baffle model has non-finite or negative damping increment.
    #[error("tank baffle invalid: {reason}")]
    InvalidBaffle {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// `initial_fill_fraction` is not in `[0, 1]` or is non-finite.
    #[error("tank initial_fill_fraction must be in [0, 1] and finite, got {value}")]
    InvalidFillFraction {
        /// Offending value.
        value: f64,
    },
    /// Mount-point coordinates are not all finite.
    #[error("tank mount point coordinates must all be finite")]
    InvalidMountPoint,
    /// `drain` was called with a non-finite or negative rate.
    #[error("tank drain rate must be finite and non-negative, got {value}")]
    InvalidDrainRate {
        /// Offending value.
        value: f64,
    },
    /// `step` received a non-finite acceleration, angular rate, or
    /// `dt`.
    #[error("tank step input is not finite: {reason}")]
    NonFiniteStepInput {
        /// Human-readable reason.
        reason: &'static str,
    },
}

// ---------------------------------------------------------------------
// Helpers shared by Phase-3.7 implementations
// ---------------------------------------------------------------------

/// Parallel-axis inertia contribution of a point mass `m` at body-
/// frame offset `r` from body origin. Returns `m · (||r||² · I3 - r
/// · rᵀ)`.
///
/// Locked operand order: `dot_product` then `outer_product` then
/// the subtraction. Used by every `MovingMassModel` impl to compose
/// the body-origin inertia delta.
#[must_use]
pub(crate) fn point_mass_inertia_about_origin(mass_kg: f64, offset_body_m: Vector3<f64>) -> Matrix3<f64> {
    let r_dot_r = offset_body_m.dot(&offset_body_m);
    let r_outer_r = offset_body_m * offset_body_m.transpose();
    let identity = Matrix3::<f64>::identity();
    mass_kg * (r_dot_r * identity - r_outer_r)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn finite_propellant() -> PropellantSpec {
        PropellantSpec {
            density_kg_m3: 1000.0,
            label: "water_textbook",
        }
    }

    #[test]
    fn cylinder_geometry_volume_matches_pi_r2_h() {
        let geometry = TankGeometry::Cylinder {
            radius_m: 0.5,
            height_m: 2.0,
        };
        let expected = std::f64::consts::PI * 0.25 * 2.0;
        assert!((geometry.volume_m3() - expected).abs() < 1e-12);
    }

    #[test]
    fn sphere_geometry_volume_matches_four_thirds_pi_r3() {
        let geometry = TankGeometry::Sphere { radius_m: 1.0 };
        let expected = (4.0 / 3.0) * std::f64::consts::PI;
        assert!((geometry.volume_m3() - expected).abs() < 1e-12);
    }

    #[test]
    fn ellipsoid_geometry_volume_matches_four_thirds_pi_abc() {
        let geometry = TankGeometry::EllipsoidTextbook {
            a_m: 1.0,
            b_m: 2.0,
            c_m: 3.0,
        };
        let expected = (4.0 / 3.0) * std::f64::consts::PI * 6.0;
        assert!((geometry.volume_m3() - expected).abs() < 1e-12);
    }

    #[test]
    fn cylinder_rejects_non_positive() {
        let bad = TankGeometry::Cylinder {
            radius_m: 0.0,
            height_m: 1.0,
        };
        assert!(matches!(bad.require_valid(), Err(TankError::InvalidGeometry { .. })));
    }

    #[test]
    fn propellant_rejects_zero_density() {
        let bad = PropellantSpec {
            density_kg_m3: 0.0,
            label: "x",
        };
        assert!(matches!(bad.require_valid(), Err(TankError::InvalidPropellant { .. })));
    }

    #[test]
    fn baffle_rejects_negative_damping() {
        let bad = BaffleModel {
            damping_increment_zeta: -0.1,
        };
        assert!(matches!(bad.require_valid(), Err(TankError::InvalidBaffle { .. })));
    }

    #[test]
    fn tank_rejects_fill_above_one() {
        let geometry = TankGeometry::Sphere { radius_m: 1.0 };
        let result = Tank::new(
            TankId::new(1),
            geometry,
            BodyId::new(1),
            Vector3::zeros(),
            finite_propellant(),
            1.5,
            None,
            Box::new(RigidLiquid::new(geometry, finite_propellant(), 1.0, Vector3::zeros()).unwrap()),
        );
        assert!(matches!(result, Err(TankError::InvalidFillFraction { .. })));
    }

    #[test]
    fn tank_initial_fluid_matches_volume_density_fill() {
        let geometry = TankGeometry::Sphere { radius_m: 1.0 };
        let propellant = finite_propellant();
        let model = RigidLiquid::new(geometry, propellant, 0.5, Vector3::zeros()).unwrap();
        let tank = Tank::new(
            TankId::new(1),
            geometry,
            BodyId::new(1),
            Vector3::zeros(),
            propellant,
            0.5,
            None,
            Box::new(model),
        )
        .unwrap();
        let expected = geometry.volume_m3() * 1000.0 * 0.5;
        assert!((tank.initial_fluid_mass_kg() - expected).abs() < 1e-9);
    }

    #[test]
    fn point_mass_inertia_zero_offset_is_zero() {
        let i = point_mass_inertia_about_origin(10.0, Vector3::zeros());
        assert!(i.iter().all(|c| c.abs() < 1e-12));
    }

    #[test]
    fn point_mass_inertia_axis_offset_matches_closed_form() {
        // Mass m at offset (d, 0, 0): I_yy = I_zz = m d², I_xx = 0,
        // off-diagonals zero.
        let m = 10.0;
        let d = 2.0;
        let i = point_mass_inertia_about_origin(m, Vector3::new(d, 0.0, 0.0));
        assert!((i[(0, 0)] - 0.0).abs() < 1e-12);
        assert!((i[(1, 1)] - m * d * d).abs() < 1e-12);
        assert!((i[(2, 2)] - m * d * d).abs() < 1e-12);
        assert!(i[(0, 1)].abs() < 1e-12);
        assert!(i[(0, 2)].abs() < 1e-12);
        assert!(i[(1, 2)].abs() < 1e-12);
    }
}
