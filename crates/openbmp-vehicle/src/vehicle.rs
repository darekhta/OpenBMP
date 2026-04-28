//! `Vehicle` trait and the Phase-2 [`BasicVehicle`] composition.
//!
//! The architecture's long-term `Vehicle` trait is:
//!
//! ```rust,ignore
//! pub trait Vehicle<S: SimState>: ForceModel<S> + MomentModel<S> {
//!     fn force_models(&self) -> &[Box<dyn ForceModel<S>>];
//!     fn moment_models(&self) -> &[Box<dyn MomentModel<S>>];
//!     fn mass_model(&self) -> &dyn MassModel;
//! }
//! ```
//!
//! Phase 2.8 ships [`BasicVehicle`], which carries ordered force /
//! moment lists plus a single mass model. It also implements
//! `ForceModel<S>` / `MomentModel<S>` so the Phase-1 kernel can consume
//! the force and moment composition through its existing generic
//! surface while the mass model is passed to the kernel separately.
//!
//! # Composition order
//!
//! `BasicVehicle::force_n_eci` evaluates the force-model list in
//! **declared order** and sums components in a left fold with
//! locked operand order:
//!
//! ```text
//! total = 0
//! for each (name, model) in force_models:
//!     component = model.force_n_eci(ctx)?       (fail-closed)
//!     total = total + component
//! ```
//!
//! Floating-point summation is not associative, so reordering the
//! force list changes the byte output. The Phase-2 plan documents
//! this as the contract: *order matters*.
//!
//! # Per-model breakdown for telemetry
//!
//! [`BasicVehicle::evaluate_force_breakdown`] /
//! [`BasicVehicle::evaluate_moment_breakdown`] evaluate the lists and
//! return a [`ForceBreakdown`] / [`MomentBreakdown`] carrying both
//! per-model components and the total. The kernel-side adapter at
//! Phase 2.10 evaluates the breakdown once per step, uses the total for
//! dynamics, and hooks the components into telemetry as
//! `force.<name>.{x,y,z}` channels.
//!
//! # Determinism
//!
//! Pure arithmetic on `f64`; locked operand order on the summation;
//! no FMA. Vehicle composition is a pure-function operation over the
//! provided context — `BasicVehicle` itself carries no per-step state.

use nalgebra::Vector3;

use openbmp_sim::{
    ForceContext, ForceModel, MassModel, ModelEvalError, MomentContext, MomentModel, SimState,
};

use crate::error::VehicleError;

// ---------------------------------------------------------------------
// NamedForceModel / NamedMomentModel
// ---------------------------------------------------------------------

/// A named entry in a vehicle's force-model list.
pub struct NamedForceModel<S: SimState> {
    /// Stable display name (used by the breakdown publisher and the
    /// Phase-2.10 telemetry channel naming).
    pub name: String,
    /// Boxed force-model impl.
    pub model: Box<dyn ForceModel<S>>,
}

impl<S: SimState> NamedForceModel<S> {
    /// Construct from `(name, model)`.
    #[must_use]
    pub fn new(name: impl Into<String>, model: Box<dyn ForceModel<S>>) -> Self {
        Self {
            name: name.into(),
            model,
        }
    }
}

impl<S: SimState> std::fmt::Debug for NamedForceModel<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NamedForceModel")
            .field("name", &self.name)
            .field("model", &"<dyn ForceModel>")
            .finish()
    }
}

/// A named entry in a vehicle's moment-model list.
pub struct NamedMomentModel<S: SimState> {
    /// Stable display name.
    pub name: String,
    /// Boxed moment-model impl.
    pub model: Box<dyn MomentModel<S>>,
}

impl<S: SimState> NamedMomentModel<S> {
    /// Construct from `(name, model)`.
    #[must_use]
    pub fn new(name: impl Into<String>, model: Box<dyn MomentModel<S>>) -> Self {
        Self {
            name: name.into(),
            model,
        }
    }
}

impl<S: SimState> std::fmt::Debug for NamedMomentModel<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NamedMomentModel")
            .field("name", &self.name)
            .field("model", &"<dyn MomentModel>")
            .finish()
    }
}

// ---------------------------------------------------------------------
// Breakdown types
// ---------------------------------------------------------------------

/// Per-model force breakdown plus the summed total.
#[derive(Clone, Debug, PartialEq)]
pub struct ForceBreakdown {
    /// Components in declared order: `(name, force_n_eci)`.
    pub components: Vec<(String, Vector3<f64>)>,
    /// Sum of every component, evaluated in declared-order left fold.
    pub total: Vector3<f64>,
}

/// Per-model moment breakdown plus the summed total.
#[derive(Clone, Debug, PartialEq)]
pub struct MomentBreakdown {
    /// Components in declared order: `(name, moment_n_m_body)`.
    pub components: Vec<(String, Vector3<f64>)>,
    /// Sum of every component, evaluated in declared-order left fold.
    pub total: Vector3<f64>,
}

// ---------------------------------------------------------------------
// Vehicle trait
// ---------------------------------------------------------------------

/// Trait implemented by Phase-2+ vehicle compositions.
///
/// Generic over `S: SimState` so the same trait surface serves
/// point-mass and rigid-body kernels.
///
/// The Phase-2.8 close criteria are the flat force / moment / mass
/// composition surface plus deterministic breakdown evaluation. The
/// `Vehicle` trait exposes the per-model lists via
/// [`force_model_names`](Self::force_model_names) and
/// [`moment_model_names`](Self::moment_model_names), and the kernel-side
/// adapter at Phase 2.10 reads
/// [`evaluate_force_breakdown`](Self::evaluate_force_breakdown) per
/// step to publish the breakdown channels.
pub trait Vehicle<S: SimState>: ForceModel<S> + MomentModel<S> {
    /// Registered force models in declared order.
    fn force_models(&self) -> &[Box<dyn ForceModel<S>>];

    /// Registered moment models in declared order.
    fn moment_models(&self) -> &[Box<dyn MomentModel<S>>];

    /// Mass model associated with this vehicle.
    fn mass_model(&self) -> &dyn MassModel;

    /// Names of the registered force models in declared order.
    fn force_model_names(&self) -> Vec<String>;

    /// Names of the registered moment models in declared order.
    fn moment_model_names(&self) -> Vec<String>;

    /// Evaluate every force model in declared order and return both
    /// the per-model components and the summed total.
    ///
    /// # Errors
    ///
    /// Returns the first model's `ModelEvalError` (short-circuit) —
    /// later models in the list are not evaluated and the breakdown
    /// is not produced.
    fn evaluate_force_breakdown(
        &self,
        ctx: ForceContext<'_, S>,
    ) -> Result<ForceBreakdown, ModelEvalError>;

    /// Evaluate every moment model in declared order and return both
    /// the per-model components and the summed total.
    ///
    /// # Errors
    ///
    /// Returns the first model's `ModelEvalError` (short-circuit) —
    /// later models in the list are not evaluated and the breakdown
    /// is not produced.
    fn evaluate_moment_breakdown(
        &self,
        ctx: MomentContext<'_, S>,
    ) -> Result<MomentBreakdown, ModelEvalError>;
}

// ---------------------------------------------------------------------
// BasicVehicle
// ---------------------------------------------------------------------

/// Phase-2 vehicle composition: ordered force-model and moment-model
/// lists plus one mass model over a single [`SimState`] type.
pub struct BasicVehicle<S: SimState> {
    force_model_names: Vec<String>,
    force_models: Vec<Box<dyn ForceModel<S>>>,
    moment_model_names: Vec<String>,
    moment_models: Vec<Box<dyn MomentModel<S>>>,
    mass_model: BoxedMassModel,
}

impl<S: SimState> std::fmt::Debug for BasicVehicle<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BasicVehicle")
            .field("force_model_names", &self.force_model_names)
            .field("force_models", &"<dyn ForceModel list>")
            .field("moment_model_names", &self.moment_model_names)
            .field("moment_models", &"<dyn MomentModel list>")
            .field("mass_model", &self.mass_model)
            .finish()
    }
}

impl<S: SimState> BasicVehicle<S> {
    /// Construct from explicit ordered force / moment lists and a mass
    /// model.
    ///
    /// # Errors
    ///
    /// Returns [`VehicleError::MalformedVehicle`] if any model name
    /// is empty, contains whitespace, or contains `.`. Returns
    /// [`VehicleError::InvalidParameter`] if a name is duplicated within
    /// the force list or within the moment list (duplicates would make
    /// per-model telemetry breakdowns ambiguous).
    pub fn new(
        force_models: Vec<NamedForceModel<S>>,
        moment_models: Vec<NamedMomentModel<S>>,
        mass_model: Box<dyn MassModel>,
    ) -> Result<Self, VehicleError> {
        for m in &force_models {
            validate_model_name(
                &m.name,
                "force-model name is empty",
                "force-model name contains whitespace",
                "force-model name contains telemetry separator '.'",
            )?;
        }
        for m in &moment_models {
            validate_model_name(
                &m.name,
                "moment-model name is empty",
                "moment-model name contains whitespace",
                "moment-model name contains telemetry separator '.'",
            )?;
        }
        // Duplicate-name detection (O(n²) for tiny n; the lists are
        // typically 2..6 long).
        for i in 0..force_models.len() {
            for j in (i + 1)..force_models.len() {
                if force_models[i].name == force_models[j].name {
                    return Err(VehicleError::InvalidParameter {
                        reason: "duplicate force-model name in vehicle list",
                    });
                }
            }
        }
        for i in 0..moment_models.len() {
            for j in (i + 1)..moment_models.len() {
                if moment_models[i].name == moment_models[j].name {
                    return Err(VehicleError::InvalidParameter {
                        reason: "duplicate moment-model name in vehicle list",
                    });
                }
            }
        }
        let (force_model_names, force_models) = force_models
            .into_iter()
            .map(|entry| (entry.name, entry.model))
            .unzip();
        let (moment_model_names, moment_models) = moment_models
            .into_iter()
            .map(|entry| (entry.name, entry.model))
            .unzip();

        Ok(Self {
            force_model_names,
            force_models,
            moment_model_names,
            moment_models,
            mass_model: BoxedMassModel(mass_model),
        })
    }

    /// Number of registered force models.
    #[must_use]
    pub fn force_model_count(&self) -> usize {
        self.force_models.len()
    }

    /// Number of registered moment models.
    #[must_use]
    pub fn moment_model_count(&self) -> usize {
        self.moment_models.len()
    }

    /// Read-only access to the force models in declared order.
    #[must_use]
    pub fn force_models(&self) -> &[Box<dyn ForceModel<S>>] {
        &self.force_models
    }

    /// Read-only access to the moment models in declared order.
    #[must_use]
    pub fn moment_models(&self) -> &[Box<dyn MomentModel<S>>] {
        &self.moment_models
    }

    /// Read-only access to the mass model.
    #[must_use]
    pub fn mass_model(&self) -> &dyn MassModel {
        self.mass_model.0.as_ref()
    }
}

impl<S: SimState> ForceModel<S> for BasicVehicle<S> {
    fn force_n_eci(&self, ctx: ForceContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError> {
        // Locked left fold in declared order. No FMA.
        let mut total = Vector3::zeros();
        for f in &self.force_models {
            let component = f.force_n_eci(ctx)?;
            total += component;
        }
        Ok(total)
    }
}

impl<S: SimState> MomentModel<S> for BasicVehicle<S> {
    fn moment_n_m_body(&self, ctx: MomentContext<'_, S>) -> Result<Vector3<f64>, ModelEvalError> {
        let mut total = Vector3::zeros();
        for m in &self.moment_models {
            let component = m.moment_n_m_body(ctx)?;
            total += component;
        }
        Ok(total)
    }
}

impl<S: SimState> Vehicle<S> for BasicVehicle<S> {
    fn force_models(&self) -> &[Box<dyn ForceModel<S>>] {
        &self.force_models
    }

    fn moment_models(&self) -> &[Box<dyn MomentModel<S>>] {
        &self.moment_models
    }

    fn mass_model(&self) -> &dyn MassModel {
        self.mass_model.0.as_ref()
    }

    fn force_model_names(&self) -> Vec<String> {
        self.force_model_names.clone()
    }

    fn moment_model_names(&self) -> Vec<String> {
        self.moment_model_names.clone()
    }

    fn evaluate_force_breakdown(
        &self,
        ctx: ForceContext<'_, S>,
    ) -> Result<ForceBreakdown, ModelEvalError> {
        let mut total = Vector3::zeros();
        let mut components = Vec::with_capacity(self.force_models.len());
        for (name, model) in self.force_model_names.iter().zip(&self.force_models) {
            let component = model.force_n_eci(ctx)?;
            components.push((name.clone(), component));
            total += component;
        }
        Ok(ForceBreakdown { components, total })
    }

    fn evaluate_moment_breakdown(
        &self,
        ctx: MomentContext<'_, S>,
    ) -> Result<MomentBreakdown, ModelEvalError> {
        let mut total = Vector3::zeros();
        let mut components = Vec::with_capacity(self.moment_models.len());
        for (name, model) in self.moment_model_names.iter().zip(&self.moment_models) {
            let component = model.moment_n_m_body(ctx)?;
            components.push((name.clone(), component));
            total += component;
        }
        Ok(MomentBreakdown { components, total })
    }
}

fn validate_model_name(
    name: &str,
    empty_reason: &'static str,
    whitespace_reason: &'static str,
    separator_reason: &'static str,
) -> Result<(), VehicleError> {
    if name.trim().is_empty() {
        return Err(VehicleError::MalformedVehicle {
            reason: empty_reason,
        });
    }
    if name.chars().any(char::is_whitespace) {
        return Err(VehicleError::MalformedVehicle {
            reason: whitespace_reason,
        });
    }
    if name.contains('.') {
        return Err(VehicleError::MalformedVehicle {
            reason: separator_reason,
        });
    }
    Ok(())
}

// ---------------------------------------------------------------------
// Mass-model passthrough
// ---------------------------------------------------------------------

/// Convenience wrapper that turns any `Box<dyn MassModel>` into a
/// type the kernel can take through its existing generic mass-model
/// surface. Phase 2.8 ships this for symmetry with `BasicVehicle`;
/// Phase 3 will introduce a `MultiStageMass` impl directly.
pub struct BoxedMassModel(pub Box<dyn MassModel>);

impl std::fmt::Debug for BoxedMassModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BoxedMassModel")
            .field("inner", &"<dyn MassModel>")
            .finish()
    }
}

impl MassModel for BoxedMassModel {
    fn mass_kg(&self, t: openbmp_core::SimTime) -> Result<f64, ModelEvalError> {
        self.0.mass_kg(t)
    }

    fn mass_rate_kg_s(&self, t: openbmp_core::SimTime) -> Result<f64, ModelEvalError> {
        self.0.mass_rate_kg_s(t)
    }

    fn validation(&self) -> openbmp_core::ValidationStatus {
        self.0.validation()
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use super::*;
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_sim::{
        ConstantGravityForce, ConstantMass, EnvironmentSample, ZeroForce, ZeroMoment,
    };
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn fixture_state() -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        )
    }

    fn ctx<'a>(
        state: &'a PointMassState,
        env: &'a EnvironmentSample,
    ) -> ForceContext<'a, PointMassState> {
        ForceContext {
            state,
            environment: env,
            mass_kg: 1.0,
            time: SimTime::ZERO,
            effector_actuals: openbmp_sim::EffectorActualsView::empty(),
            engine_snapshot: openbmp_sim::EngineSnapshotView::empty(),
        }
    }

    fn null_env() -> EnvironmentSample {
        EnvironmentSample::default()
    }

    fn test_mass_model() -> Box<dyn MassModel> {
        Box::new(ConstantMass::new(1.0))
    }

    #[test]
    fn empty_vehicle_returns_zero_force() {
        let v: BasicVehicle<PointMassState> =
            BasicVehicle::new(vec![], vec![], test_mass_model()).unwrap();
        let state = fixture_state();
        let env = null_env();
        let f = v.force_n_eci(ctx(&state, &env)).unwrap();
        assert_eq!(f, Vector3::zeros());
    }

    #[test]
    fn single_force_model_vehicle_byte_matches_raw_model() {
        let g = ConstantGravityForce::down_z(9.806_65);
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![NamedForceModel::new("gravity", Box::new(g))],
            vec![],
            test_mass_model(),
        )
        .unwrap();

        let state = fixture_state();
        let env = null_env();
        let context = ctx(&state, &env);
        let raw = g.force_n_eci(context).unwrap();
        let through_vehicle = v.force_n_eci(context).unwrap();
        // Bit-exact: x and y components are both +0.0; z is 1·(-9.80665).
        // Vector3::zeros() + raw must equal raw.
        assert_eq!(raw.x.to_bits(), through_vehicle.x.to_bits());
        assert_eq!(raw.y.to_bits(), through_vehicle.y.to_bits());
        assert_eq!(raw.z.to_bits(), through_vehicle.z.to_bits());
    }

    #[test]
    fn two_force_models_returns_ordered_left_fold_sum() {
        let g1 = ConstantGravityForce::new(Vector3::new(1.0, 0.0, 0.0));
        let g2 = ConstantGravityForce::new(Vector3::new(0.0, 2.0, 0.0));
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("a", Box::new(g1)),
                NamedForceModel::new("b", Box::new(g2)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();
        let state = fixture_state();
        let env = null_env();
        let f = v.force_n_eci(ctx(&state, &env)).unwrap();
        assert_eq!(f, Vector3::new(1.0, 2.0, 0.0));
    }

    #[test]
    fn force_breakdown_returns_per_model_components_and_total() {
        let g1 = ConstantGravityForce::new(Vector3::new(1.0, 0.0, 0.0));
        let g2 = ConstantGravityForce::new(Vector3::new(0.0, 2.0, 0.0));
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("a", Box::new(g1)),
                NamedForceModel::new("b", Box::new(g2)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();
        let state = fixture_state();
        let env = null_env();
        let breakdown = v.evaluate_force_breakdown(ctx(&state, &env)).unwrap();
        assert_eq!(breakdown.components.len(), 2);
        assert_eq!(breakdown.components[0].0, "a");
        assert_eq!(breakdown.components[0].1, Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(breakdown.components[1].0, "b");
        assert_eq!(breakdown.components[1].1, Vector3::new(0.0, 2.0, 0.0));
        assert_eq!(breakdown.total, Vector3::new(1.0, 2.0, 0.0));
    }

    #[test]
    fn first_force_model_error_short_circuits_later_models() {
        struct AlwaysFail;
        impl ForceModel<PointMassState> for AlwaysFail {
            fn force_n_eci(
                &self,
                _ctx: ForceContext<'_, PointMassState>,
            ) -> Result<Vector3<f64>, ModelEvalError> {
                Err(ModelEvalError::OutOfEnvelope {
                    model: openbmp_core::ModelId::new(0xFA17),
                    reason: std::borrow::Cow::Borrowed("test fail"),
                })
            }
        }

        // The fail model is FIRST. The trailing model would have side
        // effects (returns gravity); confirm it's not evaluated by
        // verifying the error is the fail model's error.
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("fail", Box::new(AlwaysFail)),
                NamedForceModel::new("gravity", Box::new(ConstantGravityForce::down_z(9.806_65))),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();
        let state = fixture_state();
        let env = null_env();
        let err = v.force_n_eci(ctx(&state, &env)).unwrap_err();
        assert!(matches!(err, ModelEvalError::OutOfEnvelope { .. }));
        // The breakdown path also short-circuits.
        let err2 = v.evaluate_force_breakdown(ctx(&state, &env)).unwrap_err();
        assert!(matches!(err2, ModelEvalError::OutOfEnvelope { .. }));
    }

    #[test]
    fn reordering_force_list_changes_byte_output_for_non_trivial_summands() {
        // f64 summation is not associative for non-trivial values.
        // Pick three values that exhibit catastrophic cancellation
        // when reordered.
        let big = ConstantGravityForce::new(Vector3::new(1.0e16, 0.0, 0.0));
        let neg_big = ConstantGravityForce::new(Vector3::new(-1.0e16, 0.0, 0.0));
        let small = ConstantGravityForce::new(Vector3::new(1.0, 0.0, 0.0));

        let forward: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("big", Box::new(big)),
                NamedForceModel::new("neg_big", Box::new(neg_big)),
                NamedForceModel::new("small", Box::new(small)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();

        // Same models, different order: small first, then big, then neg_big.
        let reordered: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("small", Box::new(small)),
                NamedForceModel::new("big", Box::new(big)),
                NamedForceModel::new("neg_big", Box::new(neg_big)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();

        let state = fixture_state();
        let env = null_env();
        let fwd = forward.force_n_eci(ctx(&state, &env)).unwrap();
        let rev = reordered.force_n_eci(ctx(&state, &env)).unwrap();
        // Forward: ((1e16) + (-1e16)) + 1 = 0 + 1 = 1.0
        // Reordered: ((1.0) + 1e16) + (-1e16) = 1e16 + (-1e16) = 0.0
        // Catastrophic cancellation — byte outputs differ.
        assert_ne!(fwd.x.to_bits(), rev.x.to_bits(),);
        assert_eq!(fwd.x, 1.0);
        assert_eq!(rev.x, 0.0);
    }

    #[test]
    fn force_n_eci_is_bit_stable_across_two_evaluations() {
        let g1 = ConstantGravityForce::new(Vector3::new(1.234_567, -2.345_678, 9.806_65));
        let g2 = ConstantGravityForce::new(Vector3::new(0.123_456, 0.234_567, -0.345_678));
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("a", Box::new(g1)),
                NamedForceModel::new("b", Box::new(g2)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();
        let state = fixture_state();
        let env = null_env();
        let a = v.force_n_eci(ctx(&state, &env)).unwrap();
        let b = v.force_n_eci(ctx(&state, &env)).unwrap();
        assert_eq!(a.x.to_bits(), b.x.to_bits());
        assert_eq!(a.y.to_bits(), b.y.to_bits());
        assert_eq!(a.z.to_bits(), b.z.to_bits());
    }

    #[test]
    fn moment_models_compose_in_declared_order() {
        let zm = ZeroMoment;
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![],
            vec![NamedMomentModel::new("zero", Box::new(zm))],
            test_mass_model(),
        )
        .unwrap();
        let state = fixture_state();
        let env = null_env();
        let m = v
            .moment_n_m_body(MomentContext {
                state: &state,
                environment: &env,
                time: SimTime::ZERO,
                effector_actuals: openbmp_sim::EffectorActualsView::empty(),
                engine_snapshot: openbmp_sim::EngineSnapshotView::empty(),
            })
            .unwrap();
        assert_eq!(m, Vector3::zeros());
    }

    #[test]
    fn constructor_rejects_empty_force_model_name() {
        let zf = ZeroForce;
        let err = BasicVehicle::<PointMassState>::new(
            vec![NamedForceModel::new("", Box::new(zf))],
            vec![],
            test_mass_model(),
        )
        .unwrap_err();
        assert!(matches!(err, VehicleError::MalformedVehicle { .. }));
    }

    #[test]
    fn constructor_rejects_whitespace_force_model_name() {
        let err = BasicVehicle::<PointMassState>::new(
            vec![NamedForceModel::new("bad name", Box::new(ZeroForce))],
            vec![],
            test_mass_model(),
        )
        .unwrap_err();
        assert!(matches!(err, VehicleError::MalformedVehicle { .. }));
    }

    #[test]
    fn constructor_rejects_telemetry_separator_in_force_model_name() {
        let err = BasicVehicle::<PointMassState>::new(
            vec![NamedForceModel::new("bad.name", Box::new(ZeroForce))],
            vec![],
            test_mass_model(),
        )
        .unwrap_err();
        assert!(matches!(err, VehicleError::MalformedVehicle { .. }));
    }

    #[test]
    fn constructor_rejects_duplicate_force_model_name() {
        let err = BasicVehicle::<PointMassState>::new(
            vec![
                NamedForceModel::new("dup", Box::new(ZeroForce)),
                NamedForceModel::new("dup", Box::new(ZeroForce)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap_err();
        assert!(matches!(err, VehicleError::InvalidParameter { .. }));
    }

    #[test]
    fn constructor_rejects_duplicate_moment_model_name() {
        let err = BasicVehicle::<PointMassState>::new(
            vec![],
            vec![
                NamedMomentModel::new("dup", Box::new(ZeroMoment)),
                NamedMomentModel::new("dup", Box::new(ZeroMoment)),
            ],
            test_mass_model(),
        )
        .unwrap_err();
        assert!(matches!(err, VehicleError::InvalidParameter { .. }));
    }

    #[test]
    fn vehicle_trait_exposes_mass_model() {
        let v: BasicVehicle<PointMassState> =
            BasicVehicle::new(vec![], vec![], test_mass_model()).unwrap();
        assert_eq!(v.mass_model().mass_kg(SimTime::ZERO).unwrap(), 1.0);
        assert_eq!(
            <BasicVehicle<PointMassState> as Vehicle<PointMassState>>::mass_model(&v)
                .mass_kg(SimTime::ZERO)
                .unwrap(),
            1.0
        );
    }

    #[test]
    fn vehicle_trait_methods_return_declared_order_names() {
        let v: BasicVehicle<PointMassState> = BasicVehicle::new(
            vec![
                NamedForceModel::new("gravity", Box::new(ConstantGravityForce::down_z(9.806_65))),
                NamedForceModel::new("aero", Box::new(ZeroForce)),
                NamedForceModel::new("thrust", Box::new(ZeroForce)),
            ],
            vec![],
            test_mass_model(),
        )
        .unwrap();
        let names = v.force_model_names();
        assert_eq!(names, vec!["gravity", "aero", "thrust"]);
    }
}
