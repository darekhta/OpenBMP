//! `proptest::Strategy` constructors for the OpenBMP foundation and
//! state types.
//!
//! All strategies use deterministic generators; the only randomness is
//! the `proptest` runner's own seeded PRNG. Generated values respect
//! the type's invariants (finite, normalised, etc.) where applicable.

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{
    Acceleration3, AngularVelocity3, Body, ChannelId, Displacement3, Duration, Eci, Frame, ModelId,
    Position3, Quaternion, ScenarioId, SimTime, StepIndex, Velocity3, VelocityDelta3,
};
use openbmp_state::{MassProperties, PointMassState, RigidBodyState};
use proptest::prelude::*;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

// ---------------------------------------------------------------------
// Time + IDs
// ---------------------------------------------------------------------

/// Strategy for [`SimTime`] within `[min_s, max_s]`.
///
/// The range must be finite and `min_s <= max_s`.
pub fn sim_time(min_s: f64, max_s: f64) -> impl Strategy<Value = SimTime> {
    (min_s..=max_s).prop_map(SimTime::from_seconds)
}

/// Strategy for non-negative [`SimTime`] values up to `max_s`.
pub fn sim_time_nonneg(max_s: f64) -> impl Strategy<Value = SimTime> {
    sim_time(0.0, max_s)
}

/// Strategy for [`Duration`] within `[min_s, max_s]`.
pub fn duration(min_s: f64, max_s: f64) -> impl Strategy<Value = Duration> {
    (min_s..=max_s).prop_map(Duration::from_seconds)
}

/// Strategy for strictly-positive finite [`Duration`].
pub fn duration_positive(max_s: f64) -> impl Strategy<Value = Duration> {
    (f64::MIN_POSITIVE..=max_s).prop_map(Duration::from_seconds)
}

/// Strategy for [`StepIndex`] within `[min, max]`.
pub fn step_index(min: u64, max: u64) -> impl Strategy<Value = StepIndex> {
    (min..=max).prop_map(StepIndex::new)
}

/// Strategy for [`ChannelId`] within `[min, max]`.
pub fn channel_id(min: u64, max: u64) -> impl Strategy<Value = ChannelId> {
    (min..=max).prop_map(ChannelId::new)
}

/// Strategy for [`ModelId`] within `[min, max]`.
pub fn model_id(min: u64, max: u64) -> impl Strategy<Value = ModelId> {
    (min..=max).prop_map(ModelId::new)
}

/// Strategy for [`ScenarioId`] within `[min, max]`.
pub fn scenario_id(min: u64, max: u64) -> impl Strategy<Value = ScenarioId> {
    (min..=max).prop_map(ScenarioId::new)
}

// ---------------------------------------------------------------------
// Frame-tagged vectors
// ---------------------------------------------------------------------

/// Strategy for a finite component within `[min, max]`.
pub fn finite_component(min: f64, max: f64) -> impl Strategy<Value = f64> {
    (min..=max).prop_filter("must be finite", |v: &f64| v.is_finite())
}

/// Strategy for a finite [`Vector3<f64>`] with each component in
/// `[min, max]`.
pub fn vector3(min: f64, max: f64) -> impl Strategy<Value = Vector3<f64>> {
    (
        finite_component(min, max),
        finite_component(min, max),
        finite_component(min, max),
    )
        .prop_map(|(x, y, z)| Vector3::new(x, y, z))
}

/// Strategy for a finite [`Position3<F>`] with each component in
/// `[min, max]`.
pub fn position3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = Position3<F>> {
    vector3(min, max).prop_map(Position3::from_vector)
}

/// Strategy for a finite [`Displacement3<F>`] with each component in
/// `[min, max]`.
pub fn displacement3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = Displacement3<F>> {
    vector3(min, max).prop_map(Displacement3::from_vector)
}

/// Strategy for a finite [`Velocity3<F>`] with each component in
/// `[min, max]`.
pub fn velocity3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = Velocity3<F>> {
    vector3(min, max).prop_map(Velocity3::from_vector)
}

/// Strategy for a finite [`Acceleration3<F>`] with each component in
/// `[min, max]`.
pub fn acceleration3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = Acceleration3<F>> {
    vector3(min, max).prop_map(Acceleration3::from_vector)
}

/// Strategy for a finite [`VelocityDelta3<F>`] with each component in
/// `[min, max]`.
pub fn velocity_delta3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = VelocityDelta3<F>> {
    vector3(min, max).prop_map(VelocityDelta3::from_vector)
}

/// Strategy for a finite [`AngularVelocity3<F>`] with each component
/// in `[min, max]`.
pub fn angular_velocity3<F: Frame + std::fmt::Debug>(
    min: f64,
    max: f64,
) -> impl Strategy<Value = AngularVelocity3<F>> {
    vector3(min, max).prop_map(AngularVelocity3::from_vector)
}

// ---------------------------------------------------------------------
// Quaternions (always normalised by construction)
// ---------------------------------------------------------------------

/// Strategy for a unit [`Quaternion`] from an axis-angle pair.
///
/// The axis is sampled from a non-zero candidate vector and
/// normalised; the angle is sampled from `(-π, π]`.
pub fn quaternion<From: Frame + std::fmt::Debug, To: Frame + std::fmt::Debug>()
-> impl Strategy<Value = Quaternion<From, To>> {
    use std::f64::consts::PI;
    (
        finite_component(-1.0, 1.0),
        finite_component(-1.0, 1.0),
        finite_component(-1.0, 1.0),
        finite_component(-PI, PI),
    )
        .prop_filter("axis must be non-degenerate", |&(ax, ay, az, _)| {
            (ax * ax + ay * ay + az * az).sqrt() > 1.0e-3
        })
        .prop_map(|(ax, ay, az, angle)| {
            let axis = nalgebra::Unit::new_normalize(Vector3::new(ax, ay, az));
            let q = UnitQuaternion::from_axis_angle(&axis, angle);
            Quaternion::from_unit_quaternion(q)
        })
}

// ---------------------------------------------------------------------
// State types
// ---------------------------------------------------------------------

/// Strategy for a strictly-positive [`Mass`] in kilograms within
/// `[min_kg, max_kg]`.
pub fn mass_kg(min_kg: f64, max_kg: f64) -> impl Strategy<Value = Mass> {
    (min_kg..=max_kg)
        .prop_filter("mass must be finite and positive", |mass_kg| {
            mass_kg.is_finite() && *mass_kg > 0.0
        })
        .prop_map(Mass::new::<kilogram>)
}

/// Strategy for [`MassProperties`] with strictly-positive diagonal
/// inertia within `[i_min, i_max]` (kg*m^2), rigid-body triangle
/// inequalities, and zero off-diagonal terms.
pub fn mass_properties_diagonal(
    mass_min_kg: f64,
    mass_max_kg: f64,
    i_min: f64,
    i_max: f64,
) -> impl Strategy<Value = MassProperties> {
    (
        mass_kg(mass_min_kg, mass_max_kg),
        position3::<Body>(-1.0, 1.0),
        finite_component(i_min, i_max),
        finite_component(i_min, i_max),
        finite_component(i_min, i_max),
    )
        .prop_filter(
            "principal moments must satisfy triangle inequalities",
            |(_, _, ixx, iyy, izz)| {
                let ixx = *ixx;
                let iyy = *iyy;
                let izz = *izz;
                ixx > 0.0
                    && iyy > 0.0
                    && izz > 0.0
                    && ixx + iyy >= izz
                    && ixx + izz >= iyy
                    && iyy + izz >= ixx
            },
        )
        .prop_map(|(mass, com, ixx, iyy, izz)| {
            MassProperties::with_diagonal_inertia(mass, com, ixx, iyy, izz)
        })
}

/// Strategy for a [`PointMassState`] with deterministic ranges suited
/// to academic-toy scenarios.
pub fn point_mass_state(
    time_max_s: f64,
    pos_max_m: f64,
    vel_max_m_s: f64,
    mass_min_kg: f64,
    mass_max_kg: f64,
) -> impl Strategy<Value = PointMassState> {
    (
        sim_time_nonneg(time_max_s),
        position3::<Eci>(-pos_max_m, pos_max_m),
        velocity3::<Eci>(-vel_max_m_s, vel_max_m_s),
        mass_kg(mass_min_kg, mass_max_kg),
    )
        .prop_map(|(time, position, velocity, mass)| {
            PointMassState::new(time, position, velocity, mass)
        })
}

/// Strategy for a [`RigidBodyState`] with deterministic ranges suited
/// to academic-toy scenarios. The orientation is normalised by
/// construction; the inertia tensor has a strictly-positive diagonal.
#[allow(clippy::too_many_arguments)]
pub fn rigid_body_state(
    time_max_s: f64,
    pos_max_m: f64,
    vel_max_m_s: f64,
    omega_max_rad_s: f64,
    mass_min_kg: f64,
    mass_max_kg: f64,
    i_min: f64,
    i_max: f64,
) -> impl Strategy<Value = RigidBodyState> {
    (
        sim_time_nonneg(time_max_s),
        position3::<Eci>(-pos_max_m, pos_max_m),
        velocity3::<Eci>(-vel_max_m_s, vel_max_m_s),
        quaternion::<Body, Eci>(),
        angular_velocity3::<Body>(-omega_max_rad_s, omega_max_rad_s),
        mass_properties_diagonal(mass_min_kg, mass_max_kg, i_min, i_max),
    )
        .prop_map(|(time, position, velocity, orientation, omega, props)| {
            RigidBodyState::new(time, position, velocity, orientation, omega, props)
        })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    proptest! {
        #[test]
        fn sim_time_strategy_in_range(t in sim_time(-10.0, 10.0)) {
            prop_assert!(t.as_seconds() >= -10.0);
            prop_assert!(t.as_seconds() <= 10.0);
            prop_assert!(t.is_finite());
        }

        #[test]
        fn step_index_strategy_in_range(s in step_index(0, 1_000_000)) {
            prop_assert!(s.value() <= 1_000_000);
        }

        #[test]
        fn position3_strategy_finite(p in position3::<Eci>(-1e6, 1e6)) {
            prop_assert!(p.is_finite());
        }

        #[test]
        fn displacement3_strategy_finite(d in displacement3::<Eci>(-1e6, 1e6)) {
            prop_assert!(d.is_finite());
        }

        #[test]
        fn acceleration3_strategy_finite(a in acceleration3::<Eci>(-100.0, 100.0)) {
            prop_assert!(a.is_finite());
        }

        #[test]
        fn velocity_delta3_strategy_finite(dv in velocity_delta3::<Eci>(-100.0, 100.0)) {
            prop_assert!(dv.is_finite());
        }

        #[test]
        fn quaternion_strategy_normalised(q in quaternion::<Body, Eci>()) {
            prop_assert!(q.is_normalised(1.0e-9));
        }

        #[test]
        fn mass_properties_strategy_valid(
            mp in mass_properties_diagonal(0.1, 10.0, 0.1, 10.0),
        ) {
            prop_assert!(mp.is_finite());
            prop_assert!(mp.require_valid(0.0).is_ok());
        }

        #[test]
        fn point_mass_state_strategy_valid(
            s in point_mass_state(60.0, 1.0e7, 1.0e4, 0.1, 1.0e3),
        ) {
            prop_assert!(s.require_valid().is_ok());
        }

        #[test]
        fn rigid_body_state_strategy_valid(
            s in rigid_body_state(
                60.0, 1.0e7, 1.0e4, 10.0, 0.1, 1.0e3, 0.1, 100.0,
            ),
        ) {
            prop_assert!(s.is_finite());
            prop_assert!(s.is_normalised(1.0e-9));
            prop_assert!(s.require_valid(1.0e-9, 1.0e-9).is_ok());
        }
    }
}
