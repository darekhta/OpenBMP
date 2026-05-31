use nalgebra::Vector3;
use openbmp_core::SimTime;
use openbmp_physics::AtmosphereModel;

use crate::topics::PositionEstimate;

pub(crate) const GEOCENTRIC_RADIUS_THRESHOLD_M: f64 = 1.0e6;
pub(crate) const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

pub(crate) fn altitude_m(position_eci_m: Vector3<f64>) -> f64 {
    let radius_m = position_eci_m.norm();
    if radius_m > GEOCENTRIC_RADIUS_THRESHOLD_M {
        (radius_m - EARTH_MEAN_RADIUS_M).max(0.0)
    } else {
        position_eci_m.z
    }
}

pub(crate) fn vertical_velocity_m_s(
    position_eci_m: Vector3<f64>,
    velocity_eci_m_s: Vector3<f64>,
) -> f64 {
    let radius_m = position_eci_m.norm();
    if radius_m > GEOCENTRIC_RADIUS_THRESHOLD_M {
        velocity_eci_m_s.dot(&position_eci_m) / radius_m
    } else {
        velocity_eci_m_s.z
    }
}

pub(crate) fn flight_path_angle_rad(
    position_eci_m: Vector3<f64>,
    velocity_eci_m_s: Vector3<f64>,
) -> f64 {
    let speed_m_s = velocity_eci_m_s.norm();
    if speed_m_s <= 0.0 {
        return 0.0;
    }
    (vertical_velocity_m_s(position_eci_m, velocity_eci_m_s) / speed_m_s)
        .clamp(-1.0, 1.0)
        .asin()
}

pub(crate) fn air_relative_velocity_eci_m_s(position: &PositionEstimate) -> Vector3<f64> {
    let omega = openbmp_physics::frames::WGS84_OMEGA_RAD_S;
    let r = position.position_eci_m;
    let atmosphere_velocity = Vector3::new(-omega * r.y, omega * r.x, 0.0);
    position.velocity_eci_m_s - atmosphere_velocity
}

pub(crate) fn dynamic_pressure_air_relative(position: &PositionEstimate) -> f64 {
    let altitude_m = altitude_m(position.position_eci_m).max(0.0);
    let density_kg_m3 = openbmp_physics::UsStandard1976::new()
        .sample(altitude_m, SimTime::ZERO)
        .map_or(0.0, |sample| sample.density_kg_m3);
    0.5 * density_kg_m3 * air_relative_velocity_eci_m_s(position).norm_squared()
}
