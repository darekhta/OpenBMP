use openbmp_core::SimTime;
use openbmp_physics::profile::BallisticState;

fn main() {
    let _ = BallisticState {
        position_eci_m: [0.0, 0.0, 0.0],
        velocity_eci_m_s: [0.0, 0.0, 0.0],
        ballistic_coefficient_m2_kg: 0.0,
        time: SimTime::ZERO,
    };
}
