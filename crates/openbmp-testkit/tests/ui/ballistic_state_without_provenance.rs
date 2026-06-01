use openbmp_core::SimTime;
use openbmp_physics::profile::BallisticState;

fn main() {
    let _ = BallisticState::from_forward_simulation(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        0.0,
        SimTime::ZERO,
    );
}
