use openbmp_trajopt::{MultipleShootingNode, TwoBodyCartesianState};

fn main() {
    let state =
        TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0]).unwrap();

    let _ = MultipleShootingNode {
        state,
        target_latitude_rad: 0.0,
        target_longitude_rad: 0.0,
    };
}
