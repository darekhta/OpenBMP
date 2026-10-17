use openbmp_trajopt::ScvxTerminalStateTarget;

fn main() {
    let _ = ScvxTerminalStateTarget {
        state: vec![0.0, 0.0, 0.0],
        weights: None,
        target_latitude_rad: 0.0,
        target_longitude_rad: 0.0,
        range_m: 0.0,
        aimpoint_eci_m: [0.0, 0.0, 0.0],
    };
}
