use openbmp_trajopt::LcvxTerminalStateTarget;

fn main() {
    let _ = LcvxTerminalStateTarget {
        position_m: [0.0, 0.0, 0.0],
        velocity_m_s: [0.0, 0.0, 0.0],
        target_latitude_rad: 0.0,
        target_longitude_rad: 0.0,
        range_m: 0.0,
        aimpoint_eci_m: [0.0, 0.0, 0.0],
    };
}
