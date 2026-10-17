use openbmp_mission::MissionAction;

fn main() {
    let _ = MissionAction::RequestSafeState {
        reason: "abort-to-safe".to_owned(),
        ground_aimpoint_eci_m: [0.0, 0.0, 0.0],
    };
}
