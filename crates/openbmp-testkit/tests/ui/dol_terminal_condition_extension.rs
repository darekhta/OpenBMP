use openbmp_physics::profile::TerminalCondition;

fn main() {
    let _ = TerminalCondition::DayOfLaunchLoadRatio {
        max_load_ratio: 1.0,
    };
}
