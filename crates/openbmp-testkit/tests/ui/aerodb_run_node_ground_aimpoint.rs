use openbmp_aerodb::RunNode;

fn main() {
    let node = RunNode::new(5.0, 0.0, 0.0, 1000.0, 1.0e6, 0.0).unwrap();
    let _ = node.with_ground_aimpoint_eci_m([0.0, 0.0, 0.0]);
}
