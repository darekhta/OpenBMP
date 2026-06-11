//! Compile-fail tripwire for trajectory-optimization target vocabulary.

#[test]
fn multiple_shooting_node_cannot_carry_surface_coordinates() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/trajopt_multiple_shooting_surface_coordinate.rs");
}
