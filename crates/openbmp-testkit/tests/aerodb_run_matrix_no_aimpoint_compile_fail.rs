//! Compile-fail tripwire for condition-only aero run matrices.

#![allow(clippy::expect_used)]

#[test]
fn aerodb_run_node_has_no_ground_aimpoint_builder() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/aerodb_run_node_ground_aimpoint.rs");
}
