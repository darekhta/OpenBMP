//! Compile-fail tripwire for LCvx target vocabulary.

#[test]
fn lcvx_terminal_target_has_no_surface_coordinate_fields() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/trajopt_lcvx_surface_coordinate_target.rs");
}
