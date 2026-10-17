//! Compile-fail tripwire for SCvx target vocabulary.

#[test]
fn scvx_terminal_target_has_no_surface_coordinate_fields() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/trajopt_scvx_surface_coordinate_target.rs");
}
