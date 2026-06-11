//! Compile-fail tripwire for the synthetic-only rare-event limit-state surface.

#[test]
fn limit_state_cannot_be_implemented_outside_openbmp_mc() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/limit_state_ground_aimpoint.rs");
}
