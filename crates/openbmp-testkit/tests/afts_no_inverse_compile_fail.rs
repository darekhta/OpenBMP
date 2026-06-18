//! Compile-fail tripwire for the forward-only AFTS monitor surface.

#[test]
fn afts_propagator_cannot_solve_burn_to_reach_polygon() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/afts_solve_burn_to_reach.rs");
}
