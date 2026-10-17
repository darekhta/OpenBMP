//! Compile-fail tripwire for vehicle-bound tracking observable inputs.

#[test]
fn tracking_observable_input_cannot_be_literal_constructed() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/tracking_observable_input_literal.rs");
}
