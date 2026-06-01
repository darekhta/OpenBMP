//! Compile-fail tripwires for forward-only footprint seed provenance.

#![allow(clippy::expect_used)]

#[test]
fn ballistic_state_struct_literal_is_not_constructible_downstream() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/ballistic_state_literal.rs");
}
