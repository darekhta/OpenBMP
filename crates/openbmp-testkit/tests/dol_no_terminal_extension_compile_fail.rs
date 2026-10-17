//! Compile-fail tripwire for day-of-launch terminal-condition vocabulary.

#[test]
fn dol_redesign_cannot_extend_terminal_condition() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/dol_terminal_condition_extension.rs");
}
