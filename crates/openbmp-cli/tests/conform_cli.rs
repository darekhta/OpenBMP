//! CLI smoke tests for `openbmp conform`.

#![allow(clippy::unwrap_used)]

use assert_cmd::Command;
use predicates::prelude::*;

#[test]
fn conform_runs_reference_scenario_without_writing_outputs() {
    let scenario = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios/analytic-toy/constant-acceleration-drop.toml");
    let mut cmd = Command::cargo_bin("openbmp").unwrap();
    cmd.arg("conform")
        .arg(scenario)
        .assert()
        .success()
        .stdout(predicate::str::contains("openbmp conform: ok"))
        .stdout(predicate::str::contains("constant-acceleration-drop.toml"));
}
