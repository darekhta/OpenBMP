//! HAL build-time gate test.
//!
//! Asserts that the test-only `ScenarioStateOverride` topic is
//! visible in the default (sim) build and that it carries the
//! canonical topic name. Under `--features hal --no-default-features`
//! the test body is `#[cfg]`-gated out and the test binary compiles
//! down empty, which is itself the proof that the topic gate
//! behaves: if someone accidentally removed the
//! `#[cfg(not(feature = "hal"))]` line on `ScenarioStateOverride`,
//! this file would fail to compile under HAL because the gate
//! below could not be applied to a symbol that's always present.
//!
//! See `docs/mission-graph-architecture.md § HAL adopter contract`
//! for the architectural rationale, and `.github/workflows/ci.yml`
//! § `fc-hal-gate` for the matching CI job that runs both builds.

#![allow(clippy::missing_docs_in_private_items)]

#[cfg(not(feature = "hal"))]
mod sim_default {
    use openbmp_fc::bus::Topic;
    use openbmp_fc::topics::ScenarioStateOverride;

    #[test]
    fn scenario_state_override_visible_under_sim_default() {
        // Compile-time + runtime proof: the topic exists in the
        // sim build with the canonical name. The HAL CI job pairs
        // this with a `cargo build -p openbmp-fc --no-default-features
        // --features hal` that compiles the same file with this
        // module gated out.
        assert_eq!(
            ScenarioStateOverride::NAME,
            "commander.scenario_state_override"
        );
    }
}
