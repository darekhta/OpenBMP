//! Runner-side ground-segment report wiring.
//!
//! This module materializes schema-v3 `[countdown]` declarations into the
//! dependency-light `openbmp-ground` countdown evaluator. It is deliberately
//! out-of-band for kernel behavior: the report does not alter forces, mission
//! state, or stop conditions. Scenarios that opt into `[countdown]` also get
//! deterministic summary telemetry derived from the report.

use openbmp_core::ChannelId;
use openbmp_ground::{
    CountdownEvaluationInput, CountdownFailureAction, CountdownPlan, CountdownReport,
    CountdownStep, CountdownVerdict, GROUND_COUNTDOWN_SCHEMA_VERSION, countdown_report_csv,
    evaluate_countdown,
};
use openbmp_scenario::{
    CountdownConfig, CountdownStepConfig, CountdownStepFailureConfig, ScenarioDocument,
};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};

use crate::RunnerError;

/// Runner-side countdown report emitted for `[countdown]` scenarios.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundCountdownRunReport {
    /// Evaluator report with verdict, final times, binding step, and event rows.
    pub report: CountdownReport,
    /// Byte-stable CSV rendering of `report`.
    pub csv: Vec<u8>,
}

impl GroundCountdownRunReport {
    /// Overall countdown verdict.
    #[must_use]
    pub const fn verdict(&self) -> CountdownVerdict {
        self.report.verdict
    }
}

/// Opt-in telemetry channels summarizing the ground countdown report.
#[derive(Debug)]
pub(crate) struct GroundCountdownTelemetryChannels {
    verdict: TelemetryChannel<String>,
    verdict_code: TelemetryChannel<i64>,
    final_sim_time: TelemetryChannel<f64>,
    final_count_time: TelemetryChannel<f64>,
    event_count: TelemetryChannel<i64>,
    emitted_event_count: TelemetryChannel<i64>,
    binding_step: TelemetryChannel<String>,
}

impl GroundCountdownTelemetryChannels {
    /// Construct countdown telemetry channels when `[countdown]` is present.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when a channel cannot be built.
    pub(crate) fn maybe_new(
        document: &ScenarioDocument,
        alloc: &mut impl FnMut() -> ChannelId,
    ) -> Result<Option<Self>, RunnerError> {
        if document.countdown.is_none() {
            return Ok(None);
        }
        Ok(Some(Self {
            verdict: TelemetryChannel::<String>::new(
                alloc(),
                "ground.countdown.verdict",
                "text",
                None::<&str>,
            )?,
            verdict_code: TelemetryChannel::<i64>::new(
                alloc(),
                "ground.countdown.verdict_code",
                "1",
                None::<&str>,
            )?,
            final_sim_time: TelemetryChannel::<f64>::new(
                alloc(),
                "ground.countdown.final_sim_time_s",
                "s",
                None::<&str>,
            )?,
            final_count_time: TelemetryChannel::<f64>::new(
                alloc(),
                "ground.countdown.final_count_time_s",
                "s",
                None::<&str>,
            )?,
            event_count: TelemetryChannel::<i64>::new(
                alloc(),
                "ground.countdown.event_count",
                "1",
                None::<&str>,
            )?,
            emitted_event_count: TelemetryChannel::<i64>::new(
                alloc(),
                "ground.countdown.emitted_event_count",
                "1",
                None::<&str>,
            )?,
            binding_step: TelemetryChannel::<String>::new(
                alloc(),
                "ground.countdown.binding_step",
                "text",
                None::<&str>,
            )?,
        }))
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        channels.push(self.verdict.metadata().clone());
        channels.push(self.verdict_code.metadata().clone());
        channels.push(self.final_sim_time.metadata().clone());
        channels.push(self.final_count_time.metadata().clone());
        channels.push(self.event_count.metadata().clone());
        channels.push(self.emitted_event_count.metadata().clone());
        channels.push(self.binding_step.metadata().clone());
    }

    /// Insert countdown summary values from the precomputed run report.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError::Telemetry`] when a row insertion fails or the
    /// report is missing for declared channels.
    pub(crate) fn insert_samples(
        &self,
        row: &mut TelemetryRow,
        report: Option<&GroundCountdownRunReport>,
    ) -> Result<(), RunnerError> {
        let report = report.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[countdown] telemetry requested without a countdown report".to_owned(),
        })?;
        row.insert(
            &self.verdict,
            countdown_verdict_label(report.report.verdict).to_owned(),
        )?;
        row.insert(
            &self.verdict_code,
            countdown_verdict_code(report.report.verdict),
        )?;
        row.insert(&self.final_sim_time, report.report.final_sim_time_s)?;
        row.insert(&self.final_count_time, report.report.final_count_time_s)?;
        row.insert(&self.event_count, report.report.events.len() as i64)?;
        row.insert(
            &self.emitted_event_count,
            report.report.emitted_events.len() as i64,
        )?;
        row.insert(
            &self.binding_step,
            report.report.binding_step.clone().unwrap_or_default(),
        )?;
        Ok(())
    }
}

const fn countdown_verdict_label(verdict: CountdownVerdict) -> &'static str {
    match verdict {
        CountdownVerdict::Go => "go",
        CountdownVerdict::Hold => "hold",
        CountdownVerdict::Abort => "abort",
    }
}

const fn countdown_verdict_code(verdict: CountdownVerdict) -> i64 {
    match verdict {
        CountdownVerdict::Go => 0,
        CountdownVerdict::Hold => 1,
        CountdownVerdict::Abort => 2,
    }
}

/// Evaluate the parsed `[countdown]` block, if present.
///
/// # Errors
///
/// Returns [`RunnerError`] if scenario helper parsing fails or the ground
/// countdown evaluator rejects the materialized plan.
pub fn evaluate_countdown_document(
    document: &ScenarioDocument,
) -> Result<Option<GroundCountdownRunReport>, RunnerError> {
    let Some(countdown) = document.countdown.as_ref() else {
        return Ok(None);
    };
    let plan = materialize_plan(countdown)?;
    let report = evaluate_countdown(
        &plan,
        CountdownEvaluationInput {
            criterion_timelines: &[],
            max_hold_s: countdown.max_hold_s,
        },
    )?;
    let csv = countdown_report_csv(&report).into_bytes();
    Ok(Some(GroundCountdownRunReport { report, csv }))
}

fn materialize_plan(countdown: &CountdownConfig) -> Result<CountdownPlan, RunnerError> {
    let steps = countdown
        .steps
        .iter()
        .enumerate()
        .map(|(index, step)| materialize_step(index, step))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(CountdownPlan {
        schema_version: GROUND_COUNTDOWN_SCHEMA_VERSION,
        plan_id: countdown
            .id
            .clone()
            .unwrap_or_else(|| "scenario-countdown".to_owned()),
        initial_count_s: countdown.initial_count_s()?,
        terminal_count_s: countdown.terminal_count_s()?,
        steps,
    })
}

fn materialize_step(
    index: usize,
    step: &CountdownStepConfig,
) -> Result<CountdownStep, RunnerError> {
    Ok(CountdownStep {
        id: step.id.clone(),
        at_count_s: step.at_count_s(index)?,
        window_s: step.window_s,
        requires: step.requires.clone(),
        on_fail: match step.failure_action(index)? {
            CountdownStepFailureConfig::Hold { hold_id } => {
                CountdownFailureAction::Hold { hold_id }
            }
            CountdownStepFailureConfig::Abort => CountdownFailureAction::Abort,
        },
        emits: step.emits.clone(),
    })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use openbmp_ground::CountdownVerdict;
    use openbmp_scenario::Scenario;
    use openbmp_telemetry::TelemetryValue;

    const MINIMAL: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    const COUNTDOWN_BLOCK: &str = r#"
[countdown]
id = "runner-terminal-count"
initial = "T-00:00:10"
terminal = "T0"
max_hold_s = 0.0

[[countdown.step]]
id = "autosequence_start"
at = "T-00:00:10"
on_fail = "abort"
emits = ["ground.autosequence_start"]

[[countdown.step]]
id = "engine_start"
at = "T-00:00:03"
requires = ["autosequence_start"]
on_fail = "abort"
emits = ["vehicle.engine_start"]
"#;

    fn v3_minimal(extra: &str) -> Scenario {
        let toml =
            format!("{MINIMAL}{extra}").replace("openbmp.scenario = 2", "openbmp.scenario = 3");
        Scenario::from_toml_str(&toml).expect("scenario parses")
    }

    fn has_channel(outcome: &crate::RunOutcome, name: &str) -> bool {
        outcome
            .table
            .schema()
            .channels()
            .iter()
            .any(|channel| channel.name == name)
    }

    fn first_row_value<'a>(outcome: &'a crate::RunOutcome, name: &str) -> &'a TelemetryValue {
        let channel = outcome
            .table
            .schema()
            .channels()
            .iter()
            .find(|channel| channel.name == name)
            .unwrap_or_else(|| panic!("channel `{name}` must exist"));
        let row = outcome.table.rows().first().expect("initial row exists");
        row.get(channel.id)
            .unwrap_or_else(|| panic!("channel `{name}` must have an initial value"))
    }

    #[test]
    fn runner_emits_countdown_report_and_summary_telemetry() {
        let baseline = crate::run(&v3_minimal("")).expect("baseline run");
        assert!(baseline.countdown.is_none());
        assert!(!has_channel(&baseline, "ground.countdown.verdict"));

        let counted = crate::run(&v3_minimal(COUNTDOWN_BLOCK)).expect("countdown run");
        assert_eq!(
            first_row_value(&counted, "ground.countdown.verdict"),
            &TelemetryValue::Text("go".to_owned())
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.verdict_code"),
            &TelemetryValue::Int64(0)
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.final_sim_time_s"),
            &TelemetryValue::Float64(10.0)
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.final_count_time_s"),
            &TelemetryValue::Float64(0.0)
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.event_count"),
            &TelemetryValue::Int64(2)
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.emitted_event_count"),
            &TelemetryValue::Int64(2)
        );
        assert_eq!(
            first_row_value(&counted, "ground.countdown.binding_step"),
            &TelemetryValue::Text(String::new())
        );

        let countdown = counted.countdown.expect("countdown report emitted");
        assert_eq!(countdown.verdict(), CountdownVerdict::Go);
        assert_eq!(countdown.report.plan_id, "runner-terminal-count");
        assert_eq!(countdown.report.events.len(), 2);
        assert_eq!(countdown.report.emitted_events.len(), 2);
        assert_eq!(
            std::str::from_utf8(&countdown.csv).expect("countdown CSV is UTF-8"),
            "openbmp.ground_countdown,1\n\
             plan_id,runner-terminal-count\n\
             verdict,go\n\
             final_sim_time_s,1.000000000000e1\n\
             final_count_time_s,0.000000000000e0\n\
             binding_step,\n\
             event,step_id,sim_time_s,count_time_s,detail\n\
             step_completed,autosequence_start,0.000000000000e0,-1.000000000000e1,ground.autosequence_start\n\
             step_completed,engine_start,7.000000000000e0,-3.000000000000e0,vehicle.engine_start\n\
             emitted_event\n\
             ground.autosequence_start\n\
             vehicle.engine_start\n"
        );
    }
}
