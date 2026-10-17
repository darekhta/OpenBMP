//! Deterministic ground-segment countdown primitives.
//!
//! This crate is the initial WP-18.1 substrate for host-side launch ground
//! logic. It models a declarative countdown DAG, evaluates step prerequisites
//! against deterministic criterion timelines, and separates count-clock time
//! from simulation time during holds. It intentionally has no runner,
//! simulator, flight-controller, wall-clock, or hardware dependency.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Ground countdown schema version.
pub const GROUND_COUNTDOWN_SCHEMA_VERSION: u16 = 1;

/// Errors returned by ground countdown helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum GroundError {
    /// A required identifier field was empty.
    #[error("{field} must be non-empty")]
    EmptyId {
        /// Field name.
        field: &'static str,
    },
    /// A scalar field was NaN or infinite.
    #[error("{field} must be finite")]
    NonFinite {
        /// Field name.
        field: &'static str,
    },
    /// A scalar field was outside its accepted range.
    #[error("{field} is outside the accepted range")]
    OutOfRange {
        /// Field name.
        field: &'static str,
    },
    /// The plan schema version is unsupported.
    #[error("ground countdown schema version is unsupported: {version}")]
    UnsupportedSchemaVersion {
        /// Observed schema version.
        version: u16,
    },
    /// The plan contains no steps.
    #[error("ground countdown plan must contain at least one step")]
    EmptyPlan,
    /// A step id or criterion id was declared more than once.
    #[error("duplicate id {id}")]
    DuplicateId {
        /// Duplicate id.
        id: String,
    },
    /// Countdown step anchors were not in deterministic ascending order.
    #[error("countdown steps must be sorted by ascending count time")]
    NonMonotonicSteps,
    /// Criterion samples were not strictly ordered by simulation time.
    #[error("criterion timeline samples must be strictly time ordered: {id}")]
    NonMonotonicCriterionSamples {
        /// Criterion id.
        id: String,
    },
    /// A step prerequisite references a later or same-time step.
    #[error("step {step_id} has an invalid step prerequisite {requirement_id}")]
    InvalidStepDependency {
        /// Step containing the bad prerequisite.
        step_id: String,
        /// Referenced step id.
        requirement_id: String,
    },
    /// A referenced external criterion timeline is absent.
    #[error("unknown countdown criterion {id}")]
    UnknownCriterion {
        /// Missing criterion id.
        id: String,
    },
}

/// Action when a countdown step's prerequisites are not met inside its window.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum CountdownFailureAction {
    /// Freeze the count clock at a declared hold point until prerequisites pass.
    Hold {
        /// Stable hold point id.
        hold_id: String,
    },
    /// Abort the countdown.
    Abort,
}

/// One declared countdown step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownStep {
    /// Stable step id.
    pub id: String,
    /// Count-clock target for the step, seconds relative to T-0.
    pub at_count_s: f64,
    /// Allowed simulation-time window to satisfy prerequisites, seconds.
    pub window_s: f64,
    /// Step ids or external criterion ids required before the step can complete.
    pub requires: Vec<String>,
    /// Failure action when prerequisites stay red past the window.
    pub on_fail: CountdownFailureAction,
    /// Ground/vehicle event ids emitted when the step completes.
    pub emits: Vec<String>,
}

/// Declarative countdown plan.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownPlan {
    /// Countdown schema version.
    pub schema_version: u16,
    /// Stable plan id.
    pub plan_id: String,
    /// Initial count-clock time, seconds relative to T-0.
    pub initial_count_s: f64,
    /// Terminal count-clock time, seconds relative to T-0.
    pub terminal_count_s: f64,
    /// Ordered countdown steps.
    pub steps: Vec<CountdownStep>,
}

impl CountdownPlan {
    /// Validate plan shape, ids, ordering, and known step dependencies.
    ///
    /// # Errors
    ///
    /// Returns [`GroundError`] when the plan is malformed.
    pub fn validate(&self) -> Result<(), GroundError> {
        validate_plan(self)
    }
}

/// One sample in an external criterion timeline.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownCriterionSample {
    /// Simulation time, seconds.
    pub sim_time_s: f64,
    /// Whether the criterion is green at and after this sample until changed.
    pub green: bool,
}

/// Deterministic external criterion timeline.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownCriterionTimeline {
    /// Stable criterion id referenced by countdown steps.
    pub id: String,
    /// Strictly increasing piecewise-constant samples.
    pub samples: Vec<CountdownCriterionSample>,
}

/// Inputs for evaluating a countdown plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CountdownEvaluationInput<'a> {
    /// External criterion timelines.
    pub criterion_timelines: &'a [CountdownCriterionTimeline],
    /// Maximum simulation time to remain in any one built-in hold, seconds.
    pub max_hold_s: f64,
}

/// Overall countdown verdict.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum CountdownVerdict {
    /// Countdown completed through the terminal count.
    Go,
    /// Countdown is still holding after the allowed evaluation interval.
    Hold,
    /// Countdown aborted.
    Abort,
}

impl CountdownVerdict {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Go => "go",
            Self::Hold => "hold",
            Self::Abort => "abort",
        }
    }
}

/// Event kind emitted by countdown evaluation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum CountdownEventKind {
    /// A countdown step completed and emitted its events.
    StepCompleted,
    /// A built-in hold was entered.
    HoldEntered,
    /// A built-in hold was released after prerequisites became green.
    HoldReleased,
    /// Countdown aborted on a step failure.
    Abort,
}

impl CountdownEventKind {
    const fn as_str(self) -> &'static str {
        match self {
            Self::StepCompleted => "step_completed",
            Self::HoldEntered => "hold_entered",
            Self::HoldReleased => "hold_released",
            Self::Abort => "abort",
        }
    }
}

/// One deterministic countdown event row.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownEvent {
    /// Event kind.
    pub kind: CountdownEventKind,
    /// Step associated with the event.
    pub step_id: String,
    /// Simulation time, seconds.
    pub sim_time_s: f64,
    /// Count-clock time, seconds relative to T-0.
    pub count_time_s: f64,
    /// Detail field such as hold id or abort reason.
    pub detail: String,
}

/// Deterministic countdown evaluation report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CountdownReport {
    /// Countdown schema version.
    pub schema_version: u16,
    /// Plan id.
    pub plan_id: String,
    /// Overall verdict.
    pub verdict: CountdownVerdict,
    /// Final simulation time, seconds.
    pub final_sim_time_s: f64,
    /// Final count-clock time, seconds relative to T-0.
    pub final_count_time_s: f64,
    /// First binding step for hold/abort verdicts.
    pub binding_step: Option<String>,
    /// Ordered countdown event rows.
    pub events: Vec<CountdownEvent>,
    /// Ordered emitted event ids from completed steps.
    pub emitted_events: Vec<String>,
}

/// Evaluate a countdown plan against deterministic criterion timelines.
///
/// # Errors
///
/// Returns [`GroundError`] when the plan or criterion timelines are malformed
/// or when a referenced criterion timeline is absent.
pub fn evaluate_countdown(
    plan: &CountdownPlan,
    input: CountdownEvaluationInput<'_>,
) -> Result<CountdownReport, GroundError> {
    plan.validate()?;
    validate_nonnegative_finite("max_hold_s", input.max_hold_s)?;
    let timelines = validate_criteria(input.criterion_timelines)?;
    let step_ids = step_id_set(plan);
    let mut completed_steps = BTreeSet::<String>::new();
    let mut events = Vec::<CountdownEvent>::new();
    let mut emitted_events = Vec::<String>::new();
    let mut current_count_s = plan.initial_count_s;
    let mut current_sim_time_s = 0.0_f64;

    for step in &plan.steps {
        let advance_s = step.at_count_s - current_count_s;
        current_sim_time_s += advance_s;
        current_count_s = step.at_count_s;
        let window_end_s = current_sim_time_s + step.window_s;
        if let Some(ready_time_s) = first_time_requirements_green(
            step,
            &step_ids,
            &completed_steps,
            &timelines,
            current_sim_time_s,
            window_end_s,
        )? {
            current_count_s += ready_time_s - current_sim_time_s;
            current_sim_time_s = ready_time_s;
            complete_step(
                step,
                current_sim_time_s,
                current_count_s,
                &mut completed_steps,
                &mut events,
                &mut emitted_events,
            );
            continue;
        }

        current_sim_time_s = window_end_s;
        current_count_s += step.window_s;
        match &step.on_fail {
            CountdownFailureAction::Abort => {
                events.push(CountdownEvent {
                    kind: CountdownEventKind::Abort,
                    step_id: step.id.clone(),
                    sim_time_s: current_sim_time_s,
                    count_time_s: current_count_s,
                    detail: "prerequisites_not_green".to_owned(),
                });
                return Ok(report_for(
                    plan,
                    CountdownVerdict::Abort,
                    current_sim_time_s,
                    current_count_s,
                    Some(step.id.clone()),
                    events,
                    emitted_events,
                ));
            }
            CountdownFailureAction::Hold { hold_id } => {
                events.push(CountdownEvent {
                    kind: CountdownEventKind::HoldEntered,
                    step_id: step.id.clone(),
                    sim_time_s: current_sim_time_s,
                    count_time_s: current_count_s,
                    detail: hold_id.clone(),
                });
                let hold_deadline_s = current_sim_time_s + input.max_hold_s;
                let Some(release_time_s) = first_time_requirements_green(
                    step,
                    &step_ids,
                    &completed_steps,
                    &timelines,
                    current_sim_time_s,
                    hold_deadline_s,
                )?
                else {
                    return Ok(report_for(
                        plan,
                        CountdownVerdict::Hold,
                        hold_deadline_s,
                        current_count_s,
                        Some(step.id.clone()),
                        events,
                        emitted_events,
                    ));
                };
                current_sim_time_s = release_time_s;
                events.push(CountdownEvent {
                    kind: CountdownEventKind::HoldReleased,
                    step_id: step.id.clone(),
                    sim_time_s: current_sim_time_s,
                    count_time_s: current_count_s,
                    detail: hold_id.clone(),
                });
                complete_step(
                    step,
                    current_sim_time_s,
                    current_count_s,
                    &mut completed_steps,
                    &mut events,
                    &mut emitted_events,
                );
            }
        }
    }

    if current_count_s < plan.terminal_count_s {
        current_sim_time_s += plan.terminal_count_s - current_count_s;
        current_count_s = plan.terminal_count_s;
    }

    Ok(report_for(
        plan,
        CountdownVerdict::Go,
        current_sim_time_s,
        current_count_s,
        None,
        events,
        emitted_events,
    ))
}

/// Render a byte-stable countdown report CSV.
#[must_use]
pub fn countdown_report_csv(report: &CountdownReport) -> String {
    let mut output = String::new();
    let _ = writeln!(output, "openbmp.ground_countdown,{}", report.schema_version);
    let _ = writeln!(output, "plan_id,{}", report.plan_id);
    let _ = writeln!(output, "verdict,{}", report.verdict.as_str());
    let _ = writeln!(output, "final_sim_time_s,{:.12e}", report.final_sim_time_s);
    let _ = writeln!(
        output,
        "final_count_time_s,{:.12e}",
        report.final_count_time_s
    );
    let _ = writeln!(
        output,
        "binding_step,{}",
        report.binding_step.as_deref().unwrap_or("")
    );
    let _ = writeln!(output, "event,step_id,sim_time_s,count_time_s,detail");
    for event in &report.events {
        let _ = writeln!(
            output,
            "{},{},{:.12e},{:.12e},{}",
            event.kind.as_str(),
            event.step_id,
            event.sim_time_s,
            event.count_time_s,
            event.detail
        );
    }
    let _ = writeln!(output, "emitted_event");
    for event_id in &report.emitted_events {
        let _ = writeln!(output, "{event_id}");
    }
    output
}

fn validate_plan(plan: &CountdownPlan) -> Result<(), GroundError> {
    if plan.schema_version != GROUND_COUNTDOWN_SCHEMA_VERSION {
        return Err(GroundError::UnsupportedSchemaVersion {
            version: plan.schema_version,
        });
    }
    validate_id("plan_id", &plan.plan_id)?;
    validate_finite("initial_count_s", plan.initial_count_s)?;
    validate_finite("terminal_count_s", plan.terminal_count_s)?;
    if plan.terminal_count_s < plan.initial_count_s {
        return Err(GroundError::OutOfRange {
            field: "terminal_count_s",
        });
    }
    if plan.steps.is_empty() {
        return Err(GroundError::EmptyPlan);
    }

    let mut ids = BTreeMap::<String, usize>::new();
    let mut previous_count_s = plan.initial_count_s;
    for (index, step) in plan.steps.iter().enumerate() {
        validate_id("step.id", &step.id)?;
        validate_finite("step.at_count_s", step.at_count_s)?;
        validate_nonnegative_finite("step.window_s", step.window_s)?;
        if step.at_count_s < previous_count_s || step.at_count_s > plan.terminal_count_s {
            return Err(GroundError::NonMonotonicSteps);
        }
        previous_count_s = step.at_count_s;
        if ids.insert(step.id.clone(), index).is_some() {
            return Err(GroundError::DuplicateId {
                id: step.id.clone(),
            });
        }
        match &step.on_fail {
            CountdownFailureAction::Hold { hold_id } => validate_id("hold_id", hold_id)?,
            CountdownFailureAction::Abort => {}
        }
        for requirement in &step.requires {
            validate_id("step.requires", requirement)?;
        }
        for emitted in &step.emits {
            validate_id("step.emits", emitted)?;
        }
    }

    for (index, step) in plan.steps.iter().enumerate() {
        for requirement in &step.requires {
            if let Some(requirement_index) = ids.get(requirement)
                && *requirement_index >= index
            {
                return Err(GroundError::InvalidStepDependency {
                    step_id: step.id.clone(),
                    requirement_id: requirement.clone(),
                });
            }
        }
    }
    Ok(())
}

fn validate_criteria(
    criteria: &[CountdownCriterionTimeline],
) -> Result<BTreeMap<String, CountdownCriterionTimeline>, GroundError> {
    let mut timelines = BTreeMap::new();
    for timeline in criteria {
        validate_id("criterion.id", &timeline.id)?;
        if timeline.samples.is_empty() {
            return Err(GroundError::OutOfRange {
                field: "criterion.samples",
            });
        }
        let mut previous = None::<f64>;
        for sample in &timeline.samples {
            validate_finite("criterion.sample.sim_time_s", sample.sim_time_s)?;
            if let Some(previous_time_s) = previous
                && sample.sim_time_s <= previous_time_s
            {
                return Err(GroundError::NonMonotonicCriterionSamples {
                    id: timeline.id.clone(),
                });
            }
            previous = Some(sample.sim_time_s);
        }
        if timelines
            .insert(timeline.id.clone(), timeline.clone())
            .is_some()
        {
            return Err(GroundError::DuplicateId {
                id: timeline.id.clone(),
            });
        }
    }
    Ok(timelines)
}

fn first_time_requirements_green(
    step: &CountdownStep,
    step_ids: &BTreeSet<String>,
    completed_steps: &BTreeSet<String>,
    timelines: &BTreeMap<String, CountdownCriterionTimeline>,
    start_s: f64,
    end_s: f64,
) -> Result<Option<f64>, GroundError> {
    let mut candidates = vec![start_s];
    for requirement in &step.requires {
        if step_ids.contains(requirement) {
            continue;
        }
        let timeline = timelines
            .get(requirement)
            .ok_or_else(|| GroundError::UnknownCriterion {
                id: requirement.clone(),
            })?;
        for sample in &timeline.samples {
            if sample.sim_time_s > start_s && sample.sim_time_s <= end_s {
                candidates.push(sample.sim_time_s);
            }
        }
    }
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|a, b| (*a - *b).abs() <= f64::EPSILON);
    for candidate in candidates {
        if requirements_green_at(step, step_ids, completed_steps, timelines, candidate)? {
            return Ok(Some(candidate));
        }
    }
    Ok(None)
}

fn requirements_green_at(
    step: &CountdownStep,
    step_ids: &BTreeSet<String>,
    completed_steps: &BTreeSet<String>,
    timelines: &BTreeMap<String, CountdownCriterionTimeline>,
    sim_time_s: f64,
) -> Result<bool, GroundError> {
    for requirement in &step.requires {
        if step_ids.contains(requirement) {
            if !completed_steps.contains(requirement) {
                return Ok(false);
            }
            continue;
        }
        let timeline = timelines
            .get(requirement)
            .ok_or_else(|| GroundError::UnknownCriterion {
                id: requirement.clone(),
            })?;
        if !criterion_green_at(timeline, sim_time_s) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn criterion_green_at(timeline: &CountdownCriterionTimeline, sim_time_s: f64) -> bool {
    let mut current = timeline.samples[0].green;
    for sample in &timeline.samples {
        if sample.sim_time_s > sim_time_s {
            break;
        }
        current = sample.green;
    }
    current
}

fn complete_step(
    step: &CountdownStep,
    sim_time_s: f64,
    count_time_s: f64,
    completed_steps: &mut BTreeSet<String>,
    events: &mut Vec<CountdownEvent>,
    emitted_events: &mut Vec<String>,
) {
    completed_steps.insert(step.id.clone());
    events.push(CountdownEvent {
        kind: CountdownEventKind::StepCompleted,
        step_id: step.id.clone(),
        sim_time_s,
        count_time_s,
        detail: step.emits.join("|"),
    });
    emitted_events.extend(step.emits.iter().cloned());
}

fn report_for(
    plan: &CountdownPlan,
    verdict: CountdownVerdict,
    final_sim_time_s: f64,
    final_count_time_s: f64,
    binding_step: Option<String>,
    events: Vec<CountdownEvent>,
    emitted_events: Vec<String>,
) -> CountdownReport {
    CountdownReport {
        schema_version: GROUND_COUNTDOWN_SCHEMA_VERSION,
        plan_id: plan.plan_id.clone(),
        verdict,
        final_sim_time_s,
        final_count_time_s,
        binding_step,
        events,
        emitted_events,
    }
}

fn step_id_set(plan: &CountdownPlan) -> BTreeSet<String> {
    plan.steps.iter().map(|step| step.id.clone()).collect()
}

fn validate_id(field: &'static str, value: &str) -> Result<(), GroundError> {
    if value.trim().is_empty() {
        return Err(GroundError::EmptyId { field });
    }
    Ok(())
}

fn validate_finite(field: &'static str, value: f64) -> Result<(), GroundError> {
    if !value.is_finite() {
        return Err(GroundError::NonFinite { field });
    }
    Ok(())
}

fn validate_nonnegative_finite(field: &'static str, value: f64) -> Result<(), GroundError> {
    validate_finite(field, value)?;
    if value < 0.0 {
        return Err(GroundError::OutOfRange { field });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn step(
        id: &str,
        at_count_s: f64,
        requires: &[&str],
        on_fail: CountdownFailureAction,
        emits: &[&str],
    ) -> CountdownStep {
        CountdownStep {
            id: id.to_owned(),
            at_count_s,
            window_s: 5.0,
            requires: requires.iter().map(|id| (*id).to_owned()).collect(),
            on_fail,
            emits: emits.iter().map(|id| (*id).to_owned()).collect(),
        }
    }

    fn plan(steps: Vec<CountdownStep>) -> CountdownPlan {
        CountdownPlan {
            schema_version: GROUND_COUNTDOWN_SCHEMA_VERSION,
            plan_id: "synthetic-countdown".to_owned(),
            initial_count_s: -60.0,
            terminal_count_s: 0.0,
            steps,
        }
    }

    fn criterion(id: &str, samples: &[(f64, bool)]) -> CountdownCriterionTimeline {
        CountdownCriterionTimeline {
            id: id.to_owned(),
            samples: samples
                .iter()
                .map(|(sim_time_s, green)| CountdownCriterionSample {
                    sim_time_s: *sim_time_s,
                    green: *green,
                })
                .collect(),
        }
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= 1.0e-12,
            "actual={actual:.16e} expected={expected:.16e}"
        );
    }

    #[test]
    fn countdown_completes_in_registration_order_and_emits_events() {
        let plan = plan(vec![
            step(
                "range_poll",
                -50.0,
                &["lcc.range_green"],
                CountdownFailureAction::Abort,
                &["gse.range_green"],
            ),
            step(
                "strongback_retract",
                -20.0,
                &["range_poll", "lcc.vehicle_green"],
                CountdownFailureAction::Hold {
                    hold_id: "built_in_t20".to_owned(),
                },
                &["gse.strongback_retract"],
            ),
        ]);
        let criteria = vec![
            criterion("lcc.range_green", &[(0.0, true)]),
            criterion("lcc.vehicle_green", &[(0.0, true)]),
        ];

        let report = evaluate_countdown(
            &plan,
            CountdownEvaluationInput {
                criterion_timelines: &criteria,
                max_hold_s: 120.0,
            },
        )
        .unwrap();

        assert_eq!(report.verdict, CountdownVerdict::Go);
        assert_close(report.final_count_time_s, 0.0);
        assert_close(report.final_sim_time_s, 60.0);
        assert_eq!(
            report.emitted_events,
            vec!["gse.range_green", "gse.strongback_retract"]
        );
        assert_eq!(report.events[0].step_id, "range_poll");
        assert_eq!(report.events[1].step_id, "strongback_retract");
    }

    #[test]
    fn countdown_hold_freezes_count_clock_until_requirement_green() {
        let plan = plan(vec![step(
            "strongback_retract",
            -40.0,
            &["lcc.vehicle_green"],
            CountdownFailureAction::Hold {
                hold_id: "built_in_t40".to_owned(),
            },
            &["gse.strongback_retract"],
        )]);
        let criteria = vec![criterion(
            "lcc.vehicle_green",
            &[(0.0, false), (70.0, true)],
        )];

        let report = evaluate_countdown(
            &plan,
            CountdownEvaluationInput {
                criterion_timelines: &criteria,
                max_hold_s: 120.0,
            },
        )
        .unwrap();

        assert_eq!(report.verdict, CountdownVerdict::Go);
        assert_eq!(report.events[0].kind, CountdownEventKind::HoldEntered);
        assert_close(report.events[0].sim_time_s, 25.0);
        assert_close(report.events[0].count_time_s, -35.0);
        assert_eq!(report.events[1].kind, CountdownEventKind::HoldReleased);
        assert_close(report.events[1].sim_time_s, 70.0);
        assert_close(report.events[1].count_time_s, -35.0);
        assert_eq!(report.events[2].kind, CountdownEventKind::StepCompleted);
        assert_close(report.events[2].sim_time_s, 70.0);
        assert_close(report.events[2].count_time_s, -35.0);
        assert_close(report.final_sim_time_s, 105.0);
        assert_close(report.final_count_time_s, 0.0);
    }

    #[test]
    fn countdown_abort_latches_failed_step() {
        let plan = plan(vec![step(
            "engine_start_permission",
            -10.0,
            &["lcc.engines_green"],
            CountdownFailureAction::Abort,
            &["gse.engine_start"],
        )]);
        let criteria = vec![criterion("lcc.engines_green", &[(0.0, false)])];

        let report = evaluate_countdown(
            &plan,
            CountdownEvaluationInput {
                criterion_timelines: &criteria,
                max_hold_s: 120.0,
            },
        )
        .unwrap();

        assert_eq!(report.verdict, CountdownVerdict::Abort);
        assert_eq!(
            report.binding_step.as_deref(),
            Some("engine_start_permission")
        );
        assert_eq!(report.events[0].kind, CountdownEventKind::Abort);
        assert!(report.emitted_events.is_empty());
    }

    #[test]
    fn countdown_rejects_late_step_dependency_and_unknown_criterion() {
        let invalid = plan(vec![
            step(
                "strongback_retract",
                -30.0,
                &["engine_start"],
                CountdownFailureAction::Abort,
                &["gse.strongback_retract"],
            ),
            step(
                "engine_start",
                -5.0,
                &[],
                CountdownFailureAction::Abort,
                &["gse.engine_start"],
            ),
        ]);
        assert!(matches!(
            invalid.validate(),
            Err(GroundError::InvalidStepDependency { .. })
        ));

        let unknown = plan(vec![step(
            "range_poll",
            -50.0,
            &["lcc.range_green"],
            CountdownFailureAction::Abort,
            &["gse.range_green"],
        )]);
        assert!(matches!(
            evaluate_countdown(
                &unknown,
                CountdownEvaluationInput {
                    criterion_timelines: &[],
                    max_hold_s: 120.0,
                }
            ),
            Err(GroundError::UnknownCriterion { .. })
        ));
    }

    #[test]
    fn countdown_report_csv_is_byte_stable() {
        let plan = plan(vec![step(
            "range_poll",
            -50.0,
            &["lcc.range_green"],
            CountdownFailureAction::Abort,
            &["gse.range_green"],
        )]);
        let criteria = vec![criterion("lcc.range_green", &[(0.0, true)])];
        let report = evaluate_countdown(
            &plan,
            CountdownEvaluationInput {
                criterion_timelines: &criteria,
                max_hold_s: 120.0,
            },
        )
        .unwrap();

        assert_eq!(
            countdown_report_csv(&report),
            "openbmp.ground_countdown,1\n\
             plan_id,synthetic-countdown\n\
             verdict,go\n\
             final_sim_time_s,6.000000000000e1\n\
             final_count_time_s,0.000000000000e0\n\
             binding_step,\n\
             event,step_id,sim_time_s,count_time_s,detail\n\
             step_completed,range_poll,1.000000000000e1,-5.000000000000e1,gse.range_green\n\
             emitted_event\n\
             gse.range_green\n"
        );
    }
}
