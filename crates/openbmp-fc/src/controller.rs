//! Top-level flight-controller façade.
//!
//! [`FlightController`] composes the lockstep clock, the internal
//! pub/sub bus, the cyclic scheduler, the parameter registry, the
//! table registry, and the dictionary into a single unit owned by
//! the embedder. The runner (or a downstream HAL adopter) constructs
//! one via [`FlightControllerBuilder`], registers the controller's
//! topics / params / tables / jobs, and drives it via
//! [`FlightController::step`] once per kernel tick.

use crate::bus::Bus;
use crate::clock::SimulatedClock;
use crate::dictionary::Dictionary;
use crate::error::ControllerError;
use crate::params::Parameters;
use crate::scheduler::{
    DeadlineSlipEvent, DispatchSummary, OverrunEvent, Scheduler, TimingBudgetReport,
};
use crate::tables::Tables;

/// Top-level flight-controller façade.
pub struct FlightController {
    bus: Bus,
    clock: SimulatedClock,
    scheduler: Scheduler,
    params: Parameters,
    tables: Tables,
}

impl FlightController {
    /// Borrows the internal pub/sub bus.
    #[must_use]
    pub fn bus(&self) -> &Bus {
        &self.bus
    }

    /// Borrows the clock.
    #[must_use]
    pub fn clock(&self) -> &SimulatedClock {
        &self.clock
    }

    /// Borrows the scheduler immutably.
    #[must_use]
    pub fn scheduler(&self) -> &Scheduler {
        &self.scheduler
    }

    /// Borrows the scheduler mutably; used by the embedder to register
    /// jobs at startup.
    pub fn scheduler_mut(&mut self) -> &mut Scheduler {
        &mut self.scheduler
    }

    /// Borrows the parameter registry immutably.
    #[must_use]
    pub fn params(&self) -> &Parameters {
        &self.params
    }

    /// Borrows the parameter registry mutably; used by the embedder
    /// to declare and update sections.
    pub fn params_mut(&mut self) -> &mut Parameters {
        &mut self.params
    }

    /// Borrows the table registry immutably.
    #[must_use]
    pub fn tables(&self) -> &Tables {
        &self.tables
    }

    /// Borrows the table registry mutably; used by the embedder to
    /// declare and replace tables.
    pub fn tables_mut(&mut self) -> &mut Tables {
        &mut self.tables
    }

    /// Returns a [`Dictionary`] view of the current introspectable
    /// surface. Useful for boot-time JSON dumps and CI assertions.
    #[must_use]
    pub fn dictionary(&self) -> Dictionary<'_> {
        Dictionary::new(&self.bus, &self.params, &self.tables, &self.scheduler)
    }

    /// Drives one controller step:
    /// 1. Update the clock.
    /// 2. Dispatch all due jobs.
    ///
    /// The runner advances the kernel state separately and feeds new
    /// sensor samples into the bus *before* calling `step`. Returns a
    /// [`DispatchSummary`] for telemetry / debugging.
    ///
    /// # Errors
    ///
    /// Propagates any error from a job; aborts the frame on the first
    /// error and surfaces it.
    pub fn step(
        &mut self,
        time: openbmp_core::SimTime,
        tick: openbmp_core::StepIndex,
    ) -> Result<DispatchSummary, ControllerError> {
        self.clock.set(time, tick);
        self.scheduler
            .dispatch(tick.value(), &self.bus, &self.clock)
    }
}

impl std::fmt::Debug for FlightController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FlightController")
            .field("topics", &self.bus.topics().len())
            .field("params", &self.params.sections().len())
            .field("tables", &self.tables.tables().len())
            .field("jobs", &self.scheduler.jobs().len())
            .field("clock", &self.clock)
            .finish_non_exhaustive()
    }
}

/// Builder for [`FlightController`]. Use [`FlightControllerBuilder::new`]
/// for an empty configuration; in real use the embedder wires up
/// topic registration, param/table declaration, and job registration
/// before calling [`FlightControllerBuilder::build`].
#[derive(Debug, Default)]
pub struct FlightControllerBuilder {
    frame_budget_us: u64,
}

impl FlightControllerBuilder {
    /// Constructs a builder with default frame budget (1 ms — typical
    /// 1 kHz controller). Embedders override via
    /// [`FlightControllerBuilder::frame_budget_us`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            frame_budget_us: 1_000,
        }
    }

    /// Sets the per-frame budget in microseconds.
    #[must_use]
    pub fn frame_budget_us(mut self, budget_us: u64) -> Self {
        self.frame_budget_us = budget_us;
        self
    }

    /// Builds the controller. Topics / parameters / tables / jobs are
    /// registered after this returns by calling the corresponding
    /// `_mut` accessors on the controller.
    #[must_use]
    pub fn build(self) -> FlightController {
        let controller = FlightController {
            bus: Bus::new(),
            clock: SimulatedClock::new(),
            scheduler: Scheduler::new(self.frame_budget_us),
            params: Parameters::new(),
            tables: Tables::new(),
        };
        // Always register the scheduler's overrun event topic so jobs
        // can publish overruns even before any module-level topic is
        // declared.
        let _ = controller.bus.register::<OverrunEvent>();
        let _ = controller.bus.register::<DeadlineSlipEvent>();
        let _ = controller.bus.register::<TimingBudgetReport>();
        controller
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss
)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use openbmp_core::{SimTime, StepIndex};

    use super::*;
    use crate::clock::Clock as _;
    use crate::scheduler::{Job, JobContext};

    struct Tick {
        counter: Rc<Cell<u32>>,
    }

    impl Job for Tick {
        fn name(&self) -> &'static str {
            "test.tick"
        }
        fn run(&mut self, _ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            self.counter.set(self.counter.get() + 1);
            Ok(())
        }
    }

    #[test]
    fn step_advances_clock_and_dispatches() {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        let counter = Rc::new(Cell::new(0u32));
        fc.scheduler_mut()
            .register_periodic(
                1,
                100,
                10,
                Box::new(Tick {
                    counter: counter.clone(),
                }),
            )
            .unwrap();

        for k in 0..10u64 {
            let _ = fc
                .step(SimTime::from_seconds(k as f64 * 0.001), StepIndex::new(k))
                .unwrap();
        }
        assert_eq!(counter.get(), 10);
        assert_eq!(fc.clock().now().as_seconds(), 9.0 * 0.001);
    }
}
