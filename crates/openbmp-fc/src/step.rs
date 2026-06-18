//! Portable single-step FC entry point.
//!
//! This module factors the target-facing "publish inputs, dispatch one
//! controller tick, capture outputs" boundary without depending on the
//! simulator runner or bridge crates. Host SIL/PIL transports can adapt
//! their packet format into these topic-level samples outside
//! `openbmp-fc`.

use openbmp_core::{SimTime, StepIndex};

use crate::bus::Topic;
use crate::topics::{
    ActuatorCommand, AirDataSample, BarometerSample, EffectorCommandSet, EngineCommandSet,
    EngineDemand, FailsafeFlags, FdirStatus, GnssSample, ImuIncrementWindow, ImuSample,
    MagnetometerSample, StarTrackerSample, VehicleStatus,
};
use crate::{ControllerError, DispatchSummary, FlightController, JobTimingObserver};

/// Topic-level input bundle for one FC tick.
#[derive(Clone, Debug, PartialEq)]
pub struct FcStepInput {
    /// Simulation/controller time for this tick.
    pub time: SimTime,
    /// Monotonic controller tick.
    pub tick: StepIndex,
    /// Optional IMU sample to publish before dispatch.
    pub imu: Option<ImuSample>,
    /// Optional high-rate IMU increment window to publish before dispatch.
    pub imu_increments: Option<ImuIncrementWindow>,
    /// Optional barometer sample to publish before dispatch.
    pub barometer: Option<BarometerSample>,
    /// Optional air-data sample to publish before dispatch.
    pub airdata: Option<AirDataSample>,
    /// Optional GNSS sample to publish before dispatch.
    pub gnss: Option<GnssSample>,
    /// Optional magnetometer sample to publish before dispatch.
    pub magnetometer: Option<MagnetometerSample>,
    /// Optional star-tracker sample to publish before dispatch.
    pub star_tracker: Option<StarTrackerSample>,
}

impl FcStepInput {
    /// Build an empty input bundle for a tick.
    #[must_use]
    pub const fn new(time: SimTime, tick: StepIndex) -> Self {
        Self {
            time,
            tick,
            imu: None,
            imu_increments: None,
            barometer: None,
            airdata: None,
            gnss: None,
            magnetometer: None,
            star_tracker: None,
        }
    }

    /// Attach an IMU sample.
    #[must_use]
    pub const fn with_imu(mut self, sample: ImuSample) -> Self {
        self.imu = Some(sample);
        self
    }

    /// Attach a high-rate IMU inertial-increment window.
    #[must_use]
    pub fn with_imu_increment_window(mut self, sample: ImuIncrementWindow) -> Self {
        self.imu_increments = Some(sample);
        self
    }

    /// Attach a barometer sample.
    #[must_use]
    pub const fn with_barometer(mut self, sample: BarometerSample) -> Self {
        self.barometer = Some(sample);
        self
    }

    /// Attach an air-data sample.
    #[must_use]
    pub const fn with_airdata(mut self, sample: AirDataSample) -> Self {
        self.airdata = Some(sample);
        self
    }

    /// Attach a GNSS sample.
    #[must_use]
    pub const fn with_gnss(mut self, sample: GnssSample) -> Self {
        self.gnss = Some(sample);
        self
    }

    /// Attach a magnetometer sample.
    #[must_use]
    pub const fn with_magnetometer(mut self, sample: MagnetometerSample) -> Self {
        self.magnetometer = Some(sample);
        self
    }

    /// Attach a star-tracker sample.
    #[must_use]
    pub const fn with_star_tracker(mut self, sample: StarTrackerSample) -> Self {
        self.star_tracker = Some(sample);
        self
    }
}

impl Default for FcStepInput {
    fn default() -> Self {
        Self::new(SimTime::ZERO, StepIndex::ZERO)
    }
}

/// Output snapshot captured after one FC tick.
#[derive(Copy, Clone, Debug, Default)]
pub struct FcStepOutput {
    /// Scheduler dispatch summary for the tick.
    pub dispatch: DispatchSummary,
    /// Latest raw autopilot actuator command, if published.
    pub actuator_command: Option<ActuatorCommand>,
    /// Latest phase-gated effector command set, if published.
    pub effector_commands: Option<EffectorCommandSet>,
    /// Latest raw engine demand, if published.
    pub engine_demand: Option<EngineDemand>,
    /// Latest phase-gated engine command set, if published.
    pub engine_commands: Option<EngineCommandSet>,
    /// Latest FDIR status, if published.
    pub fdir_status: Option<FdirStatus>,
    /// Latest health/failsafe flags, if published.
    pub failsafe_flags: Option<FailsafeFlags>,
    /// Latest vehicle/mission status, if published.
    pub vehicle_status: Option<VehicleStatus>,
}

/// Publish one input bundle, dispatch one FC tick, and capture output topics.
///
/// This is the stable topic-level entry point intended for host SIL and
/// future PIL firmware shims. It is deliberately not a bridge-packet API:
/// packet codecs, UART/GDB-stub framing, and simulator transports live
/// outside the hardware-portable FC crate.
///
/// # Errors
///
/// Returns [`ControllerError`] when an input or output topic has not been
/// registered, or when the controller scheduler/job dispatch fails.
pub fn fc_step(
    controller: &mut FlightController,
    input: FcStepInput,
) -> Result<FcStepOutput, ControllerError> {
    publish_inputs(controller, &input)?;
    let dispatch = controller.step(input.time, input.tick)?;
    capture_outputs(controller, dispatch)
}

/// Publish one input bundle, dispatch one FC tick through a timing observer,
/// and capture output topics.
///
/// This is the host-instrumented companion to [`fc_step`]. It preserves the
/// same topic-level input/output contract while letting runner-side SIL/PIL
/// adapters collect per-job timing evidence when configured.
///
/// # Errors
///
/// Returns [`ControllerError`] when an input or output topic has not been
/// registered, when the controller scheduler/job dispatch fails, or when the
/// observer reports a timing-side failure.
pub fn fc_step_with_timing_observer<O: JobTimingObserver>(
    controller: &mut FlightController,
    input: FcStepInput,
    observer: &mut O,
) -> Result<FcStepOutput, ControllerError> {
    publish_inputs(controller, &input)?;
    let dispatch = controller.step_with_timing_observer(input.time, input.tick, observer)?;
    capture_outputs(controller, dispatch)
}

fn capture_outputs(
    controller: &FlightController,
    dispatch: DispatchSummary,
) -> Result<FcStepOutput, ControllerError> {
    Ok(FcStepOutput {
        dispatch,
        actuator_command: latest::<ActuatorCommand>(controller)?,
        effector_commands: latest::<EffectorCommandSet>(controller)?,
        engine_demand: latest::<EngineDemand>(controller)?,
        engine_commands: latest::<EngineCommandSet>(controller)?,
        fdir_status: latest::<FdirStatus>(controller)?,
        failsafe_flags: latest::<FailsafeFlags>(controller)?,
        vehicle_status: latest::<VehicleStatus>(controller)?,
    })
}

fn publish_inputs(
    controller: &FlightController,
    input: &FcStepInput,
) -> Result<(), ControllerError> {
    if let Some(sample) = input.imu {
        controller.bus().publish(sample)?;
    }
    if let Some(sample) = &input.imu_increments {
        controller.bus().publish(sample.clone())?;
    }
    if let Some(sample) = input.barometer {
        controller.bus().publish(sample)?;
    }
    if let Some(sample) = input.airdata {
        controller.bus().publish(sample)?;
    }
    if let Some(sample) = input.gnss {
        controller.bus().publish(sample)?;
    }
    if let Some(sample) = input.magnetometer {
        controller.bus().publish(sample)?;
    }
    if let Some(sample) = input.star_tracker {
        controller.bus().publish(sample)?;
    }
    Ok(())
}

fn latest<T: Topic>(controller: &FlightController) -> Result<Option<T>, ControllerError> {
    Ok(controller.bus().latest::<T>()?.map(|(value, _)| value))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::FlightControllerBuilder;
    use crate::scheduler::{Job, JobContext};
    use crate::topics::ImuInertialIncrement;

    struct CommandJob;

    impl Job for CommandJob {
        fn name(&self) -> &'static str {
            "test.command"
        }

        fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            let (imu, _) = ctx
                .bus
                .latest::<ImuSample>()?
                .expect("IMU sample should be published before dispatch");
            ctx.bus.publish(ActuatorCommand {
                time: imu.time,
                elevator_rad: imu.gyro_rad_s.z,
                aileron_rad: imu.accel_m_s2.z,
                rudder_rad: 0.0,
                body_flap_rad: 0.0,
                saturated: false,
            })?;
            Ok(())
        }
    }

    struct IncrementWindowCommandJob;

    impl Job for IncrementWindowCommandJob {
        fn name(&self) -> &'static str {
            "test.increment_window_command"
        }

        fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            let (window, _) = ctx
                .bus
                .latest::<ImuIncrementWindow>()?
                .expect("IMU increment window should be published before dispatch");
            ctx.bus.publish(ActuatorCommand {
                time: window.time,
                elevator_rad: window.increments[0].delta_theta_rad.z,
                aileron_rad: window.increments[0].delta_v_m_s.x,
                rudder_rad: window.increments[1].seq as f64,
                body_flap_rad: 0.0,
                saturated: false,
            })?;
            Ok(())
        }
    }

    struct AirDataCommandJob;

    impl Job for AirDataCommandJob {
        fn name(&self) -> &'static str {
            "test.airdata_command"
        }

        fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
            let (airdata, _) = ctx
                .bus
                .latest::<AirDataSample>()?
                .expect("airdata sample should be published before dispatch");
            ctx.bus.publish(ActuatorCommand {
                time: airdata.time,
                elevator_rad: airdata.angle_of_attack_rad,
                aileron_rad: airdata.sideslip_rad,
                rudder_rad: airdata.mach,
                body_flap_rad: airdata.pressure_altitude_m,
                saturated: false,
            })?;
            Ok(())
        }
    }

    struct CountingTimingObserver {
        jobs: usize,
    }

    impl JobTimingObserver for CountingTimingObserver {
        fn run_job(
            &mut self,
            _job_name: &'static str,
            _declared_budget_us: u64,
            job: &mut dyn Job,
            ctx: &JobContext<'_>,
        ) -> Result<Option<u64>, ControllerError> {
            self.jobs += 1;
            job.run(ctx)?;
            Ok(Some(25))
        }
    }

    fn register_input_topics(fc: &FlightController) {
        fc.bus().register::<ImuSample>().unwrap();
        fc.bus().register::<ImuIncrementWindow>().unwrap();
        fc.bus().register::<BarometerSample>().unwrap();
        fc.bus().register::<AirDataSample>().unwrap();
        fc.bus().register::<GnssSample>().unwrap();
        fc.bus().register::<MagnetometerSample>().unwrap();
        fc.bus().register::<StarTrackerSample>().unwrap();
    }

    fn register_output_topics(fc: &FlightController) {
        fc.bus().register::<ActuatorCommand>().unwrap();
        fc.bus().register::<EffectorCommandSet>().unwrap();
        fc.bus().register::<EngineDemand>().unwrap();
        fc.bus().register::<EngineCommandSet>().unwrap();
        fc.bus().register::<FdirStatus>().unwrap();
        fc.bus().register::<FailsafeFlags>().unwrap();
        fc.bus().register::<VehicleStatus>().unwrap();
    }

    #[test]
    fn fc_step_publishes_inputs_dispatches_and_captures_outputs() {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(CommandJob))
            .unwrap();

        let time = SimTime::from_seconds(0.125);
        let output = fc_step(
            &mut fc,
            FcStepInput::new(time, StepIndex::new(7)).with_imu(ImuSample {
                time,
                gyro_rad_s: Vector3::new(0.0, 0.0, 0.25),
                accel_m_s2: Vector3::new(0.0, 0.0, 9.81),
                healthy: true,
            }),
        )
        .expect("fc step");

        assert_eq!(output.dispatch.run_count, 1);
        assert_eq!(output.dispatch.overrun_count, 0);
        let command = output
            .actuator_command
            .expect("command job should publish actuator command");
        assert_eq!(command.time, time);
        assert_eq!(command.elevator_rad, 0.25);
        assert_eq!(command.aileron_rad, 9.81);
    }

    #[test]
    fn fc_step_publishes_imu_increment_window_before_dispatch() {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(IncrementWindowCommandJob))
            .unwrap();

        let time = SimTime::from_seconds(0.125);
        let output = fc_step(
            &mut fc,
            FcStepInput::new(time, StepIndex::new(7)).with_imu_increment_window(
                ImuIncrementWindow {
                    time,
                    increments: vec![
                        ImuInertialIncrement {
                            delta_theta_rad: Vector3::new(1.0e-4, 2.0e-4, 3.0e-4),
                            delta_v_m_s: Vector3::new(0.001, 0.002, 0.003),
                            dt_s: 0.00025,
                            seq: 28,
                        },
                        ImuInertialIncrement {
                            delta_theta_rad: Vector3::new(4.0e-4, 5.0e-4, 6.0e-4),
                            delta_v_m_s: Vector3::new(0.004, 0.005, 0.006),
                            dt_s: 0.00025,
                            seq: 29,
                        },
                    ],
                    healthy: true,
                },
            ),
        )
        .expect("fc step");

        let command = output
            .actuator_command
            .expect("increment-window job should publish actuator command");
        assert_eq!(command.time, time);
        assert_eq!(command.elevator_rad.to_bits(), 3.0e-4_f64.to_bits());
        assert_eq!(command.aileron_rad.to_bits(), 0.001_f64.to_bits());
        assert_eq!(command.rudder_rad.to_bits(), 29.0_f64.to_bits());
    }

    #[test]
    fn fc_step_with_timing_observer_preserves_step_contract() {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(CommandJob))
            .unwrap();
        let mut observer = CountingTimingObserver { jobs: 0 };

        let time = SimTime::from_seconds(0.25);
        let output = fc_step_with_timing_observer(
            &mut fc,
            FcStepInput::new(time, StepIndex::new(9)).with_imu(ImuSample {
                time,
                gyro_rad_s: Vector3::new(0.0, 0.0, -0.125),
                accel_m_s2: Vector3::new(0.0, 0.0, 4.0),
                healthy: true,
            }),
            &mut observer,
        )
        .expect("fc step with observer");

        assert_eq!(observer.jobs, 1);
        assert_eq!(output.dispatch.run_count, 1);
        let command = output
            .actuator_command
            .expect("command job should publish actuator command");
        assert_eq!(command.time, time);
        assert_eq!(command.elevator_rad, -0.125);
        assert_eq!(command.aileron_rad, 4.0);
    }

    #[test]
    fn fc_step_publishes_airdata_input_before_dispatch() {
        let mut fc = FlightControllerBuilder::new()
            .frame_budget_us(1_000)
            .build();
        register_input_topics(&fc);
        register_output_topics(&fc);
        fc.scheduler_mut()
            .register_periodic(1, 100, 1, Box::new(AirDataCommandJob))
            .unwrap();

        let time = SimTime::from_seconds(0.5);
        let output = fc_step(
            &mut fc,
            FcStepInput::new(time, StepIndex::new(12)).with_airdata(AirDataSample {
                time,
                static_pressure_pa: 88_500.0,
                impact_pressure_pa: 1_250.0,
                mach: 0.15,
                calibrated_airspeed_m_s: 51.0,
                true_airspeed_m_s: 52.0,
                angle_of_attack_rad: 0.02,
                sideslip_rad: -0.01,
                pressure_altitude_m: 1_100.0,
                healthy: true,
            }),
        )
        .expect("fc step");

        let command = output
            .actuator_command
            .expect("airdata job should publish actuator command");
        assert_eq!(command.time, time);
        assert_eq!(command.elevator_rad.to_bits(), 0.02_f64.to_bits());
        assert_eq!(command.aileron_rad.to_bits(), (-0.01_f64).to_bits());
        assert_eq!(command.rudder_rad.to_bits(), 0.15_f64.to_bits());
        assert_eq!(command.body_flap_rad.to_bits(), 1_100.0_f64.to_bits());
    }

    #[test]
    fn fc_step_fails_closed_when_input_topic_is_unregistered() {
        let mut fc = FlightControllerBuilder::new().build();
        register_output_topics(&fc);
        let err = fc_step(
            &mut fc,
            FcStepInput::new(SimTime::ZERO, StepIndex::ZERO).with_barometer(BarometerSample {
                time: SimTime::ZERO,
                pressure_pa: 101_325.0,
                bias_pa: 0.0,
                healthy: true,
            }),
        )
        .expect_err("barometer input should fail before dispatch when its topic is unregistered");

        assert!(
            matches!(err, ControllerError::Bus(_)),
            "unexpected error: {err}"
        );
    }
}
