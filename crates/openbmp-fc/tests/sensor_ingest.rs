//! Sensor-ingest integration tests.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]

#[cfg(feature = "sim-default")]
mod synthetic_adapter_path {
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::{Eci, Position3, SensorId, SimTime, StepIndex, Velocity3};
    use openbmp_fc::bus::Bus;
    use openbmp_fc::clock::SimulatedClock;
    use openbmp_fc::scheduler::{Job, JobContext};
    use openbmp_fc::sensor_ingest::VotedImuIngest;
    use openbmp_fc::topics::{ImuSample, SensorStatus};
    use openbmp_fc::voter::MidValueSelectScalar;
    use openbmp_sensors::{
        SensorMeasurement, SensorTruth, SyntheticSensor, SyntheticSensorAdapter,
    };

    #[derive(Debug)]
    struct FixedSyntheticImu {
        id: SensorId,
        gyro: Vector3<f64>,
        accel: Vector3<f64>,
    }

    impl SyntheticSensor for FixedSyntheticImu {
        fn sensor_id(&self) -> SensorId {
            self.id
        }

        fn measure(
            &mut self,
            _truth: &SensorTruth,
            _step: StepIndex,
            _scenario_seed: u64,
        ) -> Result<SensorMeasurement, openbmp_sensors::SensorError> {
            Ok(SensorMeasurement::Imu {
                gyro_rad_s: self.gyro,
                accel_m_s2: self.accel,
            })
        }
    }

    fn truth(time: SimTime) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::<Eci>::origin(),
            velocity_eci: Velocity3::<Eci>::zero(),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            angular_acceleration_body_rad_s2: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::new(0.0, 0.0, 9.806_65),
            static_pressure_pa: 101_325.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time,
        }
    }

    #[test]
    fn voted_imu_uses_synthetic_sensor_adapter_and_publishes_lane_status() {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        bus.register::<SensorStatus>().unwrap();
        let clock = SimulatedClock::new();
        let now = SimTime::from_seconds(0.01);
        clock.set(now, StepIndex::new(10));

        let mut sensors = vec![
            SyntheticSensorAdapter::new(FixedSyntheticImu {
                id: SensorId::from_path("sensors.imu.a"),
                gyro: Vector3::new(0.0, 0.0, 0.0),
                accel: Vector3::new(0.0, 0.0, 9.8),
            }),
            SyntheticSensorAdapter::new(FixedSyntheticImu {
                id: SensorId::from_path("sensors.imu.b"),
                gyro: Vector3::new(0.1, 0.0, 0.0),
                accel: Vector3::new(0.1, 0.0, 9.9),
            }),
            SyntheticSensorAdapter::new(FixedSyntheticImu {
                id: SensorId::from_path("sensors.imu.c"),
                gyro: Vector3::new(5.0, 0.0, 0.0),
                accel: Vector3::new(5.0, 0.0, 14.0),
            }),
        ];
        for sensor in &mut sensors {
            sensor.prime(truth(now), StepIndex::new(10), 42);
        }

        let mut ingest = VotedImuIngest::new(
            sensors,
            MidValueSelectScalar {
                divergence_tol: 0.5,
            },
        );
        ingest
            .run(&JobContext {
                bus: &bus,
                clock: &clock,
            })
            .unwrap();

        let (imu, _) = bus.latest::<ImuSample>().unwrap().unwrap();
        assert_eq!(imu.gyro_rad_s.x, 0.1);
        assert_eq!(imu.accel_m_s2.z, 9.9);
        assert!(!imu.healthy);

        let (status, _) = bus.latest::<SensorStatus>().unwrap().unwrap();
        assert_eq!(status.lane_count, 3);
        assert!(status.any_divergent);
        assert!(!status.lanes[0].divergent);
        assert!(!status.lanes[1].divergent);
        assert!(status.lanes[2].divergent);
    }
}

mod voted_status_for_all_sensor_kinds {
    use nalgebra::Vector3;
    use openbmp_core::{SensorId, SimTime, StepIndex};
    use openbmp_fc::bus::Bus;
    use openbmp_fc::clock::FixedClock;
    use openbmp_fc::scheduler::{Job, JobContext};
    use openbmp_fc::sensor_ingest::{
        VotedBarometerIngest, VotedGnssIngest, VotedMagnetometerIngest,
    };
    use openbmp_fc::topics::{
        BarometerSample, GnssSample, MagnetometerSample, SensorKind, SensorStatus,
    };
    use openbmp_fc::voter::MidValueSelectScalar;
    use openbmp_sensors::{Sensor, SensorError, SensorMeasurement, Timestamped};

    #[derive(Clone, Debug)]
    struct FixedLane {
        id: SensorId,
        value: SensorMeasurement,
    }

    impl Sensor for FixedLane {
        type Output = SensorMeasurement;

        fn sensor_id(&self) -> SensorId {
            self.id
        }

        fn read(&mut self) -> Result<Timestamped<Self::Output>, SensorError> {
            Ok(Timestamped::new(
                SimTime::from_seconds(0.01),
                self.value.clone(),
            ))
        }
    }

    fn context<'a>(bus: &'a Bus, clock: &'a FixedClock) -> JobContext<'a> {
        JobContext { bus, clock }
    }

    #[test]
    fn barometer_ingest_publishes_lane_status() {
        let bus = Bus::new();
        bus.register::<BarometerSample>().unwrap();
        bus.register::<SensorStatus>().unwrap();
        let clock = FixedClock::new(SimTime::from_seconds(0.02), StepIndex::new(2));
        let mut ingest = VotedBarometerIngest::new(
            vec![
                FixedLane {
                    id: SensorId::from_path("sensors.baro.a"),
                    value: SensorMeasurement::Barometer {
                        pressure_pa: 100.0,
                        bias_pa: 0.0,
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.baro.b"),
                    value: SensorMeasurement::Barometer {
                        pressure_pa: 101.0,
                        bias_pa: 0.0,
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.baro.c"),
                    value: SensorMeasurement::Barometer {
                        pressure_pa: 500.0,
                        bias_pa: 0.0,
                    },
                },
            ],
            MidValueSelectScalar {
                divergence_tol: 10.0,
            },
        );

        ingest.run(&context(&bus, &clock)).unwrap();

        let (sample, _) = bus.latest::<BarometerSample>().unwrap().unwrap();
        let (status, _) = bus.latest::<SensorStatus>().unwrap().unwrap();
        assert_eq!(sample.pressure_pa, 101.0);
        assert_eq!(status.kind, SensorKind::Barometer);
        assert_eq!(status.lane_count, 3);
        assert!(status.any_divergent);
        assert!(status.lanes[2].divergent);
    }

    #[test]
    fn gnss_ingest_publishes_lane_status() {
        let bus = Bus::new();
        bus.register::<GnssSample>().unwrap();
        bus.register::<SensorStatus>().unwrap();
        let clock = FixedClock::new(SimTime::from_seconds(0.02), StepIndex::new(2));
        let mut ingest = VotedGnssIngest::new(
            vec![
                FixedLane {
                    id: SensorId::from_path("sensors.gnss.a"),
                    value: SensorMeasurement::Gnss {
                        position_eci_m: Vector3::new(0.0, 0.0, 0.0),
                        velocity_eci_m_s: Vector3::zeros(),
                        position_bias_eci_m: Vector3::zeros(),
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.gnss.b"),
                    value: SensorMeasurement::Gnss {
                        position_eci_m: Vector3::new(1.0, 0.0, 0.0),
                        velocity_eci_m_s: Vector3::zeros(),
                        position_bias_eci_m: Vector3::zeros(),
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.gnss.c"),
                    value: SensorMeasurement::Gnss {
                        position_eci_m: Vector3::new(100.0, 0.0, 0.0),
                        velocity_eci_m_s: Vector3::zeros(),
                        position_bias_eci_m: Vector3::zeros(),
                    },
                },
            ],
            MidValueSelectScalar {
                divergence_tol: 10.0,
            },
        );

        ingest.run(&context(&bus, &clock)).unwrap();

        let (sample, _) = bus.latest::<GnssSample>().unwrap().unwrap();
        let (status, _) = bus.latest::<SensorStatus>().unwrap().unwrap();
        assert_eq!(sample.position_eci_m.x, 1.0);
        assert_eq!(status.kind, SensorKind::Gnss);
        assert_eq!(status.lane_count, 3);
        assert!(status.any_divergent);
        assert!(status.lanes[2].divergent);
    }

    #[test]
    fn magnetometer_ingest_publishes_lane_status() {
        let bus = Bus::new();
        bus.register::<MagnetometerSample>().unwrap();
        bus.register::<SensorStatus>().unwrap();
        let clock = FixedClock::new(SimTime::from_seconds(0.02), StepIndex::new(2));
        let mut ingest = VotedMagnetometerIngest::new(
            vec![
                FixedLane {
                    id: SensorId::from_path("sensors.mag.a"),
                    value: SensorMeasurement::Magnetometer {
                        field_body_nt: Vector3::new(30_000.0, 0.0, 0.0),
                        hard_iron_body_nt: Vector3::zeros(),
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.mag.b"),
                    value: SensorMeasurement::Magnetometer {
                        field_body_nt: Vector3::new(30_010.0, 0.0, 0.0),
                        hard_iron_body_nt: Vector3::zeros(),
                    },
                },
                FixedLane {
                    id: SensorId::from_path("sensors.mag.c"),
                    value: SensorMeasurement::Magnetometer {
                        field_body_nt: Vector3::new(40_000.0, 0.0, 0.0),
                        hard_iron_body_nt: Vector3::zeros(),
                    },
                },
            ],
            MidValueSelectScalar {
                divergence_tol: 100.0,
            },
        );

        ingest.run(&context(&bus, &clock)).unwrap();

        let (sample, _) = bus.latest::<MagnetometerSample>().unwrap().unwrap();
        let (status, _) = bus.latest::<SensorStatus>().unwrap().unwrap();
        assert_eq!(sample.field_body_nt.x, 30_010.0);
        assert_eq!(status.kind, SensorKind::Magnetometer);
        assert_eq!(status.lane_count, 3);
        assert!(status.any_divergent);
        assert!(status.lanes[2].divergent);
    }
}

mod sensor_fault_path {
    use openbmp_core::{SensorId, SimTime, StepIndex};
    use openbmp_fc::bus::Bus;
    use openbmp_fc::clock::{Clock as _, FixedClock};
    use openbmp_fc::scheduler::{Job, JobContext};
    use openbmp_fc::sensor_ingest::{ImuIngest, VotedGnssIngest};
    use openbmp_fc::topics::{GnssSample, ImuSample, SensorKind, SensorStatus};
    use openbmp_fc::voter::MidValueSelectScalar;
    use openbmp_sensors::{Sensor, SensorError, SensorMeasurement, Timestamped};

    #[derive(Clone, Debug)]
    struct FailingLane {
        id: SensorId,
    }

    impl Sensor for FailingLane {
        type Output = SensorMeasurement;

        fn sensor_id(&self) -> SensorId {
            self.id
        }

        fn read(&mut self) -> Result<Timestamped<Self::Output>, SensorError> {
            Err(SensorError::NoSample)
        }
    }

    fn context<'a>(bus: &'a Bus, clock: &'a FixedClock) -> JobContext<'a> {
        JobContext { bus, clock }
    }

    #[test]
    fn single_imu_read_error_publishes_unhealthy_sample() {
        let bus = Bus::new();
        bus.register::<ImuSample>().unwrap();
        let clock = FixedClock::new(SimTime::from_seconds(0.02), StepIndex::new(2));
        let mut ingest = ImuIngest::new(FailingLane {
            id: SensorId::from_path("sensors.imu.failed"),
        });

        ingest.run(&context(&bus, &clock)).unwrap();

        let (sample, _) = bus.latest::<ImuSample>().unwrap().unwrap();
        assert_eq!(sample.time, clock.now());
        assert!(!sample.healthy);
    }

    #[test]
    fn voted_gnss_all_lane_errors_publish_unhealthy_sample_and_status() {
        let bus = Bus::new();
        bus.register::<GnssSample>().unwrap();
        bus.register::<SensorStatus>().unwrap();
        let clock = FixedClock::new(SimTime::from_seconds(0.03), StepIndex::new(3));
        let mut ingest = VotedGnssIngest::new(
            vec![
                FailingLane {
                    id: SensorId::from_path("sensors.gnss.a"),
                },
                FailingLane {
                    id: SensorId::from_path("sensors.gnss.b"),
                },
            ],
            MidValueSelectScalar {
                divergence_tol: 1.0,
            },
        );

        ingest.run(&context(&bus, &clock)).unwrap();

        let (sample, _) = bus.latest::<GnssSample>().unwrap().unwrap();
        let (status, _) = bus.latest::<SensorStatus>().unwrap().unwrap();
        assert_eq!(sample.time, clock.now());
        assert!(!sample.healthy);
        assert_eq!(status.kind, SensorKind::Gnss);
        assert_eq!(status.lane_count, 2);
        assert!(!status.lanes[0].healthy);
        assert!(!status.lanes[1].healthy);
    }
}
