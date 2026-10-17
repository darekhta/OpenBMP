//! Host-side optimized-reference I-load bridge evidence.

use approx::assert_abs_diff_eq;
use nalgebra::Vector3;
use openbmp_core::{SimTime, StepIndex};
use openbmp_fc::{
    Bus, Job, JobContext, SimulatedClock,
    guidance::{OptimizedReferenceProfileGuidance, OptimizedReferenceProfileSample},
    topics::{ReferenceState, VehicleStatus},
};
use openbmp_trajopt::{
    ILoadHeader, ILoadObjectiveClass, ILoadPayload, ReferenceProfileSample, SynthesisMetadata,
    TerminalCondition, decode_iload_payload, encode_iload_payload,
};

fn sample_iload_payload() -> ILoadPayload {
    let terminal = TerminalCondition::ApogeeRadius { radius_m: 120.0 };
    ILoadPayload {
        header: ILoadHeader::new_with_objective(
            &terminal,
            ILoadObjectiveClass::OptimizedReference,
            7,
            "openbmp-trajopt-testkit".to_owned(),
        ),
        metadata: SynthesisMetadata {
            scenario_digest: "sha256:test-reference-profile".to_owned(),
            source_revision: "test".to_owned(),
            method: "hermite-simpson-fixed-mesh".to_owned(),
        },
        event_bindings_postcard: vec![1, 2, 3, 4],
        gain_tables: Vec::new(),
        reference_profile: vec![
            ReferenceProfileSample {
                time_s: 0.0,
                radius_m: 100.0,
                speed_m_s: 10.0,
                flight_path_angle_rad: 0.0,
            },
            ReferenceProfileSample {
                time_s: 2.0,
                radius_m: 120.0,
                speed_m_s: 20.0,
                flight_path_angle_rad: 0.2,
            },
            ReferenceProfileSample {
                time_s: 4.0,
                radius_m: 140.0,
                speed_m_s: 30.0,
                flight_path_angle_rad: 0.4,
            },
        ],
        optimality_certificate: None,
    }
}

fn fc_samples_from_iload(
    samples: &[ReferenceProfileSample],
) -> Vec<OptimizedReferenceProfileSample> {
    samples
        .iter()
        .map(|sample| OptimizedReferenceProfileSample {
            time_s: sample.time_s,
            radius_m: sample.radius_m,
            speed_m_s: sample.speed_m_s,
            flight_path_angle_rad: sample.flight_path_angle_rad,
        })
        .collect()
}

#[test]
fn postcard_iload_reference_profile_feeds_fc_optimized_guidance()
-> Result<(), Box<dyn std::error::Error>> {
    let payload = sample_iload_payload();
    let encoded = encode_iload_payload(&payload)?;
    let decoded = decode_iload_payload(&encoded)?;
    assert_eq!(decoded, payload);

    let fc_samples = fc_samples_from_iload(&decoded.reference_profile);
    let mut guidance = OptimizedReferenceProfileGuidance::new(fc_samples.clone())?
        .with_active_phase_ids(vec![7])
        .with_reference_directions(Vector3::y(), Vector3::z())?;
    assert_eq!(guidance.samples(), fc_samples.as_slice());

    let bus = Bus::new();
    bus.register::<VehicleStatus>()?;
    bus.register::<ReferenceState>()?;
    bus.publish(VehicleStatus {
        phase_id: 7,
        armed: true,
        in_flight: true,
        safe_state_requested: false,
    })?;
    let clock = SimulatedClock::at(SimTime::from_seconds(1.0), StepIndex::new(1));

    guidance.run(&JobContext {
        bus: &bus,
        clock: &clock,
    })?;

    let Some((reference, _sequence)) = bus.latest::<ReferenceState>()? else {
        return Err(
            std::io::Error::other("optimized guidance did not publish ReferenceState").into(),
        );
    };
    let expected_radius_m = 110.0;
    let expected_speed_m_s = 15.0;
    let expected_flight_path_angle_rad: f64 = 0.1;
    assert_eq!(reference.time, SimTime::from_seconds(1.0));
    assert_abs_diff_eq!(
        reference.position_eci_m,
        Vector3::new(0.0, expected_radius_m, 0.0),
        epsilon = 1.0e-12
    );
    assert_abs_diff_eq!(
        reference.velocity_eci_m_s,
        Vector3::new(
            0.0,
            expected_speed_m_s * expected_flight_path_angle_rad.sin(),
            expected_speed_m_s * expected_flight_path_angle_rad.cos(),
        ),
        epsilon = 1.0e-12
    );

    let mut out_of_fc_envelope = sample_iload_payload();
    out_of_fc_envelope.reference_profile[1].speed_m_s = -1.0;
    let encoded_bad = encode_iload_payload(&out_of_fc_envelope)?;
    let decoded_bad = decode_iload_payload(&encoded_bad)?;
    let bad_fc_samples = fc_samples_from_iload(&decoded_bad.reference_profile);
    assert!(OptimizedReferenceProfileGuidance::new(bad_fc_samples).is_err());

    Ok(())
}
