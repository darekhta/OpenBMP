//! Compatibility re-export for the shared flight message dictionary.
//!
//! Message payloads and the stable topic-index table live in the L1
//! `openbmp-msgs` crate. `openbmp_fc::topics` remains as the
//! controller-local import path used by existing FC modules and host
//! runner code.

pub use openbmp_msgs::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::{DeadlineSlipEvent, OverrunEvent, TimingBudgetReport};

    #[test]
    #[allow(clippy::too_many_lines)]
    fn static_topic_indices_are_unique_and_in_range() {
        let topics = [
            (<ImuSample as Topic>::NAME, <ImuSample as Topic>::INDEX),
            (
                <BarometerSample as Topic>::NAME,
                <BarometerSample as Topic>::INDEX,
            ),
            (<GnssSample as Topic>::NAME, <GnssSample as Topic>::INDEX),
            (
                <MagnetometerSample as Topic>::NAME,
                <MagnetometerSample as Topic>::INDEX,
            ),
            (
                <StarTrackerSample as Topic>::NAME,
                <StarTrackerSample as Topic>::INDEX,
            ),
            (
                <AirDataSample as Topic>::NAME,
                <AirDataSample as Topic>::INDEX,
            ),
            (
                <ImuIncrementWindow as Topic>::NAME,
                <ImuIncrementWindow as Topic>::INDEX,
            ),
            (
                <SensorStatus as Topic>::NAME,
                <SensorStatus as Topic>::INDEX,
            ),
            (
                <AttitudeEstimate as Topic>::NAME,
                <AttitudeEstimate as Topic>::INDEX,
            ),
            (
                <PositionEstimate as Topic>::NAME,
                <PositionEstimate as Topic>::INDEX,
            ),
            (
                <EnvironmentEstimate as Topic>::NAME,
                <EnvironmentEstimate as Topic>::INDEX,
            ),
            (
                <PropellantState as Topic>::NAME,
                <PropellantState as Topic>::INDEX,
            ),
            (
                <EstimatorStatus as Topic>::NAME,
                <EstimatorStatus as Topic>::INDEX,
            ),
            (
                <EstimatorLaneSelection as Topic>::NAME,
                <EstimatorLaneSelection as Topic>::INDEX,
            ),
            (
                <VehicleStatus as Topic>::NAME,
                <VehicleStatus as Topic>::INDEX,
            ),
            (
                <MissionStatePublish as Topic>::NAME,
                <MissionStatePublish as Topic>::INDEX,
            ),
            (
                <MissionActionBatch as Topic>::NAME,
                <MissionActionBatch as Topic>::INDEX,
            ),
            (
                <MissionRegionStatePublish as Topic>::NAME,
                <MissionRegionStatePublish as Topic>::INDEX,
            ),
            (
                <HealthRegionStatePublish as Topic>::NAME,
                <HealthRegionStatePublish as Topic>::INDEX,
            ),
            (
                <CommsRegionStatePublish as Topic>::NAME,
                <CommsRegionStatePublish as Topic>::INDEX,
            ),
            (
                <EstimatorRegimeRegionStatePublish as Topic>::NAME,
                <EstimatorRegimeRegionStatePublish as Topic>::INDEX,
            ),
            #[cfg(not(feature = "hal"))]
            (
                <ScenarioStateOverride as Topic>::NAME,
                <ScenarioStateOverride as Topic>::INDEX,
            ),
            (
                <FailsafeFlags as Topic>::NAME,
                <FailsafeFlags as Topic>::INDEX,
            ),
            (
                <ActuatorStatus as Topic>::NAME,
                <ActuatorStatus as Topic>::INDEX,
            ),
            (
                <WatchdogStatus as Topic>::NAME,
                <WatchdogStatus as Topic>::INDEX,
            ),
            (
                <StorageStatus as Topic>::NAME,
                <StorageStatus as Topic>::INDEX,
            ),
            (
                <ActuatorCommand as Topic>::NAME,
                <ActuatorCommand as Topic>::INDEX,
            ),
            (
                <AutopilotStatus as Topic>::NAME,
                <AutopilotStatus as Topic>::INDEX,
            ),
            (
                <EffectorCommandSet as Topic>::NAME,
                <EffectorCommandSet as Topic>::INDEX,
            ),
            (
                <EngineDemand as Topic>::NAME,
                <EngineDemand as Topic>::INDEX,
            ),
            (
                <EngineCommandSet as Topic>::NAME,
                <EngineCommandSet as Topic>::INDEX,
            ),
            (<FdirStatus as Topic>::NAME, <FdirStatus as Topic>::INDEX),
            (
                <FdirGlrtDiagnostic as Topic>::NAME,
                <FdirGlrtDiagnostic as Topic>::INDEX,
            ),
            (
                <EstimatorMode as Topic>::NAME,
                <EstimatorMode as Topic>::INDEX,
            ),
            (
                <ReferenceState as Topic>::NAME,
                <ReferenceState as Topic>::INDEX,
            ),
            (
                <GuidanceCutoff as Topic>::NAME,
                <GuidanceCutoff as Topic>::INDEX,
            ),
            (
                <OverrunEvent as Topic>::NAME,
                <OverrunEvent as Topic>::INDEX,
            ),
            (
                <DeadlineSlipEvent as Topic>::NAME,
                <DeadlineSlipEvent as Topic>::INDEX,
            ),
            (
                <TimingBudgetReport as Topic>::NAME,
                <TimingBudgetReport as Topic>::INDEX,
            ),
        ];

        let expected_count = if cfg!(feature = "hal") {
            topic_index::COUNT - 1
        } else {
            topic_index::COUNT
        };
        assert_eq!(topics.len(), expected_count);

        let mut seen = [false; topic_index::COUNT];
        for (name, index) in topics {
            assert!(
                index < topic_index::COUNT,
                "{name} index {index} is out of range"
            );
            assert!(!seen[index], "{name} reuses topic index {index}");
            seen[index] = true;
        }
    }
}
