use openbmp_comm::TrackingObservableInput;

fn main() {
    let _ = TrackingObservableInput {
        site_id: "ksc".to_owned(),
        time_s: 0.0,
        slant_range_m: 1000.0,
        range_rate_m_s: 0.0,
        azimuth_rad: 0.0,
        elevation_rad: 0.0,
    };
}
