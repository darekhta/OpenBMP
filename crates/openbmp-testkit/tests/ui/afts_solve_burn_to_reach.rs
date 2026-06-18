use openbmp_afts::{ContainmentPolygon, IipPropagator, LatLon};

fn main() {
    let vertices = [
        LatLon::new_degrees(-1.0, -1.0).unwrap(),
        LatLon::new_degrees(-1.0, 1.0).unwrap(),
        LatLon::new_degrees(1.0, 1.0).unwrap(),
        LatLon::new_degrees(1.0, -1.0).unwrap(),
    ];
    let polygon = ContainmentPolygon::new("range", &vertices).unwrap();
    let propagator = IipPropagator::wgs84_spherical();

    let _ = propagator.solve_burn_to_reach(polygon);
}
