//! `uom`-typed quantity re-exports for the OpenBMP public API surface.
//!
//! Public APIs that move dimensional scalar values across crate
//! boundaries should use these `f64`-storage `uom` quantities. Internal
//! hot-loop code may use raw `f64`, but the conversion to typed
//! quantities happens at the trait boundary.
//!
//! This is the "Mars Climate Orbiter" guard: unit mismatches surface
//! at compile time.

pub use uom::si::f64::{
    Acceleration, Angle, AngularAcceleration, AngularVelocity, Force, Frequency, Length, Mass,
    MassDensity, MassRate, Power, Pressure, ThermodynamicTemperature, Velocity, Volume,
};

// Phase-6 aerothermal quantities (HeatFlux, SpecificEnergy,
// SpecificHeatCapacity, ThermalConductivity, MagneticFluxDensity, etc.)
// will be re-exported here once `openbmp-aerothermal` lands and the
// uom paths are confirmed for the chosen feature set.

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use uom::si::length::{kilometer, meter};
    use uom::si::mass::kilogram;
    use uom::si::velocity::meter_per_second;

    #[test]
    fn length_unit_round_trip() {
        let one_km = Length::new::<kilometer>(1.0);
        assert_abs_diff_eq!(one_km.get::<meter>(), 1000.0, epsilon = 1e-9);
    }

    #[test]
    fn mass_velocity_compose() {
        // Trivial: just ensure typed quantities exist and arithmetic works.
        let mass = Mass::new::<kilogram>(2.0);
        let vel = Velocity::new::<meter_per_second>(3.0);
        // momentum = mass * velocity (uom permits this)
        let momentum = mass * vel;
        // momentum has units kg·m/s
        let _ = momentum;
    }
}
