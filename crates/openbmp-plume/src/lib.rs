//! `openbmp-plume` — deterministic plume similarity primitives.
//!
//! This crate is the L2 plume/SRP substrate. It computes live,
//! coordinate-free plume similarity state from propulsion nozzle output,
//! atmosphere scalars, and vehicle-intrinsic geometry. It has no dependency on
//! the runner, simulator, mission graph, or flight-controller crates.
//!
//! The first shipped slice is intentionally diagnostic only: nozzle pressure
//! ratio, exit-pressure ratio, thrust coefficient, momentum-flux ratio,
//! Prandtl-Meyer turn angle, cluster-merge flag, and a reduced PIFS onset flag.

pub mod error;

pub use error::PlumeError;
use openbmp_propulsion::NozzleSolution;

/// Minimum positive tolerance used for singular limit checks.
const MIN_POSITIVE: f64 = 1.0e-15;

/// Atmosphere and reference-aero inputs for plume similarity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlumeFreestream {
    /// Ambient static pressure in Pa.
    pub ambient_pressure_pa: f64,
    /// Dynamic pressure in Pa.
    pub dynamic_pressure_pa: f64,
    /// Aerodynamic reference area in m^2.
    pub reference_area_m2: f64,
}

impl PlumeFreestream {
    /// Validate finite plume freestream inputs.
    ///
    /// # Errors
    ///
    /// Returns [`PlumeError`] if any scalar is non-finite, if ambient pressure
    /// is not positive, or if dynamic pressure/reference area are negative or
    /// zero where required by the similarity definitions.
    pub fn validate(self) -> Result<Self, PlumeError> {
        require_positive(self.ambient_pressure_pa, "ambient_pressure_pa")?;
        require_positive(self.dynamic_pressure_pa, "dynamic_pressure_pa")?;
        require_positive(self.reference_area_m2, "reference_area_m2")?;
        Ok(self)
    }
}

/// Propulsion-side inputs for plume similarity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlumeNozzle {
    /// Chamber pressure in Pa. Zero is the power-off limit.
    pub chamber_pressure_pa: f64,
    /// Exit static pressure in Pa.
    pub exit_pressure_pa: f64,
    /// Supersonic exit Mach number.
    pub exit_mach: f64,
    /// Exhaust specific-heat ratio.
    pub gamma: f64,
    /// Total thrust in N.
    pub total_thrust_n: f64,
    /// Momentum thrust in N.
    pub momentum_thrust_n: f64,
}

impl PlumeNozzle {
    /// Build plume nozzle inputs from a propulsion nozzle solution.
    #[must_use]
    pub fn from_nozzle_solution(
        chamber_pressure_pa: f64,
        gamma: f64,
        solution: NozzleSolution,
    ) -> Self {
        Self {
            chamber_pressure_pa,
            exit_pressure_pa: solution.exit_pressure_pa,
            exit_mach: solution.exit_mach,
            gamma,
            total_thrust_n: solution.total_thrust_n,
            momentum_thrust_n: solution.momentum_thrust_n,
        }
    }

    /// Validate finite plume nozzle inputs.
    ///
    /// # Errors
    ///
    /// Returns [`PlumeError`] if any scalar is non-finite or outside the
    /// supersonic ideal-nozzle envelope.
    pub fn validate(self) -> Result<Self, PlumeError> {
        require_non_negative(self.chamber_pressure_pa, "chamber_pressure_pa")?;
        require_non_negative(self.exit_pressure_pa, "exit_pressure_pa")?;
        require_non_negative(self.total_thrust_n, "total_thrust_n")?;
        require_non_negative(self.momentum_thrust_n, "momentum_thrust_n")?;
        require_greater_than(self.exit_mach, 1.0, "exit_mach")?;
        require_greater_than(self.gamma, 1.0, "gamma")?;
        Ok(self)
    }
}

/// Vehicle-intrinsic cluster geometry for plume merge diagnostics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlumeClusterGeometry {
    /// Number of firing nozzles represented by this state.
    pub engine_count: u32,
    /// Total nozzle exit area in m^2.
    pub exit_area_total_m2: f64,
    /// Base area in m^2 used by base-pressure consumers.
    pub base_area_m2: f64,
    /// Center-to-center spacing between adjacent nozzles in m.
    pub center_spacing_m: f64,
    /// Axial distance downstream where merge is evaluated, in m.
    pub merge_evaluation_distance_m: f64,
    /// Incipient PIFS separation angle threshold in radians.
    pub pifs_onset_angle_rad: f64,
}

impl PlumeClusterGeometry {
    /// Validate finite non-coordinate geometry inputs.
    ///
    /// # Errors
    ///
    /// Returns [`PlumeError`] if the declared geometry is malformed.
    pub fn validate(self) -> Result<Self, PlumeError> {
        if self.engine_count == 0 {
            return Err(PlumeError::InvalidParameter {
                field: "engine_count",
                rule: "be at least one",
            });
        }
        require_positive(self.exit_area_total_m2, "exit_area_total_m2")?;
        require_positive(self.base_area_m2, "base_area_m2")?;
        require_non_negative(self.center_spacing_m, "center_spacing_m")?;
        require_non_negative(
            self.merge_evaluation_distance_m,
            "merge_evaluation_distance_m",
        )?;
        require_non_negative(self.pifs_onset_angle_rad, "pifs_onset_angle_rad")?;
        Ok(self)
    }

    fn exit_diameter_per_engine_m(self) -> f64 {
        let per_engine_area = self.exit_area_total_m2 / f64::from(self.engine_count);
        (4.0 * per_engine_area / core::f64::consts::PI).sqrt()
    }
}

/// Live plume similarity state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlumeState {
    /// Nozzle pressure ratio, `pc / p_inf`.
    pub nozzle_pressure_ratio: f64,
    /// Exit static-pressure ratio, `pe / p_inf`.
    pub exit_pressure_ratio: f64,
    /// Thrust coefficient, `T / (q_inf * S_ref)`.
    pub thrust_coefficient: f64,
    /// Momentum-flux ratio, `(mdot Ve) / (q_inf * A_e,total)`.
    pub momentum_flux_ratio: f64,
    /// Initial underexpanded-plume turn angle in radians.
    pub initial_turn_angle_rad: f64,
    /// Estimated merge distance in m. `None` when a single nozzle or zero turn
    /// angle makes the reduced merge criterion inactive.
    pub merge_distance_m: Option<f64>,
    /// Whether the reduced plume cones overlap by the declared evaluation
    /// distance.
    pub cluster_merged: bool,
    /// Whether the reduced turn-angle threshold indicates PIFS onset.
    pub pifs_onset: bool,
}

impl PlumeState {
    /// Assemble plume similarity state from validated inputs.
    ///
    /// # Errors
    ///
    /// Returns [`PlumeError`] when inputs are non-finite or outside the reduced
    /// ideal-nozzle plume-similarity envelope.
    pub fn from_inputs(
        freestream: PlumeFreestream,
        nozzle: PlumeNozzle,
        geometry: PlumeClusterGeometry,
    ) -> Result<Self, PlumeError> {
        let freestream = freestream.validate()?;
        let nozzle = nozzle.validate()?;
        let geometry = geometry.validate()?;
        let nozzle_pressure_ratio = nozzle.chamber_pressure_pa / freestream.ambient_pressure_pa;
        let exit_pressure_ratio = nozzle.exit_pressure_pa / freestream.ambient_pressure_pa;
        let thrust_coefficient =
            nozzle.total_thrust_n / (freestream.dynamic_pressure_pa * freestream.reference_area_m2);
        let momentum_flux_ratio = nozzle.momentum_thrust_n
            / (freestream.dynamic_pressure_pa * geometry.exit_area_total_m2);
        let initial_turn_angle_rad = initial_turn_angle_rad(
            nozzle.exit_mach,
            nozzle.gamma,
            nozzle.exit_pressure_pa,
            freestream.ambient_pressure_pa,
        )?;
        let merge_distance_m = plume_merge_distance_m(geometry, initial_turn_angle_rad)?;
        let cluster_merged = merge_distance_m
            .is_some_and(|distance| distance <= geometry.merge_evaluation_distance_m);
        let pifs_onset =
            initial_turn_angle_rad >= geometry.pifs_onset_angle_rad && initial_turn_angle_rad > 0.0;
        for value in [
            nozzle_pressure_ratio,
            exit_pressure_ratio,
            thrust_coefficient,
            momentum_flux_ratio,
            initial_turn_angle_rad,
        ] {
            if !value.is_finite() {
                return Err(PlumeError::NonFinite {
                    field: "plume_state",
                });
            }
        }
        Ok(Self {
            nozzle_pressure_ratio,
            exit_pressure_ratio,
            thrust_coefficient,
            momentum_flux_ratio,
            initial_turn_angle_rad,
            merge_distance_m,
            cluster_merged,
            pifs_onset,
        })
    }
}

/// Prandtl-Meyer function `nu(M, gamma)` in radians.
///
/// # Errors
///
/// Returns [`PlumeError`] when `mach < 1`, `gamma <= 1`, or either scalar is
/// non-finite.
pub fn prandtl_meyer_rad(mach: f64, gamma: f64) -> Result<f64, PlumeError> {
    require_greater_or_equal(mach, 1.0, "mach")?;
    require_greater_than(gamma, 1.0, "gamma")?;
    if (mach - 1.0).abs() <= MIN_POSITIVE {
        return Ok(0.0);
    }
    let mach_sq_minus_one = mach * mach - 1.0;
    let root = mach_sq_minus_one.sqrt();
    let gamma_ratio = ((gamma + 1.0) / (gamma - 1.0)).sqrt();
    let inner = ((gamma - 1.0) / (gamma + 1.0) * mach_sq_minus_one).sqrt();
    Ok(gamma_ratio * inner.atan() - root.atan())
}

/// Initial underexpanded-plume turn angle in radians.
///
/// This is the Prandtl-Meyer expansion from nozzle exit Mach to the Mach number
/// that would isentropically match ambient pressure. Overexpanded or exactly
/// adapted nozzles return zero.
///
/// # Errors
///
/// Returns [`PlumeError`] when inputs are malformed or the expanded Mach state
/// is outside the reduced ideal-gas envelope.
pub fn initial_turn_angle_rad(
    exit_mach: f64,
    gamma: f64,
    exit_pressure_pa: f64,
    ambient_pressure_pa: f64,
) -> Result<f64, PlumeError> {
    require_greater_or_equal(exit_mach, 1.0, "exit_mach")?;
    require_greater_than(gamma, 1.0, "gamma")?;
    require_non_negative(exit_pressure_pa, "exit_pressure_pa")?;
    require_positive(ambient_pressure_pa, "ambient_pressure_pa")?;
    if exit_pressure_pa <= ambient_pressure_pa {
        return Ok(0.0);
    }
    let pressure_ratio = exit_pressure_pa / ambient_pressure_pa;
    let gm1_half = 0.5 * (gamma - 1.0);
    let exit_stagnation_ratio = 1.0 + gm1_half * exit_mach * exit_mach;
    let expanded_stagnation_ratio =
        exit_stagnation_ratio * pressure_ratio.powf((gamma - 1.0) / gamma);
    let expanded_mach_sq = (expanded_stagnation_ratio - 1.0) / gm1_half;
    if !expanded_mach_sq.is_finite() || expanded_mach_sq < 1.0 {
        return Err(PlumeError::InvalidParameter {
            field: "expanded_mach",
            rule: "remain supersonic and finite",
        });
    }
    let expanded_mach = expanded_mach_sq.sqrt();
    let turn = prandtl_meyer_rad(expanded_mach, gamma)? - prandtl_meyer_rad(exit_mach, gamma)?;
    Ok(turn.max(0.0))
}

/// Reduced plume-cone merge distance in m.
///
/// # Errors
///
/// Returns [`PlumeError`] when geometry or turn angle is malformed.
pub fn plume_merge_distance_m(
    geometry: PlumeClusterGeometry,
    initial_turn_angle_rad: f64,
) -> Result<Option<f64>, PlumeError> {
    let geometry = geometry.validate()?;
    require_non_negative(initial_turn_angle_rad, "initial_turn_angle_rad")?;
    if geometry.engine_count < 2 || initial_turn_angle_rad <= 0.0 {
        return Ok(None);
    }
    let gap_m = (geometry.center_spacing_m - geometry.exit_diameter_per_engine_m()).max(0.0);
    if gap_m <= 0.0 {
        return Ok(Some(0.0));
    }
    let tangent = initial_turn_angle_rad.tan();
    if !tangent.is_finite() || tangent <= 0.0 {
        return Ok(None);
    }
    Ok(Some(gap_m / (2.0 * tangent)))
}

fn require_positive(value: f64, field: &'static str) -> Result<(), PlumeError> {
    if !value.is_finite() {
        return Err(PlumeError::NonFinite { field });
    }
    if value <= 0.0 {
        return Err(PlumeError::InvalidParameter {
            field,
            rule: "be positive",
        });
    }
    Ok(())
}

fn require_non_negative(value: f64, field: &'static str) -> Result<(), PlumeError> {
    if !value.is_finite() {
        return Err(PlumeError::NonFinite { field });
    }
    if value < 0.0 {
        return Err(PlumeError::InvalidParameter {
            field,
            rule: "be non-negative",
        });
    }
    Ok(())
}

fn require_greater_than(value: f64, threshold: f64, field: &'static str) -> Result<(), PlumeError> {
    if !value.is_finite() || !threshold.is_finite() {
        return Err(PlumeError::NonFinite { field });
    }
    if value <= threshold {
        return Err(PlumeError::InvalidParameter {
            field,
            rule: "be greater than the threshold",
        });
    }
    Ok(())
}

fn require_greater_or_equal(
    value: f64,
    threshold: f64,
    field: &'static str,
) -> Result<(), PlumeError> {
    if !value.is_finite() || !threshold.is_finite() {
        return Err(PlumeError::NonFinite { field });
    }
    if value < threshold {
        return Err(PlumeError::InvalidParameter {
            field,
            rule: "be greater than or equal to the threshold",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn freestream() -> PlumeFreestream {
        PlumeFreestream {
            ambient_pressure_pa: 25_000.0,
            dynamic_pressure_pa: 40_000.0,
            reference_area_m2: 10.0,
        }
    }

    fn nozzle() -> PlumeNozzle {
        PlumeNozzle {
            chamber_pressure_pa: 2_000_000.0,
            exit_pressure_pa: 50_000.0,
            exit_mach: 3.0,
            gamma: 1.22,
            total_thrust_n: 500_000.0,
            momentum_thrust_n: 480_000.0,
        }
    }

    fn geometry() -> PlumeClusterGeometry {
        PlumeClusterGeometry {
            engine_count: 3,
            exit_area_total_m2: 0.6,
            base_area_m2: 5.0,
            center_spacing_m: 0.7,
            merge_evaluation_distance_m: 3.0,
            pifs_onset_angle_rad: 0.02,
        }
    }

    #[test]
    fn prandtl_meyer_known_air_value() {
        let nu = prandtl_meyer_rad(2.0, 1.4).unwrap();
        assert_abs_diff_eq!(nu, 0.460_413_682_082_694_73, epsilon = 1.0e-15);
    }

    #[test]
    fn initial_turn_angle_is_zero_for_adapted_or_overexpanded_nozzle() {
        assert_eq!(
            initial_turn_angle_rad(3.0, 1.22, 25_000.0, 25_000.0)
                .unwrap()
                .to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            initial_turn_angle_rad(3.0, 1.22, 20_000.0, 25_000.0)
                .unwrap()
                .to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn initial_turn_angle_matches_closed_form_expansion() {
        let gamma = 1.22;
        let exit_mach = 3.0;
        let exit_pressure_pa = 50_000.0;
        let ambient_pressure_pa = 25_000.0;
        let got = initial_turn_angle_rad(exit_mach, gamma, exit_pressure_pa, ambient_pressure_pa)
            .unwrap();

        let gm1_half = 0.5 * (gamma - 1.0);
        let exit_stagnation_ratio = 1.0 + gm1_half * exit_mach * exit_mach;
        let pressure_ratio = exit_pressure_pa / ambient_pressure_pa;
        let expanded_stagnation_ratio =
            exit_stagnation_ratio * pressure_ratio.powf((gamma - 1.0) / gamma);
        let expanded_mach = ((expanded_stagnation_ratio - 1.0) / gm1_half).sqrt();
        let expected = prandtl_meyer_rad(expanded_mach, gamma).unwrap()
            - prandtl_meyer_rad(exit_mach, gamma).unwrap();

        assert_abs_diff_eq!(got, expected, epsilon = 1.0e-15);
        assert!(got > 0.0);
    }

    #[test]
    fn merge_distance_matches_cone_overlap_geometry() {
        let geometry = geometry();
        let turn = 0.1;
        let distance = plume_merge_distance_m(geometry, turn).unwrap().unwrap();
        let exit_diameter =
            (4.0 * (geometry.exit_area_total_m2 / 3.0) / core::f64::consts::PI).sqrt();
        let expected = (geometry.center_spacing_m - exit_diameter).max(0.0) / (2.0 * turn.tan());
        assert_abs_diff_eq!(distance, expected, epsilon = 1.0e-15);
    }

    #[test]
    fn plume_state_reports_similarity_scalars() {
        let state = PlumeState::from_inputs(freestream(), nozzle(), geometry()).unwrap();

        assert_abs_diff_eq!(state.nozzle_pressure_ratio, 80.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.exit_pressure_ratio, 2.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.thrust_coefficient, 1.25, epsilon = 1.0e-15);
        assert_abs_diff_eq!(state.momentum_flux_ratio, 20.0, epsilon = 1.0e-15);
        assert!(state.initial_turn_angle_rad > 0.0);
        assert!(state.cluster_merged);
        assert!(state.pifs_onset);
    }

    #[test]
    fn plume_state_rejects_coordinate_like_invalid_inputs() {
        let err = PlumeState::from_inputs(
            PlumeFreestream {
                ambient_pressure_pa: 0.0,
                ..freestream()
            },
            nozzle(),
            geometry(),
        )
        .unwrap_err();
        assert!(matches!(
            err,
            PlumeError::InvalidParameter {
                field: "ambient_pressure_pa",
                ..
            }
        ));
    }
}
