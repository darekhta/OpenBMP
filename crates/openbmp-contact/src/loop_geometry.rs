//! Geometry helpers for CFE loop-closure rows.

use crate::{
    ContactError,
    error::{require_finite, require_positive},
};

/// Planar four-bar loop geometry for one scalar CFE closure row.
///
/// The fixed pivots are `ground_pivot_a_m` and `ground_pivot_d_m`. The input
/// link rotates about A, the output link rotates about D, and the coupler
/// closes the distance between their moving endpoints. The generated CFE row
/// enforces `distance(input_endpoint, output_endpoint) - coupler_link_m = 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarFourBarLoop {
    ground_pivot_a_m: [f64; 2],
    ground_pivot_d_m: [f64; 2],
    input_link_m: f64,
    coupler_link_m: f64,
    output_link_m: f64,
}

impl PlanarFourBarLoop {
    /// Creates a validated planar four-bar loop.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError`] when pivots are non-finite/coincident, link
    /// lengths are non-positive, or the four lengths cannot form a closed
    /// quadrilateral in any configuration.
    pub fn new(
        ground_pivot_a_m: [f64; 2],
        ground_pivot_d_m: [f64; 2],
        input_link_m: f64,
        coupler_link_m: f64,
        output_link_m: f64,
    ) -> Result<Self, ContactError> {
        let ground_pivot_a_m = finite_vector2("ground_pivot_a_m", ground_pivot_a_m)?;
        let ground_pivot_d_m = finite_vector2("ground_pivot_d_m", ground_pivot_d_m)?;
        let ground_link_m = norm2(sub2(ground_pivot_d_m, ground_pivot_a_m));
        if ground_link_m <= 0.0 {
            return Err(ContactError::InvalidVector {
                field: "ground_pivot_d_m",
                reason: "must not coincide with ground_pivot_a_m",
            });
        }
        let input_link_m = require_positive("input_link_m", input_link_m)?;
        let coupler_link_m = require_positive("coupler_link_m", coupler_link_m)?;
        let output_link_m = require_positive("output_link_m", output_link_m)?;
        validate_quadrilateral_lengths(ground_link_m, input_link_m, coupler_link_m, output_link_m)?;

        Ok(Self {
            ground_pivot_a_m,
            ground_pivot_d_m,
            input_link_m,
            coupler_link_m,
            output_link_m,
        })
    }

    /// Returns fixed pivot A.
    #[must_use]
    pub const fn ground_pivot_a_m(self) -> [f64; 2] {
        self.ground_pivot_a_m
    }

    /// Returns fixed pivot D.
    #[must_use]
    pub const fn ground_pivot_d_m(self) -> [f64; 2] {
        self.ground_pivot_d_m
    }

    /// Returns the input crank length.
    #[must_use]
    pub const fn input_link_m(self) -> f64 {
        self.input_link_m
    }

    /// Returns the coupler length.
    #[must_use]
    pub const fn coupler_link_m(self) -> f64 {
        self.coupler_link_m
    }

    /// Returns the output rocker length.
    #[must_use]
    pub const fn output_link_m(self) -> f64 {
        self.output_link_m
    }

    /// Evaluates the scalar CFE row for the supplied angles and rates.
    ///
    /// The returned Jacobian is ordered `[input_angle, output_angle]`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError`] when any state component is non-finite or the
    /// two moving endpoints coincide, making the distance Jacobian undefined.
    pub fn evaluate(
        self,
        input_angle_rad: f64,
        output_angle_rad: f64,
        input_rate_rad_s: f64,
        output_rate_rad_s: f64,
    ) -> Result<PlanarFourBarCfeRow, ContactError> {
        let input_angle_rad = require_finite("input_angle_rad", input_angle_rad)?;
        let output_angle_rad = require_finite("output_angle_rad", output_angle_rad)?;
        let input_rate_rad_s = require_finite("input_rate_rad_s", input_rate_rad_s)?;
        let output_rate_rad_s = require_finite("output_rate_rad_s", output_rate_rad_s)?;

        let input_endpoint_m = self.input_endpoint_m(input_angle_rad);
        let output_endpoint_m = self.output_endpoint_m(output_angle_rad);
        let endpoint_delta_m = sub2(input_endpoint_m, output_endpoint_m);
        let coupler_distance_m = norm2(endpoint_delta_m);
        if coupler_distance_m <= f64::EPSILON {
            return Err(ContactError::InvalidParameter {
                field: "four_bar_loop",
                reason: "moving endpoints must not coincide",
            });
        }

        let input_endpoint_derivative = rotate90(self.input_link_vector(input_angle_rad));
        let output_endpoint_derivative = rotate90(self.output_link_vector(output_angle_rad));
        let delta_derivative_input = input_endpoint_derivative;
        let delta_derivative_output = scale2(output_endpoint_derivative, -1.0);
        let jacobian_input = dot2(endpoint_delta_m, delta_derivative_input) / coupler_distance_m;
        let jacobian_output = dot2(endpoint_delta_m, delta_derivative_output) / coupler_distance_m;
        let velocity_residual_per_s =
            jacobian_input * input_rate_rad_s + jacobian_output * output_rate_rad_s;

        Ok(PlanarFourBarCfeRow {
            input_endpoint_m,
            output_endpoint_m,
            coupler_distance_m,
            position_residual_m: coupler_distance_m - self.coupler_link_m,
            velocity_residual_m_s: velocity_residual_per_s,
            jacobian_row: [jacobian_input, jacobian_output],
        })
    }

    /// Solves the two circle-intersection output-angle branches for an input angle.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError`] when the input angle is non-finite or that
    /// crank position has no finite closed four-bar configuration.
    pub fn output_angle_solutions(
        self,
        input_angle_rad: f64,
    ) -> Result<PlanarFourBarOutputAngleSolutions, ContactError> {
        let input_angle_rad = require_finite("input_angle_rad", input_angle_rad)?;
        let input_endpoint_m = self.input_endpoint_m(input_angle_rad);
        let from_output_pivot_to_input = sub2(input_endpoint_m, self.ground_pivot_d_m);
        let pivot_distance_m = norm2(from_output_pivot_to_input);
        if pivot_distance_m <= f64::EPSILON {
            return Err(ContactError::InvalidParameter {
                field: "four_bar_loop",
                reason: "input endpoint must not coincide with output pivot",
            });
        }

        let a_m = (self.output_link_m * self.output_link_m
            - self.coupler_link_m * self.coupler_link_m
            + pivot_distance_m * pivot_distance_m)
            / (2.0 * pivot_distance_m);
        let h2_m2 = self.output_link_m * self.output_link_m - a_m * a_m;
        if h2_m2 < -1.0e-12 {
            return Err(ContactError::InvalidParameter {
                field: "four_bar_loop",
                reason: "input angle has no closed output solution",
            });
        }
        let h_m = h2_m2.max(0.0).sqrt();
        let axis = scale2(from_output_pivot_to_input, 1.0 / pivot_distance_m);
        let perpendicular = rotate90(axis);
        let base = add2(self.ground_pivot_d_m, scale2(axis, a_m));
        let positive_endpoint_m = add2(base, scale2(perpendicular, h_m));
        let negative_endpoint_m = sub2(base, scale2(perpendicular, h_m));

        Ok(PlanarFourBarOutputAngleSolutions {
            positive_branch_output_angle_rad: angle_of(sub2(
                positive_endpoint_m,
                self.ground_pivot_d_m,
            )),
            negative_branch_output_angle_rad: angle_of(sub2(
                negative_endpoint_m,
                self.ground_pivot_d_m,
            )),
        })
    }

    fn input_endpoint_m(self, input_angle_rad: f64) -> [f64; 2] {
        add2(
            self.ground_pivot_a_m,
            self.input_link_vector(input_angle_rad),
        )
    }

    fn output_endpoint_m(self, output_angle_rad: f64) -> [f64; 2] {
        add2(
            self.ground_pivot_d_m,
            self.output_link_vector(output_angle_rad),
        )
    }

    fn input_link_vector(self, input_angle_rad: f64) -> [f64; 2] {
        [
            self.input_link_m * input_angle_rad.cos(),
            self.input_link_m * input_angle_rad.sin(),
        ]
    }

    fn output_link_vector(self, output_angle_rad: f64) -> [f64; 2] {
        [
            self.output_link_m * output_angle_rad.cos(),
            self.output_link_m * output_angle_rad.sin(),
        ]
    }
}

/// Materialized scalar CFE row for a planar four-bar loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarFourBarCfeRow {
    /// Moving endpoint of the input link.
    pub input_endpoint_m: [f64; 2],
    /// Moving endpoint of the output link.
    pub output_endpoint_m: [f64; 2],
    /// Current distance between moving endpoints.
    pub coupler_distance_m: f64,
    /// Position residual `distance - coupler_link_m`.
    pub position_residual_m: f64,
    /// Velocity residual `J * qd`.
    pub velocity_residual_m_s: f64,
    /// Constraint Jacobian row ordered `[input_angle, output_angle]`.
    pub jacobian_row: [f64; 2],
}

/// The two output-angle branches produced by closing a planar four-bar loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarFourBarOutputAngleSolutions {
    /// Output angle for the positive circle-intersection branch.
    pub positive_branch_output_angle_rad: f64,
    /// Output angle for the negative circle-intersection branch.
    pub negative_branch_output_angle_rad: f64,
}

fn validate_quadrilateral_lengths(
    ground_link_m: f64,
    input_link_m: f64,
    coupler_link_m: f64,
    output_link_m: f64,
) -> Result<(), ContactError> {
    let mut longest = ground_link_m;
    let mut sum_other = input_link_m + coupler_link_m + output_link_m;
    for length in [input_link_m, coupler_link_m, output_link_m] {
        if length > longest {
            sum_other += longest - length;
            longest = length;
        }
    }
    if longest <= sum_other {
        Ok(())
    } else {
        Err(ContactError::InvalidParameter {
            field: "four_bar_loop",
            reason: "link lengths must admit a closed loop",
        })
    }
}

fn finite_vector2(field: &'static str, vector: [f64; 2]) -> Result<[f64; 2], ContactError> {
    if vector.iter().all(|component| component.is_finite()) {
        Ok(vector)
    } else {
        Err(ContactError::InvalidVector {
            field,
            reason: "all components must be finite",
        })
    }
}

fn add2(lhs: [f64; 2], rhs: [f64; 2]) -> [f64; 2] {
    [lhs[0] + rhs[0], lhs[1] + rhs[1]]
}

fn sub2(lhs: [f64; 2], rhs: [f64; 2]) -> [f64; 2] {
    [lhs[0] - rhs[0], lhs[1] - rhs[1]]
}

fn scale2(vector: [f64; 2], scalar: f64) -> [f64; 2] {
    [vector[0] * scalar, vector[1] * scalar]
}

fn dot2(lhs: [f64; 2], rhs: [f64; 2]) -> f64 {
    lhs[0] * rhs[0] + lhs[1] * rhs[1]
}

fn norm2(vector: [f64; 2]) -> f64 {
    dot2(vector, vector).sqrt()
}

fn rotate90(vector: [f64; 2]) -> [f64; 2] {
    [-vector[1], vector[0]]
}

fn angle_of(vector: [f64; 2]) -> f64 {
    vector[1].atan2(vector[0])
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn planar_four_bar_closed_rectangle_materializes_zero_residual() -> Result<(), ContactError> {
        let loop_geometry = PlanarFourBarLoop::new([0.0, 0.0], [1.0, 0.0], 0.5, 1.0, 0.5)?;

        let row = loop_geometry.evaluate(
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
            0.0,
            0.0,
        )?;

        assert_abs_diff_eq!(row.input_endpoint_m[0], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(row.input_endpoint_m[1], 0.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(row.output_endpoint_m[0], 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(row.output_endpoint_m[1], 0.5, epsilon = 1.0e-15);
        assert_abs_diff_eq!(row.coupler_distance_m, 1.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(row.position_residual_m, 0.0, epsilon = 1.0e-15);
        Ok(())
    }

    #[test]
    fn planar_four_bar_jacobian_matches_centered_difference() -> Result<(), ContactError> {
        let loop_geometry = PlanarFourBarLoop::new([0.0, 0.0], [1.2, 0.0], 0.7, 0.9, 0.8)?;
        let input_angle = 0.6_f64;
        let output_angle = 2.1_f64;
        let row = loop_geometry.evaluate(input_angle, output_angle, 0.0, 0.0)?;
        let step = 1.0e-6_f64;
        let input_plus = loop_geometry
            .evaluate(input_angle + step, output_angle, 0.0, 0.0)?
            .position_residual_m;
        let input_minus = loop_geometry
            .evaluate(input_angle - step, output_angle, 0.0, 0.0)?
            .position_residual_m;
        let output_plus = loop_geometry
            .evaluate(input_angle, output_angle + step, 0.0, 0.0)?
            .position_residual_m;
        let output_minus = loop_geometry
            .evaluate(input_angle, output_angle - step, 0.0, 0.0)?
            .position_residual_m;

        assert_abs_diff_eq!(
            row.jacobian_row[0],
            (input_plus - input_minus) / (2.0 * step),
            epsilon = 1.0e-10
        );
        assert_abs_diff_eq!(
            row.jacobian_row[1],
            (output_plus - output_minus) / (2.0 * step),
            epsilon = 1.0e-10
        );
        Ok(())
    }

    #[test]
    fn planar_four_bar_velocity_residual_is_jacobian_times_rates() -> Result<(), ContactError> {
        let loop_geometry = PlanarFourBarLoop::new([0.0, 0.0], [1.2, 0.0], 0.7, 0.9, 0.8)?;
        let row = loop_geometry.evaluate(0.6, 2.1, 0.25, -0.4)?;

        assert_abs_diff_eq!(
            row.velocity_residual_m_s,
            row.jacobian_row[0] * 0.25 + row.jacobian_row[1] * -0.4,
            epsilon = 1.0e-15
        );
        Ok(())
    }

    #[test]
    fn planar_four_bar_output_angle_solutions_close_loop() -> Result<(), ContactError> {
        let loop_geometry = PlanarFourBarLoop::new([0.0, 0.0], [1.0, 0.0], 0.5, 1.0, 0.5)?;
        let solutions = loop_geometry.output_angle_solutions(std::f64::consts::FRAC_PI_2)?;

        let positive_row = loop_geometry.evaluate(
            std::f64::consts::FRAC_PI_2,
            solutions.positive_branch_output_angle_rad,
            0.0,
            0.0,
        )?;
        let negative_row = loop_geometry.evaluate(
            std::f64::consts::FRAC_PI_2,
            solutions.negative_branch_output_angle_rad,
            0.0,
            0.0,
        )?;

        assert_abs_diff_eq!(positive_row.position_residual_m, 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(negative_row.position_residual_m, 0.0, epsilon = 1.0e-15);
        let distance_to_vertical = (solutions.positive_branch_output_angle_rad
            - std::f64::consts::FRAC_PI_2)
            .abs()
            .min((solutions.negative_branch_output_angle_rad - std::f64::consts::FRAC_PI_2).abs());
        assert_abs_diff_eq!(distance_to_vertical, 0.0, epsilon = 1.0e-15);
        Ok(())
    }

    #[test]
    fn planar_four_bar_rejects_invalid_geometry_and_open_input_pose() -> Result<(), ContactError> {
        assert!(PlanarFourBarLoop::new([0.0, 0.0], [0.0, 0.0], 0.5, 1.0, 0.5).is_err());
        assert!(PlanarFourBarLoop::new([0.0, 0.0], [3.0, 0.0], 0.5, 1.0, 0.5).is_err());
        assert!(
            PlanarFourBarLoop::new([0.0, 0.0], [1.0, 0.0], 0.5, 0.1, 0.5)?
                .output_angle_solutions(std::f64::consts::FRAC_PI_2)
                .is_err()
        );
        assert!(
            PlanarFourBarLoop::new([0.0, 0.0], [1.0, 0.0], 0.5, 0.5, 0.5)?
                .evaluate(0.0, std::f64::consts::PI, 0.0, 0.0)
                .is_err()
        );
        Ok(())
    }
}
