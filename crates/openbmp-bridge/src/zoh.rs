//! Zero-order-hold coupling-error checks for lockstep bridge studies.
//!
//! The in-house bridge holds actuator commands constant over a controller
//! minor frame. For a smooth command `u(t)`, that input extrapolation error
//! is first-order in the controller frame period. These helpers provide a
//! small analytic toy used by the parity gate: for a linear command ramp,
//! the mean absolute ZOH error is `|du/dt| * h_c / 2`, so halving `h_c`
//! halves the error.

use thiserror::Error;

/// One ZOH coupling-error sample at a controller frame period.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZohCouplingErrorSample {
    /// Controller frame period, seconds.
    pub frame_period_s: f64,
    /// Mean absolute `|u(t) - u_k|` over one held frame.
    pub mean_abs_error: f64,
}

/// Coupling-error ratio for one controller-frame halving.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZohCouplingHalvingReport {
    /// Coarse-frame error sample.
    pub coarse: ZohCouplingErrorSample,
    /// Fine-frame error sample.
    pub fine: ZohCouplingErrorSample,
    /// `fine.mean_abs_error / coarse.mean_abs_error`.
    pub error_ratio: f64,
}

/// ZOH coupling-error helper validation failure.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ZohCouplingError {
    /// A supplied scalar was zero, negative, NaN, or infinite.
    #[error("invalid ZOH coupling parameter {field}: {value}; {rule}")]
    InvalidParameter {
        /// Parameter name.
        field: &'static str,
        /// Rejected value.
        value: f64,
        /// Validation rule.
        rule: &'static str,
    },
}

/// Mean absolute ZOH error for a linear command ramp.
///
/// For `u(t) = u0 + slope_per_s * t`, sampled and held at the start of
/// a controller frame of width `frame_period_s`, the mean absolute error
/// over that frame is `|slope_per_s| * frame_period_s / 2`.
///
/// # Errors
///
/// Returns [`ZohCouplingError`] if the frame period is not positive and
/// finite, or if `slope_per_s` is not finite.
pub fn zoh_linear_ramp_mean_abs_error(
    frame_period_s: f64,
    slope_per_s: f64,
) -> Result<ZohCouplingErrorSample, ZohCouplingError> {
    require_positive_finite("frame_period_s", frame_period_s)?;
    require_finite("slope_per_s", slope_per_s)?;
    Ok(ZohCouplingErrorSample {
        frame_period_s,
        mean_abs_error: 0.5 * slope_per_s.abs() * frame_period_s,
    })
}

/// Compute the ZOH error ratio after halving the controller frame.
///
/// The function does not require `fine_frame_s == coarse_frame_s / 2`;
/// the returned ratio exposes whatever pair was supplied. The parity
/// gate uses an exact halving pair and checks the ratio against the
/// tolerance table.
///
/// # Errors
///
/// Returns [`ZohCouplingError`] if a frame period is not positive and
/// finite, if `slope_per_s` is not finite, or if the ramp slope is zero
/// and the ratio would be undefined.
pub fn zoh_linear_ramp_halving_report(
    coarse_frame_s: f64,
    fine_frame_s: f64,
    slope_per_s: f64,
) -> Result<ZohCouplingHalvingReport, ZohCouplingError> {
    require_non_zero_finite("slope_per_s", slope_per_s)?;
    let coarse = zoh_linear_ramp_mean_abs_error(coarse_frame_s, slope_per_s)?;
    let fine = zoh_linear_ramp_mean_abs_error(fine_frame_s, slope_per_s)?;
    Ok(ZohCouplingHalvingReport {
        coarse,
        fine,
        error_ratio: fine.mean_abs_error / coarse.mean_abs_error,
    })
}

fn require_finite(field: &'static str, value: f64) -> Result<(), ZohCouplingError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ZohCouplingError::InvalidParameter {
            field,
            value,
            rule: "must be finite",
        })
    }
}

fn require_non_zero_finite(field: &'static str, value: f64) -> Result<(), ZohCouplingError> {
    require_finite(field, value)?;
    if value == 0.0 {
        return Err(ZohCouplingError::InvalidParameter {
            field,
            value,
            rule: "must be non-zero",
        });
    }
    Ok(())
}

fn require_positive_finite(field: &'static str, value: f64) -> Result<(), ZohCouplingError> {
    require_finite(field, value)?;
    if value <= 0.0 {
        return Err(ZohCouplingError::InvalidParameter {
            field,
            value,
            rule: "must be positive",
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use std::path::PathBuf;

    use openbmp_testkit::tolerance::ToleranceTable;

    use super::*;

    #[test]
    fn zoh_coupling_error_halves_as_controller_frame_halves() {
        let report =
            zoh_linear_ramp_halving_report(0.1, 0.05, 2.0).expect("ZOH halving report computes");

        let mut path: PathBuf = std::env::var_os("CARGO_MANIFEST_DIR")
            .expect("CARGO_MANIFEST_DIR must be set for cargo test")
            .into();
        path.push("tests");
        path.push("expected");
        path.push("zoh-coupling.toml");
        let table = ToleranceTable::from_path(path).expect("tolerance table must parse");

        table
            .check_metric("coarse_mean_abs_error", report.coarse.mean_abs_error)
            .expect("coarse mean absolute ZOH error within tolerance");
        table
            .check_metric("fine_mean_abs_error", report.fine.mean_abs_error)
            .expect("fine mean absolute ZOH error within tolerance");
        table
            .check_metric("mean_abs_error_halving_ratio", report.error_ratio)
            .expect("ZOH error halving ratio within tolerance");
    }

    #[test]
    fn zoh_coupling_error_rejects_invalid_inputs() {
        assert!(matches!(
            zoh_linear_ramp_mean_abs_error(0.0, 1.0),
            Err(ZohCouplingError::InvalidParameter {
                field: "frame_period_s",
                ..
            })
        ));
        assert!(matches!(
            zoh_linear_ramp_mean_abs_error(0.1, f64::INFINITY),
            Err(ZohCouplingError::InvalidParameter {
                field: "slope_per_s",
                ..
            })
        ));
        assert!(matches!(
            zoh_linear_ramp_halving_report(0.1, 0.05, 0.0),
            Err(ZohCouplingError::InvalidParameter {
                field: "slope_per_s",
                rule: "must be non-zero",
                ..
            })
        ));
    }
}
