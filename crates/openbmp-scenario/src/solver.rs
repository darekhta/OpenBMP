//! Optional solver-profile section.
//!
//! The Phase-1 kernel ships only fixed-step explicit (RK4); the rest of
//! these fields are validated structurally so a Phase-6 solver can
//! advertise its profile in scenarios without parser churn. Setting
//! `bit-stable` determinism with anything other than `fixed-step-explicit`
//! is rejected at scenario load.

use serde::Deserialize;

use crate::checks::{require_positive, require_positive_u32, require_supported};
use crate::error::ScenarioError;

/// Optional solver profile.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolverConfig {
    /// Solver profile.
    pub profile: Option<String>,
    /// Trajectory method.
    pub trajectory_method: Option<String>,
    /// Determinism claim.
    pub determinism: Option<String>,
    /// Adaptive-solver controls.
    pub adaptive: Option<AdaptiveSolverConfig>,
    /// Source-term sub-step controls.
    pub source_terms: Option<SourceTermSolverConfig>,
}

impl SolverConfig {
    /// Validate solver constraints.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for unsupported solver settings or
    /// invalid tolerances.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        let profile = self.profile.as_deref().unwrap_or("fixed-step-explicit");
        require_supported(
            "solver.profile",
            profile,
            &[
                "fixed-step-explicit",
                "adaptive-explicit",
                "implicit-source-term",
                "partitioned-hypersonic",
            ],
        )?;
        let method = self.trajectory_method.as_deref().unwrap_or("rk4");
        require_supported(
            "solver.trajectory_method",
            method,
            &["rk4", "dopri54", "dopri853", "rkf78"],
        )?;
        let determinism = self.determinism.as_deref().unwrap_or("bit-stable");
        require_supported(
            "solver.determinism",
            determinism,
            &["bit-stable", "state-stable"],
        )?;

        if profile == "adaptive-explicit" && self.adaptive.is_none() {
            return Err(ScenarioError::UnsupportedValue {
                field: "solver.adaptive".to_owned(),
                value: "missing".to_owned(),
            });
        }
        if determinism == "bit-stable" && profile != "fixed-step-explicit" {
            return Err(ScenarioError::UnsupportedValue {
                field: "solver.determinism".to_owned(),
                value: "bit-stable adaptive/implicit solver".to_owned(),
            });
        }

        if let Some(adaptive) = &self.adaptive {
            adaptive.validate()?;
        }
        if let Some(source_terms) = &self.source_terms {
            source_terms.validate()?;
        }
        Ok(())
    }
}

/// Adaptive solver controls.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct AdaptiveSolverConfig {
    /// Relative tolerance.
    pub rtol: f64,
    /// Absolute tolerance.
    pub atol: f64,
    /// Minimum time step in seconds.
    pub min_dt_s: f64,
    /// Maximum time step in seconds.
    pub max_dt_s: f64,
    /// Whether dense output is enabled.
    pub dense_output: bool,
}

impl AdaptiveSolverConfig {
    /// Validate adaptive solver controls.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for invalid numeric controls.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        require_positive("solver.adaptive.rtol", self.rtol)?;
        require_positive("solver.adaptive.atol", self.atol)?;
        require_positive("solver.adaptive.min_dt_s", self.min_dt_s)?;
        require_positive("solver.adaptive.max_dt_s", self.max_dt_s)?;
        if self.max_dt_s < self.min_dt_s {
            return Err(ScenarioError::InvalidNumber {
                field: "solver.adaptive.max_dt_s".to_owned(),
                value: self.max_dt_s,
                rule: "must be greater than or equal to min_dt_s",
            });
        }
        Ok(())
    }
}

/// Source-term sub-step solver controls.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SourceTermSolverConfig {
    /// Chemistry integration method.
    pub chemistry_method: String,
    /// Number of chemistry substeps.
    pub chemistry_substeps: u32,
    /// Material integration method.
    pub material_method: String,
    /// Number of material substeps.
    pub material_substeps: u32,
    /// Nonlinear solve tolerance.
    pub nonlinear_tolerance: f64,
    /// Maximum nonlinear iterations.
    pub nonlinear_max_iter: u32,
}

impl SourceTermSolverConfig {
    /// Validate source-term solver controls.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] for unsupported methods or invalid
    /// numeric controls.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        require_supported(
            "solver.source_terms.chemistry_method",
            &self.chemistry_method,
            &["implicit-euler", "rosenbrock-wanner", "bdf"],
        )?;
        require_supported(
            "solver.source_terms.material_method",
            &self.material_method,
            &["implicit-euler"],
        )?;
        require_positive_u32(
            "solver.source_terms.chemistry_substeps",
            self.chemistry_substeps,
        )?;
        require_positive_u32(
            "solver.source_terms.material_substeps",
            self.material_substeps,
        )?;
        require_positive(
            "solver.source_terms.nonlinear_tolerance",
            self.nonlinear_tolerance,
        )?;
        require_positive_u32(
            "solver.source_terms.nonlinear_max_iter",
            self.nonlinear_max_iter,
        )?;
        Ok(())
    }
}
