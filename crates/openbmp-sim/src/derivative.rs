//! State-space derivative types — re-exported from `openbmp-models`.
//!
//! The derivative trait + impls live in `openbmp-models`
//! so model authors and HAL adopters can implement / consume them
//! without depending on the simulator. This module is now a thin
//! re-export to preserve every existing `openbmp_sim::derivative::*`
//! import path.

pub use openbmp_models::derivative::{
    PointMassDerivative, RigidBodyDerivative, SimStateDerivative,
};
