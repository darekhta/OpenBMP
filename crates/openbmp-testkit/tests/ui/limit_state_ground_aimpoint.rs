use openbmp_mc::{LimitState, MonteCarloError, StandardNormalPoint};

struct GroundAimpointLimitState;

impl LimitState for GroundAimpointLimitState {
    fn label(&self) -> &'static str {
        "ground-aimpoint"
    }

    fn dimension(&self) -> usize {
        2
    }

    fn evaluate(&self, point: &StandardNormalPoint) -> Result<f64, MonteCarloError> {
        Ok(1.0 - point.coordinates()[0])
    }
}

fn main() {}
