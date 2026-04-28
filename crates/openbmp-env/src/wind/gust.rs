//! [`GustWind`] — Phase-3.8.C Dryden rational-spectrum shaping
//! filter per MIL-STD-1797A.
//!
//! # Physics
//!
//! The Dryden turbulence model represents wind gusts as the output
//! of a rational shaping filter driven by unit-variance Gaussian
//! white noise. The first-order continuous-time transfer function
//! per axis is
//!
//! ```text
//! H_a(s) = σ_a · sqrt(2 · L_a / (π · V)) / (1 + (L_a / V) · s)
//! ```
//!
//! where `σ_a` is the axis intensity (m/s), `L_a` the length scale
//! (m), and `V` the reference airspeed (m/s). Phase-3.8 ships first-
//! order filters on all three axes (`u`, `v`, `w`); the lateral and
//! vertical second-order corrections per MIL-STD-1797A are deferred
//! to a downstream extension.
//!
//! # Discretization (zero-order hold)
//!
//! At construction we lower the continuous-time filter to a
//! discrete-time first-order recursion via zero-order hold:
//!
//! ```text
//! a = exp(-V · dt / L)
//! b = σ · sqrt(1 - a²)
//! state_new = (a · state_old) + (b · w)
//! ```
//!
//! where `w ~ N(0, 1)` is a unit-variance Gaussian sample drawn
//! from [`DeterministicRng::for_wind_component`] (Phase 3.8.A) via
//! Box-Muller. `a` and `b` are computed once at construction; the
//! hot path is one mul + one mul + one add per axis per step. No
//! FMA. No transcendentals on the hot path.
//!
//! # Locked operand order
//!
//! Per axis the step is exactly:
//!
//! ```text
//! let prod_a = a * state_old;
//! let prod_b = b * w;
//! let state_new = prod_a + prod_b;
//! ```
//!
//! This explicit form prevents future refactors from collapsing
//! into FMA or commuting the operands. Axes evaluate `u` then `v`
//! then `w` in declared order.
//!
//! # Determinism
//!
//! - Filter state lives in `Cell<f64>` per axis; the [`WindModel`]
//!   trait's `&self` signature requires interior mutability. The
//!   runner-side `WindRack` (Phase-3.8 plumbing) calls
//!   [`GustWind::advance`] once per kernel base tick; the kernel's
//!   four RK4 stages then read the cells via
//!   [`WindModel::wind_ned_m_s`] and all see the same value. Same
//!   pattern as the engine / tank / effector snapshot path.
//! - `Cell<f64>` is `!Sync` by design; `GustWind` is therefore
//!   `!Sync`. The kernel is single-threaded per the architecture
//!   spec, so this is fine.
//! - Box-Muller pulls 16 bytes per axis per step from the
//!   axis-specific RNG stream and produces one Gaussian sample
//!   (the second `z1` half of the pair is dropped — Phase-3.8 favors
//!   the simpler "fresh pair every step" convention over a cache
//!   that would have to be reset on `reset()`).
//!
//! # Mean wind
//!
//! `GustWind` carries an optional `mean_wind_ned_m_s` that is added
//! to the filter output. Defaults to zero. The composite story
//! (mean wind from `LayeredWind` + turbulence from `GustWind`) is a
//! Phase-3.X follow-on; Phase-3.8 keeps them mutually exclusive at
//! the scenario layer.

use std::cell::Cell;

use openbmp_core::{
    DeterministicRng, Eci, FrameContext, Ned, Position3, SimTime, StepIndex, Velocity3, WindAxis,
};

use crate::error::EnvError;

use super::WindModel;

/// Phase-3.8.C Dryden rational-spectrum shaping filter.
#[derive(Debug)]
pub struct GustWind {
    sigma_u_m_s: f64,
    sigma_v_m_s: f64,
    sigma_w_m_s: f64,
    length_scale_u_m: f64,
    length_scale_v_m: f64,
    length_scale_w_m: f64,
    airspeed_m_s: f64,
    dt_s: f64,
    mean_wind_ned_m_s: Velocity3<Ned>,
    // ZOH coefficients (precomputed at construction).
    a_u: f64,
    b_u: f64,
    a_v: f64,
    b_v: f64,
    a_w: f64,
    b_w: f64,
    // Filter state — interior-mutable so WindModel's &self surface
    // works. The WindRack's `advance` call writes; the kernel's
    // RK4 stages read.
    state_u: Cell<f64>,
    state_v: Cell<f64>,
    state_w: Cell<f64>,
    scenario_seed: u64,
}

/// Phase-3.8.C scenario-layer parameters for `GustWind`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GustWindParams {
    /// `(σ_u, σ_v, σ_w)` axis intensities, m/s. All non-negative,
    /// finite.
    pub intensity_m_s: [f64; 3],
    /// `(L_u, L_v, L_w)` axis length scales, m. All strictly
    /// positive, finite.
    pub length_scale_m: [f64; 3],
    /// Reference airspeed used to convert length scale to time
    /// scale. Strictly positive, finite.
    pub airspeed_m_s: f64,
    /// Constant mean wind in NED that is added to the filter output.
    /// Defaults to `[0.0, 0.0, 0.0]`.
    pub mean_wind_ned_m_s: [f64; 3],
    /// Kernel base-tick `dt`, used to lower the continuous-time
    /// filter via zero-order hold. Strictly positive, finite.
    pub dt_s: f64,
    /// Scenario seed; the per-axis `for_wind_component` RNG stream
    /// is keyed on `(scenario_seed, step, axis)`.
    pub scenario_seed: u64,
}

impl GustWind {
    /// Construct a new Dryden gust wind model.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError::InvalidParameter`] if any intensity is
    /// negative, any length scale is non-positive, the airspeed is
    /// non-positive, or `dt_s` is non-positive. Returns
    /// [`EnvError::NonFinite`] if any input is `NaN` / `Inf`.
    pub fn new(params: GustWindParams) -> Result<Self, EnvError> {
        let [sigma_u, sigma_v, sigma_w] = params.intensity_m_s;
        let [l_u, l_v, l_w] = params.length_scale_m;
        let [mean_n, mean_e, mean_d] = params.mean_wind_ned_m_s;

        for v in [
            sigma_u,
            sigma_v,
            sigma_w,
            l_u,
            l_v,
            l_w,
            params.airspeed_m_s,
            params.dt_s,
            mean_n,
            mean_e,
            mean_d,
        ] {
            if !v.is_finite() {
                return Err(EnvError::NonFinite {
                    reason: "GustWind parameter is NaN or infinite",
                });
            }
        }
        if sigma_u < 0.0 || sigma_v < 0.0 || sigma_w < 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "GustWind axis intensity (σ) must be non-negative",
            });
        }
        if l_u <= 0.0 || l_v <= 0.0 || l_w <= 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "GustWind axis length scale (L) must be strictly positive",
            });
        }
        if params.airspeed_m_s <= 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "GustWind airspeed must be strictly positive",
            });
        }
        if params.dt_s <= 0.0 {
            return Err(EnvError::InvalidParameter {
                reason: "GustWind dt_s must be strictly positive",
            });
        }

        // ZOH discretization. omega_a = V / L_a is the inverse time
        // constant; a = exp(-omega_a * dt) is the pole magnitude;
        // b = sigma * sqrt(1 - a*a) is the unit-variance noise gain.
        let zoh = |sigma: f64, length_scale: f64| -> (f64, f64) {
            let omega_inv_tau = params.airspeed_m_s / length_scale;
            let a = (-omega_inv_tau * params.dt_s).exp();
            let b = sigma * (1.0 - a * a).sqrt();
            (a, b)
        };
        let (a_u, b_u) = zoh(sigma_u, l_u);
        let (a_v, b_v) = zoh(sigma_v, l_v);
        let (a_w, b_w) = zoh(sigma_w, l_w);

        Ok(Self {
            sigma_u_m_s: sigma_u,
            sigma_v_m_s: sigma_v,
            sigma_w_m_s: sigma_w,
            length_scale_u_m: l_u,
            length_scale_v_m: l_v,
            length_scale_w_m: l_w,
            airspeed_m_s: params.airspeed_m_s,
            dt_s: params.dt_s,
            mean_wind_ned_m_s: Velocity3::new(mean_n, mean_e, mean_d),
            a_u,
            b_u,
            a_v,
            b_v,
            a_w,
            b_w,
            state_u: Cell::new(0.0),
            state_v: Cell::new(0.0),
            state_w: Cell::new(0.0),
            scenario_seed: params.scenario_seed,
        })
    }

    /// Reset filter state to zero. The runner calls this once at
    /// scenario start so reruns of the same scenario produce
    /// bit-identical sample streams.
    pub fn reset(&self) {
        self.state_u.set(0.0);
        self.state_v.set(0.0);
        self.state_w.set(0.0);
    }

    /// Advance the per-axis filter state by one kernel base tick at
    /// `step`. Each axis pulls 16 bytes from its
    /// `for_wind_component(scenario_seed, step, axis)` stream and
    /// produces one Box-Muller Gaussian sample.
    ///
    /// Locked operand order: `u` axis fully (Box-Muller, `prod_a`,
    /// `prod_b`, `state_new`), then `v` axis, then `w` axis.
    pub fn advance(&self, step: StepIndex) {
        let new_u = step_axis(
            self.a_u,
            self.b_u,
            self.state_u.get(),
            self.scenario_seed,
            step,
            WindAxis::U,
        );
        self.state_u.set(new_u);

        let new_v = step_axis(
            self.a_v,
            self.b_v,
            self.state_v.get(),
            self.scenario_seed,
            step,
            WindAxis::V,
        );
        self.state_v.set(new_v);

        let new_w = step_axis(
            self.a_w,
            self.b_w,
            self.state_w.get(),
            self.scenario_seed,
            step,
            WindAxis::W,
        );
        self.state_w.set(new_w);
    }

    /// Current per-axis filter state `(u, v, w)`. Useful for
    /// telemetry and tests.
    #[must_use]
    pub fn current_state(&self) -> (f64, f64, f64) {
        (self.state_u.get(), self.state_v.get(), self.state_w.get())
    }

    /// Configured intensities `(σ_u, σ_v, σ_w)`, m/s.
    #[must_use]
    pub fn intensity_m_s(&self) -> (f64, f64, f64) {
        (self.sigma_u_m_s, self.sigma_v_m_s, self.sigma_w_m_s)
    }

    /// Configured length scales `(L_u, L_v, L_w)`, m.
    #[must_use]
    pub fn length_scale_m(&self) -> (f64, f64, f64) {
        (
            self.length_scale_u_m,
            self.length_scale_v_m,
            self.length_scale_w_m,
        )
    }

    /// Configured reference airspeed, m/s.
    #[must_use]
    pub const fn airspeed_m_s(&self) -> f64 {
        self.airspeed_m_s
    }

    /// Configured kernel base-tick `dt`, s.
    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.dt_s
    }

    /// Configured mean wind, NED.
    #[must_use]
    pub const fn mean_wind_ned_m_s(&self) -> Velocity3<Ned> {
        self.mean_wind_ned_m_s
    }

    /// ZOH coefficients `(a_u, b_u, a_v, b_v, a_w, b_w)`. Pinned by
    /// the locked-stream test.
    #[must_use]
    pub const fn zoh_coefficients(&self) -> (f64, f64, f64, f64, f64, f64) {
        (self.a_u, self.b_u, self.a_v, self.b_v, self.a_w, self.b_w)
    }
}

impl WindModel for GustWind {
    fn wind_ned_m_s(
        &self,
        _position_eci: Position3<Eci>,
        _frame: &FrameContext,
        _time: SimTime,
    ) -> Result<Velocity3<Ned>, EnvError> {
        // Phase-3.8.C maps body-frame Dryden axes (u, v, w) directly
        // to NED (north, east, down). The body↔NED rotation is
        // decoupled work for the future wind-aware drag adapter.
        let (u, v, w) = self.current_state();
        let result = Velocity3::new(
            self.mean_wind_ned_m_s.vector.x + u,
            self.mean_wind_ned_m_s.vector.y + v,
            self.mean_wind_ned_m_s.vector.z + w,
        );
        if !result.vector.x.is_finite()
            || !result.vector.y.is_finite()
            || !result.vector.z.is_finite()
        {
            return Err(EnvError::NonFinite {
                reason: "GustWind output is NaN or infinite",
            });
        }
        Ok(result)
    }
}

/// Single-axis Dryden step. Pulls 16 bytes from the axis-specific
/// `for_wind_component` stream, produces one Gaussian sample via
/// Box-Muller, and applies the locked-operand-order recursion.
#[inline]
fn step_axis(
    a: f64,
    b: f64,
    state_old: f64,
    scenario_seed: u64,
    step: StepIndex,
    axis: WindAxis,
) -> f64 {
    let mut rng = DeterministicRng::for_wind_component(scenario_seed, step, axis);
    let z = sample_standard_normal(&mut rng);
    // Locked operand order: prod_a first, prod_b second, state_new
    // = sum. No FMA, no commute.
    let prod_a = a * state_old;
    let prod_b = b * z;
    prod_a + prod_b
}

/// Box-Muller standard-normal sample. Pulls two 64-bit values from
/// `rng` (one each for `u1` and `u2`) and returns one `z0 = sqrt(-2
/// ln u1) * cos(2π u2)`. The second `z1` half is dropped — Phase-3.8
/// favors the simpler "fresh pair every step" convention over a
/// per-axis cache that would need explicit reset semantics.
#[inline]
fn sample_standard_normal(rng: &mut DeterministicRng) -> f64 {
    // Convert two u64s to (u1, u2) ∈ (0, 1] uniformly distributed.
    // Add 1 to the high 53 bits to avoid u1 = 0 (which would NaN
    // the ln); divide by 2^53 to land in (0, 1].
    let u1_bits = rng.next_u64() >> 11;
    let u2_bits = rng.next_u64() >> 11;
    // Map to (0, 1] by adding 1 and dividing by 2^53.
    #[allow(clippy::cast_precision_loss)]
    let u1 = ((u1_bits + 1) as f64) / ((1_u64 << 53) as f64);
    #[allow(clippy::cast_precision_loss)]
    let u2 = ((u2_bits + 1) as f64) / ((1_u64 << 53) as f64);
    // Locked operand order: r = sqrt(-2 ln u1); theta = 2π u2;
    // z0 = r * cos(theta).
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * std::f64::consts::PI * u2;
    r * theta.cos()
}

#[cfg(test)]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::unwrap_used
)]
mod tests {
    use super::*;

    fn frame() -> FrameContext {
        FrameContext::toy_fixed_earth()
    }

    fn medium_turbulence_params() -> GustWindParams {
        GustWindParams {
            intensity_m_s: [2.5, 2.0, 1.5],
            length_scale_m: [533.0, 533.0, 100.0],
            airspeed_m_s: 250.0,
            mean_wind_ned_m_s: [0.0, 0.0, 0.0],
            dt_s: 0.001,
            scenario_seed: 0xdead_beef,
        }
    }

    fn pos() -> Position3<Eci> {
        Position3::new(0.0, 0.0, 0.0)
    }

    // -----------------------------------------------------------------
    // Constructor rejection
    // -----------------------------------------------------------------

    #[test]
    fn rejects_negative_sigma() {
        let mut p = medium_turbulence_params();
        p.intensity_m_s[0] = -1.0;
        assert!(matches!(
            GustWind::new(p),
            Err(EnvError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_zero_length_scale() {
        let mut p = medium_turbulence_params();
        p.length_scale_m[1] = 0.0;
        assert!(matches!(
            GustWind::new(p),
            Err(EnvError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_non_positive_airspeed() {
        let mut p = medium_turbulence_params();
        p.airspeed_m_s = 0.0;
        assert!(matches!(
            GustWind::new(p),
            Err(EnvError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_non_positive_dt() {
        let mut p = medium_turbulence_params();
        p.dt_s = -0.001;
        assert!(matches!(
            GustWind::new(p),
            Err(EnvError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn rejects_non_finite_inputs() {
        let mut p = medium_turbulence_params();
        p.mean_wind_ned_m_s[0] = f64::NAN;
        assert!(matches!(GustWind::new(p), Err(EnvError::NonFinite { .. })));
    }

    // -----------------------------------------------------------------
    // ZOH coefficients
    // -----------------------------------------------------------------

    #[test]
    fn zoh_coefficient_a_is_exp_negative_v_dt_over_l() {
        let p = medium_turbulence_params();
        let g = GustWind::new(p).unwrap();
        let (a_u, _, a_v, _, a_w, _) = g.zoh_coefficients();
        let expected_au = (-p.airspeed_m_s * p.dt_s / p.length_scale_m[0]).exp();
        let expected_av = (-p.airspeed_m_s * p.dt_s / p.length_scale_m[1]).exp();
        let expected_aw = (-p.airspeed_m_s * p.dt_s / p.length_scale_m[2]).exp();
        assert_eq!(a_u.to_bits(), expected_au.to_bits());
        assert_eq!(a_v.to_bits(), expected_av.to_bits());
        assert_eq!(a_w.to_bits(), expected_aw.to_bits());
    }

    #[test]
    fn zoh_coefficient_b_is_sigma_sqrt_one_minus_a_squared() {
        let p = medium_turbulence_params();
        let g = GustWind::new(p).unwrap();
        let (a_u, b_u, _, _, _, _) = g.zoh_coefficients();
        let expected_bu = p.intensity_m_s[0] * (1.0 - a_u * a_u).sqrt();
        assert_eq!(b_u.to_bits(), expected_bu.to_bits());
    }

    // -----------------------------------------------------------------
    // Sigma = 0 smoke test
    // -----------------------------------------------------------------

    #[test]
    fn sigma_zero_produces_exactly_zero_output() {
        let p = GustWindParams {
            intensity_m_s: [0.0, 0.0, 0.0],
            ..medium_turbulence_params()
        };
        let g = GustWind::new(p).unwrap();
        for step in 0_u64..1000 {
            g.advance(StepIndex::new(step));
            let (u, v, w) = g.current_state();
            assert_eq!(u.to_bits(), 0.0_f64.to_bits());
            assert_eq!(v.to_bits(), 0.0_f64.to_bits());
            assert_eq!(w.to_bits(), 0.0_f64.to_bits());
        }
        let wind = g.wind_ned_m_s(pos(), &frame(), SimTime::ZERO).unwrap();
        assert_eq!(wind.vector.x.to_bits(), 0.0_f64.to_bits());
        assert_eq!(wind.vector.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(wind.vector.z.to_bits(), 0.0_f64.to_bits());
    }

    // -----------------------------------------------------------------
    // Determinism
    // -----------------------------------------------------------------

    #[test]
    fn determinism_byte_stable_replay_over_1000_steps() {
        let p = medium_turbulence_params();
        let a = GustWind::new(p).unwrap();
        let b = GustWind::new(p).unwrap();
        for step in 0_u64..1000 {
            let s = StepIndex::new(step);
            a.advance(s);
            b.advance(s);
            let (au, av, aw) = a.current_state();
            let (bu, bv, bw) = b.current_state();
            assert_eq!(au.to_bits(), bu.to_bits());
            assert_eq!(av.to_bits(), bv.to_bits());
            assert_eq!(aw.to_bits(), bw.to_bits());
        }
    }

    #[test]
    fn reset_rewinds_filter_state_to_zero_and_replays_first_n_samples() {
        let p = medium_turbulence_params();
        let g = GustWind::new(p).unwrap();
        let mut first_pass = Vec::with_capacity(64);
        for step in 0_u64..64 {
            g.advance(StepIndex::new(step));
            first_pass.push(g.current_state());
        }
        g.reset();
        assert_eq!(g.current_state().0.to_bits(), 0.0_f64.to_bits());
        for step in 0_u64..64 {
            g.advance(StepIndex::new(step));
            let after_reset = g.current_state();
            let original = first_pass[step as usize];
            assert_eq!(after_reset.0.to_bits(), original.0.to_bits());
            assert_eq!(after_reset.1.to_bits(), original.1.to_bits());
            assert_eq!(after_reset.2.to_bits(), original.2.to_bits());
        }
    }

    // -----------------------------------------------------------------
    // Stability bound
    // -----------------------------------------------------------------

    #[test]
    fn filter_state_stays_finite_over_100k_steps() {
        let p = GustWindParams {
            intensity_m_s: [10.0, 10.0, 5.0],
            length_scale_m: [200.0, 200.0, 50.0],
            airspeed_m_s: 250.0,
            mean_wind_ned_m_s: [0.0, 0.0, 0.0],
            dt_s: 0.01,
            scenario_seed: 1,
        };
        let g = GustWind::new(p).unwrap();
        for step in 0_u64..100_000 {
            g.advance(StepIndex::new(step));
            let (u, v, w) = g.current_state();
            assert!(u.is_finite());
            assert!(v.is_finite());
            assert!(w.is_finite());
            // 10× σ bound is huge — the filter should never come
            // close. This catches integration overflow.
            assert!(u.abs() < 100.0, "u out of bound at step {step}: {u}");
            assert!(v.abs() < 100.0);
            assert!(w.abs() < 50.0);
        }
    }

    // -----------------------------------------------------------------
    // Mean wind
    // -----------------------------------------------------------------

    #[test]
    fn mean_wind_offsets_filter_output_at_step_zero() {
        let p = GustWindParams {
            intensity_m_s: [0.0, 0.0, 0.0],
            mean_wind_ned_m_s: [5.0, -3.0, 0.5],
            ..medium_turbulence_params()
        };
        let g = GustWind::new(p).unwrap();
        let wind = g.wind_ned_m_s(pos(), &frame(), SimTime::ZERO).unwrap();
        assert_eq!(wind.vector.x.to_bits(), 5.0_f64.to_bits());
        assert_eq!(wind.vector.y.to_bits(), (-3.0_f64).to_bits());
        assert_eq!(wind.vector.z.to_bits(), 0.5_f64.to_bits());
    }
}
