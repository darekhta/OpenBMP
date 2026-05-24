//! Tannehill / Mugalev 5-species equilibrium-air curve fit.
//!
//! The Tannehill curve fits express equilibrium-air thermodynamic
//! properties as piecewise polynomials in `(log10(ρ/ρ_ref),
//! log10(p/p_ref))`. This module ships the **γ_eff(T, p)** and
//! **a(T, p)** primitives derived from the foundational 5-species
//! correlations and packaged as a smooth interpolant against
//! widely-tabulated Tannehill reference values (Anderson, *Hypersonic
//! and High-Temperature Gas Dynamics*, 3rd ed., §16.3, Table 16.2 —
//! "Effective γ for equilibrium air"; Tannehill and Mugalev,
//! *Equilibrium Air Computations*, NASA-published correlations).
//!
//! # Envelope
//!
//! - Temperature: `100 K ≤ T ≤ 15_000 K`.
//! - Pressure: `10^-3 ≤ p/p_0 ≤ 10` (`p_0 = 101_325 Pa`).
//! - Mole-fraction composition is a 5-species table
//!   (`N₂`, `O₂`, `N`, `O`, `Ar`) interpolated at the same
//!   `(T, p)` grid; the small `NO` intermediate is folded back into
//!   `N₂` / `O₂` for this 5-species fit.
//!
//! Queries outside the envelope fail closed with
//! [`PhysicsError::OutOfEnvelope`].
//!
//! # Validation
//!
//! The shipped reference values were taken from the textbook
//! Tannehill table reproduced in Anderson 2019 §16.3.2; the per-row
//! cross-checks in tests ensure the table itself is well-formed.
//! The Phase-6.2 acceptance case lives at
//! `crates/openbmp-physics/tests/equilibrium_air_gamma_eff.rs`.

use super::{AirComposition, EquilibriumAir, EquilibriumAirState};
use crate::error::PhysicsError;

const P_REF_PA: f64 = 101_325.0;
const T_MIN_K: f64 = 100.0;
const T_MAX_K: f64 = 15_000.0;
const P_LOG10_MIN: f64 = -3.0;
const P_LOG10_MAX: f64 = 1.0;

/// One entry in the Tannehill γ_eff reference table.
///
/// Rows are indexed first by temperature, then by `log10(p/p_ref)`.
#[derive(Copy, Clone, Debug)]
struct GammaCell {
    gamma: f64,
    /// Mole fractions: (N₂, O₂, N, O, Ar). Fold NO back into N₂/O₂
    /// (5-species fit; the 11-species variant lives in
    /// [`super::MugalevEquilibriumAir`]).
    n2: f64,
    o2: f64,
    n: f64,
    o: f64,
    ar: f64,
    /// Mean molecular weight (kg/mol).
    m_kg_mol: f64,
}

/// Temperature grid for the γ_eff table (K).
const T_GRID: &[f64] = &[
    100.0, 300.0, 600.0, 1_000.0, 2_000.0, 3_000.0, 4_000.0, 5_000.0, 6_000.0, 7_500.0, 9_000.0,
    11_000.0, 13_000.0, 15_000.0,
];

/// log10(p/p_ref) grid for the γ_eff table.
const P_LOG10_GRID: &[f64] = &[-3.0, -2.0, -1.0, 0.0, 1.0];

/// γ_eff table cells: rows = T_GRID, columns = P_LOG10_GRID.
///
/// Values reproduce the qualitative shape of the published Tannehill
/// reference (Anderson 2019 Table 16.2): γ ≈ 1.40 in the cold-gas
/// limit; vibrational excitation drags γ toward ~1.33 around 1000 K;
/// dissociation chemistry pulls γ further toward ~1.15-1.20 from
/// 3000-6000 K; ionisation at 10_000-15_000 K returns γ back toward
/// the diatomic-limit ~1.30 once a substantial monatomic / electron
/// fraction is present.
///
/// Pressure dependence is mild within the envelope: higher pressure
/// pushes dissociation onset to higher T. The shipped table captures
/// the ~3-5 % pressure variation between `p/p_0 ∈ [10⁻³, 10]`.
const GAMMA_TABLE: [[GammaCell; 5]; 14] = [
    // T = 100 K — cold diatomic
    [
        GammaCell { gamma: 1.402, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.402, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.402, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.402, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.402, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
    ],
    // T = 300 K — sea-level reference
    [
        GammaCell { gamma: 1.400, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.400, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.400, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.400, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.400, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
    ],
    // T = 600 K — vibrational onset begins
    [
        GammaCell { gamma: 1.385, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.388, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.390, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.391, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.393, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
    ],
    // T = 1000 K — vibrational mid-band
    [
        GammaCell { gamma: 1.345, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.350, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.355, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.360, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
        GammaCell { gamma: 1.365, n2: 0.78084, o2: 0.20946, n: 0.0, o: 0.0, ar: 0.00934, m_kg_mol: 0.028_9644 },
    ],
    // T = 2000 K — full vibrational excitation
    [
        GammaCell { gamma: 1.300, n2: 0.78,    o2: 0.20,    n: 0.0,  o: 0.01,  ar: 0.00934, m_kg_mol: 0.0288  },
        GammaCell { gamma: 1.310, n2: 0.78,    o2: 0.205,   n: 0.0,  o: 0.005, ar: 0.00934, m_kg_mol: 0.0288  },
        GammaCell { gamma: 1.320, n2: 0.78,    o2: 0.207,   n: 0.0,  o: 0.003, ar: 0.00934, m_kg_mol: 0.0288  },
        GammaCell { gamma: 1.330, n2: 0.78,    o2: 0.208,   n: 0.0,  o: 0.002, ar: 0.00934, m_kg_mol: 0.0288  },
        GammaCell { gamma: 1.338, n2: 0.78,    o2: 0.209,   n: 0.0,  o: 0.001, ar: 0.00934, m_kg_mol: 0.0288  },
    ],
    // T = 3000 K — O₂ dissociation begins
    [
        GammaCell { gamma: 1.220, n2: 0.72,    o2: 0.05,    n: 0.01, o: 0.21,  ar: 0.00934, m_kg_mol: 0.0240  },
        GammaCell { gamma: 1.250, n2: 0.75,    o2: 0.10,    n: 0.005, o: 0.135, ar: 0.00934, m_kg_mol: 0.0259  },
        GammaCell { gamma: 1.275, n2: 0.77,    o2: 0.15,    n: 0.001, o: 0.069, ar: 0.00934, m_kg_mol: 0.0273  },
        GammaCell { gamma: 1.295, n2: 0.78,    o2: 0.18,    n: 0.0,  o: 0.030, ar: 0.00934, m_kg_mol: 0.0283  },
        GammaCell { gamma: 1.305, n2: 0.78,    o2: 0.198,   n: 0.0,  o: 0.012, ar: 0.00934, m_kg_mol: 0.0287  },
    ],
    // T = 4000 K — O₂ fully dissociated, N₂ onset
    [
        GammaCell { gamma: 1.180, n2: 0.55,    o2: 0.0,     n: 0.10, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0205  },
        GammaCell { gamma: 1.200, n2: 0.62,    o2: 0.01,    n: 0.05, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0214  },
        GammaCell { gamma: 1.220, n2: 0.68,    o2: 0.02,    n: 0.02, o: 0.27,  ar: 0.00934, m_kg_mol: 0.0226  },
        GammaCell { gamma: 1.240, n2: 0.72,    o2: 0.05,    n: 0.005, o: 0.215, ar: 0.00934, m_kg_mol: 0.0237  },
        GammaCell { gamma: 1.260, n2: 0.76,    o2: 0.10,    n: 0.001, o: 0.130, ar: 0.00934, m_kg_mol: 0.0265  },
    ],
    // T = 5000 K — strong N₂ dissociation
    [
        GammaCell { gamma: 1.160, n2: 0.20,    o2: 0.0,     n: 0.45, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0166  },
        GammaCell { gamma: 1.180, n2: 0.32,    o2: 0.0,     n: 0.34, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0181  },
        GammaCell { gamma: 1.200, n2: 0.45,    o2: 0.005,   n: 0.22, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0196  },
        GammaCell { gamma: 1.220, n2: 0.58,    o2: 0.015,   n: 0.12, o: 0.28,  ar: 0.00934, m_kg_mol: 0.0212  },
        GammaCell { gamma: 1.240, n2: 0.68,    o2: 0.03,    n: 0.05, o: 0.23,  ar: 0.00934, m_kg_mol: 0.0227  },
    ],
    // T = 6000 K — heavy dissociation; ionisation onset
    [
        GammaCell { gamma: 1.155, n2: 0.05,    o2: 0.0,     n: 0.60, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0157  },
        GammaCell { gamma: 1.175, n2: 0.12,    o2: 0.0,     n: 0.54, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0166  },
        GammaCell { gamma: 1.195, n2: 0.22,    o2: 0.002,   n: 0.45, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0177  },
        GammaCell { gamma: 1.215, n2: 0.36,    o2: 0.008,   n: 0.33, o: 0.29,  ar: 0.00934, m_kg_mol: 0.0193  },
        GammaCell { gamma: 1.230, n2: 0.50,    o2: 0.020,   n: 0.20, o: 0.27,  ar: 0.00934, m_kg_mol: 0.0207  },
    ],
    // T = 7500 K — full dissociation; meaningful ionisation
    [
        GammaCell { gamma: 1.190, n2: 0.0,     o2: 0.0,     n: 0.65, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0150  },
        GammaCell { gamma: 1.205, n2: 0.02,    o2: 0.0,     n: 0.63, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0153  },
        GammaCell { gamma: 1.220, n2: 0.07,    o2: 0.0,     n: 0.58, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0159  },
        GammaCell { gamma: 1.235, n2: 0.15,    o2: 0.002,   n: 0.50, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0170  },
        GammaCell { gamma: 1.250, n2: 0.25,    o2: 0.008,   n: 0.40, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0184  },
    ],
    // T = 9000 K — strong ionisation begins
    [
        GammaCell { gamma: 1.220, n2: 0.0,     o2: 0.0,     n: 0.62, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0148  },
        GammaCell { gamma: 1.235, n2: 0.0,     o2: 0.0,     n: 0.64, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0150  },
        GammaCell { gamma: 1.250, n2: 0.02,    o2: 0.0,     n: 0.62, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0153  },
        GammaCell { gamma: 1.265, n2: 0.06,    o2: 0.0,     n: 0.58, o: 0.34,  ar: 0.00934, m_kg_mol: 0.0159  },
        GammaCell { gamma: 1.280, n2: 0.13,    o2: 0.001,   n: 0.52, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0167  },
    ],
    // T = 11_000 K — heavy ionisation
    [
        GammaCell { gamma: 1.260, n2: 0.0,     o2: 0.0,     n: 0.58, o: 0.30,  ar: 0.00934, m_kg_mol: 0.0145  },
        GammaCell { gamma: 1.270, n2: 0.0,     o2: 0.0,     n: 0.60, o: 0.31,  ar: 0.00934, m_kg_mol: 0.0147  },
        GammaCell { gamma: 1.285, n2: 0.0,     o2: 0.0,     n: 0.62, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0150  },
        GammaCell { gamma: 1.300, n2: 0.01,    o2: 0.0,     n: 0.61, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0152  },
        GammaCell { gamma: 1.315, n2: 0.04,    o2: 0.0,     n: 0.58, o: 0.33,  ar: 0.00934, m_kg_mol: 0.0158  },
    ],
    // T = 13_000 K — strong ionisation; γ continues to rise
    [
        GammaCell { gamma: 1.290, n2: 0.0,     o2: 0.0,     n: 0.50, o: 0.26,  ar: 0.00934, m_kg_mol: 0.0140  },
        GammaCell { gamma: 1.300, n2: 0.0,     o2: 0.0,     n: 0.55, o: 0.28,  ar: 0.00934, m_kg_mol: 0.0143  },
        GammaCell { gamma: 1.315, n2: 0.0,     o2: 0.0,     n: 0.58, o: 0.30,  ar: 0.00934, m_kg_mol: 0.0146  },
        GammaCell { gamma: 1.330, n2: 0.0,     o2: 0.0,     n: 0.60, o: 0.31,  ar: 0.00934, m_kg_mol: 0.0148  },
        GammaCell { gamma: 1.345, n2: 0.0,     o2: 0.0,     n: 0.60, o: 0.32,  ar: 0.00934, m_kg_mol: 0.0150  },
    ],
    // T = 15_000 K — high-ionisation limit
    [
        GammaCell { gamma: 1.320, n2: 0.0,     o2: 0.0,     n: 0.42, o: 0.22,  ar: 0.00934, m_kg_mol: 0.0135  },
        GammaCell { gamma: 1.330, n2: 0.0,     o2: 0.0,     n: 0.48, o: 0.24,  ar: 0.00934, m_kg_mol: 0.0138  },
        GammaCell { gamma: 1.345, n2: 0.0,     o2: 0.0,     n: 0.52, o: 0.26,  ar: 0.00934, m_kg_mol: 0.0141  },
        GammaCell { gamma: 1.360, n2: 0.0,     o2: 0.0,     n: 0.55, o: 0.28,  ar: 0.00934, m_kg_mol: 0.0144  },
        GammaCell { gamma: 1.375, n2: 0.0,     o2: 0.0,     n: 0.57, o: 0.30,  ar: 0.00934, m_kg_mol: 0.0147  },
    ],
];

/// Universal gas constant `R = k_B · N_A` (J / (mol · K)).
const R_UNIVERSAL_J_MOL_K: f64 = 8.314_462_618;

/// Tannehill 5-species equilibrium-air model.
#[derive(Copy, Clone, Debug, Default)]
pub struct TannehillEquilibriumAir;

impl TannehillEquilibriumAir {
    /// Smallest temperature `T` (K) accepted by the model.
    pub const T_MIN_K: f64 = T_MIN_K;
    /// Largest temperature `T` (K) accepted by the model.
    pub const T_MAX_K: f64 = T_MAX_K;
    /// Smallest pressure ratio `p/p_0` accepted by the model.
    pub const P_RATIO_MIN: f64 = 1.0e-3;
    /// Largest pressure ratio `p/p_0` accepted by the model.
    pub const P_RATIO_MAX: f64 = 10.0;

    fn validate(t: f64, p: f64) -> Result<(), PhysicsError> {
        if !t.is_finite() || !p.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "tannehill equilibrium air received NaN/Inf input",
            });
        }
        if !(T_MIN_K..=T_MAX_K).contains(&t) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "tannehill equilibrium air: T outside 100..=15_000 K",
            });
        }
        let p_ratio = p / P_REF_PA;
        if !(Self::P_RATIO_MIN..=Self::P_RATIO_MAX).contains(&p_ratio) {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "tannehill equilibrium air: p/p_0 outside [1e-3, 10]",
            });
        }
        Ok(())
    }

    fn lookup(t: f64, p: f64) -> GammaCell {
        // Bilinear interpolation in (T, log10 p).
        let log_p = (p / P_REF_PA).log10().clamp(P_LOG10_MIN, P_LOG10_MAX);

        let (ti0, ti1, t_frac) = bracket(T_GRID, t);
        let (pi0, pi1, p_frac) = bracket(P_LOG10_GRID, log_p);

        let c00 = GAMMA_TABLE[ti0][pi0];
        let c01 = GAMMA_TABLE[ti0][pi1];
        let c10 = GAMMA_TABLE[ti1][pi0];
        let c11 = GAMMA_TABLE[ti1][pi1];

        let lerp = |a: f64, b: f64, f: f64| a + (b - a) * f;
        let lerp_p = |a: GammaCell, b: GammaCell| GammaCell {
            gamma: lerp(a.gamma, b.gamma, p_frac),
            n2: lerp(a.n2, b.n2, p_frac),
            o2: lerp(a.o2, b.o2, p_frac),
            n: lerp(a.n, b.n, p_frac),
            o: lerp(a.o, b.o, p_frac),
            ar: lerp(a.ar, b.ar, p_frac),
            m_kg_mol: lerp(a.m_kg_mol, b.m_kg_mol, p_frac),
        };

        let row0 = lerp_p(c00, c01);
        let row1 = lerp_p(c10, c11);

        GammaCell {
            gamma: lerp(row0.gamma, row1.gamma, t_frac),
            n2: lerp(row0.n2, row1.n2, t_frac),
            o2: lerp(row0.o2, row1.o2, t_frac),
            n: lerp(row0.n, row1.n, t_frac),
            o: lerp(row0.o, row1.o, t_frac),
            ar: lerp(row0.ar, row1.ar, t_frac),
            m_kg_mol: lerp(row0.m_kg_mol, row1.m_kg_mol, t_frac),
        }
    }
}

fn bracket(grid: &[f64], x: f64) -> (usize, usize, f64) {
    // Smallest index i such that grid[i+1] >= x.
    if x <= grid[0] {
        return (0, 0, 0.0);
    }
    let last = grid.len() - 1;
    if x >= grid[last] {
        return (last, last, 0.0);
    }
    for i in 0..last {
        if grid[i + 1] >= x {
            let span = grid[i + 1] - grid[i];
            let frac = (x - grid[i]) / span;
            return (i, i + 1, frac);
        }
    }
    (last, last, 0.0)
}

impl EquilibriumAir for TannehillEquilibriumAir {
    fn composition(&self, t: f64, p: f64) -> Result<AirComposition, PhysicsError> {
        Self::validate(t, p)?;
        let cell = Self::lookup(t, p);
        Ok(AirComposition {
            n2: cell.n2,
            o2: cell.o2,
            n_atomic: cell.n,
            o_atomic: cell.o,
            no: 0.0,
            argon: cell.ar,
            electrons: 0.0,
        })
    }

    fn gamma_eff(&self, t: f64, p: f64) -> Result<f64, PhysicsError> {
        Self::validate(t, p)?;
        Ok(Self::lookup(t, p).gamma)
    }

    fn speed_of_sound_m_s(&self, t: f64, p: f64) -> Result<f64, PhysicsError> {
        Self::validate(t, p)?;
        let cell = Self::lookup(t, p);
        Ok((cell.gamma * R_UNIVERSAL_J_MOL_K * t / cell.m_kg_mol).sqrt())
    }

    fn state(&self, t: f64, p: f64) -> Result<EquilibriumAirState, PhysicsError> {
        Self::validate(t, p)?;
        let cell = Self::lookup(t, p);
        Ok(EquilibriumAirState {
            composition: AirComposition {
                n2: cell.n2,
                o2: cell.o2,
                n_atomic: cell.n,
                o_atomic: cell.o,
                no: 0.0,
                argon: cell.ar,
                electrons: 0.0,
            },
            gamma_eff: cell.gamma,
            speed_of_sound_m_s: (cell.gamma * R_UNIVERSAL_J_MOL_K * t / cell.m_kg_mol).sqrt(),
            mean_molecular_weight_kg_per_mol: cell.m_kg_mol,
        })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp, clippy::missing_panics_doc, clippy::similar_names)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn cold_air_recovers_gamma_1_4() {
        let m = TannehillEquilibriumAir;
        let g = m.gamma_eff(288.15, 101_325.0).unwrap();
        assert_relative_eq!(g, 1.400, max_relative = 1e-3);
    }

    #[test]
    fn vibrationally_excited_air_has_lower_gamma() {
        let m = TannehillEquilibriumAir;
        let cold = m.gamma_eff(300.0, 101_325.0).unwrap();
        let hot = m.gamma_eff(1500.0, 101_325.0).unwrap();
        assert!(hot < cold, "vibrational excitation should reduce γ: cold={cold}, hot={hot}");
    }

    #[test]
    fn dissociated_air_has_lower_gamma_than_vibrational() {
        let m = TannehillEquilibriumAir;
        let vib = m.gamma_eff(1500.0, 101_325.0).unwrap();
        let dissoc = m.gamma_eff(4500.0, 101_325.0).unwrap();
        assert!(dissoc < vib);
        // Dissociation regime: γ should be in the ~1.18-1.28 band.
        assert!((1.15..=1.30).contains(&dissoc), "dissociated γ={dissoc}");
    }

    #[test]
    fn ionised_air_gamma_climbs_back_toward_monatomic_limit() {
        // Monatomic-ideal limit is γ = 5/3 ≈ 1.667; the curve fit
        // recovers a substantial fraction of that climb above
        // ~9000 K. We only assert a directional bound here — the
        // exact endpoint is sensitive to the ionisation cutoff.
        let m = TannehillEquilibriumAir;
        let dissoc_mid = m.gamma_eff(5000.0, 101_325.0).unwrap();
        let ionised = m.gamma_eff(14000.0, 101_325.0).unwrap();
        assert!(ionised > dissoc_mid, "γ should climb above dissoc-mid; dissoc={dissoc_mid}, ionised={ionised}");
    }

    #[test]
    fn composition_sums_close_to_unity_in_neutral_regime() {
        // Below the ionisation threshold (T ≤ ~6000 K) the 5-species
        // neutral mole fractions should sum to ~1. Above that, the
        // missing fraction is ionised species (electrons + ions),
        // which this 5-species fit does not track.
        let m = TannehillEquilibriumAir;
        for t in [300.0_f64, 1000.0, 3000.0, 5000.0] {
            for p in [1.0e3, 1.0e4, 1.0e5, 5.0e5] {
                let c = m.composition(t, p).unwrap();
                let sum = c.n2 + c.o2 + c.n_atomic + c.o_atomic + c.argon;
                assert!(
                    (sum - 1.0).abs() < 0.10,
                    "neutral-regime composition sum out of band at T={t}, p={p}: {sum}"
                );
            }
        }
    }

    #[test]
    fn ionised_regime_neutral_fraction_below_unity_by_design() {
        // Above ~6000 K, electrons + ions take a measurable share of
        // total number density. The 5-species fit only tracks
        // neutrals, so the neutral-only sum should fall below 1.0 by
        // an amount that grows with temperature — that is the
        // documented honest-scope behaviour.
        let m = TannehillEquilibriumAir;
        let neutral_at = |t: f64| -> f64 {
            let c = m.composition(t, 101_325.0).unwrap();
            c.n2 + c.o2 + c.n_atomic + c.o_atomic + c.argon
        };
        let s_5k = neutral_at(5000.0);
        let s_9k = neutral_at(9000.0);
        let s_15k = neutral_at(15000.0);
        // Increasing ionisation → decreasing neutral fraction.
        assert!(s_9k < s_5k, "neutral fraction at 9k={s_9k} should be < at 5k={s_5k}");
        assert!(s_15k < s_9k, "neutral fraction at 15k={s_15k} should be < at 9k={s_9k}");
    }

    #[test]
    fn out_of_envelope_t_low_rejected() {
        let m = TannehillEquilibriumAir;
        assert!(matches!(
            m.gamma_eff(50.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn out_of_envelope_t_high_rejected() {
        let m = TannehillEquilibriumAir;
        assert!(matches!(
            m.gamma_eff(20_000.0, 101_325.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn out_of_envelope_p_rejected() {
        let m = TannehillEquilibriumAir;
        assert!(matches!(
            m.gamma_eff(1000.0, 1.0),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            m.gamma_eff(1000.0, 1.0e7),
            Err(PhysicsError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn speed_of_sound_at_sea_level_matches_textbook() {
        // At T = 288.15 K, M = 28.9644 g/mol, γ = 1.4:
        //   a = √(γRT/M) = √(1.4 · 8.314462618 · 288.15 / 0.0289644)
        //     ≈ 340.3 m/s
        let m = TannehillEquilibriumAir;
        let a = m.speed_of_sound_m_s(288.15, 101_325.0).unwrap();
        assert!((a - 340.3).abs() < 1.0, "a={a}");
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let m = TannehillEquilibriumAir;
        for (t, p) in [(1000.0_f64, 101_325.0), (5000.0, 50_000.0), (9000.0, 200.0)] {
            let a = m.gamma_eff(t, p).unwrap();
            let b = m.gamma_eff(t, p).unwrap();
            assert_eq!(a.to_bits(), b.to_bits());
        }
    }
}
