//! Phase-3.7 runner-side tank rack.
//!
//! The rack owns a `BTreeMap<TankId, openbmp_vehicle::Tank>` resolved
//! from the scenario's `[[vehicle.assembly.tanks]]` block, plus the
//! construction-time `dt` that's passed to each tank's `step(...)`
//! call and a cached pair `(accel_body, omega_body)` from the prior
//! kernel step's solution.
//!
//! Each kernel base tick the runner:
//!
//! 1. Drains every tank by its scenario-declared
//!    `drain_rate_kg_per_s` (Phase-3.7 ships drain decoupled from
//!    engine-cluster mdot; that coupling is a Phase-3.X follow-on).
//! 2. Steps every tank using the **prior step's** cached
//!    `(accel_body, omega_body)`. The runner caches the freshly-
//!    computed drivers from the step that just completed via
//!    [`TankRack::update_drivers`]. The first step uses zeros.
//! 3. Packs the resulting `mass_contribution()` and
//!    `reaction_body()` observations into a `BTreeMap<TankId,
//!    TankSnapshot>` via [`TankRack::snapshot_map`] and pushes it
//!    into the kernel via `set_tank_snapshot(...)`.
//!
//! Phase-3.7 leaves the kernel's force / moment / mass evaluation
//! untouched when the rack is empty — legacy scenarios produce
//! byte-identical Parquet because the runner short-circuits every
//! rack-related operation on `is_empty()`.
//!
//! # Determinism
//!
//! - Tanks are stored in a `BTreeMap<TankId, Tank>` (defeats macOS
//!   `SipHash` randomisation); iteration is in `TankId` order which
//!   for `from_path`-derived ids is deterministic across reruns.
//! - The kernel-pushed snapshot map is also `BTreeMap<TankId,
//!   TankSnapshot>`.
//! - One-step lag on `(accel_body, omega_body)` is the only
//!   defensible bit-stable solution to the circular dependency
//!   between tank reaction force / mass contribution and the
//!   kernel's per-step force / mass evaluation. Initial step: zeros.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use openbmp_core::{Duration, TankId};
use openbmp_scenario::{
    InitialSloshConfig, MovingMassKindConfig, PropellantSpecConfig, ScenarioDocument, TankConfig,
    TankGeometryConfig,
};
use openbmp_sim::TankSnapshot;
use openbmp_vehicle::{
    BaffleModel, BaffledPendulum, EquivalentPendulum, EquivalentSpringMass, MovingMassModel,
    PropellantSpec, RigidLiquid, Tank, TankGeometry,
};

use crate::error::CliError;

/// Runner-side tank rack. Built once per `openbmp run` invocation;
/// consumed by the per-step kernel loop.
pub struct TankRack {
    tanks: BTreeMap<TankId, Tank>,
    dt: Duration,
    /// Per-tank scenario-declared drain rates (kg/s). Phase-3.7
    /// ships drain decoupled from engines; future phase ties this
    /// to cluster mdot.
    drain_rates_kg_per_s: BTreeMap<TankId, f64>,
    /// Cached `(accel_body_m_s2, omega_body_rad_s)` from the prior
    /// kernel step. Initialised to zeros at construction.
    last_drivers: (Vector3<f64>, Vector3<f64>),
}

impl std::fmt::Debug for TankRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TankRack")
            .field("tank_count", &self.tanks.len())
            .field("dt", &self.dt)
            .field("drain_rates_kg_per_s", &self.drain_rates_kg_per_s)
            .field("last_drivers", &self.last_drivers)
            .finish()
    }
}

impl TankRack {
    /// Build the rack from a parsed scenario document. Returns an
    /// empty rack when no `[[vehicle.assembly.tanks]]` block is
    /// declared.
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Tank`] when a tank config fails one of
    /// the moving-mass model constructors (invalid geometry,
    /// damping, fill fraction, etc.).
    pub fn build(document: &ScenarioDocument) -> Result<Self, CliError> {
        let dt = Duration::from_seconds(document.time.dt_s);
        let mut tanks: BTreeMap<TankId, Tank> = BTreeMap::new();
        let mut drain_rates: BTreeMap<TankId, f64> = BTreeMap::new();

        if let Some(assembly) = &document.vehicle.assembly {
            for config in &assembly.tanks {
                let (id, tank) = build_tank(config)?;
                drain_rates.insert(id, config.drain_rate_kg_per_s.unwrap_or(0.0));
                if tanks.insert(id, tank).is_some() {
                    return Err(CliError::Tank {
                        field: format!("vehicle.assembly.tanks.{id}", id = config.id),
                        reason: "duplicate tank id (collision in fnv1a-64 hash)".to_owned(),
                    });
                }
            }
        }

        Ok(Self {
            tanks,
            dt,
            drain_rates_kg_per_s: drain_rates,
            last_drivers: (Vector3::zeros(), Vector3::zeros()),
        })
    }

    /// `true` when the rack carries no tanks. The runner gates
    /// every per-step rack operation on this so legacy scenarios
    /// never touch the rack code path.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tanks.is_empty()
    }

    /// Number of tanks in the rack.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tanks.len()
    }

    /// Tank ids in `BTreeMap` (sorted) order.
    #[must_use]
    pub fn tank_ids(&self) -> Vec<TankId> {
        self.tanks.keys().copied().collect()
    }

    /// Replace the cached prior-step `(accel_body, omega_body)`.
    /// The runner calls this with the freshly-computed body-frame
    /// translational acceleration and angular rate after the kernel
    /// step that just completed; the next [`Self::step`] uses these
    /// as the slosh-dynamics drivers.
    pub fn update_drivers(
        &mut self,
        accel_body_m_s2: Vector3<f64>,
        omega_body_rad_s: Vector3<f64>,
    ) {
        self.last_drivers = (accel_body_m_s2, omega_body_rad_s);
    }

    /// Drain every tank by its scenario-declared rate, then advance
    /// every tank's slosh state using the cached `(accel, omega)`
    /// from the prior step (one-step lag).
    ///
    /// # Errors
    ///
    /// Returns [`CliError::Tank`] when a tank's `drain` or `step`
    /// rejects (non-finite rate, non-finite drivers).
    pub fn step(&mut self) -> Result<(), CliError> {
        let (accel, omega) = self.last_drivers;
        for (id, tank) in &mut self.tanks {
            let rate = self.drain_rates_kg_per_s.get(id).copied().unwrap_or(0.0);
            tank.drain(rate).map_err(|err| CliError::Tank {
                field: format!("vehicle.assembly.tanks.{id_v}", id_v = id.value()),
                reason: err.to_string(),
            })?;
            tank.step(accel, omega, self.dt)
                .map_err(|err| CliError::Tank {
                    field: format!("vehicle.assembly.tanks.{id_v}", id_v = id.value()),
                    reason: err.to_string(),
                })?;
        }
        Ok(())
    }

    /// Produce the per-tank snapshot map the kernel consumes via
    /// `set_tank_snapshot`.
    #[must_use]
    pub fn snapshot_map(&self) -> BTreeMap<TankId, TankSnapshot> {
        let mut out = BTreeMap::new();
        for (id, tank) in &self.tanks {
            let mass = tank.mass_contribution();
            let reaction = tank.reaction_body();
            out.insert(
                *id,
                TankSnapshot {
                    mass_kg: mass.mass_kg,
                    cg_offset_body_m: mass.cg_offset_body_m,
                    inertia_delta_body_kg_m2: mass.inertia_delta_body_kg_m2,
                    reaction_force_body_n: reaction.force_body_n,
                    reaction_moment_body_n_m: reaction.moment_body_n_m,
                    fluid_remaining_kg: tank.fluid_remaining_kg(),
                },
            );
        }
        out
    }
}

/// Phase-3.7 tank resolver: scenario `TankConfig` → openbmp-vehicle
/// `Tank` with the appropriate `Box<dyn MovingMassModel>` inner.
fn build_tank(config: &TankConfig) -> Result<(TankId, Tank), CliError> {
    let path = format!("vehicle.assembly.tanks.{id}", id = config.id);
    let id = TankId::from_path(&path);
    let geometry = build_geometry(&config.geometry);
    let propellant = build_propellant(&config.propellant);
    let mount = Vector3::new(
        config.mount_point_body_m[0],
        config.mount_point_body_m[1],
        config.mount_point_body_m[2],
    );
    let body_id = openbmp_core::BodyId::from_path(&format!(
        "vehicle.assembly.bodies.{body}",
        body = config.mounted_to
    ));

    let baffle = config.baffle_model.map(|b| BaffleModel {
        damping_increment_zeta: b.damping_increment_zeta,
    });

    let moving_mass: Box<dyn MovingMassModel> = match config.moving_mass {
        MovingMassKindConfig::RigidLiquid => Box::new(
            RigidLiquid::new(geometry, propellant, config.initial_fill_fraction, mount).map_err(
                |err| CliError::Tank {
                    field: path.clone(),
                    reason: err.to_string(),
                },
            )?,
        ),
        MovingMassKindConfig::EquivalentPendulum { damping_ratio_zeta } => {
            let mut model = EquivalentPendulum::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                damping_ratio_zeta,
            )
            .map_err(|err| CliError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_pendulum(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
        MovingMassKindConfig::EquivalentSpringMass { damping_ratio_zeta } => {
            let mut model = EquivalentSpringMass::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                damping_ratio_zeta,
            )
            .map_err(|err| CliError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_spring_mass(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
        MovingMassKindConfig::BaffledPendulum {
            base_damping_ratio_zeta,
        } => {
            let baffle_model = baffle.unwrap_or(BaffleModel {
                damping_increment_zeta: 0.0,
            });
            let mut model = BaffledPendulum::new(
                geometry,
                propellant,
                config.initial_fill_fraction,
                mount,
                base_damping_ratio_zeta,
                baffle_model,
            )
            .map_err(|err| CliError::Tank {
                field: path.clone(),
                reason: err.to_string(),
            })?;
            apply_initial_slosh_baffled_pendulum(&mut model, config.initial_slosh.as_ref())?;
            Box::new(model)
        }
    };

    let tank = Tank::new(
        id,
        geometry,
        body_id,
        mount,
        propellant,
        config.initial_fill_fraction,
        baffle,
        moving_mass,
    )
    .map_err(|err| CliError::Tank {
        field: path.clone(),
        reason: err.to_string(),
    })?;

    Ok((id, tank))
}

fn build_geometry(config: &TankGeometryConfig) -> TankGeometry {
    match *config {
        TankGeometryConfig::Cylinder { radius_m, height_m } => {
            TankGeometry::Cylinder { radius_m, height_m }
        }
        TankGeometryConfig::Sphere { radius_m } => TankGeometry::Sphere { radius_m },
        TankGeometryConfig::EllipsoidTextbook { a_m, b_m, c_m } => {
            TankGeometry::EllipsoidTextbook { a_m, b_m, c_m }
        }
    }
}

fn build_propellant(config: &PropellantSpecConfig) -> PropellantSpec {
    PropellantSpec {
        density_kg_m3: config.density_kg_m3,
        // Leak the scenario-supplied label into a static string so
        // `PropellantSpec` (which holds `&'static str`) can carry it
        // verbatim. One-time leak per scenario load, ≈ 30 bytes per
        // tank — negligible.
        label: Box::leak(config.label.clone().into_boxed_str()),
    }
}

fn apply_initial_slosh_pendulum(
    model: &mut EquivalentPendulum,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), CliError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let angles = initial.angles_rad.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.angles_rad".to_owned(),
        reason: "required for equivalent_pendulum".to_owned(),
    })?;
    let rates = initial.rates_rad_s.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.rates_rad_s".to_owned(),
        reason: "required for equivalent_pendulum".to_owned(),
    })?;
    model
        .set_initial_slosh((angles[0], angles[1]), (rates[0], rates[1]))
        .map_err(|err| CliError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

fn apply_initial_slosh_spring_mass(
    model: &mut EquivalentSpringMass,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), CliError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let displacement = initial.displacement_body_m.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.displacement_body_m".to_owned(),
        reason: "required for equivalent_spring_mass".to_owned(),
    })?;
    let velocity = initial.velocity_body_m_s.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.velocity_body_m_s".to_owned(),
        reason: "required for equivalent_spring_mass".to_owned(),
    })?;
    model
        .set_initial_slosh(
            (displacement[0], displacement[1]),
            (velocity[0], velocity[1]),
        )
        .map_err(|err| CliError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}

fn apply_initial_slosh_baffled_pendulum(
    model: &mut BaffledPendulum,
    initial: Option<&InitialSloshConfig>,
) -> Result<(), CliError> {
    let Some(initial) = initial else {
        return Ok(());
    };
    let angles = initial.angles_rad.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.angles_rad".to_owned(),
        reason: "required for baffled_pendulum".to_owned(),
    })?;
    let rates = initial.rates_rad_s.ok_or_else(|| CliError::Tank {
        field: "vehicle.assembly.tanks[*].initial_slosh.rates_rad_s".to_owned(),
        reason: "required for baffled_pendulum".to_owned(),
    })?;
    model
        .set_initial_slosh((angles[0], angles[1]), (rates[0], rates[1]))
        .map_err(|err| CliError::Tank {
            field: "vehicle.assembly.tanks[*].initial_slosh".to_owned(),
            reason: err.to_string(),
        })
}
