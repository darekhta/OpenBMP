# Aerothermal Radiative Data Provenance

## `data/aerothermal/radiative/tauber-sutton-earth-v1.toml`

- dataset id: `tauber-sutton-earth-air-radiative-v1`
- validation label: `checked`
- source: Tauber, M. E.; Sutton, K., "Stagnation-point radiative
  heating relations for Earth and Mars entries," Journal of Spacecraft
  and Rockets 28(1), 1991, doi:10.2514/3.26206.
- secondary public reproduction: Brykina et al., Journal of Applied
  Mechanics and Technical Physics, 2019.
- purpose: Pin the WP-04.1-a Tauber-Sutton Earth-air stagnation-point
  radiative-heating coefficient set and velocity function used by the
  opt-in `TauberSuttonRadiative::earth_air()` model.
- scope boundary: deterministic table interpolation, envelope rejection,
  unit conversion, and total-heating composition only. This does not
  claim Apollo-4, FIRE-II, NEQAIR, or coupled radiation-transport
  validation.
