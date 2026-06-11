#!/usr/bin/env python3
"""Generate the LOX/LCH4 Cantera thermochemistry reference tables.

This script is optional developer tooling. It requires Cantera to be installed
in the invoking Python environment and prints TOML fragments for the checked
deck and tolerance table committed under data/thermochem.
"""

from __future__ import annotations

import math


UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K = 8.314_462_618_153_24
STANDARD_GRAVITY_M_S2 = 9.806_65
THROAT_AREA_M2 = 0.02
EXIT_AREA_M2 = 0.24
AMBIENT_PRESSURE_PA = 101_325.0
NOZZLE_MACH_BISECTION_ITERS = 96
C_STAR_EFFICIENCY_MIN = 0.96
C_STAR_EFFICIENCY_NOMINAL = 0.98
C_STAR_EFFICIENCY_MAX = 1.0


def choked_mass_flux_gamma(gamma: float) -> float:
    exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0))
    return math.sqrt(gamma) * (2.0 / (gamma + 1.0)) ** exponent


def characteristic_velocity_m_s(
    chamber_temperature_k: float,
    gamma: float,
    molecular_weight_kg_per_mol: float,
) -> float:
    specific_gas_constant = (
        UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K / molecular_weight_kg_per_mol
    )
    return (
        math.sqrt(specific_gas_constant * chamber_temperature_k)
        / choked_mass_flux_gamma(gamma)
    )


def nozzle_area_ratio(gamma: float, mach: float) -> float:
    gm1 = gamma - 1.0
    gp1 = gamma + 1.0
    bracket = (2.0 / gp1) * (1.0 + 0.5 * gm1 * mach * mach)
    return (1.0 / mach) * bracket ** (gp1 / (2.0 * gm1))


def supersonic_mach_for_area_ratio(gamma: float, expansion_ratio: float) -> float:
    lo = 1.0
    hi = 50.0
    for _ in range(NOZZLE_MACH_BISECTION_ITERS):
        mid = 0.5 * (lo + hi)
        if nozzle_area_ratio(gamma, mid) < expansion_ratio:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def exit_pressure_ratio(gamma: float, mach: float) -> float:
    return (1.0 + 0.5 * (gamma - 1.0) * mach * mach) ** (
        -gamma / (gamma - 1.0)
    )


def ideal_momentum_thrust_coefficient(gamma: float, expansion_ratio: float) -> float:
    exit_mach = supersonic_mach_for_area_ratio(gamma, expansion_ratio)
    pe_pc = exit_pressure_ratio(gamma, exit_mach)
    term = 1.0 - pe_pc ** ((gamma - 1.0) / gamma)
    return math.sqrt(
        (2.0 * gamma * gamma / (gamma - 1.0))
        * (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (gamma - 1.0))
        * term
    )


def nozzle_performance(
    chamber_pressure_pa: float,
    gamma: float,
    c_star_m_s: float,
) -> tuple[float, float, float]:
    expansion_ratio = EXIT_AREA_M2 / THROAT_AREA_M2
    mass_flow_kg_per_s = chamber_pressure_pa * THROAT_AREA_M2 / c_star_m_s
    exit_mach = supersonic_mach_for_area_ratio(gamma, expansion_ratio)
    exit_pressure_pa = chamber_pressure_pa * exit_pressure_ratio(gamma, exit_mach)
    momentum_thrust_n = (
        ideal_momentum_thrust_coefficient(gamma, expansion_ratio)
        * THROAT_AREA_M2
        * chamber_pressure_pa
    )
    pressure_thrust_n = (exit_pressure_pa - AMBIENT_PRESSURE_PA) * EXIT_AREA_M2
    total_thrust_n = momentum_thrust_n + pressure_thrust_n
    isp_s = total_thrust_n / (STANDARD_GRAVITY_M_S2 * mass_flow_kg_per_s)
    return mass_flow_kg_per_s, total_thrust_n, isp_s


def main() -> None:
    try:
        import cantera as ct
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Cantera is required: install it in a throwaway venv with "
            "`python -m pip install cantera`."
        ) from exc

    gas = ct.Solution("gri30.yaml")
    methane_mw = gas.molecular_weights[gas.species_index("CH4")]
    oxygen_mw = gas.molecular_weights[gas.species_index("O2")]
    pressures = [1_000_000.0, 4_000_000.0]
    mixture_ratios = [2.8, 3.4]

    print(f"# Generated with Cantera {ct.__version__} from {gas.source}")
    for chamber_pressure_pa in pressures:
        for mixture_ratio in mixture_ratios:
            oxygen_moles = mixture_ratio * methane_mw / oxygen_mw
            gas.TPX = 298.15, chamber_pressure_pa, {"CH4": 1.0, "O2": oxygen_moles}
            gas.equilibrate("HP", solver="auto")

            gamma = gas.cp_mass / gas.cv_mass
            molecular_weight_kg_per_mol = gas.mean_molecular_weight / 1000.0
            c_star_m_s = characteristic_velocity_m_s(
                gas.T,
                gamma,
                molecular_weight_kg_per_mol,
            )
            ideal_mass_flow_kg_per_s, total_thrust_n, ideal_isp_s = nozzle_performance(
                chamber_pressure_pa,
                gamma,
                c_star_m_s,
            )
            nominal_mass_flow_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_NOMINAL
            mass_flow_min_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_MAX
            mass_flow_max_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_MIN
            nominal_isp_s = total_thrust_n / (
                STANDARD_GRAVITY_M_S2 * nominal_mass_flow_kg_per_s
            )

            print()
            print(f"# pc={chamber_pressure_pa:.1f} Pa, MR={mixture_ratio:.1f}")
            print(f"chamber_temperature_k = {gas.T!r}")
            print(f"gamma = {gamma!r}")
            print(f"molecular_weight_kg_per_mol = {molecular_weight_kg_per_mol!r}")
            print(f"c_star_m_s = {c_star_m_s!r}")
            print(
                "c_star_efficiency = "
                f"{{ min = {C_STAR_EFFICIENCY_MIN!r}, "
                f"nominal = {C_STAR_EFFICIENCY_NOMINAL!r}, "
                f"max = {C_STAR_EFFICIENCY_MAX!r} }}"
            )
            print(f"ideal_mass_flow_kg_per_s = {ideal_mass_flow_kg_per_s!r}")
            print(f"expected_nominal_mass_flow_kg_per_s = {nominal_mass_flow_kg_per_s!r}")
            print(f"expected_mass_flow_min_kg_per_s = {mass_flow_min_kg_per_s!r}")
            print(f"expected_mass_flow_max_kg_per_s = {mass_flow_max_kg_per_s!r}")
            print(f"sea_level_total_thrust_n = {total_thrust_n!r}")
            print(f"ideal_sea_level_isp_s = {ideal_isp_s!r}")
            print(f"expected_nominal_sea_level_isp_s = {nominal_isp_s!r}")


if __name__ == "__main__":
    main()
