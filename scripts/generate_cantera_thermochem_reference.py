#!/usr/bin/env python3
"""Generate Cantera thermochemistry reference tables.

This script is optional developer tooling. It requires Cantera to be installed
in the invoking Python environment and prints checked deck or tolerance-table
TOML committed under data/thermochem.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import sys


UNIVERSAL_GAS_CONSTANT_J_PER_MOL_K = 8.314_462_618_153_24
STANDARD_GRAVITY_M_S2 = 9.806_65
THROAT_AREA_M2 = 0.02
EXIT_AREA_M2 = 0.24
AMBIENT_PRESSURE_PA = 101_325.0
NOZZLE_MACH_BISECTION_ITERS = 96
C_STAR_EFFICIENCY_MIN = 0.96
C_STAR_EFFICIENCY_NOMINAL = 0.98
C_STAR_EFFICIENCY_MAX = 1.0


@dataclass(frozen=True)
class PairConfig:
    slug: str
    propellant_pair: str
    mechanism: str
    file_stem: str
    dataset_slug: str
    fuel_species: str
    oxidizer_species: str
    pressure_axis_pa: list[float]
    mixture_ratio_axis: list[float]
    interpolation_cases: list[tuple[str, float, float]]

    @property
    def deck_file(self) -> str:
        return f"{self.file_stem}-schema-1.toml"

    @property
    def reference_file(self) -> str:
        return f"{self.file_stem}-tolerance-v1.toml"

    @property
    def dataset_id(self) -> str:
        return f"openbmp.thermochem.{self.dataset_slug}.v1"


PRESSURE_AXIS_PA = [1_000_000.0, 4_000_000.0, 10_000_000.0, 30_000_000.0]
PAIR_CONFIGS = {
    "lox-lch4": PairConfig(
        slug="lox-lch4",
        propellant_pair="LOX/LCH4",
        mechanism="gri30.yaml",
        file_stem="lox-lch4-cantera-gri30",
        dataset_slug="lox_lch4_cantera_gri30",
        fuel_species="CH4",
        oxidizer_species="O2",
        pressure_axis_pa=PRESSURE_AXIS_PA,
        mixture_ratio_axis=[2.6, 2.8, 3.1, 3.4, 3.6],
        interpolation_cases=[
            ("pc1p5mpa_mr2p7_interpolated", 1_500_000.0, 2.7),
            ("pc2mpa_mr3p1_log_pressure_interpolated", 2_000_000.0, 3.1),
            ("pc6mpa_mr3p25_interpolated", 6_000_000.0, 3.25),
            ("pc18mpa_mr3p5_interpolated", 18_000_000.0, 3.5),
        ],
    ),
    "lox-lh2": PairConfig(
        slug="lox-lh2",
        propellant_pair="LOX/LH2",
        mechanism="gri30.yaml",
        file_stem="lox-lh2-cantera-gri30",
        dataset_slug="lox_lh2_cantera_gri30",
        fuel_species="H2",
        oxidizer_species="O2",
        pressure_axis_pa=PRESSURE_AXIS_PA,
        mixture_ratio_axis=[4.5, 5.0, 5.5, 6.0, 6.5],
        interpolation_cases=[
            ("pc1p5mpa_mr4p75_interpolated", 1_500_000.0, 4.75),
            ("pc2mpa_mr5p5_log_pressure_interpolated", 2_000_000.0, 5.5),
            ("pc6mpa_mr5p75_interpolated", 6_000_000.0, 5.75),
            ("pc18mpa_mr6p25_interpolated", 18_000_000.0, 6.25),
        ],
    ),
    "lox-rp1-ndodecane": PairConfig(
        slug="lox-rp1-ndodecane",
        propellant_pair="LOX/RP-1 n-dodecane surrogate",
        mechanism="nDodecane_Reitz.yaml",
        file_stem="lox-rp1-ndodecane-cantera-reitz",
        dataset_slug="lox_rp1_ndodecane_cantera_reitz",
        fuel_species="c12h26",
        oxidizer_species="o2",
        pressure_axis_pa=PRESSURE_AXIS_PA,
        mixture_ratio_axis=[2.2, 2.4, 2.6, 2.8, 3.0],
        interpolation_cases=[
            ("pc1p5mpa_mr2p3_interpolated", 1_500_000.0, 2.3),
            ("pc2mpa_mr2p6_log_pressure_interpolated", 2_000_000.0, 2.6),
            ("pc6mpa_mr2p7_interpolated", 6_000_000.0, 2.7),
            ("pc18mpa_mr2p9_interpolated", 18_000_000.0, 2.9),
        ],
    ),
}
DEFAULT_PAIR = "lox-lch4"


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


def lerp_tuple(
    a: tuple[float, ...],
    b: tuple[float, ...],
    fraction: float,
) -> tuple[float, ...]:
    return tuple((1.0 - fraction) * av + fraction * bv for av, bv in zip(a, b))


def bracket_fraction(
    axis: list[float],
    query: float,
    transform=lambda value: value,
) -> tuple[int, int, float]:
    transformed_axis = [transform(value) for value in axis]
    transformed_query = transform(query)
    for low in range(len(axis) - 1):
        high = low + 1
        if transformed_axis[low] <= transformed_query <= transformed_axis[high]:
            fraction = (transformed_query - transformed_axis[low]) / (
                transformed_axis[high] - transformed_axis[low]
            )
            return low, high, fraction
    raise ValueError(f"query {query} outside interpolation axis {axis}")


def interpolated_state(
    states: dict[tuple[float, float], tuple[float, float, float, float]],
    pressures: list[float],
    mixture_ratios: list[float],
    chamber_pressure_pa: float,
    mixture_ratio: float,
) -> tuple[float, float, float, float]:
    p0, p1, pressure_fraction = bracket_fraction(
        pressures,
        chamber_pressure_pa,
        math.log,
    )
    m0, m1, mixture_fraction = bracket_fraction(mixture_ratios, mixture_ratio)
    low_pressure = lerp_tuple(
        states[(pressures[p0], mixture_ratios[m0])],
        states[(pressures[p0], mixture_ratios[m1])],
        mixture_fraction,
    )
    high_pressure = lerp_tuple(
        states[(pressures[p1], mixture_ratios[m0])],
        states[(pressures[p1], mixture_ratios[m1])],
        mixture_fraction,
    )
    return lerp_tuple(low_pressure, high_pressure, pressure_fraction)


def pressure_label(chamber_pressure_pa: float) -> str:
    label = f"{chamber_pressure_pa / 1_000_000.0:g}".replace(".", "p")
    return f"{label}mpa"


def mixture_ratio_label(mixture_ratio: float) -> str:
    return f"{mixture_ratio:g}".replace(".", "p")


def toml_float(value: float) -> str:
    return repr(float(value))


def toml_array(values: list[float]) -> str:
    return "[" + ", ".join(toml_float(value) for value in values) + "]"


def print_deck(
    config: PairConfig,
    states: dict[tuple[float, float], tuple[float, float, float, float]],
) -> None:
    print("openbmp.thermochem_deck = 1")
    print()
    print("[meta]")
    print(f'propellant_pair = "{config.propellant_pair}"')
    print(
        f'provenance = "Generated with Cantera 3.2.0 {config.mechanism} gas-phase HP equilibrium; see data/thermochem/provenance.md"'
    )
    print('validation = "research"')
    print()
    print("[axes]")
    print(f"chamber_pressure_pa = {toml_array(config.pressure_axis_pa)}")
    print(f"mixture_ratio = {toml_array(config.mixture_ratio_axis)}")
    for chamber_pressure_pa in config.pressure_axis_pa:
        for mixture_ratio in config.mixture_ratio_axis:
            chamber_temperature_k, gamma, molecular_weight_kg_per_mol, c_star_m_s = states[
                (chamber_pressure_pa, mixture_ratio)
            ]
            print()
            print("[[state]]")
            print(f"chamber_pressure_pa = {toml_float(chamber_pressure_pa)}")
            print(f"mixture_ratio = {toml_float(mixture_ratio)}")
            print(f"chamber_temperature_k = {toml_float(chamber_temperature_k)}")
            print(f"gamma = {toml_float(gamma)}")
            print(
                "molecular_weight_kg_per_mol = "
                f"{toml_float(molecular_weight_kg_per_mol)}"
            )
            print(f"c_star_m_s = {toml_float(c_star_m_s)}")
            print(
                "c_star_efficiency = "
                f"{{ min = {toml_float(C_STAR_EFFICIENCY_MIN)}, "
                f"nominal = {toml_float(C_STAR_EFFICIENCY_NOMINAL)}, "
                f"max = {toml_float(C_STAR_EFFICIENCY_MAX)} }}"
            )


def print_reference_header(config: PairConfig) -> None:
    print("openbmp.thermochem_reference = 1")
    print()
    print("[meta]")
    print(f'dataset_id = "{config.dataset_id}"')
    print('source_class = "public-open-source-tool"')
    print(
        f'source_title = "Cantera 3.2.0 {config.mechanism} {config.propellant_pair} HP-equilibrium thermochemistry tolerance table"'
    )
    print('source_url = "https://cantera.org/"')
    print(f'mechanism = "{config.mechanism}"')
    print(
        'license_or_terms = "Cantera BSD-3-Clause; GRI-Mech 3.0 mechanism terms documented by Cantera distribution"'
    )
    print('validation = "research"')
    print()
    print("[deck]")
    print(f'file = "{config.deck_file}"')
    print()
    print("[nozzle]")
    print(f"throat_area_m2 = {toml_float(THROAT_AREA_M2)}")
    print(f"exit_area_m2 = {toml_float(EXIT_AREA_M2)}")
    print(f"ambient_pressure_pa = {toml_float(AMBIENT_PRESSURE_PA)}")
    print('separation = "off"')
    print()
    print("[tolerances]")
    print("relative = 1.0e-10")
    print("absolute_temperature_k = 1.0e-8")
    print("absolute_gamma = 1.0e-12")
    print("absolute_molecular_weight_kg_per_mol = 1.0e-14")
    print("absolute_c_star_m_s = 1.0e-8")
    print("absolute_mass_flow_kg_per_s = 1.0e-10")
    print("absolute_isp_s = 1.0e-8")
    print("absolute_thrust_n = 1.0e-7")


def print_reference_case(
    name: str,
    chamber_pressure_pa: float,
    mixture_ratio: float,
    chamber_temperature_k: float,
    gamma: float,
    molecular_weight_kg_per_mol: float,
    c_star_m_s: float,
) -> None:
    ideal_mass_flow_kg_per_s, total_thrust_n, ideal_isp_s = nozzle_performance(
        chamber_pressure_pa,
        gamma,
        c_star_m_s,
    )
    nominal_mass_flow_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_NOMINAL
    mass_flow_min_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_MAX
    mass_flow_max_kg_per_s = ideal_mass_flow_kg_per_s / C_STAR_EFFICIENCY_MIN
    nominal_isp_s = total_thrust_n / (STANDARD_GRAVITY_M_S2 * nominal_mass_flow_kg_per_s)

    print()
    print("[[case]]")
    print(f'name = "{name}"')
    print(f"chamber_pressure_pa = {toml_float(chamber_pressure_pa)}")
    print(f"mixture_ratio = {toml_float(mixture_ratio)}")
    print(f"expected_chamber_temperature_k = {toml_float(chamber_temperature_k)}")
    print(f"expected_gamma = {toml_float(gamma)}")
    print(f"expected_molecular_weight_kg_per_mol = {toml_float(molecular_weight_kg_per_mol)}")
    print(f"expected_c_star_m_s = {toml_float(c_star_m_s)}")
    print(
        "expected_nominal_mass_flow_kg_per_s = "
        f"{toml_float(nominal_mass_flow_kg_per_s)}"
    )
    print(f"expected_mass_flow_min_kg_per_s = {toml_float(mass_flow_min_kg_per_s)}")
    print(f"expected_mass_flow_max_kg_per_s = {toml_float(mass_flow_max_kg_per_s)}")
    print(f"expected_thrust_n = {toml_float(total_thrust_n)}")
    print(f"expected_isp_s = {toml_float(nominal_isp_s)}")


def cantera_states(config: PairConfig):
    try:
        import cantera as ct
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Cantera is required: install it in a throwaway venv with "
            "`python -m pip install cantera==3.2.0`."
        ) from exc


    gas = ct.Solution(config.mechanism)
    fuel_mw = gas.molecular_weights[gas.species_index(config.fuel_species)]
    oxidizer_mw = gas.molecular_weights[gas.species_index(config.oxidizer_species)]
    states: dict[tuple[float, float], tuple[float, float, float, float]] = {}

    if ct.__version__ != "3.2.0":
        raise SystemExit(f"expected Cantera 3.2.0, got {ct.__version__}")
    if gas.source != config.mechanism:
        raise SystemExit(f"expected {config.mechanism}, got {gas.source}")

    for chamber_pressure_pa in config.pressure_axis_pa:
        for mixture_ratio in config.mixture_ratio_axis:
            oxidizer_moles = mixture_ratio * fuel_mw / oxidizer_mw
            gas.TPX = (
                298.15,
                chamber_pressure_pa,
                {config.fuel_species: 1.0, config.oxidizer_species: oxidizer_moles},
            )
            gas.equilibrate("HP", solver="auto")

            gamma = gas.cp_mass / gas.cv_mass
            molecular_weight_kg_per_mol = gas.mean_molecular_weight / 1000.0
            c_star_m_s = characteristic_velocity_m_s(
                gas.T,
                gamma,
                molecular_weight_kg_per_mol,
            )
            states[(chamber_pressure_pa, mixture_ratio)] = (
                gas.T,
                gamma,
                molecular_weight_kg_per_mol,
                c_star_m_s,
            )
    return states


def print_reference(
    config: PairConfig,
    states: dict[tuple[float, float], tuple[float, float, float, float]],
) -> None:
    print_reference_header(config)
    for chamber_pressure_pa in config.pressure_axis_pa:
        for mixture_ratio in config.mixture_ratio_axis:
            print_reference_case(
                f"pc{pressure_label(chamber_pressure_pa)}_mr{mixture_ratio_label(mixture_ratio)}",
                chamber_pressure_pa,
                mixture_ratio,
                *states[(chamber_pressure_pa, mixture_ratio)],
            )

    for name, chamber_pressure_pa, mixture_ratio in config.interpolation_cases:
        print_reference_case(
            name,
            chamber_pressure_pa,
            mixture_ratio,
            *interpolated_state(
                states,
                config.pressure_axis_pa,
                config.mixture_ratio_axis,
                chamber_pressure_pa,
                mixture_ratio,
            ),
        )


def main() -> None:
    if len(sys.argv) not in {2, 3} or sys.argv[1] not in {"deck", "reference"}:
        raise SystemExit(
            "usage: generate_cantera_thermochem_reference.py "
            "{deck|reference} [lox-lch4|lox-lh2|lox-rp1-ndodecane]"
        )
    pair = sys.argv[2] if len(sys.argv) == 3 else DEFAULT_PAIR
    if pair not in PAIR_CONFIGS:
        raise SystemExit(f"unknown pair {pair!r}; expected one of {sorted(PAIR_CONFIGS)}")
    config = PAIR_CONFIGS[pair]

    states = cantera_states(config)
    if sys.argv[1] == "deck":
        print_deck(config, states)
    else:
        print_reference(config, states)


if __name__ == "__main__":
    main()
