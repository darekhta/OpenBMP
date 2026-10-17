#!/usr/bin/env python3
"""Generate NASA CEA rocket-mode thermochemistry reference tables.

This script is optional developer tooling. It requires the official NASA CEA
Python package to be installed in the invoking Python environment and prints
checked deck or tolerance-table TOML committed under data/thermochem.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys

from generate_cantera_thermochem_reference import PRESSURE_AXIS_PA
from generate_cantera_thermochem_reference import characteristic_velocity_m_s
from generate_cantera_thermochem_reference import print_deck
from generate_cantera_thermochem_reference import print_reference


CEA_PRESSURE_RATIO = 68.0457
CEA_SUPERSONIC_AREA_RATIO = 12.0


@dataclass(frozen=True)
class CeaPairConfig:
    slug: str
    propellant_pair: str
    file_stem: str
    dataset_slug: str
    reactants: list[str]
    fuel_amounts: list[float]
    oxidizer_amounts: list[float]
    pressure_axis_pa: list[float]
    mixture_ratio_axis: list[float]
    interpolation_cases: list[tuple[str, float, float]]
    mechanism: str = "thermo.lib"
    species_subset = None

    @property
    def deck_file(self) -> str:
        return f"{self.file_stem}-schema-1.toml"

    @property
    def reference_file(self) -> str:
        return f"{self.file_stem}-tolerance-v1.toml"

    @property
    def dataset_id(self) -> str:
        return f"openbmp.thermochem.{self.dataset_slug}.v1"

    @property
    def source_description(self) -> str:
        return "thermo.lib"

    @property
    def source_url(self) -> str:
        return "https://github.com/nasa/cea"

    @property
    def tool_version(self) -> str:
        return "NASA CEA 3.2.1"

    @property
    def provenance_equilibrium_description(self) -> str:
        return "rocket-equilibrium"

    @property
    def reference_equilibrium_description(self) -> str:
        return "rocket-equilibrium"

    @property
    def license_or_terms(self) -> str:
        return (
            "NASA CEA Apache-2.0; thermo.lib species data terms documented by "
            "NASA CEA distribution"
        )


PAIR_CONFIGS = {
    "nto-mmh": CeaPairConfig(
        slug="nto-mmh",
        propellant_pair="NTO/MMH",
        file_stem="nto-mmh-nasa-cea-rocket",
        dataset_slug="nto_mmh_nasa_cea_rocket",
        reactants=["CH6N2(L)", "N2O4(L)"],
        fuel_amounts=[1.0, 0.0],
        oxidizer_amounts=[0.0, 1.0],
        pressure_axis_pa=PRESSURE_AXIS_PA,
        mixture_ratio_axis=[1.6, 2.0, 2.2, 2.5, 2.8],
        interpolation_cases=[
            ("pc1p5mpa_mr1p8_interpolated", 1_500_000.0, 1.8),
            ("pc2mpa_mr2p2_log_pressure_interpolated", 2_000_000.0, 2.2),
            ("pc6mpa_mr2p35_interpolated", 6_000_000.0, 2.35),
            ("pc18mpa_mr2p65_interpolated", 18_000_000.0, 2.65),
        ],
    ),
    "lox-rp1": CeaPairConfig(
        slug="lox-rp1",
        propellant_pair="LOX/RP-1",
        file_stem="lox-rp1-nasa-cea-rocket",
        dataset_slug="lox_rp1_nasa_cea_rocket",
        reactants=["RP-1", "O2(L)"],
        fuel_amounts=[1.0, 0.0],
        oxidizer_amounts=[0.0, 1.0],
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
DEFAULT_PAIR = "nto-mmh"


def cea_states(config: CeaPairConfig):
    try:
        import cea
        from cea import matlab
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "NASA CEA is required: install it in a throwaway venv with "
            "`python -m pip install cea==3.2.1`."
        ) from exc

    if cea.__version__ != "3.2.1":
        raise SystemExit(f"expected NASA CEA 3.2.1, got {cea.__version__}")

    states: dict[tuple[float, float], tuple[float, float, float, float]] = {}
    for chamber_pressure_pa in config.pressure_axis_pa:
        for mixture_ratio in config.mixture_ratio_axis:
            solution = matlab.rocket_solve(
                config.reactants,
                pc=chamber_pressure_pa / 100_000.0,
                pi_p=CEA_PRESSURE_RATIO,
                supar=[CEA_SUPERSONIC_AREA_RATIO],
                T_reac=[298.15] * len(config.reactants),
                fuel_amounts=config.fuel_amounts,
                oxid_amounts=config.oxidizer_amounts,
                of_ratio=mixture_ratio,
                iac=True,
                n_frz=2,
            )
            if not solution.converged:
                raise SystemExit(
                    f"NASA CEA did not converge for {config.slug} "
                    f"pc={chamber_pressure_pa} MR={mixture_ratio}: "
                    f"error {solution.last_error}"
                )

            chamber_temperature_k = float(solution.T[0])
            gamma = float(solution.gamma_s[0])
            molecular_weight_kg_per_mol = float(solution.MW[0]) / 1000.0
            c_star_m_s = characteristic_velocity_m_s(
                chamber_temperature_k,
                gamma,
                molecular_weight_kg_per_mol,
            )

            states[(chamber_pressure_pa, mixture_ratio)] = (
                chamber_temperature_k,
                gamma,
                molecular_weight_kg_per_mol,
                c_star_m_s,
            )
    return states


def main() -> None:
    if len(sys.argv) not in {2, 3} or sys.argv[1] not in {"deck", "reference"}:
        raise SystemExit(
            "usage: generate_nasa_cea_thermochem_reference.py "
            "{deck|reference} [nto-mmh|lox-rp1]"
        )
    pair = sys.argv[2] if len(sys.argv) == 3 else DEFAULT_PAIR
    if pair not in PAIR_CONFIGS:
        raise SystemExit(f"unknown pair {pair!r}; expected one of {sorted(PAIR_CONFIGS)}")
    config = PAIR_CONFIGS[pair]

    states = cea_states(config)
    if sys.argv[1] == "deck":
        print_deck(config, states)
    else:
        print_reference(config, states)


if __name__ == "__main__":
    main()
