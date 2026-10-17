# Slosh Data Provenance

Canonical OpenBMP provenance record for checked-in slosh mechanical-analog
coefficient decks.

## `abramson-cylinder-table-v1.toml`

- Deck path: `data/slosh/abramson-cylinder-table-v1.toml`
- Scope: cylindrical-tank Abramson/Dodge fixed-mass plus N slosh-mass mechanical
  analog checks for the first three lateral antisymmetric modes.
- Source: Dodge, F. T., *The New "Dynamic Behavior of Liquids in Moving
  Containers"*, update to NASA SP-106.
- Source URL: <https://www2.swri.org/www2/fluid-slosh/dodge-sp-106-update.pdf>
- Source PDF SHA-256 at retrieval:
  `2985277d1b7a0b7ad445cc11c5bc56587a89a5b8998d5ddc549976d75c5f0d31`
- Deck SHA-256:
  `92a9b642aada4b1d65a38e88c4cc16c09a666613adc5069c0d986acc69717cbe`
- Derivation: hand-authored numerical evaluations of Table 3.2 cylindrical-tank
  formulas for `xi_n`, `m_n / m_liq`, `L_n / d`, hinge height over diameter, and
  slosh-mass height over diameter at representative `h / d` values.
- Verification: `cargo test -p openbmp-vehicle abramson_multi_mass --locked`
  parses the deck, checks this provenance file carries the source SHA pin, and
  compares the production closed-form helper to every deck row within the 1%
  WP-02.1-c tolerance.
