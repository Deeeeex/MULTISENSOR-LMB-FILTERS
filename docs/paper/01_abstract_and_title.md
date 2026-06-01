# Abstract And Title

## Recommended Title

Recommended:

`Communication-Aware Adaptive Weights for Consensus-Oriented Distributed KLA-Based LMB Fusion`

Backup 1:

`Distributed KLA-Based LMB Fusion With Communication-Aware Adaptive Weights`

Backup 2:

`Communication-Aware Adaptive KLA Fusion for Distributed LMB Multi-Object Tracking`

## Title Direction

- Related papers in this line usually keep the title centered on one method noun phrase plus one setting qualifier, rather than enumerating every internal module.
- Here the added qualifier `consensus-oriented` is intentional: it makes the paper's primary objective explicit while avoiding ambiguity with statistical filter-consistency terminology.
- The strongest common templates are:
  - `X fusion/filter for Y`
  - `X fusion with Z`
  - `X ... using Y`
- Therefore `existence confidence` and `weak structure-aware decoupling` should remain in the abstract and method sections, not in the title itself.

## Paper-Ready Abstract Draft

Distributed multi-sensor multi-object tracking over peer-to-peer networks requires fusion rules that remain reliable under unknown cross-correlations and heterogeneous communication. Kullback-Leibler average (KLA), or geometric-average fusion, is well suited to this setting, but fixed or topology-derived fusion weights cannot reflect time-varying posterior informativeness or realized packet delivery. This paper proposes a communication-aware adaptive weighting family for consensus-oriented distributed KLA-based LMB fusion. The method builds fusion weights from posterior covariance, realized link quality, and Bernoulli existence confidence, applies a weak structure-aware decoupled refinement, and can inject an FID-FIA information-geometric score only into the existence branch. The method is evaluated in an eight-sensor distributed formation scenario with tiered heterogeneous packet loss, where network-level disagreement metrics, conventional local tracking metrics, and filter/fusion runtime are treated as complementary evaluation axes. Relative to Fixed Metropolis, the Cardinality-critical mode reduces OSPA consensus error, matched localization disagreement, and cardinality dispersion from `2.454`, `2.336`, and `0.715` to `1.669`, `1.528`, and `0.061`, respectively, while also improving mean local E-OSPA and local cardinality error and keeping local RMSE below FID-FIA-weighted GA. The complementary Balanced mode provides the lower-overhead position-oriented option when spatial accuracy or runtime is the limiting requirement, whereas the Cardinality-critical mode spends additional information-geometric computation for stronger target-number agreement. These results indicate that communication-aware adaptive weighting with branch-specific operating modes is an effective way to improve distributed KLA-based LMB fusion under heterogeneous network conditions without hiding the spatial-versus-cardinality-versus-cost tradeoff.

## Terminology Rule

- Do not use bare `4+4` in the paper body, abstract, figure captions, or tables.
- Preferred wording: `an eight-sensor distributed formation scenario composed of two four-sensor formations`
- Shorter wording after first mention: `the dual-formation eight-sensor scenario`

## Abstract Notes

- The abstract normally should not carry citations.
- If final Monte Carlo headline numbers are confirmed, they can be inserted into the fifth sentence without changing the overall structure.
- The citation-backed background for this abstract is provided in `10_reference_seed.md` and expanded in `02_introduction.md`.
