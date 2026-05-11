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

Distributed multi-sensor multi-object tracking over peer-to-peer networks requires fusion rules that remain reliable under unknown cross-correlations and heterogeneous communication. Kullback-Leibler average (KLA), or geometric-average fusion, is well suited to this setting, but fixed or topology-derived fusion weights cannot reflect time-varying posterior informativeness or realized packet delivery. This paper proposes a communication-aware adaptive weighting scheme for consensus-oriented distributed KLA-based LMB fusion. The method builds fusion weights from three complementary cues, namely posterior covariance, realized link quality, and Bernoulli existence confidence, and then applies a weak structure-aware decoupled refinement to the spatial and existence branches. The method is evaluated in an eight-sensor distributed formation scenario with tiered heterogeneous packet loss, where network-level consensus metrics are treated as the primary outcome and conventional local tracking metrics are used as safeguards. Relative to fixed weights, the proposed method reduces consensus OSPA, consensus RMSE, and consensus cardinality disagreement from `2.454`, `2.336`, and `0.715` to `1.786`, `1.563`, and `0.193`, respectively, while also improving mean local E-OSPA and local cardinality error and reducing local RMSE. These results indicate that communication-aware adaptive weighting is an effective way to improve distributed KLA-based LMB fusion under heterogeneous network conditions without sacrificing local tracking quality.

## Terminology Rule

- Do not use bare `4+4` in the paper body, abstract, figure captions, or tables.
- Preferred wording: `an eight-sensor distributed formation scenario composed of two four-sensor formations`
- Shorter wording after first mention: `the dual-formation eight-sensor scenario`

## Abstract Notes

- The abstract normally should not carry citations.
- If final Monte Carlo headline numbers are confirmed, they can be inserted into the fifth sentence without changing the overall structure.
- The citation-backed background for this abstract is provided in `10_reference_seed.md` and expanded in `02_introduction.md`.
