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

Distributed multi-sensor multi-object tracking over peer-to-peer networks requires fusion rules that remain conservative under unknown cross-correlations while responding to geometry-induced sensing variation and heterogeneous communication loss. Kullback-Leibler average (KLA), or geometric-average fusion, provides a natural foundation, but fixed or topology-derived weights cannot express posterior informativeness or realized packet delivery. This paper proposes communication-aware adaptive weighting for consensus-oriented distributed KLA-based LMB fusion. The retained configuration combines covariance quality, realized link quality, and existence confidence, then applies a branch-aware refinement that decouples spatial and existence weighting with weak structure-aware modulation. A controlled 50-trial component study evaluates network disagreement, local tracking accuracy, and filter/fusion runtime. Realized link quality supplies the dominant gain, while the combined branch-aware refinement provides a smaller but repeatable spatial-consensus improvement. The retained instantaneous-weight Balanced mode reduces OSPA consensus error, matched localization disagreement, and cardinality dispersion by `28.8%`, `35.5%`, and `85.4%` relative to Fixed Metropolis, while requiring about `1.10x` the fixed runtime. A Cardinality-critical mode adds FID-FIA only to the existence branch and reduces cardinality dispersion by a further `34.8%` relative to Balanced, at the cost of higher localization disagreement and local RMSE. EMA smoothing and final-weight floors are excluded from both modes.

## Terminology Rule

- Do not use bare `4+4` in the paper body, abstract, figure captions, or tables.
- Preferred wording: `an eight-sensor distributed formation scenario composed of two four-sensor formations`
- Shorter wording after first mention: `the dual-formation eight-sensor scenario`

## Abstract Notes

- The abstract normally should not carry citations.
- If final Monte Carlo headline numbers are confirmed, they can be inserted into the fifth sentence without changing the overall structure.
- The citation-backed background for this abstract is provided in `10_reference_seed.md` and expanded in `02_introduction.md`.
