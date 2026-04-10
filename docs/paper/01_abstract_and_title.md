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

Distributed multi-sensor multi-object tracking over peer-to-peer networks requires fusion rules that remain reliable when local posteriors are heterogeneous and communication links are imperfect. Kullback-Leibler average (KLA), or geometric-average fusion, is attractive because it remains conservative under unknown cross-correlations, yet fixed or topology-derived weights cannot reflect time-varying posterior quality or realized link reliability. This paper proposes a communication-aware adaptive fusion-weight allocation method for distributed GA-LMB fusion under packet loss. The proposed weighting rule combines posterior covariance quality, realized link quality, and an existence-confidence term that measures how decisively a local Bernoulli posterior supports target existence. A weak structure-aware decoupled KLA refinement is then introduced to modulate the spatial and existence branches without letting topology dominate the fusion rule. Experiments in an eight-sensor distributed formation scenario composed of two four-sensor formations under tiered heterogeneous packet loss show clear consensus gains over fixed-weight and partial-factor baselines. In particular, existence confidence provides the missing improvement beyond covariance-and-link-only weighting, while the weak structure-aware refinement delivers the best overall consensus OSPA, RMSE, and cardinality agreement. These results show that communication-aware adaptive weighting is an effective way to improve distributed KLA-based LMB fusion in heterogeneous network conditions.

## Terminology Rule

- Do not use bare `4+4` in the paper body, abstract, figure captions, or tables.
- Preferred wording: `an eight-sensor distributed formation scenario composed of two four-sensor formations`
- Shorter wording after first mention: `the dual-formation eight-sensor scenario`

## Abstract Notes

- The abstract normally should not carry citations.
- If final Monte Carlo headline numbers are confirmed, they can be inserted into the fifth sentence without changing the overall structure.
- The citation-backed background for this abstract is provided in `10_reference_seed.md` and expanded in `02_introduction.md`.
