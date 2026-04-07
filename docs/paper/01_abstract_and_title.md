# Abstract And Title

## Recommended Title

Candidate A:

`Communication-Aware Adaptive Fusion Weights for Distributed KLA-Based LMB Fusion Under Heterogeneous Packet Loss`

Candidate B:

`Adaptive KLA-Based Distributed LMB Fusion with Existence Confidence Under Communication Constraints`

Candidate C:

`Communication-Aware and Existence-Aware Adaptive Fusion for Distributed GA-LMB Tracking`

## Paper-Ready Abstract Draft

Distributed multi-sensor multi-object tracking over peer-to-peer networks requires fusion rules that remain reliable when local posteriors are heterogeneous and communication links are imperfect. Kullback-Leibler average (KLA), or geometric-average fusion, is attractive for distributed fusion because it is conservative under unknown cross-correlations, but fixed or topology-only fusion weights cannot reflect time-varying posterior quality and realized communication reliability. This paper proposes a communication-aware adaptive fusion-weight allocation method for distributed GA-LMB fusion under packet loss. The proposed weighting model combines three complementary factors: posterior covariance quality, realized link quality, and an existence-confidence term that measures how decisively a local Bernoulli posterior supports target existence. A weak structure-aware decoupled KLA refinement is further introduced to regularize the spatial and existence branches without letting topology dominate the fusion rule. Experiments on an eight-sensor distributed formation-tracking scenario composed of two four-sensor formations and evaluated under tiered heterogeneous packet loss show that the proposed strategy substantially improves inter-sensor consensus over fixed-weight and partial-factor baselines. In particular, adding existence confidence improves consensus beyond covariance-and-link-only weighting, while the weak structure-aware decoupled refinement delivers the best overall consensus OSPA, RMSE, and cardinality agreement. These results indicate that communication-aware and existence-aware adaptive weighting is an effective way to make distributed KLA-based LMB fusion more robust in heterogeneous network conditions.

## Terminology Rule

- Do not use bare `4+4` in the paper body, abstract, figure captions, or tables.
- Preferred wording: `an eight-sensor distributed formation scenario composed of two four-sensor formations`
- Shorter wording after first mention: `the dual-formation eight-sensor scenario`

## Abstract Notes

- The abstract normally should not carry citations.
- If final Monte Carlo headline numbers are confirmed, they can be inserted into the fifth sentence without changing the overall structure.
- The citation-backed background for this abstract is provided in `10_reference_seed.md` and expanded in `02_introduction.md`.
