# Abstract And Title

## Title Candidates

Candidate A:

`Adaptive Fusion-Weight Allocation with Existence Confidence for KLA-Based Distributed LMB Fusion Under Communication Constraints`

Candidate B:

`Communication-Aware and Existence-Confidence-Aware Adaptive KLA Fusion for Distributed Multi-Sensor LMB Tracking`

Candidate C:

`Adaptive KLA Fusion with Posterior-Quality and Existence-Confidence Weighting Under Heterogeneous Packet Loss`

## Abstract Skeleton

Paragraph logic:

1. Background problem
   Fixed-weight distributed KLA fusion is sensitive to heterogeneous sensor quality and communication degradation.
2. Method
   Propose an adaptive fusion-weight allocation scheme for GA-LMB/KLA fusion using availability, covariance quality, realized link quality, and existence-confidence.
3. Key technical point
   Introduce an existence-confidence factor that captures how decisively each local posterior supports target existence, complementing covariance and link quality.
4. Main evidence
   Report gains on 4+4 distributed formation scenarios with tiered heterogeneous packet loss, including improvements beyond the `covariance + link quality` baseline and a further gain from weak structure-aware decoupled KLA.
5. Final takeaway
   The method primarily improves inter-sensor consensus while preserving acceptable local tracking behavior.

## One-Sentence Summary

We propose a communication-aware adaptive weighting strategy for distributed KLA-based GA-LMB fusion that combines covariance, realized link quality, and existence-confidence, and show that a weak structure-aware decoupled refinement achieves the strongest consensus performance under tiered packet loss.

## Numbers To Consider In Abstract

- Tiered GA main result: fixed -> full adaptive gives consensus OSPA `2.624 -> 1.864`
- Tiered GA main result: fixed -> full adaptive gives consensus RMSE `2.703 -> 1.750`
- Tiered GA main result: fixed -> full adaptive gives consensus cardinality `0.879 -> 0.245`
- Tiered targeted ablation: `cov + link` -> `cov + link + structure-aware decoupled KLA` improves OSPA and RMSE while preserving cardinality

## Abstract Warnings

- Use Monte Carlo-backed numbers when possible.
- If the abstract uses the stronger `2.203 -> 1.686` result, clearly treat it as the main scenario result, not the sole proof of generality.
