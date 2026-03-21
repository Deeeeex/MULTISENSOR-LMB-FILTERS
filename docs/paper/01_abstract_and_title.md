# Abstract And Title

## Title Candidates

Candidate A:

`Consistency-Aware Adaptive Weight Allocation for KLA-Based Distributed LMB Fusion Under Communication Constraints`

Candidate B:

`Adaptive Fusion-Weight Allocation for KLA-Based Multi-Sensor LMB Tracking with Decoupled NIS Consistency Penalty`

Candidate C:

`Robust Consistency-Aware Adaptive KLA Fusion for Distributed Multi-Sensor LMB Tracking`

## Abstract Skeleton

Paragraph logic:

1. Background problem
   Fixed-weight distributed KLA fusion is sensitive to heterogeneous sensor quality and communication degradation.
2. Method
   Propose an adaptive fusion-weight allocation scheme for GA-LMB/KLA fusion using availability, covariance quality, link quality, and NIS-based consistency.
3. Key technical point
   Replace monotonic NIS scoring with a decoupled robust consistency penalty based on chi-square consistency intervals.
4. Main evidence
   Report gains on 4+4 distributed formation scenarios and show `robust NIS` is more stable than plain `NIS`.
5. Final takeaway
   The method primarily improves inter-sensor consensus while preserving acceptable local tracking behavior.

## One-Sentence Summary

We propose a communication-aware and consistency-aware adaptive weighting strategy for distributed KLA-based GA-LMB fusion, where robust decoupled NIS improves consensus stability over both fixed weights and plain NIS weighting.

## Numbers To Consider In Abstract

- GA main result: consensus OSPA `2.203 -> 1.686`
- GA main result: consensus RMSE `1.942 -> 1.440`
- GA main result: consensus cardinality `0.604 -> 0.158`
- GA NIS ablation: plain `NIS` degrades all three consensus metrics relative to `robust NIS`

## Abstract Warnings

- Use Monte Carlo-backed numbers when possible.
- If the abstract uses the stronger `2.203 -> 1.686` result, clearly treat it as the main scenario result, not the sole proof of generality.
