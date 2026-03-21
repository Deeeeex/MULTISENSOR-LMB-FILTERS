# Method: Adaptive KLA Fusion

## Recommended Section Layout

1. Baseline KLA or GA fusion
2. Adaptive fusion-weight factorization
3. Decoupled robust NIS consistency penalty
4. Temporal smoothing and minimum-weight stabilization
5. Optional extensions

## Core Weight Factorization

Use the current implementation as the paper backbone:

```text
baseScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t)
rawScore_j(t)  = baseScore_j(t) * innovationPenalty_j(t) * associationScore_j(t) * historyScore_j(t)
```

Recommended main paper version:

```text
rawScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t) * innovationPenalty_j(t)
```

Reason:

- `historyScore` is currently weak as a headline method point.
- `associationScore` is better treated as an extension unless later evidence improves.

## Terms To Explain

`mask_j(t)`:

- availability under communication and outage constraints

`covScore_j(t)`:

- posterior concentration proxy
- currently implemented by inverse mean posterior covariance trace

`linkQuality_j(t)`:

- delivered over delivered plus dropped measurements

`innovationPenalty_j(t)`:

- consistency penalty from aggregated NIS statistics

## NIS Narrative

This is the main technical subsection.

Required points:

- NIS and posterior covariance are structurally coupled through the innovation covariance.
- A monotonic `smaller NIS is better` mapping causes double counting.
- The fix is to treat NIS as a consistency test, not a second quality reward.
- `robust NIS` uses quantile or median aggregation.
- The penalty is asymmetric: weaker for too-small NIS, stronger for too-large NIS.

## Useful Equation Skeleton

Suggested equations to include:

- KLA or geometric-average fusion rule
- covariance score definition
- NIS definition and normalized form
- chi-square consistency interval
- asymmetric soft penalty
- EMA smoothing of the final weights

## Optional Modules

Describe briefly and demote:

- `historyScore`: optional temporal-stability term
- `associationScore`: optional ambiguity-aware term

Recommended treatment:

- include in the method as optional extensions
- evaluate in ablation
- do not make them part of the claimed core method

## Source Files

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `docs/ADAPTIVE_FUSION_WEIGHTS_CN.md`
- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`
