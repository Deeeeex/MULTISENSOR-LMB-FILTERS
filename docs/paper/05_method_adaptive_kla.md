# Method: Adaptive KLA Fusion

## Recommended Section Layout

1. Baseline KLA or GA fusion
2. Adaptive fusion-weight factorization
3. Existence-confidence weighting
4. Weak structure-aware decoupled refinement
5. Temporal smoothing and minimum-weight stabilization
6. Optional consistency and extension modules

## Core Weight Factorization

Use the current implementation as the paper backbone:

```text
baseScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t)
rawScore_j(t)  = baseScore_j(t) * existenceConfidence_j(t) * innovationPenalty_j(t) * associationScore_j(t) * historyScore_j(t)
```

Recommended main paper version:

```text
rawScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t) * existenceConfidence_j(t)
```

Reason:

- `historyScore` is currently weak as a headline method point.
- `associationScore` is better treated as an extension unless later evidence improves.
- `robust NIS` is useful as a consistency-analysis module, but not the strongest current headline gain.

Current best refinement layer:

```text
spatialScore_j(t)   = blend(rawScore_j(t), spatialDedicatedScore_j(t), eta_s) * structurePrior^gamma_s
existenceScore_j(t) = blend(rawScore_j(t), existenceDedicatedScore_j(t), eta_e) * structurePrior^gamma_e
```

where the structure-aware part is intentionally weak on the existence branch.

## Terms To Explain

`mask_j(t)`:

- availability under communication and outage constraints

`covScore_j(t)`:

- posterior concentration proxy
- currently implemented by inverse mean posterior covariance trace

`linkQuality_j(t)`:

- delivered over delivered plus dropped measurements

`existenceConfidence_j(t)`:

- confidence derived from posterior Bernoulli existence probabilities
- high when `r` is close to `0` or `1`, low when `r` is close to `0.5`
- intended to capture local existence and cardinality decisiveness

`innovationPenalty_j(t)`:

- optional consistency penalty from aggregated NIS statistics

`structurePrior_j(t)`:

- weak local-graph refinement based on neighborhood overlap and communication reliability
- used to slightly reshape the decoupled spatial and existence branches
- not intended to replace posterior-quality factors as the main weight signal

## Existence-Confidence Narrative

This should be the main new method subsection.

Required points:

- Covariance measures spatial concentration, but not whether existence decisions are decisive.
- Link quality measures realized communication success, but not whether the transmitted local posterior is trustworthy in cardinality terms.
- Bernoulli existence probabilities provide a direct signal for how confidently a sensor asserts the presence or absence of targets.
- A weighted confidence score from local existence probabilities adds a complementary dimension to covariance and link quality.

## Weak Structure-Aware Decoupled Narrative

This should be a short refinement subsection after the three-factor design.

Required points:

- The strongest current result does not come from replacing the three-factor score, but from refining it.
- Spatial and existence fusion need not share exactly the same weight dynamics.
- The spatial branch can tolerate a modest graph-aware refinement.
- The existence branch is much more sensitive, so only a very weak structure-aware adjustment is retained.
- Structure should be framed as a light prior layered on top of posterior quality and link quality, not as a topology-only weighting scheme.

## NIS Narrative

This is now a secondary but still useful subsection.

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
- link-quality definition from delivered versus dropped packets
- existence-confidence score definition
- decoupled spatial and existence score definitions
- weak structure-prior modulation
- optional NIS definition and normalized form
- EMA smoothing of the final weights

## Optional Modules

Describe briefly and demote:

- `innovationPenalty`: optional NIS-based consistency term
- `historyScore`: optional temporal-stability term
- `associationScore`: optional ambiguity-aware term
- strong structure-aware priors on the existence branch

Recommended treatment:

- include in the method as optional extensions
- evaluate in ablation
- do not make them part of the claimed core method

## Source Files

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `docs/ADAPTIVE_FUSION_WEIGHTS_CN.md`
- `docs/COMMUNICATION_TIERED_DROP_UPDATE_CN.md`
- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`
