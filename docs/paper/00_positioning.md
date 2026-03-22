# Positioning

## Recommended Scope

Recommended title scope:

- `KLA-based distributed LMB fusion`
- `adaptive fusion-weight allocation`
- `communication-constrained multi-sensor tracking`

Recommended method scope:

- Main algorithm family: `GA-LMB`
- Main mechanism: adaptive fusion weights
- Main technical novelty: factorized `covariance + link quality + existence confidence` weighting, plus a weak structure-aware decoupled refinement
- Secondary modules: decoupled `robust NIS`, `history`, `freshness`, `association ambiguity`

## What The Paper Should Claim

- Fixed KLA weights are brittle when communication quality and local posterior quality vary across sensors.
- A factorized adaptive weighting mechanism can improve distributed consensus quality.
- Existence-confidence weighting adds a missing dimension beyond state precision and communication reliability.
- A weak structure-aware decoupled KLA refinement can further improve consensus without sacrificing cardinality.
- Interpreting NIS as a consistency penalty is still useful, but it is not the strongest current headline improvement.

## What The Paper Should Not Claim

- Do not claim a brand-new multi-sensor filter family.
- Do not claim `history` is a major innovation.
- Do not claim `freshness` is a useful core module with current evidence.
- Do not claim `association ambiguity` is a major innovation with current evidence.
- Do not claim universal local tracking gains for every sensor and every metric.
- Do not make `AA` a co-equal main line with `GA/KLA`.

## Recommended Contributions

Use a 3-point contribution list:

1. An adaptive KLA fusion-weight allocation scheme for distributed GA-LMB fusion under communication constraints.
2. A three-factor quality model that combines posterior covariance, realized link quality, and existence-confidence for communication-constrained distributed fusion.
3. An empirical study under tiered heterogeneous packet-loss conditions showing that a weak structure-aware decoupled KLA refinement further improves consensus OSPA and RMSE beyond the three-factor baseline while preserving cardinality.

## Current Evidence Hierarchy

Most convincing:

- Tiered GA main result in the 4+4 formation scenario
- Tiered ablation `fixed -> +covariance -> +link quality -> +structure-aware decoupled KLA`
- Communication-aware interpretation of tiered packet-loss heterogeneity

Useful but secondary:

- GA `w/o NIS -> robust NIS -> NIS` ablation
- GA NIS parameter grid
- Communication-level robustness analysis
- AA three-wave scenario

Weak or negative:

- `freshness`
- `history`
- `association ambiguity`
- `cardinality consensus`

## Writing Rule

If a paragraph does not clearly serve the `GA-LMB/KLA + adaptive weights + tiered communication + existence confidence + weak structure-aware decoupling + consensus improvement` story, it should likely be shortened or moved to appendix.
