# Positioning

## Recommended Scope

Recommended title scope:

- `KLA-based distributed LMB fusion`
- `adaptive fusion-weight allocation`
- `communication-constrained multi-sensor tracking`

Recommended method scope:

- Main algorithm family: `GA-LMB`
- Main mechanism: adaptive fusion weights
- Main technical novelty: decoupled `robust NIS` consistency penalty
- Secondary modules: `history`, `association ambiguity`

## What The Paper Should Claim

- Fixed KLA weights are brittle when communication quality and local posterior quality vary across sensors.
- A factorized adaptive weighting mechanism can improve distributed consensus quality.
- Interpreting NIS as a consistency penalty is better than treating it as a monotonic quality reward.
- Robust NIS aggregation is more stable than plain mean-based NIS in the current GA setting.

## What The Paper Should Not Claim

- Do not claim a brand-new multi-sensor filter family.
- Do not claim `history` is a major innovation.
- Do not claim `association ambiguity` is a major innovation with current evidence.
- Do not claim universal local tracking gains for every sensor and every metric.
- Do not make `AA` a co-equal main line with `GA/KLA`.

## Recommended Contributions

Use a 3-point contribution list:

1. An adaptive KLA fusion-weight allocation scheme for distributed GA-LMB fusion under communication constraints.
2. A decoupled robust NIS design that converts NIS from a quality reward into a chi-square consistency penalty.
3. An empirical study showing substantial consensus gains in GA formation scenarios and improved stability over plain NIS.

## Current Evidence Hierarchy

Most convincing:

- GA main result in the 4+4 formation scenario
- GA `w/o NIS -> robust NIS -> NIS` ablation
- GA NIS parameter grid

Useful but secondary:

- Communication-level robustness analysis
- AA three-wave scenario

Weak or negative:

- `history`
- `association ambiguity`

## Writing Rule

If a paragraph does not clearly serve the `GA-LMB/KLA + adaptive weights + robust NIS + consensus improvement` story, it should likely be shortened or moved to appendix.
