# Related Work

## Suggested Related-Work Buckets

1. Distributed multi-sensor RFS tracking
2. KLA or GCI style fusion for multi-object posteriors
3. Adaptive or reliability-aware fusion weights
4. NIS and innovation-based consistency assessment
5. Communication-constrained sensor fusion

## Positioning Against Prior Work

Recommended differentiation:

- Prior KLA or GCI fusion often uses fixed weights or topology-derived weights.
- Prior adaptive weighting often focuses on generic confidence scores, not on the covariance and NIS coupling issue.
- Prior NIS usage often treats NIS as monotonic quality, while this work reframes it as a consistency penalty.
- This work targets distributed `GA-LMB` realization with communication-aware and consistency-aware weighting.
- This work uses topology only as a weak refinement layered on top of posterior-quality signals, not as the primary weight source.

## What To Cite For Contrast

- KLA or GCI fusion in distributed Bayesian tracking
- LMB or GLMB distributed fusion papers
- Innovation consistency and NIS-based filter health monitoring
- Communication-aware distributed estimation and consensus

## Writing Advice

- Keep this section functional.
- Tie every related-work paragraph to one precise gap that your method addresses.
- Avoid a broad survey that weakens the paper's focus.
