# M24 joint-baseline counterfactual-teacher audit: seed 7

- Protocol: `m24-joint-baseline-counterfactual-teacher-seed7-v1`
- Experiment commit: `35496bc`
- Evidence split: `development`
- Design-seen seed: `7`
- Audit git commit: `4fc158b7877774bcb756e8be3a72acdd319392e7`
- Audit tracked worktree dirty: `0`
- Audit untracked source dirty: `0`
- Scope: truth-free centralized offline teacher; requires all current full posteriors and is excluded from primary and held-out evidence.

## Registered reference and gates

- Code-24 E-OSPA: `22.3449`
- Code-24 worst-node E-OSPA: `34.5380`
- Code-24 posterior disagreement: `0.9525`
- Code-24 attempted bytes: `3472848`
- Joint gate: at least 5% mean tracking gain, no worst-node regression, at most 2% attempted-byte mismatch, complete selected rolling-B3 safety, no repair, emergency, truth use or infeasibility.

## Complete candidate table

| Code | Objective | k | E-OSPA | Mean gain | Worst node | Tail gain | Posterior disagr. | Posterior gain | Byte change | Policy s | Pass |
|--:|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 40 | `mean` | 1 | 22.9468 | -2.69% | 34.5859 | -0.14% | 0.9488 | +0.39% | -0.44% | 63.44 | 0 |
| 41 | `mean` | 2 | 24.5328 | -9.79% | 34.5380 | +0.00% | 0.9158 | +3.85% | -0.87% | 63.38 | 0 |
| 42 | `mean` | 3 | 26.5984 | -19.04% | 44.4231 | -28.62% | 0.8756 | +8.08% | -1.26% | 75.45 | 0 |
| 43 | `top-fraction` | 1 | 24.2297 | -8.43% | 44.4508 | -28.70% | 0.9149 | +3.95% | +0.22% | 75.81 | 0 |
| 44 | `top-fraction` | 2 | 25.6842 | -14.94% | 45.0244 | -30.36% | 0.8872 | +6.85% | -0.16% | 74.27 | 0 |
| 45 | `top-fraction` | 3 | 27.9337 | -25.01% | 44.4508 | -28.70% | 0.8526 | +10.49% | -0.62% | 73.57 | 0 |

## Decision

- Posterior-disagreement improvements: `6 / 6`
- Joint tracking/tail passes: `0 / 6`

All six actions fail the tracking/tail gate despite communication matching. The least harmful mean-k1 action lowers posterior disagreement but worsens E-OSPA, directly falsifying one-step posterior consensus as a sufficient tracking surrogate for this M24 checkpoint. Larger k and the global top-fraction objective worsen tracking further. Retain this implementation only as a negative offline diagnostic; do not train a deployable scorer from these labels.

## Source summaries

- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_postfusion_b01of02/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_063121.mat`
- `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_postfusion_b02of02/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_063125.mat`
