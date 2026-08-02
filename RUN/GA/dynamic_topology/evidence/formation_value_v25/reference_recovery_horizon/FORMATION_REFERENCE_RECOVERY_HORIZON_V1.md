# Formation reference-recovery horizon probe

- Contract / protocol: `formation-reference-recovery-horizon-probe-result-v1 / formation-reference-recovery-horizon-v1`
- Generation commit: `27b2c879c634a4e32705c53cf413caa7df9fb6d1`
- Cache protocol / commit: `formation-h3-event-conditioned-sentinel-v1 / c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Active prefix / horizons: `H=3 / [4 5]`
- Strong safe recovery found: `0`
- Broader teacher collection authorized: `0`
- GNN training authorized: `0`

## Frozen active prefixes

| Candidate | Role | Three-step mode sequence | H=3 targets |
|--:|:--|:--|:--|
| 1 | `v21-positive-tails-lowest-consensus-debt` | `[1 1 3 1] -> [1 1 1 4] -> [1 4 4 2]` | `[5.774941 0 0.04212573 -1.527435 1.027062 1.073903]` |
| 2 | `v24-balanced-high-return` | `[1 4 3 1] -> [1 1 1 4] -> [1 1 4 2]` | `[5.752299 0 0.1002752 -1.704093 1.809861 1.011454]` |
| 3 | `v24-maximum-mean-headroom` | `[1 4 3 1] -> [1 1 1 4] -> [1 1 1 2]` | `[8.648597 0 0.1002752 -3.6817 1.020828 0.1864362]` |
| 4 | `v24-low-return-boundary` | `[1 1 2 2] -> [1 4 1 1] -> [1 1 4 1]` | `[3.891414 0 -0.002126497 -1.675203 0.1614886 0.1688536]` |

## Equal-horizon results

| H | Candidate | Mean | Min. formation | Worst sensor | Window cons. | Attempted | Delivered | Final-step cons. | Strict | Strong |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 4 | 1 | +6.728% | +0.000% | +0.055% | -3.603% | -0.422% | -0.440% | -10.094% | 0 | 0 |
| 4 | 2 | +7.128% | +0.000% | +0.082% | -1.883% | +0.224% | -0.424% | -2.444% | 0 | 0 |
| 4 | 3 | +9.293% | +0.000% | -1.635% | -3.158% | -0.339% | -1.009% | -1.519% | 0 | 0 |
| 4 | 4 | +4.258% | +0.000% | -0.005% | -2.325% | +0.040% | +0.042% | -4.357% | 0 | 0 |
| 5 | 1 | +9.091% | +0.000% | +1.400% | -5.038% | -0.758% | -0.801% | -11.147% | 0 | 0 |
| 5 | 2 | +9.735% | +0.000% | +1.400% | -3.282% | -0.079% | -0.624% | -9.238% | 0 | 0 |
| 5 | 3 | +11.289% | +0.000% | +1.400% | -3.810% | -0.835% | -1.400% | -6.589% | 0 | 0 |
| 5 | 4 | +4.740% | +0.000% | -1.619% | -2.800% | -0.128% | -0.157% | -4.824% | 0 | 0 |

## Horizon summary

| H | Strict | Strong | Prefix reproduction max error | Closest candidate | Closest targets |
|--:|--:|--:|--:|--:|:--|
| 4 | 0/4 | 0/4 | 3.55e-15 | 2 | `[7.128027 0 0.08204159 -1.883399 0.2236136 -0.4240176 -2.444057]` |
| 5 | 0/4 | 0/4 | 3.55e-15 | 3 | `[11.28887 0 1.399529 -3.810253 -0.8347933 -1.399742 -6.588808]` |

## Metric meaning

The first six signed targets preserve the v21-v24 ordering: mean tracking, minimum formation, worst sensor, consensus averaged over the equal-length horizon, attempted bytes, and delivered bytes. The seventh target compares consensus at the final step only. Positive values are improvements over the all-reference trajectory of the same horizon. Strict feasibility requires all seven values to be nonnegative; strong safety additionally requires at least 3% mean tracking gain.

## Evidence boundary

The four active H=3 prefixes are selected after inspecting v21-v24 and frozen before any H=4/H=5 outcome. Each prefix is followed only by registered reference actions and compared with an all-reference trajectory of equal horizon. Both window-average and final-step consensus are offline targets. This privileged opened-state probe cannot support M24, X36, learned-model, or final-seed claims.
