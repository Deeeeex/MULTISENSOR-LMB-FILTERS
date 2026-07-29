# Expanded M24 proposal-head LOSO diagnostic

- Generated: 2026-07-29 15:35:55
- Model contract: `rolling-safe-expanded-proposal-head-model-m24-v1`
- Training commit: `a7c83d188c49916195a6b6ed6da4708ee3c569f0`
- Dataset SHA-256: `ce815847e6e540e379934e2f09bbdb4f461cc2b4c7dc4ac918f35ff56feccf06`
- Selected family/context: `mlp / graph-context`
- Selected hidden width / lambda: `64 / 1e-05`
- Evaluation top-K: `16`
- Expanded target-state capture: `2 / 54 (3.704%)`
- Minimum expanded seed capture: `0.000%`
- Expanded target-graph recall: `2 / 210 (0.952%)`
- Expanded mean best edge F1: `0.197531`
- Legacy value-state capture: `1 / 10 (10.000%)`
- Legacy value-graph recall: `1 / 34 (2.941%)`
- Legacy value mean best edge F1: `0.300000`
- Expanded / legacy capture gate: `0 / 0`
- H=3 proposal-return generation authorized: `0`
- Critic/X36 authorized: `0 / 0`
- Evidence boundary: Hyperparameters use only LOSO current-truth graph imitation on states collected by a frozen truth-free behavior policy. The older value-filtered states are an external architecture diagnostic and do not select the model. Passing authorizes only H=3 proposal-return generation on the expanded M24 development states, never critic training or X36.

## Single-shot selection screen

| Family | Context | Hidden | Lambda | State | Min seed | Graph recall | Best F1 | Distinct | Failures |
|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| mlp | graph-context | 64 | 1e-05 | 3.7% | 0.0% | 1.0% | 0.198 | 3.46 | 4 |
| ridge | graph-context | 0 | 0.01 | 1.9% | 0.0% | 0.5% | 0.259 | 3.48 | 4 |
| ridge | graph-context | 0 | 100 | 1.9% | 0.0% | 0.5% | 0.235 | 2.67 | 5 |
| mlp | graph-context | 64 | 0.001 | 1.9% | 0.0% | 0.5% | 0.228 | 3.06 | 7 |
| ridge | graph-context | 0 | 0.0001 | 1.9% | 0.0% | 0.5% | 0.216 | 3.46 | 3 |
| ridge | graph-context | 0 | 1 | 0.0% | 0.0% | 0.0% | 0.253 | 3.41 | 4 |
| mlp | graph-context | 32 | 0.001 | 0.0% | 0.0% | 0.0% | 0.210 | 2.93 | 3 |
| ridge | raw | 0 | 0.0001 | 0.0% | 0.0% | 0.0% | 0.198 | 3.19 | 0 |
| ridge | raw | 0 | 1 | 0.0% | 0.0% | 0.0% | 0.191 | 3.13 | 2 |
| mlp | graph-context | 32 | 1e-05 | 0.0% | 0.0% | 0.0% | 0.185 | 3.28 | 4 |
| ridge | raw | 0 | 0.01 | 0.0% | 0.0% | 0.0% | 0.179 | 2.98 | 2 |
| ridge | raw | 0 | 100 | 0.0% | 0.0% | 0.0% | 0.148 | 2.43 | 2 |
| mlp | graph-context | 16 | 0.001 | 0.0% | 0.0% | 0.0% | 0.136 | 2.89 | 2 |
| mlp | graph-context | 16 | 1e-05 | 0.0% | 0.0% | 0.0% | 0.123 | 2.80 | 4 |

## Selected top-16 expanded states

| Seed | Time | Targets | Captured | Graphs | Best F1 | Distinct | Failures |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 7 | 77 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 7 | 78 | 4 | 0 | 0 | 0.667 | 3 | 0 |
| 7 | 79 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 7 | 80 | 4 | 0 | 0 | 0.000 | 2 | 0 |
| 7 | 81 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 7 | 82 | 4 | 0 | 0 | 0.333 | 3 | 0 |
| 7 | 83 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 11 | 75 | 4 | 0 | 0 | 0.667 | 3 | 0 |
| 11 | 76 | 4 | 0 | 0 | 0.667 | 4 | 0 |
| 11 | 77 | 4 | 1 | 1 | 1.000 | 5 | 0 |
| 11 | 78 | 4 | 0 | 0 | 0.333 | 5 | 2 |
| 11 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 80 | 4 | 0 | 0 | 0.333 | 3 | 0 |
| 11 | 81 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 83 | 4 | 0 | 0 | 0.000 | 3 | 0 |
| 17 | 75 | 4 | 1 | 1 | 1.000 | 4 | 0 |
| 17 | 76 | 4 | 0 | 0 | 0.000 | 3 | 0 |
| 17 | 77 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 78 | 4 | 0 | 0 | 0.000 | 3 | 0 |
| 17 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 17 | 80 | 3 | 0 | 0 | 0.333 | 6 | 0 |
| 17 | 81 | 4 | 0 | 0 | 0.000 | 3 | 1 |
| 17 | 82 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 17 | 83 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 19 | 75 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 19 | 76 | 4 | 0 | 0 | 0.333 | 7 | 0 |
| 19 | 77 | 4 | 0 | 0 | 0.000 | 2 | 0 |
| 19 | 78 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 19 | 79 | 4 | 0 | 0 | 0.333 | 3 | 0 |
| 19 | 80 | 4 | 0 | 0 | 0.333 | 5 | 0 |
| 19 | 81 | 4 | 0 | 0 | 0.000 | 5 | 0 |
| 19 | 82 | 4 | 0 | 0 | 0.000 | 2 | 0 |
| 19 | 83 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 23 | 76 | 3 | 0 | 0 | 0.000 | 3 | 1 |
| 23 | 77 | 4 | 0 | 0 | 0.000 | 3 | 0 |
| 23 | 78 | 3 | 0 | 0 | 0.000 | 6 | 0 |
| 23 | 79 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 80 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 81 | 4 | 0 | 0 | 0.000 | 2 | 0 |
| 23 | 82 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 83 | 4 | 0 | 0 | 0.667 | 4 | 0 |
| 29 | 75 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 76 | 4 | 0 | 0 | 0.000 | 3 | 1 |
| 29 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 78 | 4 | 0 | 0 | 0.000 | 5 | 0 |
| 29 | 79 | 4 | 0 | 0 | 0.000 | 3 | 0 |
| 29 | 80 | 3 | 0 | 0 | 0.000 | 3 | 0 |
| 29 | 81 | 4 | 0 | 0 | 0.000 | 2 | 0 |
| 29 | 82 | 4 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 83 | 4 | 0 | 0 | 0.333 | 8 | 0 |

## Selected top-16 legacy value states

| Seed | Time | Targets | Captured | Graphs | Best F1 | Distinct | Failures |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 76 | 4 | 0 | 0 | 0.333 | 3 | 0 |
| 7 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 11 | 75 | 4 | 0 | 0 | 0.667 | 3 | 0 |
| 11 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 |
| 11 | 77 | 1 | 0 | 0 | 0.000 | 3 | 1 |
| 17 | 75 | 4 | 1 | 1 | 1.000 | 4 | 0 |
| 17 | 77 | 3 | 0 | 0 | 0.333 | 4 | 0 |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 6 | 0 |
| 29 | 75 | 3 | 0 | 0 | 0.000 | 4 | 0 |
| 29 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 |

## Decision

FAIL: redesign the proposal scorer before H=3 return generation. Critic training and X36 remain blocked.
