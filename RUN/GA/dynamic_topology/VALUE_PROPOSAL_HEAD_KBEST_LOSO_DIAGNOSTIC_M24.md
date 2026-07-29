# M24 multi-head value-proposal LOSO diagnostic

- Generated: 2026-07-29 13:51:07
- Model: `ridge-multihead-kbest-rolling-safe-value-proposal-v2`
- Training commit: `aa1c567ec1c774ad6c8775d607780365b7310d85`
- Dataset SHA-256: `cfda289bb32ecf35161786476878210b76a0ccdb3b4dfb4ee719b576e66bea63`
- Heads / top-K: `[0 90 91 92] / 16`
- Exact current cross edges per head: `3`
- Selected supervision: `all-privileged`
- Selected ridge lambda: `1`
- Target-state capture: `0 / 10 (0.000%)`
- Target-graph recall: `0 / 34 (0.000%)`
- Mean best target edge F1: `0.300000`
- Mean distinct proposals per state: `5.900`
- Projection failures: `0`
- Development capture gate (>= 80.0%): `0`
- Online truth-free capture evaluated: `0`
- Critic/X36 authorized: `0 / 0`
- Evidence boundary: The four score heads read only frozen observable edge features and every single-shot or exclusion proposal uses the exact rolling-B=3 projector. Their teacher graphs, value weights and all state histories are design-seen privileged data. LOSO capture is an architecture diagnostic only. Fresh online truth-free rollouts must pass the same capture gate before critic or X36 experiments.

## Hyperparameter screen

| Supervision | Lambda | State capture | Target recall | Mean best F1 | Distinct | Failures |
|:--|--:|--:|--:|--:|--:|--:|
| all-privileged | 1 | 0.0% | 0.0% | 0.300 | 5.90 | 0 |
| value-weighted | 1 | 0.0% | 0.0% | 0.300 | 4.80 | 1 |
| all-privileged | 0.01 | 0.0% | 0.0% | 0.267 | 5.20 | 1 |
| all-privileged | 0.0001 | 0.0% | 0.0% | 0.267 | 5.20 | 2 |
| value-only | 0.01 | 0.0% | 0.0% | 0.267 | 5.50 | 2 |
| value-weighted | 0.01 | 0.0% | 0.0% | 0.233 | 5.80 | 0 |
| value-only | 1 | 0.0% | 0.0% | 0.233 | 5.20 | 0 |
| value-weighted | 0.0001 | 0.0% | 0.0% | 0.233 | 5.00 | 1 |
| value-only | 0.0001 | 0.0% | 0.0% | 0.233 | 4.50 | 4 |
| value-weighted | 100 | 0.0% | 0.0% | 0.167 | 3.40 | 1 |
| value-only | 100 | 0.0% | 0.0% | 0.167 | 3.50 | 1 |
| all-privileged | 100 | 0.0% | 0.0% | 0.167 | 2.60 | 2 |

## Selected LOSO states

| Seed | Time | Targets | Captured | Captured graphs | Best edge F1 | Distinct proposals | Failures | Matches |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| 7 | 76 | 4 | 0 | 0 | 0.333 | 7 | 0 | `-,-,-,-,-,-,-` |
| 7 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 | `-,-,-,-` |
| 11 | 75 | 4 | 0 | 0 | 0.667 | 10 | 0 | `-,-,-,-,-,-,-,-,-,-` |
| 11 | 76 | 4 | 0 | 0 | 0.667 | 4 | 0 | `-,-,-,-` |
| 11 | 77 | 1 | 0 | 0 | 0.667 | 8 | 0 | `-,-,-,-,-,-,-,-` |
| 17 | 75 | 4 | 0 | 0 | 0.333 | 6 | 0 | `-,-,-,-,-,-` |
| 17 | 77 | 3 | 0 | 0 | 0.333 | 3 | 0 | `-,-,-` |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 8 | 0 | `-,-,-,-,-,-,-,-` |
| 29 | 75 | 3 | 0 | 0 | 0.000 | 5 | 0 | `-,-,-,-,-` |
| 29 | 76 | 4 | 0 | 0 | 0.000 | 4 | 0 | `-,-,-,-` |

## Decision

FAIL: the four linear truth-free score heads do not reach the 80% development capture gate. Do not train a critic or run X36; redesign the proposal model first.
