# M24 multi-head value-proposal LOSO diagnostic

- Generated: 2026-07-29 13:36:59
- Model: `ridge-multihead-rolling-safe-value-proposal-v1`
- Training commit: `f6d6ae26b0da72c825cf0446776591ad5234ed4a`
- Dataset SHA-256: `cfda289bb32ecf35161786476878210b76a0ccdb3b4dfb4ee719b576e66bea63`
- Heads / top-K: `[0 90 91 92] / 4`
- Exact current cross edges per head: `3`
- Selected supervision: `value-weighted`
- Selected ridge lambda: `1`
- Target-state capture: `0 / 10 (0.000%)`
- Target-graph recall: `0 / 34 (0.000%)`
- Mean best target edge F1: `0.266667`
- Mean distinct proposals per state: `3.300`
- Projection failures: `1`
- Development capture gate (>= 80.0%): `0`
- Online truth-free capture evaluated: `0`
- Critic/X36 authorized: `0 / 0`
- Evidence boundary: The four score heads read only frozen observable edge features and use the exact rolling-B=3 projector. Their teacher graphs, value weights and all state histories are design-seen privileged data. LOSO capture is an architecture diagnostic only. Fresh online truth-free rollouts must pass the same capture gate before critic or X36 experiments.

## Hyperparameter screen

| Supervision | Lambda | State capture | Target recall | Mean best F1 | Distinct | Failures |
|:--|--:|--:|--:|--:|--:|--:|
| value-weighted | 1 | 0.0% | 0.0% | 0.267 | 3.30 | 1 |
| all-privileged | 1 | 0.0% | 0.0% | 0.233 | 3.60 | 0 |
| value-only | 0.01 | 0.0% | 0.0% | 0.233 | 3.90 | 0 |
| all-privileged | 0.0001 | 0.0% | 0.0% | 0.233 | 3.40 | 1 |
| value-weighted | 0.0001 | 0.0% | 0.0% | 0.233 | 3.50 | 1 |
| value-only | 0.0001 | 0.0% | 0.0% | 0.233 | 3.50 | 4 |
| all-privileged | 0.01 | 0.0% | 0.0% | 0.200 | 3.30 | 0 |
| value-weighted | 0.01 | 0.0% | 0.0% | 0.200 | 3.80 | 0 |
| value-only | 1 | 0.0% | 0.0% | 0.200 | 3.30 | 0 |
| value-only | 100 | 0.0% | 0.0% | 0.167 | 2.70 | 0 |
| value-weighted | 100 | 0.0% | 0.0% | 0.167 | 2.60 | 1 |
| all-privileged | 100 | 0.0% | 0.0% | 0.133 | 2.00 | 2 |

## Selected LOSO states

| Seed | Time | Targets | Captured | Captured graphs | Best edge F1 | Distinct proposals | Failures | Matches |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| 7 | 76 | 4 | 0 | 0 | 0.333 | 3 | 0 | `-,-,-,-` |
| 7 | 77 | 3 | 0 | 0 | 0.000 | 4 | 0 | `-,-,-,-` |
| 11 | 75 | 4 | 0 | 0 | 0.667 | 4 | 0 | `-,-,-,-` |
| 11 | 76 | 4 | 0 | 0 | 0.333 | 4 | 0 | `-,-,-,-` |
| 11 | 77 | 1 | 0 | 0 | 0.000 | 2 | 1 | `-,-,-,-` |
| 17 | 75 | 4 | 0 | 0 | 0.667 | 4 | 0 | `-,-,-,-` |
| 17 | 77 | 3 | 0 | 0 | 0.667 | 3 | 0 | `-,-,-,-` |
| 23 | 75 | 4 | 0 | 0 | 0.000 | 4 | 0 | `-,-,-,-` |
| 29 | 75 | 3 | 0 | 0 | 0.000 | 3 | 0 | `-,-,-,-` |
| 29 | 76 | 4 | 0 | 0 | 0.000 | 2 | 0 | `-,-,-,-` |

## Decision

FAIL: the four linear truth-free score heads do not reach the 80% development capture gate. Do not train a critic or run X36; redesign the proposal model first.
