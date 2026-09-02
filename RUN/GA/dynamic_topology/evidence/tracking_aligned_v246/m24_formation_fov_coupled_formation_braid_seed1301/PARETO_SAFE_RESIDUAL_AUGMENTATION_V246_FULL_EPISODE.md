# V246 Pareto-safe residual augmentation

- Preset: `m24-formation-fov-coupled-formation-braid`
- Seed: `1301`
- Generation commit: `6e886023f4cd8a29c8dae3d577e3dce34d95ef04`
- Balanced direction over fixed passed: `1`
- Incremental direction over V242 passed: `1`
- Paper threshold passed: `0`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| Fixed formation tree | 126.724 | 9.052 | 135.624 | 34363568 | 46--48 |
| Full causal repair | 125.394 | 8.947 | 133.109 | 38421672 | 48--48 |
| Minimum causal backbone | 125.991 | 9.579 | 132.815 | 26465520 | 30--30 |
| Pareto-safe residual augmentation | 125.964 | 8.984 | 128.124 | 28025384 | 30--32 |

## V246 over fixed tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.600%` |
| Full RMSE | `+0.745%` |
| Focus consistency | `+5.530%` |
| Attempted-byte saving | `+18.444%` |
| Weakest formation E-OSPA | `-2.304%` |
| Weakest formation RMSE | `-7.322%` |

## V246 over V242 minimum backbone

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.021%` |
| Full RMSE | `+6.210%` |
| Focus consistency | `+3.532%` |
| Attempted-byte saving | `-5.894%` |
| Weakest formation E-OSPA | `-2.653%` |
| Weakest formation RMSE | `-2.890%` |

## Runtime selection

- Pages with at least one residual: `0.931`
- Mean / maximum selected residuals: `1.562 / 2`
- Mean / maximum admissible candidates: `2.300 / 6`
- Mean / maximum policy selection seconds: `22.124 / 28.977`
- Current-step Pareto guard passed on every page: `1`

## Evidence boundary

V246 augments the causal V242 minimum formation backbone only with currently physical local residual inputs omitted from the matched V240 route. It uses current local posteriors and current link reliabilities to enumerate the registered one-round mixture-aware LMB-KLA delivery outcomes. The final route must not increase the current-step task-risk, network-disagreement or per-formation mean-tail risk proxies relative to V242, and it must satisfy robust existence-retention, message and estimated-byte bounds. These are current-step surrogate guarantees, not truth-level or recursive tracking guarantees. Truth, future measurements, realized future delivery outcomes and tracking scores are not read online.
