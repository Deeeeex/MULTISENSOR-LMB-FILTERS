# V33 safe gateway trust-backoff preflight

- Contract: `formation-safe-gateway-trust-backoff-v33-preflight-v1`
- Generation commit: `ee7503e877f41282ab585c991a14339113f93346`
- Tracked worktree dirty / untracked source: `0 / 0`
- Preset / seed / reconnect time: `m24-formation-fov / 211 / 74`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Frozen v32 candidate bank SHA-256: `d5a53c158fa368a7fcb913e8847c359489c938cec8979191a228e5fbb00221d8`
- Promising v32 candidates: `[8 10 12]`
- Trust grid: `[0.0125 0.025 0.0375]`
- Safe / eligible route-weight pairs: `0 / 0`
- Selected source candidate / weight: `NaN / NaN`
- Selected action: `reference`
- Predicted disagreement improvement: `+0.0000%`
- Tracking outcome scored / rerun authorized: `0 / 0`
- Replay / control construction: `134.41 / 53.80 s`

## t=74 trust-backoff diagnostics

| V32 candidate | Changed receivers | Weight | Disagreement improvement | Retention risk | Min form. d-card | Min label retention | Threshold drops | Safe | Eligible |
|--:|:--|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 8 | `[2 10]` | 0.0125 | -1.9858% | 0.043301 | +0.000000 | 0.026950 | 2 | 0 | 0 |
| 8 | `[2 10]` | 0.0250 | -0.5916% | 0.095588 | +0.000000 | 0.026951 | 4 | 0 | 0 |
| 8 | `[2 10]` | 0.0375 | +0.3029% | 0.134172 | -0.015230 | 0.026952 | 6 | 0 | 0 |
| 10 | `[2 9]` | 0.0125 | -2.0858% | 0.038801 | +0.000000 | 0.027348 | 1 | 0 | 0 |
| 10 | `[2 9]` | 0.0250 | -0.6503% | 0.089104 | +0.000000 | 0.027349 | 4 | 0 | 0 |
| 10 | `[2 9]` | 0.0375 | +0.2983% | 0.126413 | +0.000000 | 0.027350 | 5 | 0 | 0 |
| 12 | `[2 11]` | 0.0125 | -2.0501% | 0.038037 | +0.000000 | 0.025702 | 1 | 0 | 0 |
| 12 | `[2 11]` | 0.0250 | -0.9579% | 0.078656 | +0.000000 | 0.025703 | 3 | 0 | 0 |
| 12 | `[2 11]` | 0.0375 | +0.3133% | 0.124864 | +0.000000 | 0.025704 | 5 | 0 | 0 |

## Decision

No reduced-trust alternative gateway is both label-safe and above the registered disagreement-improvement margin. Stop before rerunning tracking.

## Evidence boundary

v33 reuses only the frozen v32 radius-one gateway bank and the already-opened v30 causal t=74 state. Structural candidates are preselected only when their frozen weight-0.05 one-round disagreement improvement is at least 0.25 percent. For those routes, only the two changed cross inputs are evaluated at the preregistered lower weights 0.0125, 0.025, and 0.0375; all other edges, weights, messages, and the formation-level cycle remain fixed. Exact current-state label retention and disagreement may select a route, but target truth and later outcomes may not be read. A paired tracking rerun is authorized only after a clean preflight finds a nonreference route that passes every safety, parity, rolling-B3, and disagreement gate. Other M24 states, GNN, X36, X48, and validation remain closed.
