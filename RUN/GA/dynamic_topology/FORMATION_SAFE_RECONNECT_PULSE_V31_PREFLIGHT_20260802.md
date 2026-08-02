# V31 safe reconnect-pulse preflight

- Contract: `formation-safe-reconnect-pulse-v31-preflight-v1`
- Generation commit: `89579bcbe15f3c69273f218c4cf100775ff128a0`
- Tracked worktree dirty: `0`
- Untracked source files: `0`
- Preset / seed / reconnect time: `m24-formation-fov / 211 / 74`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Source trajectory replay matched frozen v30: `1`
- Tracking outcome scored: `0`
- Safe / eligible single pulses: `2 / 0`
- Requested / selected formations: `[] / []`
- Selected action: `reference`
- Predicted disagreement improvement: `+0.0000%`
- Tracking rerun authorized: `0`
- Replay / control construction: `137.44 / 78.65 s`

## t=74 pulse diagnostics

| Formation | Pulse weight | Disagreement improvement | Retention risk | Min form. d-card | Min label retention | Threshold drops | Safe | Eligible |
|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 1 | 0.075 | +0.0422% | 0.001964 | -0.013243 | 0.948947 | 0 | 1 | 0 |
| 1 | 0.100 | +0.1099% | 0.005803 | -0.038871 | 0.837826 | 0 | 1 | 0 |
| 1 | 0.150 | +0.2157% | 0.012252 | -0.081910 | 0.731085 | 0 | 0 | 0 |
| 2 | 0.075 | +0.2554% | 0.029722 | -0.124720 | 0.356796 | 0 | 0 | 0 |
| 2 | 0.100 | +0.4797% | 0.062127 | -0.260694 | 0.279448 | 1 | 0 | 0 |
| 2 | 0.150 | +0.9336% | 0.120756 | -0.506710 | 0.090925 | 3 | 0 | 0 |
| 3 | 0.075 | +0.2100% | 0.012722 | -0.079484 | 0.174940 | 0 | 0 | 0 |
| 3 | 0.100 | +0.3117% | 0.017602 | -0.109976 | 0.134168 | 0 | 0 | 0 |
| 3 | 0.150 | +0.6006% | 0.029322 | -0.183204 | 0.134175 | 1 | 0 | 0 |
| 4 | 0.075 | +0.1762% | 0.014845 | -0.068137 | 0.262635 | 0 | 0 | 0 |
| 4 | 0.100 | +0.4550% | 0.034194 | -0.156945 | 0.262497 | 1 | 0 | 0 |
| 4 | 0.150 | +0.7399% | 0.055060 | -0.252717 | 0.071962 | 1 | 0 | 0 |

## Decision

No nonreference pulse is both label-safe and predicted to reduce one-round disagreement by the registered margin. Stop before rerunning the t=72 tracking outcome.

## Evidence boundary

v31 may replay only the already-opened v30 controller trajectory through t=74 to recover the causal pre-fusion posterior and selected topology history. The reconnect-pulse bank reads that current posterior, current physical graph, current link reliability, and history; it reads no target truth or later outcome. The probe may measure exact one-round posterior disagreement and reference-relative label retention, but it may not score tracking, rerun the t=72 outcome, open t=60 or another M24 state, train a GNN, or open X36/X48 before a clean frozen preflight explicitly authorizes it.
