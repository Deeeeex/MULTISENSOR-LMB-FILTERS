# V32 label-compatible gateway preflight

- Contract: `formation-label-compatible-gateway-v32-preflight-v1`
- Generation commit: `90a3d2ca499b2fa0cd834a55cb3095a6cf3479c5`
- Tracked worktree dirty: `0`
- Untracked source files: `0`
- Preset / seed / reconnect time: `m24-formation-fov / 211 / 74`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Candidate bank SHA-256: `d5a53c158fa368a7fcb913e8847c359489c938cec8979191a228e5fbb00221d8`
- Source trajectory replay matched frozen v30: `1`
- Tracking outcome scored: `0`
- Proposed / safe / eligible routes: `12 / 0 / 0`
- Selected candidate / action: `NaN / reference`
- Predicted disagreement improvement: `+0.0000%`
- Message / formation graph / residual weight parity: `1 / 1 / 1`
- Tracking rerun authorized: `0`
- Replay / control construction: `137.39 / 81.01 s`

## t=74 alternative-gateway diagnostics

| Candidate | Proposal rank | Cut changes | Cut receivers | Cross senders | Cross receivers | Proposal score | Disagreement improvement | Retention risk | Min form. d-card | Min label retention | Threshold drops | Safe | Eligible |
|--:|--:|--:|:--|:--|:--|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 1 | 1 | 1 | `[2 20 17 8]` | `[3 21 18 9]` | `[20 17 8 2]` | 1.134401 | -0.0276% | 0.048055 | -0.055628 | 0.057340 | 2 | 0 | 0 |
| 2 | 2 | 1 | `[2 20 16 8]` | `[3 21 17 9]` | `[20 16 8 2]` | 1.132711 | +0.0749% | 0.034822 | -0.076252 | 0.060395 | 1 | 0 | 0 |
| 3 | 4 | 1 | `[2 20 15 8]` | `[3 21 16 9]` | `[20 15 8 2]` | 1.108266 | -0.0454% | 0.033604 | -0.014514 | 0.060995 | 1 | 0 | 0 |
| 4 | 5 | 1 | `[2 22 14 8]` | `[3 23 15 9]` | `[22 14 8 2]` | 1.107927 | +0.0763% | 0.106720 | -0.061851 | 0.025482 | 4 | 0 | 0 |
| 5 | 6 | 1 | `[2 21 14 8]` | `[3 22 15 9]` | `[21 14 8 2]` | 1.106603 | +0.1127% | 0.106595 | -0.063235 | 0.025613 | 5 | 0 | 0 |
| 6 | 7 | 1 | `[5 20 14 8]` | `[6 21 15 9]` | `[20 14 8 5]` | 1.085314 | +0.0196% | 0.016046 | -0.005212 | 0.292610 | 0 | 0 | 0 |
| 7 | 8 | 1 | `[3 20 14 8]` | `[4 21 15 9]` | `[20 14 8 3]` | 1.074792 | -0.0635% | 0.016281 | -0.007877 | 0.294238 | 0 | 0 | 0 |
| 8 | 9 | 1 | `[2 20 14 10]` | `[3 21 15 11]` | `[20 14 10 2]` | 1.071148 | +0.5471% | 0.149490 | -0.124124 | 0.026952 | 6 | 0 | 0 |
| 9 | 10 | 1 | `[4 20 14 8]` | `[5 21 15 9]` | `[20 14 8 4]` | 1.069174 | -0.1326% | 0.015466 | -0.001942 | 0.293917 | 0 | 0 | 0 |
| 10 | 11 | 1 | `[2 20 14 9]` | `[3 21 15 10]` | `[20 14 9 2]` | 1.065021 | +0.6280% | 0.141676 | -0.074138 | 0.027351 | 6 | 0 | 0 |
| 11 | 12 | 1 | `[2 23 14 8]` | `[3 24 15 9]` | `[23 14 8 2]` | 1.057232 | +0.0414% | 0.104758 | -0.023927 | 0.050985 | 4 | 0 | 0 |
| 12 | 13 | 1 | `[2 20 14 11]` | `[3 21 15 12]` | `[20 14 11 2]` | 1.053282 | +0.6459% | 0.142594 | -0.081201 | 0.025704 | 6 | 0 | 0 |

## Decision

No alternative gateway cycle is both label-safe and predicted to reduce one-round disagreement by the registered margin. Stop before rerunning the tracking outcome.

## Evidence boundary

v32 may replay only the already-opened v30 controller trajectory through t=74. It keeps the registered formation-level cycle, dominant route, residual weight, and directed message count fixed, and ranks at most sixteen reference-anchored sensor-gateway realizations with at most one formation cut change, using only current label-wise KLA compatibility and current link reliability. Exact one-round disagreement and reference-relative existence retention may project this proposal bank, but target truth and later outcomes may not be read. A tracking rerun is permitted only after a clean frozen preflight selects a nonreference route that passes every safety, parity, rolling-B3, and 0.25-percent disagreement-improvement gate. No other M24 state, GNN, X36, or X48 is opened by this probe.
