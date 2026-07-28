# Structured rolling-safe oracle-k audit

- Generated: 2026-07-29 04:59:33
- Protocol: `m24-rolling-safe-rollout-v2-projector-replay-f1`
- Dataset SHA-256: `05b06759cadda5738d9ea4b6bef02b4b178aba4c84318d68acc49a0813d5a710`
- Training seeds: `[11 17 19 23 29]`
- Audit seeds: `7`
- Feature context: `graph-context`
- Retained linear features: 136 / 168
- Lambda grid: `[0.001 0.01]`
- Selected lambda: 0.001
- Epochs: 30
- Oracle cardinality used: `1`
- Deployable: `0`
- Edge-ranking ceiling passed: `0`
- Training git commit: `910fe07397f8fba33b4df0c377322e432e143a4a`
- Training worktree dirty: `0`
- Evidence boundary: Oracle-k supplies each recorded target cross-edge count to the exact rolling-B3 projector. The audit isolates edge-ranking learnability and cannot be deployed or used as closed-loop tracking evidence. Hyperparameters use only leave-one-training-seed-out graph imitation; seed 7 remains design-seen audit data.

## Leave-one-training-seed-out selection

| Lambda | Min seed F1 | Mean F1 | Min block F1 | Min seed exact | Mean exact |
|--:|--:|--:|--:|--:|--:|
| 0.001 | 0.0000 | 0.0889 | 0.0000 | 0.0000 | 0.0667 |
| 0.01 | 0.0000 | 0.0889 | 0.0000 | 0.0000 | 0.0667 |

## Training replay

| Seed | Time | Code | Oracle k | Precision | Recall | F1 | Exact graph | Repair | Infeasible |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 11 | 75 | 00 | 3 | 0.3333 | 0.3333 | 0.3333 | 0 | 0 | 0 |
| 11 | 76 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 11 | 77 | 00 | 3 | 0.6667 | 0.6667 | 0.6667 | 0 | 0 | 0 |
| 17 | 75 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 17 | 76 | 24 | 0 | 1.0000 | 1.0000 | 1.0000 | 1 | 0 | 0 |
| 17 | 77 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 19 | 75 | 24 | 1 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 19 | 76 | 00 | 3 | 0.3333 | 0.3333 | 0.3333 | 0 | 0 | 0 |
| 19 | 77 | 00 | 3 | 0.3333 | 0.3333 | 0.3333 | 0 | 0 | 0 |
| 23 | 75 | 90 | 3 | 0.3333 | 0.3333 | 0.3333 | 0 | 0 | 0 |
| 23 | 76 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 23 | 77 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 29 | 75 | 92 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 29 | 76 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 29 | 77 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |

## Design-seen audit replay

| Seed | Time | Code | Oracle k | Precision | Recall | F1 | Exact graph | Repair | Infeasible |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 91 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 7 | 76 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
| 7 | 77 | 00 | 3 | 0.0000 | 0.0000 | 0.0000 | 0 | 0 | 0 |
