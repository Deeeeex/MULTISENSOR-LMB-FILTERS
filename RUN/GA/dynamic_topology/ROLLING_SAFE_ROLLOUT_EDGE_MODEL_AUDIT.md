# Rolling-safe rollout edge-model audit

- Generated: 2026-07-29 04:10:49
- Protocol: `m24-rolling-safe-rollout-v2-projector-replay-f1`
- Dataset generation commit: `18f09ac2a7788854e5fafef6e45bf37db2bf05d9`
- Payload tolerance fraction: `inf`
- Filter RNG offset: `100000`
- Continuation-cache set SHA-256: `44a226757dc1d8436e314e89e68813dc63435323423718cfc6ec9b20b92719ef`
- Training seeds: `[11 17 19 23 29]`
- Audit seeds: `7`
- Registered design seeds: `[7 11 17 19 23 27 29 31 37]`
- Retained features: 136 / 168
- Dataset feature context: `raw`
- Model feature context: `graph-context`
- Hyperparameter grid: hidden `[8 16]`, lambda `0.01`
- Grid boundary: The compact width/lambda grid was narrowed using the rejected v1 development audit; no held-out validation seed selected it.
- Selected hidden width: 16
- Selected lambda: 0.01
- Accepted for closed-loop screen: `0`
- Artifact ID: `b3a3d4e3295cd54cf80c4a3134a5635132669c8041efe4ce4c2a63ded6c3de24`
- Dataset SHA-256: `05b06759cadda5738d9ea4b6bef02b4b178aba4c84318d68acc49a0813d5a710`
- Feature-contract SHA-256: `6b1606e6c1ddcf39e0b2dde6f89cc2c59e0b1bfdf8c0e006262eeaef781322ce`
- Training-config SHA-256: `078bea1693c179e9ec4e9a9937b618b0543434d0c842340b8b54dbe7b70756bf`
- Training git commit: `8495b33207444a55300351e6478a7582985e5524`
- Training worktree dirty: `0`
- Rejection reasons: `minimum held-out-seed projected edge F1 | mean held-out projected edge F1 | minimum held-out block projected edge F1 | held-out projected edge recall | held-out projected exact-graph fraction | zero-edge exact imitation | single-edge exact imitation`
- Acceptance thresholds: min-seed projected F1 0.3333, mean projected F1 0.5000, min-block F1 0.6667, min-seed exact 0.3333, mean exact 0.3333
- Evidence boundary: The model is selected only by leave-one-training-seed-out imitation of privileged rollout graphs. Audit-seed labels do not select hyperparameters. Joint-projector replay uses the recorded teacher-forced state and history in each block; neither split is on-policy closed-loop validation.

## Leave-one-training-seed-out selection

| Hidden | Lambda | Min projected F1 | Mean projected F1 | Min block F1 | Zero exact | Single exact | Min projected recall | Mean projected recall | Min exact graph | Mean exact graph | Min raw recall | Mean raw recall |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 16 | 0.01 | 0.0000 | 0.0222 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0222 | 0.0000 | 0.0000 | 0.0000 | 0.1111 |
| 8 | 0.01 | 0.0000 | 0.0222 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0222 | 0.0000 | 0.0000 | 0.0000 | 0.0667 |

## Training imitation

| Seed | Time | Code | Projected recall | Projected F1 | Exact graph | Raw recall | Raw exact | Receiver top-1 | Mean positive rank | Margin |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 11 | 75 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | 0.2650 |
| 11 | 76 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | 2.6347 |
| 11 | 77 | 00 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 1.0000 | 1.0000 | -0.2861 |
| 17 | 75 | 00 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 0.6667 | 1.6667 | -1.7102 |
| 17 | 76 | 24 | 0.0000 | 0.0000 | 0 | 1.0000 | 1 | 1.0000 | 0.0000 | -2.3217 |
| 17 | 77 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | 1.2147 |
| 19 | 75 | 24 | 1.0000 | 0.5000 | 0 | 1.0000 | 1 | 1.0000 | 1.0000 | 1.4930 |
| 19 | 76 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | -1.1919 |
| 19 | 77 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | 0.1820 |
| 23 | 75 | 90 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 1.0000 | 1.0000 | -0.2645 |
| 23 | 76 | 00 | 1.0000 | 1.0000 | 1 | 0.6667 | 0 | 1.0000 | 1.0000 | -0.0801 |
| 23 | 77 | 00 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 1.0000 | 1.0000 | -0.4975 |
| 29 | 75 | 92 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 0.6667 | 1.3333 | -0.6682 |
| 29 | 76 | 00 | 1.0000 | 1.0000 | 1 | 1.0000 | 1 | 1.0000 | 1.0000 | 0.1181 |
| 29 | 77 | 00 | 0.6667 | 0.6667 | 0 | 0.6667 | 0 | 1.0000 | 1.0000 | -0.4784 |

## Audit-seed imitation

| Seed | Time | Code | Projected recall | Projected F1 | Exact graph | Raw recall | Raw exact | Receiver top-1 | Mean positive rank | Margin |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 75 | 91 | 0.0000 | 0.0000 | 0 | 0.0000 | 0 | 0.3333 | 2.6667 | -4.0158 |
| 7 | 76 | 00 | 0.0000 | 0.0000 | 0 | 0.0000 | 0 | 0.3333 | 3.3333 | -5.8638 |
| 7 | 77 | 00 | 0.0000 | 0.0000 | 0 | 0.0000 | 0 | 0.3333 | 4.0000 | -4.0807 |
