# V167 confidence-gated one-hop action-value gate

- Model: `confidence-gated-one-hop-label-action-value-v167`
- Ridge lambda / maximum edits: `0.01 / 1`
- Selected-action correction: `1.204517`
- Heldout gate passed: `0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Independent capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 0/0 | 0/0 | +0.0000 / +0.0000 | 0.000 |
| Heldout F5 t=79 | 0/0 | 0/0 | +0.0000 / +0.0000 | 0.000 |

Uncalibrated diagnostic (same frozen ridge/K; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Independent capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 5/5 | +39.9412 / +141.6394 | 0.783 |
| Heldout raw | 6/6 | 6/6 | +41.2476 / -9.4396 | -0.943 |

## Candidate configurations

| Lambda | K | Correction | Cal actions | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|:--:|
| 0.01 | 1 | 1.20452 | 0 | 0 | 0.000 | 0 |
| 0.01 | 2 | 1.20452 | 0 | 0 | 0.000 | 0 |
| 0.01 | 3 | 1.20452 | 0 | 0 | 0.000 | 0 |
| 0.01 | 4 | 5.92215 | 0 | 0 | 0.000 | 0 |
| 0.1 | 1 | 1.20322 | 0 | 0 | 0.000 | 0 |
| 0.1 | 2 | 1.20322 | 0 | 0 | 0.000 | 0 |
| 0.1 | 3 | 1.20322 | 0 | 0 | 0.000 | 0 |
| 0.1 | 4 | 5.92227 | 0 | 0 | 0.000 | 0 |
| 1 | 1 | 1.19375 | 0 | 0 | 0.000 | 0 |
| 1 | 2 | 1.19375 | 0 | 0 | 0.000 | 0 |
| 1 | 3 | 1.19375 | 0 | 0 | 0.000 | 0 |
| 1 | 4 | 5.91744 | 0 | 0 | 0.000 | 0 |
| 10 | 1 | 1.14196 | 0 | 0 | 0.000 | 0 |
| 10 | 2 | 1.14196 | 0 | 0 | 0.000 | 0 |
| 10 | 3 | 1.14196 | 0 | 0 | 0.000 | 0 |
| 10 | 4 | 5.85841 | 0 | 0 | 0.000 | 0 |

## Evidence boundary

V167 is a same-seed grouped learnability gate. Two linear ridge heads use truth-free V166 features and current-truth single-action targets. Lambda, maximum edits, support box and selected-action overprediction correction are frozen using t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 heldout rows are evaluated. Label keys are used only to enforce a unique-label projection, never as numeric features. Even a heldout pass only authorizes a nonlinear/graph or recursive development probe; it is not independent validation or generalization evidence.
