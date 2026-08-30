# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `compact-scalar-20-byte-second-rollout-aware / 27`
- Safe-candidate ranking: `safety-probability`
- Hidden width / weight decay: `16 / 0.001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.000 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

- Heldout safety gate passed: `1`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +41.6836 / +147.7111 | 0.640 |
| Heldout F5 t=79 | 4/4 | 0/0 | +23.3055 / +0.4899 | 0.056 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +31.3212 / +159.6818 | 0.440 |
| Training F5 t=78 | 18/18 | 0/0 | +44.9130 / +0.8327 | 0.043 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 0/0 | +41.6836 / +147.7111 | 0.640 |
| Heldout raw | 6/6 | 2/2 | +23.3043 / +0.4854 | 0.056 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 1 | 0.631 | 0 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 1 | 0.631 | 0 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 1 | 0.631 | 0 |
| 16 | 0.0001 | 1.00 | 0.80 | 3 | 0 | 0.623 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 0.50 | 0.80 | 5 | 0 | 0.636 | 1 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.640 | 1 |
| 16 | 0.001 | 1.00 | 0.80 | 5 | 0 | 0.636 | 1 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.632 | 1 |
| 32 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.632 | 1 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.613 | 1 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.613 | 1 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.613 | 1 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.613 | 1 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.005 | 1 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.005 | 1 |

## Evidence boundary

V181 is the second opened seed-211 DAgger-style learnability gate. The MLP trains on V166 early/static cells, V176 t=78 and the previously opened V176 t=79 rollout cells. The original V166 F3 calibration boundary remains unchanged; duplicate V180 t=78 states are excluded. Width, decay, uncertainty and threshold are frozen before the new V180 F5 t=79 rollout cells are read. Inference features are truth-free and current truth supplies immediate action signs. A pass authorizes one recursive development probe, not validation or generalization.
