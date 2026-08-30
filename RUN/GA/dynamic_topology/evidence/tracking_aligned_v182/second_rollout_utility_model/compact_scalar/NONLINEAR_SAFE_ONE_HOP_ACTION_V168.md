# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `compact-scalar-second-rollout-positive-utility / 27`
- Safe-candidate ranking: `positive-utility`
- Hidden width / weight decay: `32 / 0.0001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.500 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

- Heldout safety gate passed: `0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +36.1748 / +157.0249 | 0.738 |
| Heldout F5 t=79 | 4/4 | 1/1 | +0.1529 / +0.6919 | 0.092 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 2/2 | +15.5694 / -218.3236 | 0.342 |
| Training F5 t=78 | 18/18 | 1/1 | +114.4438 / +2.8795 | 0.173 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 3/3 | +24.2976 / +16.1694 | 0.803 |
| Heldout raw | 6/6 | 3/3 | +0.1517 / +0.6874 | 0.092 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 2 | 0.835 | 0 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 3 | 0.775 | 0 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 3 | 0.673 | 0 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.740 | 0 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 3 | 0.676 | 0 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 2 | 0.736 | 0 |
| 16 | 0.0001 | 1.00 | 0.80 | 3 | 0 | 0.627 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 5 | 0.562 | 0 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 5 | 0.562 | 0 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 4 | 0.567 | 0 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.631 | 1 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 5 | 0.562 | 0 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 4 | 0.567 | 0 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 4 | 0.567 | 0 |
| 16 | 0.001 | 0.50 | 0.80 | 5 | 0 | 0.636 | 1 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 4 | 0.664 | 0 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 4 | 0.567 | 0 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 2 | 0.576 | 0 |
| 16 | 0.001 | 1.00 | 0.80 | 5 | 0 | 0.636 | 1 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 3 | 0.803 | 0 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 2 | 0.808 | 0 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 1 | 0.807 | 0 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.713 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.738 | 1 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.735 | 1 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.732 | 1 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 2 | 0.734 | 0 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.735 | 1 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.735 | 1 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 2 | 0.728 | 0 |
| 32 | 0.0001 | 1.00 | 0.80 | 6 | 2 | 0.734 | 0 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 1 | 0.966 | 0 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 1 | 0.964 | 0 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 3 | 0.802 | 0 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 2 | 0.802 | 0 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 1 | 0.966 | 0 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 1 | 0.964 | 0 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 2 | 0.806 | 0 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 3 | 0.108 | 0 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 1 | 0.375 | 0 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 1 | 0.321 | 0 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 2 | 0.220 | 0 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 1 | 0.008 | 0 |

## Evidence boundary

V181 is the second opened seed-211 DAgger-style learnability gate. The MLP trains on V166 early/static cells, V176 t=78 and the previously opened V176 t=79 rollout cells. The original V166 F3 calibration boundary remains unchanged; duplicate V180 t=78 states are excluded. Width, decay, uncertainty and threshold are frozen before the new V180 F5 t=79 rollout cells are read. Inference features are truth-free and current truth supplies immediate action signs. A pass authorizes one recursive development probe, not validation or generalization.
