# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `compact-scalar-20-byte / 27`
- Safe-candidate ranking: `positive-utility`
- Hidden width / weight decay: `32 / 0.001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `1.000 / 0.800`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +41.6919 / +147.7287 | 0.680 |
| Heldout F5 t=79 | 6/6 | 6/6 | +41.8429 / -4.5928 | 0.000 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +37.7677 / +190.7502 | 0.603 |
| Training F5 t=78 | 6/6 | 0/0 | +42.8892 / +1.4759 | 0.235 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 1/1 | +36.6382 / +158.7983 | 0.795 |
| Heldout raw | 6/6 | 0/0 | +42.8096 / +0.8874 | 0.094 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 3 | 0.787 | 0 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 3 | 0.787 | 0 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 3 | 0.787 | 0 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 1 | 0.676 | 0 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 2 | 0.764 | 0 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 2 | 0.764 | 0 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 1 | 0.676 | 0 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.676 | 0 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 2 | 0.672 | 0 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 2 | 0.672 | 0 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 1 | 0.676 | 0 |
| 16 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 2 | 0.663 | 0 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 0.50 | 0.80 | 6 | 1 | 0.770 | 0 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 5 | 0.601 | 0 |
| 16 | 0.001 | 1.00 | 0.80 | 6 | 1 | 0.766 | 0 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 4 | 0.730 | 0 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 4 | 0.730 | 0 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 3 | 0.637 | 0 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.642 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 2 | 0.733 | 0 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 2 | 0.640 | 0 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 1 | 0.642 | 0 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.642 | 0 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 3 | 0.637 | 0 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 3 | 0.637 | 0 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 1 | 0.641 | 0 |
| 32 | 0.0001 | 1.00 | 0.80 | 5 | 1 | 0.641 | 0 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 1 | 0.795 | 0 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 4 | 0.732 | 0 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 4 | 0.732 | 0 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 4 | 0.747 | 0 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 3 | 0.195 | 0 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 2 | 0.734 | 0 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 3 | 0.734 | 0 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 2 | 0.660 | 0 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 3 | 0.195 | 0 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 2 | 0.740 | 0 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 2 | 0.764 | 0 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.680 | 1 |

## Evidence boundary

V168 is a same-seed grouped nonlinear learnability gate. A shared MLP classifier consumes the truth-free V166 action features and learns only the signs of current-truth immediate E-OSPA and RMSE gains. Width, weight decay, ensemble uncertainty penalty and probability threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. One action per receiver avoids assuming that independently evaluated actions add. A heldout pass authorizes only a recursive development experiment; it is not independent-seed or cross-scenario evidence.
