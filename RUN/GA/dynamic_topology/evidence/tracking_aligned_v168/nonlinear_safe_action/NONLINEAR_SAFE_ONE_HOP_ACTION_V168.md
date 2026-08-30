# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `full-observable / 37`
- Hidden width / weight decay: `16 / 0.001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.000 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `1`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +30.9724 / +166.5604 | 0.862 |
| Heldout F5 t=79 | 6/6 | 0/0 | +28.8094 / +3.4915 | 0.330 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +49.5122 / +175.3005 | 0.526 |
| Training F5 t=78 | 6/6 | 0/0 | +35.8317 / +2.2633 | 0.344 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 0/0 | +30.9724 / +166.5604 | 0.862 |
| Heldout raw | 6/6 | 0/0 | +28.8094 / +3.4915 | 0.330 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.766 | 1 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.766 | 1 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.766 | 1 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.766 | 1 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.768 | 1 |
| 16 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.768 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.862 | 1 |
| 16 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.862 | 1 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 1 | 0.675 | 0 |
| 32 | 0.0001 | 1.00 | 0.80 | 6 | 1 | 0.675 | 0 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 1 | 0.799 | 0 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 1 | 0.799 | 0 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 1 | 0.799 | 0 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 1 | 0.799 | 0 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 1 | 0.810 | 0 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 1 | 0.810 | 0 |

## Evidence boundary

V168 is a same-seed grouped nonlinear learnability gate. A shared MLP classifier consumes the truth-free V166 action features and learns only the signs of current-truth immediate E-OSPA and RMSE gains. Width, weight decay, ensemble uncertainty penalty and probability threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. One action per receiver avoids assuming that independently evaluated actions add. A heldout pass authorizes only a recursive development experiment; it is not independent-seed or cross-scenario evidence.
