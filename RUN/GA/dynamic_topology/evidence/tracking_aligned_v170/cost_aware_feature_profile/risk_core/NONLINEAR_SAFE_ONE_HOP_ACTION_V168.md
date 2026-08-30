# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `risk-core-16-byte / 20`
- Hidden width / weight decay: `32 / 0.0001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.500 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +30.3880 / +161.3003 | 0.810 |
| Heldout F5 t=79 | 6/6 | 0/0 | +42.8701 / +1.3592 | 0.144 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 1/1 | +42.0999 / +172.3089 | 0.525 |
| Training F5 t=78 | 6/6 | 0/0 | +42.8858 / +1.4581 | 0.232 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 0/0 | +29.3460 / +156.3455 | 0.763 |
| Heldout raw | 6/6 | 0/0 | +42.8701 / +1.3592 | 0.144 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 1 | 0.285 | 0 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 1 | 0.285 | 0 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 1 | 0.285 | 0 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 1 | 0.285 | 0 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 1 | 0.843 | 0 |
| 16 | 0.0001 | 1.00 | 0.80 | 6 | 1 | 0.843 | 0 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 0.50 | 0.80 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 1 | 0.199 | 0 |
| 16 | 0.001 | 1.00 | 0.80 | 6 | 1 | 0.199 | 0 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.763 | 1 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.763 | 1 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.763 | 1 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.763 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.810 | 1 |
| 32 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.810 | 1 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 1 | 0.199 | 0 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 1 | 0.199 | 0 |

## Evidence boundary

V168 is a same-seed grouped nonlinear learnability gate. A shared MLP classifier consumes the truth-free V166 action features and learns only the signs of current-truth immediate E-OSPA and RMSE gains. Width, weight decay, ensemble uncertainty penalty and probability threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. One action per receiver avoids assuming that independently evaluated actions add. A heldout pass authorizes only a recursive development experiment; it is not independent-seed or cross-scenario evidence.
