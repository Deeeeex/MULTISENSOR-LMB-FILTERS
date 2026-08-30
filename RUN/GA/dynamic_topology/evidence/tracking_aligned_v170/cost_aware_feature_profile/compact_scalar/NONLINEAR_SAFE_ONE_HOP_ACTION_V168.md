# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `compact-scalar-20-byte / 27`
- Safe-candidate ranking: `safety-probability`
- Hidden width / weight decay: `16 / 0.001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.500 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

- Heldout safety gate passed: `1`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +36.2816 / +157.2428 | 0.771 |
| Heldout F5 t=79 | 6/6 | 0/0 | +42.8701 / +1.3592 | 0.144 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +49.6118 / +175.5924 | 0.527 |
| Training F5 t=78 | 6/6 | 0/0 | +42.8858 / +1.4581 | 0.232 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 0/0 | +41.6842 / +147.7126 | 0.680 |
| Heldout raw | 6/6 | 0/0 | +14.8706 / +6.6146 | 0.640 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.680 | 1 |
| 16 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.680 | 1 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.771 | 1 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.771 | 1 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.771 | 1 |
| 16 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.771 | 1 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.762 | 1 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.762 | 1 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.762 | 1 |
| 16 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.762 | 1 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.677 | 1 |
| 32 | 0.0001 | 1.00 | 0.80 | 5 | 0 | 0.676 | 1 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.680 | 1 |
| 32 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.680 | 1 |

## Evidence boundary

V168 is a same-seed grouped nonlinear learnability gate. A shared MLP classifier consumes the truth-free V166 action features and learns only the signs of current-truth immediate E-OSPA and RMSE gains. Width, weight decay, ensemble uncertainty penalty and probability threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. One action per receiver avoids assuming that independently evaluated actions add. A heldout pass authorizes only a recursive development experiment; it is not independent-seed or cross-scenario evidence.
