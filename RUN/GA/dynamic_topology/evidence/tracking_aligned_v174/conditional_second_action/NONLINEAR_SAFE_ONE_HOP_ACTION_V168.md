# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `conditional-compact-packed-summary / 40`
- Safe-candidate ranking: `safety-probability`
- Hidden width / weight decay: `16 / 0.0001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.000 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

- Heldout safety gate passed: `0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 1/1 | 0/0 | +9.0151 / +5.3026 | 0.158 |
| Heldout F5 t=79 | 0/0 | 0/0 | +0.0000 / +0.0000 | 0.000 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +25.1004 / +59.3427 | 0.596 |
| Training F5 t=78 | 6/6 | 0/0 | +0.5707 / +5.5831 | 0.761 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 3/3 | +8.9128 / +5.1964 | 0.160 |
| Heldout raw | 6/6 | 1/1 | +40.1605 / +0.2854 | 0.021 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.00 | 0.60 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.00 | 0.70 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.00 | 0.80 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.50 | 0.50 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.50 | 0.60 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.50 | 0.70 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 0.50 | 0.80 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 1.00 | 0.50 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 1.00 | 0.60 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 1.00 | 0.70 | 1 | 0 | 0.158 | 0 |
| 16 | 0.0001 | 1.00 | 0.80 | 1 | 0 | 0.158 | 0 |
| 16 | 0.001 | 0.00 | 0.50 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.00 | 0.60 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.00 | 0.70 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.00 | 0.80 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.50 | 0.50 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.50 | 0.60 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.50 | 0.70 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 0.50 | 0.80 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 1.00 | 0.50 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 1.00 | 0.60 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 1.00 | 0.70 | 1 | 0 | 0.009 | 0 |
| 16 | 0.001 | 1.00 | 0.80 | 1 | 0 | 0.009 | 0 |
| 32 | 0.0001 | 0.00 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.00 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.00 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.00 | 0.80 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.50 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.50 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.50 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 0.50 | 0.80 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 1.00 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 1.00 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 1.00 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.0001 | 1.00 | 0.80 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.00 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.00 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.00 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.00 | 0.80 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.50 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.50 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.50 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 0.50 | 0.80 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 1.00 | 0.50 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 1.00 | 0.60 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 1.00 | 0.70 | 1 | 0 | 0.154 | 0 |
| 32 | 0.001 | 1.00 | 0.80 | 1 | 0 | 0.154 | 0 |

## Evidence boundary

This is a same-seed grouped conditional-action learnability gate. The frozen compact first-stage selector chooses one label without truth; the MLP then consumes truth-free updated-posterior and set-cardinality features and learns only the signs of the incremental second-action E-OSPA and RMSE gains. Width, weight decay, uncertainty penalty and threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. A pass authorizes only a sequential nonrecursive and recursive development probe, not validation or generalization.
