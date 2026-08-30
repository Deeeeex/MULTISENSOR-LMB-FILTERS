# V168 nonlinear safe one-hop action gate

- Model: `nonlinear-safe-one-hop-label-action-v168`
- Feature profile / count: `compact-scalar-20-byte-rollout-aware / 27`
- Safe-candidate ranking: `safety-probability`
- Hidden width / weight decay: `16 / 0.001`
- Ensemble seeds: `[1701 1709 1721]`
- Uncertainty penalty / probability threshold: `0.000 / 0.500`
- Maximum edits per receiver: `1`
- Heldout gate passed: `0`

- Heldout safety gate passed: `1`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration F3 t=79 | 6/6 | 0/0 | +36.2347 / +157.0772 | 0.757 |
| Heldout F5 t=79 | 6/6 | 0/0 | +44.9956 / +1.3406 | 0.150 |

Opened training-cell readout (diagnostic only):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 t=76/78 | 12/12 | 0/0 | +48.5638 / +169.1948 | 0.495 |
| Training F5 t=78 | 12/12 | 0/0 | +78.6265 / +2.7266 | 0.236 |

Uncalibrated diagnostic (ensemble mean > 0.5; not a policy):

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Calibration raw | 6/6 | 0/0 | +36.2347 / +157.0772 | 0.757 |
| Heldout raw | 6/6 | 0/0 | +44.9956 / +1.3406 | 0.150 |

## Candidate configurations

| Width | Decay | Uncertainty | Threshold | Cal cells | Harmful | Capture | Pass |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 16 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.664 | 1 |
| 16 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.664 | 1 |
| 16 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.757 | 1 |
| 16 | 0.001 | 1.00 | 0.80 | 6 | 0 | 0.757 | 1 |
| 32 | 0.0001 | 0.00 | 0.50 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.00 | 0.60 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.00 | 0.70 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.00 | 0.80 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.50 | 0.50 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.50 | 0.60 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.50 | 0.70 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 0.50 | 0.80 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 1.00 | 0.50 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 1.00 | 0.60 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 1.00 | 0.70 | 6 | 0 | 0.663 | 1 |
| 32 | 0.0001 | 1.00 | 0.80 | 6 | 0 | 0.663 | 1 |
| 32 | 0.001 | 0.00 | 0.50 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.00 | 0.60 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.00 | 0.70 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.00 | 0.80 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.50 | 0.50 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.50 | 0.60 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.50 | 0.70 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 0.50 | 0.80 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 1.00 | 0.50 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 1.00 | 0.60 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 1.00 | 0.70 | 6 | 0 | 0.661 | 1 |
| 32 | 0.001 | 1.00 | 0.80 | 5 | 0 | 0.657 | 1 |

## Evidence boundary

V178 is an opened seed-211 policy-iteration learnability gate. The MLP trains on grouped V166 early/static cells plus V176 t=78 states actually visited by recursive V169. Width, weight decay, uncertainty penalty and threshold are frozen using only the V166 F3 t=79 calibration cells before the V176 F5 t=79 rollout cells are opened. Inference features are truth-free; current truth supplies immediate action signs. A pass authorizes one recursive development probe, not validation or generalization.
