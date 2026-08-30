# V174 conditional second-label safety gate

- Model artifact: `RUN/GA/dynamic_topology/evidence/tracking_aligned_v174/conditional_second_action/NONLINEAR_SAFE_ONE_HOP_ACTION_V168_MODEL.mat`
- Compact conditional feature count: `40`
- Heldout safety / efficiency gate: `0 / 0`

| Split | Selected cells/actions | Harmful cells/actions | E-OSPA / RMSE gain | Joint-utility capture |
|:--|:--|:--|:--|--:|
| Training F3 | 12/12 | 0/0 | +25.1004 / +59.3427 | 0.596 |
| Training F5 | 6/6 | 0/0 | +0.5707 / +5.5831 | 0.761 |
| Calibration F3 t=79 | 1/1 | 0/0 | +9.0151 / +5.3026 | 0.158 |
| Heldout F5 t=79 | 0/0 | 0/0 | +0.0000 / +0.0000 | 0.000 |

## Evidence boundary

This is a same-seed grouped conditional-action learnability gate. The frozen compact first-stage selector chooses one label without truth; the MLP then consumes truth-free updated-posterior and set-cardinality features and learns only the signs of the incremental second-action E-OSPA and RMSE gains. Width, weight decay, uncertainty penalty and threshold are frozen with t=76/78 training cells and t=79 F3 calibration cells before t=79 F5 is opened. A pass authorizes only a sequential nonrecursive and recursive development probe, not validation or generalization.
