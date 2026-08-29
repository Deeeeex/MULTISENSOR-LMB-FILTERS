# V152 safe graph-codebook oracle pilot

- Preset / seed / window: `x36-formation-fov / 83 / 60:67`
- Better static reference: `backbone-residual-spliced-cycle-cw-a70-e05`
- Selected oracle action: `backbone-residual-spliced-cycle-cw-a70-e05`
- Admissible dynamic graphs: `0 / 6`

| Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Attempted-byte saving | 5% pilot gate |
|--:|--:|--:|--:|--:|:--:|
| +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 0 |

## Codebook

| Arm | Mean E-OSPA | Worst sensor | Consensus | Bytes | Admissible |
|:--|--:|--:|--:|--:|:--:|
| `backbone-residual-spliced-cycle-cw-a70-e05` | 92.345953 | 103.105520 | 80.654813 | 25409592 | 1 |
| `backbone-residual-spliced-cycle-ccw-a70-e05` | 93.673437 | 102.598649 | 82.923919 | 25301256 | 0 |
| `v152-safe-graph-rank1` | 94.487662 | 104.902024 | 83.500717 | 25828608 | 0 |
| `v152-safe-graph-rank2` | 93.977983 | 104.902024 | 83.067806 | 25885704 | 0 |
| `v152-safe-graph-rank3` | 94.042523 | 104.290869 | 83.183677 | 25648848 | 0 |
| `v152-safe-graph-rank4` | 93.773551 | 104.902026 | 82.726191 | 25724424 | 0 |
| `v152-safe-graph-rank5` | 92.264821 | 104.310987 | 81.695208 | 25740000 | 0 |
| `v152-safe-graph-rank6` | 94.515733 | 103.125646 | 83.093969 | 25731720 | 0 |

## Dynamic constraint decomposition

| Rank | Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Byte saving | Structure pass | Admissible |
|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 1 | -2.319% | -1.742% | -5.643% | -3.528% | -1.649% | 1 | 0 |
| 2 | -1.767% | -1.742% | -6.908% | -2.992% | -1.874% | 1 | 0 |
| 3 | -1.837% | -1.150% | -4.093% | -3.135% | -0.942% | 1 | 0 |
| 4 | -1.546% | -1.742% | -4.233% | -2.568% | -1.239% | 1 | 0 |
| 5 | +0.088% | -1.169% | -4.372% | -1.290% | -1.300% | 1 | 0 |
| 6 | -2.350% | -0.020% | -6.764% | -3.024% | -1.268% | 1 | 0 |

## Boundary

Opened-development action-space headroom only. The pinned M24 score generates scale-shared feasible graph diversity; it is not an X36 value predictor. Truth and future outcomes select the offline oracle only.
