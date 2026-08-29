# V152 safe graph-codebook oracle pilot

- Preset / seed / window: `m24-formation-fov / 83 / 70:77`
- Better static reference: `backbone-residual-spliced-cycle-cw-a70-e05`
- Selected oracle action: `backbone-residual-spliced-cycle-cw-a70-e05`
- Admissible dynamic graphs: `0 / 6`

| Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Attempted-byte saving | 5% pilot gate |
|--:|--:|--:|--:|--:|:--:|
| +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 0 |

## Codebook

| Arm | Mean E-OSPA | Worst sensor | Consensus | Bytes | Admissible |
|:--|--:|--:|--:|--:|:--:|
| `backbone-residual-spliced-cycle-cw-a70-e05` | 23.087786 | 63.338432 | 29.506247 | 16014976 | 1 |
| `backbone-residual-spliced-cycle-ccw-a70-e05` | 26.907930 | 59.323228 | 30.530771 | 16126528 | 0 |
| `v152-safe-graph-rank1` | 21.337686 | 55.947326 | 26.858049 | 16244704 | 0 |
| `v152-safe-graph-rank2` | 20.871903 | 55.956629 | 27.103877 | 16213000 | 0 |
| `v152-safe-graph-rank3` | 20.898673 | 40.007492 | 26.835127 | 16309072 | 0 |
| `v152-safe-graph-rank4` | 20.083482 | 39.808933 | 25.586116 | 16329136 | 0 |
| `v152-safe-graph-rank5` | 20.468155 | 39.119547 | 26.219443 | 16236736 | 0 |
| `v152-safe-graph-rank6` | 23.348161 | 52.767533 | 29.370490 | 16286056 | 0 |

## Dynamic constraint decomposition

| Rank | Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Byte saving | Structure pass | Admissible |
|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 1 | +7.580% | +11.669% | -9.382% | +8.975% | -1.434% | 1 | 0 |
| 2 | +9.598% | +11.655% | -7.231% | +8.142% | -1.236% | 1 | 0 |
| 3 | +9.482% | +36.835% | -21.479% | +9.053% | -1.836% | 1 | 0 |
| 4 | +13.013% | +37.149% | +4.262% | +13.286% | -1.962% | 1 | 0 |
| 5 | +11.346% | +38.237% | -21.879% | +11.139% | -1.385% | 1 | 0 |
| 6 | -1.128% | +16.690% | -35.550% | +0.460% | -1.693% | 1 | 0 |

## Boundary

Opened-development action-space headroom only. The pinned M24 score generates scale-shared feasible graph diversity; it is not an X36 value predictor. Truth and future outcomes select the offline oracle only.
