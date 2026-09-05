# V284 untouched-prior exclusion: paired prefix

X36, seed 1301, steps 1--40 of the original 160-step scene. Source `d9c42fdff3d51b52731e9eff1b8c6a8cd6d15ac3`.

Development evidence, self-check only. The reference is reused from V282; no baseline filter rerun.

| Metric (lower is better) | V242 reference | V284 intervention |
| --- | ---: | ---: |
| E-OSPA | 135.180030 | 117.117031 |
| Count absolute error | 19.501389 | 14.473611 |
| Conditional matched RMSE | 8.425317 | 22.303075 |
| Representative disagreement, entire prefix | 144.669699 | 115.117106 |
| Worst-node E-OSPA | 137.719448 | 124.052048 |
| Attempted payload bytes | 18435344.000000 | 18442200.000000 |
| Delivered payload bytes | 17667904.000000 | 17675856.000000 |
| Attempted messages | 1840.000000 | 1840.000000 |
| Delivered messages | 1766.000000 | 1766.000000 |

Common-finite-cell RMSE: 8.425317 -> 22.303075 over 1440 cells; change +164.715%.
Finite RMSE fractions (not an accuracy metric): 1.000000 -> 1.000000.
Attempted / delivered route differences: 0 / 0. Metadata bytes: 352792 (included above); excluded source-label pools: 1670.

| Formation | V242 E-OSPA | V284 E-OSPA | Change (%) |
| --- | ---: | ---: | ---: |
| 1 | 131.108544 | 111.213457 | -15.175 |
| 2 | 133.828655 | 111.640596 | -16.579 |
| 3 | 136.465141 | 114.093008 | -16.394 |
| 4 | 136.742390 | 120.361768 | -11.979 |
| 5 | 137.278990 | 123.152466 | -10.290 |
| 6 | 135.656459 | 122.240893 | -9.889 |

Screen evaluated: 1. Screen passed: 0. A two-step run is integration only.

This is not a full-episode or held-out result. Prefix disagreement is not the paper focus-window metric. Lower conditional RMSE alone cannot establish set recovery.
A passing prefix requires a full-episode fixed-routing arm with the same semantic rule before attributing gains to routing; a failed screen closes startup-only follow-ups under the design.
