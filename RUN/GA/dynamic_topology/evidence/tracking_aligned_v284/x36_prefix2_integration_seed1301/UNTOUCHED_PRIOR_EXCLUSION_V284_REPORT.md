# V284 untouched-prior exclusion: paired prefix

X36, seed 1301, steps 1--2 of the original 160-step scene. Source `fd5722f2900ad86c44c0b7b27507c4fb4ae586dd`.

Development evidence, self-check only. The reference is reused from V282; no baseline filter rerun.

| Metric (lower is better) | V242 reference | V284 intervention |
| --- | ---: | ---: |
| E-OSPA | 139.697349 | 136.361972 |
| Count absolute error | 20.805556 | 19.833333 |
| Conditional matched RMSE | 11.831774 | 12.761410 |
| Representative disagreement, entire prefix | 149.479424 | 136.282593 |
| Worst-node E-OSPA | 146.843905 | 146.841752 |
| Attempted payload bytes | 677584.000000 | 683152.000000 |
| Delivered payload bytes | 670880.000000 | 676256.000000 |
| Attempted messages | 92.000000 | 92.000000 |
| Delivered messages | 91.000000 | 91.000000 |

Common-finite-cell RMSE: 11.831774 -> 12.761410 over 72 cells; change +7.857%.
Finite RMSE fractions (not an accuracy metric): 1.000000 -> 1.000000.
Attempted / delivered route differences: 0 / 0. Metadata bytes: 17664 (included above); excluded source-label pools: 182.

| Formation | V242 E-OSPA | V284 E-OSPA | Change (%) |
| --- | ---: | ---: | ---: |
| 1 | 146.037161 | 143.886608 | -1.473 |
| 2 | 137.840218 | 137.246960 | -0.430 |
| 3 | 138.717712 | 133.204988 | -3.974 |
| 4 | 138.431519 | 134.952033 | -2.514 |
| 5 | 138.958282 | 133.826818 | -3.693 |
| 6 | 138.199199 | 135.054423 | -2.276 |

Screen evaluated: 0. Screen passed: 0. A two-step run is integration only.

This is not a full-episode or held-out result. Prefix disagreement is not the paper focus-window metric. Lower conditional RMSE alone cannot establish set recovery.
A passing prefix requires a full-episode fixed-routing arm with the same semantic rule before attributing gains to routing; a failed screen closes startup-only follow-ups under the design.
