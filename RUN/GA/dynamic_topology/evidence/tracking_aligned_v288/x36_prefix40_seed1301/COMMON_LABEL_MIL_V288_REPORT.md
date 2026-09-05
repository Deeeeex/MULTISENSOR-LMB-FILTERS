# V288 common-label LMB-MIL: paired prefix

X36, seed 1301, steps 1--40 of the original 160-step scene. Source `f67a1bbc5f19c56fe4005749888b4ef964437230`.

Development evidence; self-check only. Reference filter reused from V282. MIL changes the fusion family, not the V242 route or shared labels.

| Metric (lower is better) | V242 KLA reference | V288 reduced-GM MIL |
| --- | ---: | ---: |
| E-OSPA | 135.180030 | 132.617637 |
| Absolute count error | 19.501389 | 18.631944 |
| Conditional matched RMSE | 8.425317 | 19.735674 |
| Representative disagreement, entire prefix | 144.669699 | 140.162516 |
| Worst-node E-OSPA | 137.719448 | 134.289701 |
| Attempted payload bytes | 18435344.000000 | 59867264.000000 |
| Delivered payload bytes | 17667904.000000 | 57266464.000000 |
| Attempted messages | 1840.000000 | 1840.000000 |
| Delivered messages | 1766.000000 | 1766.000000 |

Common-finite-cell RMSE: 8.425317 -> 19.735674, 1440 cells, change +134.242%.
Finite RMSE coverage: 1.000000 -> 1.000000; this is not an accuracy metric.
Attempted / delivered route differences: 0 / 0. Runtime 2205.5 seconds.

Eight-component spatial reduction: 22342 / 34560 fused labels truncated; mean discarded conditional mass 0.074527607, maximum 0.663394592.

Zero-extended missing source-label cases: 0 across 0 label pools. The earlier two-step integration did not retain these two counters.

| Formation | V242 E-OSPA | V288 E-OSPA | Change (%) |
| --- | ---: | ---: | ---: |
| 1 | 131.108544 | 130.474519 | -0.484 |
| 2 | 133.828655 | 131.117019 | -2.026 |
| 3 | 136.465141 | 133.420935 | -2.231 |
| 4 | 136.742390 | 132.892224 | -2.816 |
| 5 | 137.278990 | 133.719252 | -2.593 |
| 6 | 135.656459 | 134.081873 | -1.161 |

Screen evaluated: 1. Screen passed: 0. Two steps are integration only.
This is not a full-episode, held-out, label-matching or complete different-FoV MIL reproduction result. No routing improvement may be attributed to a cross-fusion comparison.
