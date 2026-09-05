# V288 common-label LMB-MIL: paired prefix

X36, seed 1301, steps 1--2 of the original 160-step scene. Source `d5fa9e4f460168064494df331c0cb1e6ef8b9704`.

Development evidence; self-check only. Reference filter reused from V282. MIL changes the fusion family, not the V242 route or shared labels.

| Metric (lower is better) | V242 KLA reference | V288 reduced-GM MIL |
| --- | ---: | ---: |
| E-OSPA | 139.697349 | 139.270653 |
| Absolute count error | 20.805556 | 20.666667 |
| Conditional matched RMSE | 11.831774 | 13.737502 |
| Representative disagreement, entire prefix | 149.479424 | 147.362762 |
| Worst-node E-OSPA | 146.843905 | 146.843246 |
| Attempted payload bytes | 677584.000000 | 964696.000000 |
| Delivered payload bytes | 670880.000000 | 957992.000000 |
| Attempted messages | 92.000000 | 92.000000 |
| Delivered messages | 91.000000 | 91.000000 |

Common-finite-cell RMSE: 11.831774 -> 13.737502, 72 cells, change +16.107%.
Finite RMSE coverage: 1.000000 -> 1.000000; this is not an accuracy metric.
Attempted / delivered route differences: 0 / 0. Runtime 25.9 seconds.

Eight-component spatial reduction: 164 / 1728 fused labels truncated; mean discarded conditional mass 0.024773690, maximum 0.535584399.

| Formation | V242 E-OSPA | V288 E-OSPA | Change (%) |
| --- | ---: | ---: | ---: |
| 1 | 146.037161 | 145.768885 | -0.184 |
| 2 | 137.840218 | 137.920597 | +0.058 |
| 3 | 138.717712 | 137.636486 | -0.779 |
| 4 | 138.431519 | 137.940086 | -0.355 |
| 5 | 138.958282 | 138.702248 | -0.184 |
| 6 | 138.199199 | 137.655612 | -0.393 |

Screen evaluated: 0. Screen passed: 0. Two steps are integration only.
This is not a full-episode, held-out, label-matching or complete different-FoV MIL reproduction result. No routing improvement may be attributed to a cross-fusion comparison.
