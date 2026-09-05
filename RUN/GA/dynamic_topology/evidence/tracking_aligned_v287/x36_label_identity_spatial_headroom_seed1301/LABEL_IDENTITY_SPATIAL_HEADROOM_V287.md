# V287: geometric identity and same-label spatial headroom

X36 seed 1301, unchanged stored prefix 1--40. Diagnostic / self-check only; no filter, relabeling, observation resimulation or communication action. The scene has 24 distinct initial birth components, one per explicit truth trajectory, active throughout this prefix. Birth identity is a diagnostic anchor, not a change to unlabeled OSPA/RMSE.

## Emitted states

| Arm | Steps | Outputs | Birth / assigned truth agrees (%) | Birth / nearest truth agrees (%) | Geometric pooled RMSE | Birth-anchored pooled RMSE |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 1--40 | 6478 | 28.47 | 28.62 | 9.109935 | 24.791733 |
| 1 | 1--5 | 642 | 67.91 | 48.91 | 12.534183 | 15.459359 |
| 1 | 6--20 | 2156 | 37.06 | 37.52 | 8.704195 | 18.541690 |
| 1 | 21--40 | 3680 | 16.55 | 19.86 | 8.619496 | 28.962683 |
| 2 | 1--40 | 13718 | 55.14 | 37.25 | 26.909607 | 31.443734 |
| 2 | 1--5 | 1002 | 72.26 | 47.70 | 13.154831 | 15.322640 |
| 2 | 6--20 | 5549 | 62.48 | 40.71 | 15.554616 | 19.736204 |
| 2 | 21--40 | 7167 | 47.06 | 33.11 | 34.270933 | 39.471914 |

Arm 1 is V242; arm 2 is V284. Pooled RMSE is not the official mean per-cell RMSE. A coherent common label permutation would not itself invalidate label-wise fusion; birth disagreement alone is not a causal diagnosis.

## Cross-node same-label output correspondence

| Arm | Steps | Pool | Label pairs | Assignment disagreement (%) | Nearest disagreement (%) | Mean truth separation when assigned identities disagree (m) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 1 | 1--40 | 1 | 22491 | 34.73 | 40.56 | 20.408 |
| 1 | 1--40 | 2 | 6341 | 24.68 | 28.24 | 19.645 |
| 1 | 1--5 | 1 | 1808 | 44.03 | 48.06 | 23.495 |
| 1 | 1--5 | 2 | 567 | 40.39 | 40.56 | 22.087 |
| 1 | 6--20 | 1 | 6659 | 44.75 | 47.12 | 20.803 |
| 1 | 6--20 | 2 | 2051 | 31.74 | 33.54 | 19.631 |
| 1 | 21--40 | 1 | 14024 | 28.77 | 36.47 | 19.508 |
| 1 | 21--40 | 2 | 3723 | 18.40 | 23.45 | 18.841 |
| 2 | 1--40 | 1 | 124196 | 47.23 | 42.68 | 25.796 |
| 2 | 1--40 | 2 | 13981 | 20.86 | 17.76 | 21.630 |
| 2 | 1--5 | 1 | 5109 | 42.34 | 48.27 | 23.268 |
| 2 | 1--5 | 2 | 1019 | 37.59 | 33.46 | 22.183 |
| 2 | 6--20 | 1 | 52422 | 46.73 | 49.41 | 22.656 |
| 2 | 6--20 | 2 | 5559 | 25.42 | 21.75 | 20.408 |
| 2 | 21--40 | 1 | 66665 | 48.01 | 36.96 | 28.370 |
| 2 | 21--40 | 2 | 7403 | 15.13 | 12.60 | 22.982 |

Pool 1 counts undirected global pairs; pool 2 counts directed delivered edges. Outputs are from round end, not the actual incoming packets earlier in the same round. These are geometric correspondences, not confirmed identity switches. Close targets and one-to-one assignment competition can affect correspondence; no distance gate is introduced.

## Restricted source oracle

| Birth stratum | Lag | Pool | Queries | Receiver | Original oracle | Assignment-coherent oracle | Nearest-coherent oracle |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| -1 | 0 | 1 | 7850 | 34.730995 | 11.104842 | 22.535599 | 21.112490 |
| -1 | 0 | 4 | 7850 | 34.730995 | 33.963849 | 34.185749 | 34.295225 |
| -1 | 0 | 5 | 7850 | 34.730995 | 29.655729 | 31.691452 | 31.998599 |
| -1 | 1 | 1 | 7818 | 34.786640 | 11.083094 | 22.349591 | 21.179909 |
| -1 | 1 | 4 | 7818 | 34.786640 | 34.380086 | 34.532953 | 34.548844 |
| -1 | 1 | 5 | 7818 | 34.786640 | 29.978635 | 32.021646 | 32.199818 |
| 0 | 0 | 1 | 2188 | 44.469576 | 16.422120 | 28.855997 | 26.216276 |
| 0 | 0 | 4 | 2188 | 44.469576 | 43.604744 | 43.887550 | 44.135072 |
| 0 | 0 | 5 | 2188 | 44.469576 | 39.660885 | 41.396164 | 42.625447 |
| 0 | 1 | 1 | 2184 | 44.501706 | 16.465175 | 28.534766 | 26.594317 |
| 0 | 1 | 4 | 2184 | 44.501706 | 44.135464 | 44.278284 | 44.348965 |
| 0 | 1 | 5 | 2184 | 44.501706 | 39.835875 | 41.852446 | 42.733556 |
| 1 | 0 | 1 | 5662 | 30.136084 | 8.170403 | 19.553335 | 18.772090 |
| 1 | 0 | 4 | 5662 | 30.136084 | 29.403307 | 29.596604 | 29.629945 |
| 1 | 0 | 5 | 5662 | 30.136084 | 24.727714 | 27.023163 | 26.785435 |
| 1 | 1 | 1 | 5634 | 30.191216 | 8.084541 | 19.429379 | 18.663236 |
| 1 | 1 | 4 | 5634 | 30.191216 | 29.750176 | 29.913146 | 29.898014 |
| 1 | 1 | 5 | 5634 | 30.191216 | 25.138590 | 27.273814 | 27.034199 |

Birth strata: -1 all, 0 mismatch, 1 agreement. Source pools: 1 global, 4 delivered incoming, 5 physical one-hop. Both restrictions require source-time geometric correspondence to the receiver query target; nearest and assignment restrictions are separate. All oracles may retain the receiver and use truth, so none is an online selector. Lag 1 predicts prior output one step; lag 0 uses current end-of-round output. Neither accounts for actual multihop transit or a changed tracking trajectory.

Maximum official per-cell RMSE discrepancy 1.42e-14; assignment solver fallbacks 0. Original V285 oracle readback discrepancy 0. No source-local positive-measurement timestamp is established here.
