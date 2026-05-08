# GA Tiered Link Main Paired Confirmation (20 trials)
Generated: 2026-05-06 23:37:33
Source reports:
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED1_20260506_233600.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED6_20260506_233535.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED11_20260506_233603.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N5_SEED16_20260506_233553.md`

## Run Config
- Arms: fixed weights vs +structure-aware decoupled KLA
- Trial seeds: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21
- Chunk baseSeeds: 1, 6, 11, 16; each chunk uses five deterministic trials.
- Scenario: tiered heterogeneous packet loss, communication level 2, global delivered-measurement cap 80, fixed link loss, pDrop levels [0, 0.1, 0.2, 0.5] with counts [1, 4, 1, 2].

## Per-Trial Consensus Metrics
| Seed | Arm | OSPA | RMSE | Cardinality |
|-----:|:----|-----:|-----:|------------:|
| 2 | fixed weights | 2.590531 | 2.268101 | 0.868750 |
| 2 | +structure-aware decoupled KLA | 1.892404 | 1.611468 | 0.241250 |
| 3 | fixed weights | 2.623249 | 3.319849 | 0.761250 |
| 3 | +structure-aware decoupled KLA | 1.928044 | 1.541570 | 0.267500 |
| 4 | fixed weights | 2.681097 | 2.507659 | 0.871250 |
| 4 | +structure-aware decoupled KLA | 1.835859 | 1.487550 | 0.237500 |
| 5 | fixed weights | 2.398507 | 2.920391 | 0.625000 |
| 5 | +structure-aware decoupled KLA | 1.764445 | 2.213187 | 0.191250 |
| 6 | fixed weights | 2.826942 | 2.497009 | 1.267500 |
| 6 | +structure-aware decoupled KLA | 1.890466 | 1.894265 | 0.283750 |
| 7 | fixed weights | 2.541049 | 2.575987 | 0.747500 |
| 7 | +structure-aware decoupled KLA | 1.823612 | 1.828964 | 0.205000 |
| 8 | fixed weights | 2.386825 | 2.412256 | 0.563750 |
| 8 | +structure-aware decoupled KLA | 1.762642 | 1.481461 | 0.185000 |
| 9 | fixed weights | 2.456704 | 2.596131 | 0.663750 |
| 9 | +structure-aware decoupled KLA | 1.756928 | 1.546963 | 0.172500 |
| 10 | fixed weights | 2.571992 | 3.155163 | 0.736250 |
| 10 | +structure-aware decoupled KLA | 1.823264 | 2.505816 | 0.215000 |
| 11 | fixed weights | 2.175224 | 2.634355 | 0.416250 |
| 11 | +structure-aware decoupled KLA | 1.831138 | 1.611837 | 0.212500 |
| 12 | fixed weights | 2.235333 | 2.348212 | 0.482500 |
| 12 | +structure-aware decoupled KLA | 1.797235 | 1.885002 | 0.225000 |
| 13 | fixed weights | 2.380325 | 2.213801 | 0.656250 |
| 13 | +structure-aware decoupled KLA | 1.786939 | 1.575583 | 0.281250 |
| 14 | fixed weights | 2.363292 | 2.665071 | 0.552500 |
| 14 | +structure-aware decoupled KLA | 1.850337 | 1.788203 | 0.222500 |
| 15 | fixed weights | 2.147298 | 2.230920 | 0.382500 |
| 15 | +structure-aware decoupled KLA | 1.798971 | 1.592700 | 0.247500 |
| 16 | fixed weights | 2.048554 | 2.956503 | 0.333750 |
| 16 | +structure-aware decoupled KLA | 1.765436 | 2.136723 | 0.210000 |
| 17 | fixed weights | 2.600917 | 2.376823 | 0.802500 |
| 17 | +structure-aware decoupled KLA | 1.863439 | 1.430953 | 0.268750 |
| 18 | fixed weights | 2.530244 | 2.232345 | 0.762500 |
| 18 | +structure-aware decoupled KLA | 1.873030 | 1.746324 | 0.237500 |
| 19 | fixed weights | 2.440243 | 2.295666 | 0.741250 |
| 19 | +structure-aware decoupled KLA | 1.703659 | 1.650134 | 0.208750 |
| 20 | fixed weights | 2.498598 | 2.662959 | 0.777500 |
| 20 | +structure-aware decoupled KLA | 1.851685 | 1.651547 | 0.253750 |
| 21 | fixed weights | 2.462637 | 2.808408 | 0.576250 |
| 21 | +structure-aware decoupled KLA | 1.829428 | 1.815996 | 0.266250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.447978 +/- 0.191065 | [2.358558, 2.537398] | 20 |
| fixed weights | RMSE | 2.583880 +/- 0.315832 | [2.436068, 2.731693] | 20 |
| fixed weights | Cardinality | 0.679437 +/- 0.208768 | [0.581732, 0.777143] | 20 |
| +structure-aware decoupled KLA | OSPA | 1.821448 +/- 0.055189 | [1.795619, 1.847277] | 20 |
| +structure-aware decoupled KLA | RMSE | 1.749812 +/- 0.274055 | [1.621552, 1.878073] | 20 |
| +structure-aware decoupled KLA | Cardinality | 0.231625 +/- 0.032213 | [0.216549, 0.246701] | 20 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.626530 +/- 0.167457 | [0.548159, 0.704901] | 25.59% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | RMSE | 0.834068 +/- 0.290377 | [0.698169, 0.969967] | 32.28% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | Cardinality | 0.447813 +/- 0.197420 | [0.355418, 0.540207] | 65.91% | 20/20 | 1.907e-06 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.895679 +/- 0.113006 | [2.842791, 2.948567] | 20 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 20 |
| fixed weights | RMSE | 1.620730 +/- 0.063031 | [1.591231, 1.650229] | 20 |
| fixed weights | CardErr | 1.568688 +/- 0.226516 | [1.462676, 1.674699] | 20 |
| +structure-aware decoupled KLA | E-OSPA | 2.394398 +/- 0.089133 | [2.352683, 2.436113] | 20 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 20 |
| +structure-aware decoupled KLA | RMSE | 1.599062 +/- 0.060202 | [1.570887, 1.627237] | 20 |
| +structure-aware decoupled KLA | CardErr | 0.712625 +/- 0.076198 | [0.676963, 0.748287] | 20 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.501281 +/- 0.079082 | [0.464270, 0.538292] | 17.31% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 8/20 | 0.5034 |
| +structure-aware decoupled KLA | RMSE | 0.021667 +/- 0.033377 | [0.006047, 0.037288] | 1.34% | 16/20 | 0.01182 |
| +structure-aware decoupled KLA | CardErr | 0.856062 +/- 0.201443 | [0.761785, 0.950340] | 54.57% | 20/20 | 1.907e-06 |
