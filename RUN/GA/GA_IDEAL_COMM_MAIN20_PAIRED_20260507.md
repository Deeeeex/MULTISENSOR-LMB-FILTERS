# GA Ideal Communication Main Paired Confirmation (20 trials)
Generated: 2026-05-07 01:08:13
Source reports:
- `RUN/GA/GA_IDEAL_COMM_COMPARE_N5_SEED1_20260507_010611.md`
- `RUN/GA/GA_IDEAL_COMM_COMPARE_N5_SEED6_20260507_010557.md`
- `RUN/GA/GA_IDEAL_COMM_COMPARE_N5_SEED11_20260507_010617.md`
- `RUN/GA/GA_IDEAL_COMM_COMPARE_N5_SEED16_20260507_010609.md`

## Run Config
- Arms: ordinary GA vs structure-aware decoupled KLA
- Trial seeds: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21
- Communication level: 0 (ideal communication)

## Per-Trial Consensus Metrics
| Seed | Arm | OSPA | RMSE | Cardinality |
|-----:|:----|-----:|-----:|------------:|
| 2 | ordinary GA | 1.748653 | 1.413558 | 0.163750 |
| 2 | structure-aware decoupled KLA | 1.529450 | 1.323827 | 0.133750 |
| 3 | ordinary GA | 1.668612 | 1.599501 | 0.148750 |
| 3 | structure-aware decoupled KLA | 1.430569 | 1.202858 | 0.105000 |
| 4 | ordinary GA | 1.687680 | 1.482259 | 0.156250 |
| 4 | structure-aware decoupled KLA | 1.460573 | 1.189964 | 0.123750 |
| 5 | ordinary GA | 1.747289 | 1.533046 | 0.180000 |
| 5 | structure-aware decoupled KLA | 1.533769 | 1.219654 | 0.152500 |
| 6 | ordinary GA | 1.675510 | 1.601136 | 0.153750 |
| 6 | structure-aware decoupled KLA | 1.518010 | 1.511913 | 0.180000 |
| 7 | ordinary GA | 1.682324 | 1.372530 | 0.161250 |
| 7 | structure-aware decoupled KLA | 1.428726 | 1.186161 | 0.113750 |
| 8 | ordinary GA | 1.674547 | 1.583875 | 0.161250 |
| 8 | structure-aware decoupled KLA | 1.433605 | 1.188096 | 0.098750 |
| 9 | ordinary GA | 1.699581 | 1.440882 | 0.178750 |
| 9 | structure-aware decoupled KLA | 1.487433 | 1.217020 | 0.168750 |
| 10 | ordinary GA | 1.655732 | 1.450167 | 0.130000 |
| 10 | structure-aware decoupled KLA | 1.412383 | 1.211172 | 0.085000 |
| 11 | ordinary GA | 1.750999 | 1.574666 | 0.171250 |
| 11 | structure-aware decoupled KLA | 1.495216 | 1.232282 | 0.123750 |
| 12 | ordinary GA | 1.651846 | 1.407507 | 0.135000 |
| 12 | structure-aware decoupled KLA | 1.429862 | 1.221540 | 0.097500 |
| 13 | ordinary GA | 1.670232 | 1.500101 | 0.180000 |
| 13 | structure-aware decoupled KLA | 1.455867 | 1.174099 | 0.151250 |
| 14 | ordinary GA | 1.785560 | 1.726749 | 0.170000 |
| 14 | structure-aware decoupled KLA | 1.550565 | 1.272314 | 0.155000 |
| 15 | ordinary GA | 1.663041 | 1.538469 | 0.146250 |
| 15 | structure-aware decoupled KLA | 1.460064 | 1.191457 | 0.120000 |
| 16 | ordinary GA | 1.718071 | 1.737580 | 0.176250 |
| 16 | structure-aware decoupled KLA | 1.499582 | 1.231569 | 0.148750 |
| 17 | ordinary GA | 1.684848 | 1.773106 | 0.162500 |
| 17 | structure-aware decoupled KLA | 1.505622 | 1.374197 | 0.163750 |
| 18 | ordinary GA | 1.720182 | 1.661495 | 0.156250 |
| 18 | structure-aware decoupled KLA | 1.494448 | 1.227218 | 0.141250 |
| 19 | ordinary GA | 1.776884 | 1.512271 | 0.183750 |
| 19 | structure-aware decoupled KLA | 1.569992 | 1.219516 | 0.167500 |
| 20 | ordinary GA | 1.675784 | 1.338376 | 0.156250 |
| 20 | structure-aware decoupled KLA | 1.459963 | 1.172760 | 0.097500 |
| 21 | ordinary GA | 1.732708 | 1.402462 | 0.158750 |
| 21 | structure-aware decoupled KLA | 1.479245 | 1.197196 | 0.116250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | OSPA | 1.703504 +/- 0.041063 | [1.684286, 1.722722] | 20 |
| ordinary GA | RMSE | 1.532487 +/- 0.125044 | [1.473965, 1.591009] | 20 |
| ordinary GA | Cardinality | 0.161500 +/- 0.014704 | [0.154618, 0.168382] | 20 |
| structure-aware decoupled KLA | OSPA | 1.481747 +/- 0.044369 | [1.460982, 1.502512] | 20 |
| structure-aware decoupled KLA | RMSE | 1.238241 +/- 0.081211 | [1.200233, 1.276248] | 20 |
| structure-aware decoupled KLA | Cardinality | 0.132187 +/- 0.028000 | [0.119083, 0.145292] | 20 |

## Paired Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | OSPA | 0.221757 +/- 0.024360 | [0.210356, 0.233158] | 13.02% | 20/20 | 1.907e-06 |
| structure-aware decoupled KLA | RMSE | 0.294246 +/- 0.118774 | [0.238659, 0.349833] | 19.20% | 20/20 | 1.907e-06 |
| structure-aware decoupled KLA | Cardinality | 0.029312 +/- 0.020934 | [0.019515, 0.039110] | 18.15% | 18/20 | 0.0004025 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | E-OSPA | 1.962844 +/- 0.046381 | [1.941137, 1.984551] | 20 |
| ordinary GA | H-OSPA | 0.499995 +/- 0.000003 | [0.499994, 0.499997] | 20 |
| ordinary GA | RMSE | 1.444517 +/- 0.033413 | [1.428879, 1.460154] | 20 |
| structure-aware decoupled KLA | E-OSPA | 1.885250 +/- 0.052455 | [1.860701, 1.909799] | 20 |
| structure-aware decoupled KLA | H-OSPA | 0.499993 +/- 0.000003 | [0.499992, 0.499995] | 20 |
| structure-aware decoupled KLA | RMSE | 1.371501 +/- 0.035621 | [1.354830, 1.388172] | 20 |

## Paired Local-Metric Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | E-OSPA | 0.077594 +/- 0.013835 | [0.071119, 0.084069] | 3.95% | 20/20 | 1.907e-06 |
| structure-aware decoupled KLA | H-OSPA | 0.000002 +/- 0.000002 | [0.000001, 0.000003] | 0.00% | 16/20 | 0.01182 |
| structure-aware decoupled KLA | RMSE | 0.073015 +/- 0.007407 | [0.069549, 0.076482] | 5.05% | 20/20 | 1.907e-06 |
