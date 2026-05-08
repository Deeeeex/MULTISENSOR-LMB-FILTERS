# GA Ideal Communication Comparison (2026-05-07 01:06:09)

Comparison order: ordinary GA -> structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 16 (fixed=1)
- trialSeeds: [17 18 19 20 21]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- comm level: 0
- linkModel: fixed
- pDrop target mean: 0.000
- adaptive useDecoupledKla: 1
- adaptive useStructureAwareKla: 1
- adaptive usePosteriorStructureConsistency: 0
- adaptive spatialStructureStrength: 0.450
- adaptive existenceStructureStrength: 0.080
- adaptive structureReliabilityPower: 0.300

## Mean pDropBySensor Across Trials
- [0 0 0 0 0 0 0 0]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 17 | ordinary GA | 1.684848 | 1.773106 | 0.162500 |
| 1 | 17 | structure-aware decoupled KLA | 1.505622 | 1.374197 | 0.163750 |
| 2 | 18 | ordinary GA | 1.720182 | 1.661495 | 0.156250 |
| 2 | 18 | structure-aware decoupled KLA | 1.494448 | 1.227218 | 0.141250 |
| 3 | 19 | ordinary GA | 1.776884 | 1.512271 | 0.183750 |
| 3 | 19 | structure-aware decoupled KLA | 1.569992 | 1.219516 | 0.167500 |
| 4 | 20 | ordinary GA | 1.675784 | 1.338376 | 0.156250 |
| 4 | 20 | structure-aware decoupled KLA | 1.459963 | 1.172760 | 0.097500 |
| 5 | 21 | ordinary GA | 1.732708 | 1.402462 | 0.158750 |
| 5 | 21 | structure-aware decoupled KLA | 1.479245 | 1.197196 | 0.116250 |

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.149 | 1.991 | 0.500 | 0.500 | 1.492 | 1.398 |
| 2 | 2.103 | 1.940 | 0.500 | 0.500 | 1.472 | 1.374 |
| 3 | 2.039 | 2.007 | 0.500 | 0.500 | 1.457 | 1.394 |
| 4 | 1.952 | 1.959 | 0.500 | 0.500 | 1.456 | 1.381 |
| 5 | 1.911 | 1.850 | 0.500 | 0.500 | 1.427 | 1.367 |
| 6 | 1.860 | 1.797 | 0.500 | 0.500 | 1.412 | 1.347 |
| 7 | 1.909 | 1.830 | 0.500 | 0.500 | 1.423 | 1.365 |
| 8 | 1.857 | 1.800 | 0.500 | 0.500 | 1.430 | 1.358 |

## Aggregated Local Metrics (mean across sensors and trials)
- E-OSPA: 1.972 -> 1.897
- H-OSPA: 0.500 -> 0.500
- RMSE: 1.446 -> 1.373

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.718 -> 1.502
- Position (RMSE): 1.538 -> 1.238
- Cardinality: 0.164 -> 0.137

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | OSPA | 1.718081 +/- 0.040526 | [1.667769, 1.768393] | 5 |
| structure-aware decoupled KLA | OSPA | 1.501854 +/- 0.041767 | [1.450002, 1.553706] | 5 |
| ordinary GA | RMSE | 1.537542 +/- 0.179866 | [1.314245, 1.760839] | 5 |
| structure-aware decoupled KLA | RMSE | 1.238178 +/- 0.078944 | [1.140171, 1.336184] | 5 |
| ordinary GA | Cardinality | 0.163500 +/- 0.011605 | [0.149092, 0.177908] | 5 |
| structure-aware decoupled KLA | Cardinality | 0.137250 +/- 0.030226 | [0.099726, 0.174774] | 5 |

## Paired Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | OSPA | 0.216227 +/- 0.027080 | [0.182609, 0.249846] | 12.59% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | RMSE | 0.299364 +/- 0.117151 | [0.153926, 0.444803] | 19.47% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | Cardinality | 0.026250 +/- 0.023995 | [-0.003540, 0.056040] | 16.06% | 4/5 | 0.375 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | E-OSPA | 1.972432 +/- 0.030162 | [1.934988, 2.009877] | 5 |
| structure-aware decoupled KLA | E-OSPA | 1.896734 +/- 0.031210 | [1.857988, 1.935481] | 5 |
| ordinary GA | H-OSPA | 0.499996 +/- 0.000002 | [0.499993, 0.499999] | 5 |
| structure-aware decoupled KLA | H-OSPA | 0.499994 +/- 0.000003 | [0.499991, 0.499997] | 5 |
| ordinary GA | RMSE | 1.446106 +/- 0.023107 | [1.417419, 1.474793] | 5 |
| structure-aware decoupled KLA | RMSE | 1.372873 +/- 0.022857 | [1.344497, 1.401249] | 5 |

## Paired Local-Metric Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | E-OSPA | 0.075698 +/- 0.018918 | [0.052211, 0.099185] | 3.84% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | H-OSPA | 0.000002 +/- 0.000003 | [-0.000002, 0.000006] | 0.00% | 3/5 | 1 |
| structure-aware decoupled KLA | RMSE | 0.073233 +/- 0.006345 | [0.065356, 0.081110] | 5.06% | 5/5 | 0.0625 |
