# GA Ideal Communication Comparison (2026-05-07 01:06:17)

Comparison order: ordinary GA -> structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
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
| 1 | 12 | ordinary GA | 1.651846 | 1.407507 | 0.135000 |
| 1 | 12 | structure-aware decoupled KLA | 1.429862 | 1.221540 | 0.097500 |
| 2 | 13 | ordinary GA | 1.670232 | 1.500101 | 0.180000 |
| 2 | 13 | structure-aware decoupled KLA | 1.455867 | 1.174099 | 0.151250 |
| 3 | 14 | ordinary GA | 1.785560 | 1.726749 | 0.170000 |
| 3 | 14 | structure-aware decoupled KLA | 1.550565 | 1.272314 | 0.155000 |
| 4 | 15 | ordinary GA | 1.663041 | 1.538469 | 0.146250 |
| 4 | 15 | structure-aware decoupled KLA | 1.460064 | 1.191457 | 0.120000 |
| 5 | 16 | ordinary GA | 1.718071 | 1.737580 | 0.176250 |
| 5 | 16 | structure-aware decoupled KLA | 1.499582 | 1.231569 | 0.148750 |

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.139 | 1.963 | 0.500 | 0.500 | 1.477 | 1.373 |
| 2 | 2.098 | 1.974 | 0.500 | 0.500 | 1.456 | 1.374 |
| 3 | 2.002 | 1.994 | 0.500 | 0.500 | 1.450 | 1.388 |
| 4 | 1.989 | 1.934 | 0.500 | 0.500 | 1.444 | 1.366 |
| 5 | 1.941 | 1.871 | 0.500 | 0.500 | 1.435 | 1.374 |
| 6 | 1.922 | 1.822 | 0.500 | 0.500 | 1.477 | 1.383 |
| 7 | 1.895 | 1.818 | 0.500 | 0.500 | 1.424 | 1.359 |
| 8 | 1.861 | 1.832 | 0.500 | 0.500 | 1.412 | 1.348 |

## Aggregated Local Metrics (mean across sensors and trials)
- E-OSPA: 1.981 -> 1.901
- H-OSPA: 0.500 -> 0.500
- RMSE: 1.447 -> 1.371

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.698 -> 1.479
- Position (RMSE): 1.582 -> 1.218
- Cardinality: 0.162 -> 0.135

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | OSPA | 1.697750 +/- 0.055210 | [1.629208, 1.766292] | 5 |
| structure-aware decoupled KLA | OSPA | 1.479188 +/- 0.047046 | [1.420782, 1.537593] | 5 |
| ordinary GA | RMSE | 1.582081 +/- 0.145093 | [1.401954, 1.762209] | 5 |
| structure-aware decoupled KLA | RMSE | 1.218196 +/- 0.038008 | [1.171011, 1.265381] | 5 |
| ordinary GA | Cardinality | 0.161500 +/- 0.019792 | [0.136929, 0.186071] | 5 |
| structure-aware decoupled KLA | Cardinality | 0.134500 +/- 0.024915 | [0.103568, 0.165432] | 5 |

## Paired Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | OSPA | 0.218562 +/- 0.011644 | [0.204107, 0.233017] | 12.87% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | RMSE | 0.363886 +/- 0.124272 | [0.209607, 0.518164] | 23.00% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | Cardinality | 0.027000 +/- 0.008033 | [0.017027, 0.036973] | 16.72% | 5/5 | 0.0625 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | E-OSPA | 1.980911 +/- 0.043529 | [1.926871, 2.034951] | 5 |
| structure-aware decoupled KLA | E-OSPA | 1.901139 +/- 0.048604 | [1.840799, 1.961479] | 5 |
| ordinary GA | H-OSPA | 0.499995 +/- 0.000003 | [0.499991, 0.499999] | 5 |
| structure-aware decoupled KLA | H-OSPA | 0.499994 +/- 0.000001 | [0.499992, 0.499995] | 5 |
| ordinary GA | RMSE | 1.446693 +/- 0.045807 | [1.389825, 1.503560] | 5 |
| structure-aware decoupled KLA | RMSE | 1.370521 +/- 0.046347 | [1.312983, 1.428059] | 5 |

## Paired Local-Metric Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | E-OSPA | 0.079772 +/- 0.006538 | [0.071655, 0.087889] | 4.03% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | H-OSPA | 0.000001 +/- 0.000002 | [-0.000002, 0.000004] | 0.00% | 4/5 | 0.375 |
| structure-aware decoupled KLA | RMSE | 0.076171 +/- 0.010360 | [0.063309, 0.089033] | 5.27% | 5/5 | 0.0625 |
