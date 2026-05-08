# GA Ideal Communication Comparison (2026-05-07 01:06:11)

Comparison order: ordinary GA -> structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6]
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
| 1 | 2 | ordinary GA | 1.748653 | 1.413558 | 0.163750 |
| 1 | 2 | structure-aware decoupled KLA | 1.529450 | 1.323827 | 0.133750 |
| 2 | 3 | ordinary GA | 1.668612 | 1.599501 | 0.148750 |
| 2 | 3 | structure-aware decoupled KLA | 1.430569 | 1.202858 | 0.105000 |
| 3 | 4 | ordinary GA | 1.687680 | 1.482259 | 0.156250 |
| 3 | 4 | structure-aware decoupled KLA | 1.460573 | 1.189964 | 0.123750 |
| 4 | 5 | ordinary GA | 1.747289 | 1.533046 | 0.180000 |
| 4 | 5 | structure-aware decoupled KLA | 1.533769 | 1.219654 | 0.152500 |
| 5 | 6 | ordinary GA | 1.675510 | 1.601136 | 0.153750 |
| 5 | 6 | structure-aware decoupled KLA | 1.518010 | 1.511913 | 0.180000 |

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.061 | 1.897 | 0.500 | 0.500 | 1.460 | 1.376 |
| 2 | 2.041 | 1.928 | 0.500 | 0.500 | 1.445 | 1.370 |
| 3 | 1.907 | 1.908 | 0.500 | 0.500 | 1.411 | 1.345 |
| 4 | 1.898 | 1.872 | 0.500 | 0.500 | 1.438 | 1.360 |
| 5 | 1.948 | 1.878 | 0.500 | 0.500 | 1.457 | 1.383 |
| 6 | 1.919 | 1.851 | 0.500 | 0.500 | 1.444 | 1.372 |
| 7 | 1.911 | 1.843 | 0.500 | 0.500 | 1.425 | 1.367 |
| 8 | 1.912 | 1.839 | 0.500 | 0.500 | 1.455 | 1.382 |

## Aggregated Local Metrics (mean across sensors and trials)
- E-OSPA: 1.950 -> 1.877
- H-OSPA: 0.500 -> 0.500
- RMSE: 1.442 -> 1.369

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.706 -> 1.494
- Position (RMSE): 1.526 -> 1.290
- Cardinality: 0.161 -> 0.139

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | OSPA | 1.705549 +/- 0.039326 | [1.656727, 1.754371] | 5 |
| structure-aware decoupled KLA | OSPA | 1.494474 +/- 0.046245 | [1.437062, 1.551886] | 5 |
| ordinary GA | RMSE | 1.525900 +/- 0.080084 | [1.426479, 1.625322] | 5 |
| structure-aware decoupled KLA | RMSE | 1.289643 +/- 0.135036 | [1.122001, 1.457285] | 5 |
| ordinary GA | Cardinality | 0.160500 +/- 0.012171 | [0.145391, 0.175609] | 5 |
| structure-aware decoupled KLA | Cardinality | 0.139000 +/- 0.028633 | [0.103453, 0.174547] | 5 |

## Paired Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | OSPA | 0.211075 +/- 0.031332 | [0.172178, 0.249972] | 12.38% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | RMSE | 0.236257 +/- 0.139555 | [0.063004, 0.409510] | 15.48% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | Cardinality | 0.021500 +/- 0.027406 | [-0.012524, 0.055524] | 13.40% | 4/5 | 0.375 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | E-OSPA | 1.949511 +/- 0.053419 | [1.883194, 2.015829] | 5 |
| structure-aware decoupled KLA | E-OSPA | 1.876801 +/- 0.063668 | [1.797760, 1.955843] | 5 |
| ordinary GA | H-OSPA | 0.499996 +/- 0.000004 | [0.499991, 0.500001] | 5 |
| structure-aware decoupled KLA | H-OSPA | 0.499994 +/- 0.000004 | [0.499989, 0.499998] | 5 |
| ordinary GA | RMSE | 1.441872 +/- 0.027975 | [1.407142, 1.476602] | 5 |
| structure-aware decoupled KLA | RMSE | 1.369361 +/- 0.030997 | [1.330879, 1.407843] | 5 |

## Paired Local-Metric Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | E-OSPA | 0.072710 +/- 0.017300 | [0.051233, 0.094187] | 3.73% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | H-OSPA | 0.000003 +/- 0.000001 | [0.000001, 0.000004] | 0.00% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | RMSE | 0.072511 +/- 0.006478 | [0.064469, 0.080554] | 5.03% | 5/5 | 0.0625 |
