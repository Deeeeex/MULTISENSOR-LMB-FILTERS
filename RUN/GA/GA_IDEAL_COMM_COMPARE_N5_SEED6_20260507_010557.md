# GA Ideal Communication Comparison (2026-05-07 01:05:57)

Comparison order: ordinary GA -> structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 6 (fixed=1)
- trialSeeds: [7 8 9 10 11]
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
| 1 | 7 | ordinary GA | 1.682324 | 1.372530 | 0.161250 |
| 1 | 7 | structure-aware decoupled KLA | 1.428726 | 1.186161 | 0.113750 |
| 2 | 8 | ordinary GA | 1.674547 | 1.583875 | 0.161250 |
| 2 | 8 | structure-aware decoupled KLA | 1.433605 | 1.188096 | 0.098750 |
| 3 | 9 | ordinary GA | 1.699581 | 1.440882 | 0.178750 |
| 3 | 9 | structure-aware decoupled KLA | 1.487433 | 1.217020 | 0.168750 |
| 4 | 10 | ordinary GA | 1.655732 | 1.450167 | 0.130000 |
| 4 | 10 | structure-aware decoupled KLA | 1.412383 | 1.211172 | 0.085000 |
| 5 | 11 | ordinary GA | 1.750999 | 1.574666 | 0.171250 |
| 5 | 11 | structure-aware decoupled KLA | 1.495216 | 1.232282 | 0.123750 |

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.114 | 1.925 | 0.500 | 0.500 | 1.478 | 1.381 |
| 2 | 2.060 | 1.937 | 0.500 | 0.500 | 1.430 | 1.379 |
| 3 | 1.985 | 1.918 | 0.500 | 0.500 | 1.443 | 1.362 |
| 4 | 1.931 | 1.920 | 0.500 | 0.500 | 1.443 | 1.369 |
| 5 | 1.921 | 1.832 | 0.500 | 0.500 | 1.444 | 1.367 |
| 6 | 1.857 | 1.800 | 0.500 | 0.500 | 1.425 | 1.370 |
| 7 | 1.863 | 1.806 | 0.500 | 0.500 | 1.449 | 1.388 |
| 8 | 1.857 | 1.794 | 0.500 | 0.500 | 1.436 | 1.370 |

## Aggregated Local Metrics (mean across sensors and trials)
- E-OSPA: 1.949 -> 1.866
- H-OSPA: 0.500 -> 0.500
- RMSE: 1.443 -> 1.373

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.693 -> 1.451
- Position (RMSE): 1.484 -> 1.207
- Cardinality: 0.161 -> 0.118

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | OSPA | 1.692637 +/- 0.036228 | [1.647661, 1.737612] | 5 |
| structure-aware decoupled KLA | OSPA | 1.451473 +/- 0.037321 | [1.405141, 1.497805] | 5 |
| ordinary GA | RMSE | 1.484424 +/- 0.091684 | [1.370602, 1.598247] | 5 |
| structure-aware decoupled KLA | RMSE | 1.206946 +/- 0.019676 | [1.182519, 1.231373] | 5 |
| ordinary GA | Cardinality | 0.160500 +/- 0.018574 | [0.137441, 0.183559] | 5 |
| structure-aware decoupled KLA | Cardinality | 0.118000 +/- 0.031962 | [0.078320, 0.157680] | 5 |

## Paired Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | OSPA | 0.241164 +/- 0.017429 | [0.219527, 0.262801] | 14.25% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | RMSE | 0.277478 +/- 0.087841 | [0.168426, 0.386530] | 18.69% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | Cardinality | 0.042500 +/- 0.019445 | [0.018359, 0.066641] | 26.48% | 5/5 | 0.0625 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| ordinary GA | E-OSPA | 1.948523 +/- 0.059655 | [1.874464, 2.022583] | 5 |
| structure-aware decoupled KLA | E-OSPA | 1.866326 +/- 0.068282 | [1.781557, 1.951096] | 5 |
| ordinary GA | H-OSPA | 0.499995 +/- 0.000002 | [0.499992, 0.499998] | 5 |
| structure-aware decoupled KLA | H-OSPA | 0.499992 +/- 0.000003 | [0.499988, 0.499996] | 5 |
| ordinary GA | RMSE | 1.443396 +/- 0.043231 | [1.389726, 1.497066] | 5 |
| structure-aware decoupled KLA | RMSE | 1.373249 +/- 0.048813 | [1.312649, 1.433849] | 5 |

## Paired Local-Metric Improvements Relative to Ordinary GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| structure-aware decoupled KLA | E-OSPA | 0.082197 +/- 0.011939 | [0.067375, 0.097018] | 4.22% | 5/5 | 0.0625 |
| structure-aware decoupled KLA | H-OSPA | 0.000003 +/- 0.000003 | [-0.000001, 0.000007] | 0.00% | 4/5 | 0.375 |
| structure-aware decoupled KLA | RMSE | 0.070147 +/- 0.006924 | [0.061551, 0.078744] | 4.86% | 5/5 | 0.0625 |
