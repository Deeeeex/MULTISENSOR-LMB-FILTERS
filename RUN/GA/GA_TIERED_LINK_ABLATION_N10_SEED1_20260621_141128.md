# GA Tiered Link Ablation (2026-06-21 14:11:28)

Comparison order: +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 10
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: fidFiaExistenceRefinement

## Arm Configs
### +structure-aware decoupled KLA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### +FID-FIA existence refinement
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 1
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.5 0.2]
- Trial 3: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0 0.1 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 5: [0.1 0.1 0.1 0.5 0.2 0 0.1 0.5]
- Trial 6: [0.1 0.2 0.1 0.5 0.1 0.5 0.1 0]
- Trial 7: [0.1 0 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 8: [0 0.1 0.5 0.1 0.1 0.2 0.5 0.1]
- Trial 9: [0.5 0.1 0.1 0 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0 0.5 0.1 0.2 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | +structure-aware decoupled KLA | 1.786737 | 1.505206 | 0.091250 |
| 1 | 2 | +FID-FIA existence refinement | 1.864203 | 1.916324 | 0.078750 |
| 2 | 3 | +structure-aware decoupled KLA | 1.737537 | 1.665656 | 0.068750 |
| 2 | 3 | +FID-FIA existence refinement | 1.763895 | 2.076323 | 0.062500 |
| 3 | 4 | +structure-aware decoupled KLA | 1.709809 | 1.403508 | 0.067500 |
| 3 | 4 | +FID-FIA existence refinement | 1.849800 | 2.159309 | 0.082500 |
| 4 | 5 | +structure-aware decoupled KLA | 1.721621 | 1.382597 | 0.082500 |
| 4 | 5 | +FID-FIA existence refinement | 1.731440 | 1.720011 | 0.077500 |
| 5 | 6 | +structure-aware decoupled KLA | 1.635518 | 1.402009 | 0.063750 |
| 5 | 6 | +FID-FIA existence refinement | 1.719365 | 1.673258 | 0.053750 |
| 6 | 7 | +structure-aware decoupled KLA | 1.629848 | 1.346446 | 0.083750 |
| 6 | 7 | +FID-FIA existence refinement | 1.709856 | 1.632654 | 0.060000 |
| 7 | 8 | +structure-aware decoupled KLA | 1.874411 | 1.628423 | 0.091250 |
| 7 | 8 | +FID-FIA existence refinement | 1.832476 | 1.847821 | 0.077500 |
| 8 | 9 | +structure-aware decoupled KLA | 1.679424 | 1.366166 | 0.091250 |
| 8 | 9 | +FID-FIA existence refinement | 1.777212 | 1.551432 | 0.088750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.723533 | 1.539905 | 0.095000 |
| 9 | 10 | +FID-FIA existence refinement | 1.780593 | 1.639807 | 0.065000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.823001 | 1.440625 | 0.095000 |
| 10 | 11 | +FID-FIA existence refinement | 1.991834 | 2.037224 | 0.108750 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| +structure-aware decoupled KLA | 1.732144 | 1.468054 | 0.083000 |
| +FID-FIA existence refinement | 1.802067 | 1.825416 | 0.075500 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | OSPA | 1.732144 +/- 0.078009 | [1.676344, 1.787944] | 10 |
| +FID-FIA existence refinement | OSPA | 1.802067 +/- 0.085597 | [1.740839, 1.863296] | 10 |
| +structure-aware decoupled KLA | RMSE | 1.468054 +/- 0.112082 | [1.387881, 1.548227] | 10 |
| +FID-FIA existence refinement | RMSE | 1.825416 +/- 0.213178 | [1.672928, 1.977905] | 10 |
| +structure-aware decoupled KLA | Cardinality | 0.083000 +/- 0.012050 | [0.074380, 0.091620] | 10 |
| +FID-FIA existence refinement | Cardinality | 0.075500 +/- 0.016129 | [0.063963, 0.087037] | 10 |

## Paired Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | OSPA | -0.069923 +/- 0.061472 | [-0.113895, -0.025952] | -4.04% | 1/10 | 0.02148 |
| +FID-FIA existence refinement | RMSE | -0.357362 +/- 0.197071 | [-0.498329, -0.216396] | -24.34% | 0/10 | 0.001953 |
| +FID-FIA existence refinement | Cardinality | 0.007500 +/- 0.014264 | [-0.002703, 0.017703] | 9.04% | 8/10 | 0.1094 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| +structure-aware decoupled KLA | 46.982082 +/- 2.757103 | 1.957587 | 1.000x | 10 |
| +FID-FIA existence refinement | 101.721166 +/- 5.620379 | 4.238382 | 2.167x | 10 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +FID-FIA existence refinement | 54.739085 +/- 3.983010 | 116.67% | 10/10 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | +structure-aware decoupled KLA | 54.343242 | 2.264302 | 1.000x |
| 1 | 2 | +FID-FIA existence refinement | 110.916368 | 4.621515 | 2.041x |
| 2 | 3 | +structure-aware decoupled KLA | 47.163731 | 1.965155 | 1.000x |
| 2 | 3 | +FID-FIA existence refinement | 103.096019 | 4.295667 | 2.186x |
| 3 | 4 | +structure-aware decoupled KLA | 45.987605 | 1.916150 | 1.000x |
| 3 | 4 | +FID-FIA existence refinement | 100.540673 | 4.189195 | 2.186x |
| 4 | 5 | +structure-aware decoupled KLA | 45.986603 | 1.916108 | 1.000x |
| 4 | 5 | +FID-FIA existence refinement | 101.868601 | 4.244525 | 2.215x |
| 5 | 6 | +structure-aware decoupled KLA | 45.017387 | 1.875724 | 1.000x |
| 5 | 6 | +FID-FIA existence refinement | 92.506031 | 3.854418 | 2.055x |
| 6 | 7 | +structure-aware decoupled KLA | 47.184293 | 1.966012 | 1.000x |
| 6 | 7 | +FID-FIA existence refinement | 109.186698 | 4.549446 | 2.314x |
| 7 | 8 | +structure-aware decoupled KLA | 45.961559 | 1.915065 | 1.000x |
| 7 | 8 | +FID-FIA existence refinement | 103.685689 | 4.320237 | 2.256x |
| 8 | 9 | +structure-aware decoupled KLA | 47.430775 | 1.976282 | 1.000x |
| 8 | 9 | +FID-FIA existence refinement | 100.561943 | 4.190081 | 2.120x |
| 9 | 10 | +structure-aware decoupled KLA | 46.345360 | 1.931057 | 1.000x |
| 9 | 10 | +FID-FIA existence refinement | 100.038970 | 4.168290 | 2.159x |
| 10 | 11 | +structure-aware decoupled KLA | 44.400262 | 1.850011 | 1.000x |
| 10 | 11 | +FID-FIA existence refinement | 94.810673 | 3.950445 | 2.135x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| +structure-aware decoupled KLA | 2.197304 | 4.223026 | 0.212000 |
| +FID-FIA existence refinement | 2.133102 | 4.511292 | 0.149500 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | E-OSPA | 2.197304 +/- 0.058597 | [2.155389, 2.239219] | 10 |
| +FID-FIA existence refinement | E-OSPA | 2.133102 +/- 0.078411 | [2.077015, 2.189190] | 10 |
| +structure-aware decoupled KLA | RMSE | 4.223026 +/- 0.099020 | [4.152196, 4.293856] | 10 |
| +FID-FIA existence refinement | RMSE | 4.511292 +/- 0.312846 | [4.287511, 4.735073] | 10 |
| +structure-aware decoupled KLA | CardErr | 0.212000 +/- 0.028554 | [0.191575, 0.232425] | 10 |
| +FID-FIA existence refinement | CardErr | 0.149500 +/- 0.015581 | [0.138355, 0.160645] | 10 |

## Paired Local-Metric Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | E-OSPA | 0.064202 +/- 0.053410 | [0.025997, 0.102406] | 2.92% | 9/10 | 0.02148 |
| +FID-FIA existence refinement | RMSE | -0.288267 +/- 0.384194 | [-0.563083, -0.013450] | -6.83% | 1/10 | 0.02148 |
| +FID-FIA existence refinement | CardErr | 0.062500 +/- 0.029226 | [0.041594, 0.083406] | 29.48% | 10/10 | 0.001953 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | +structure-aware decoupled KLA | 2.243447 | 4.215836 | 0.227000 |
| 1 | +FID-FIA existence refinement | 2.185379 | 4.566460 | 0.159000 |
| 2 | +structure-aware decoupled KLA | 2.322767 | 4.472656 | 0.238000 |
| 2 | +FID-FIA existence refinement | 2.228587 | 4.563147 | 0.161000 |
| 3 | +structure-aware decoupled KLA | 2.261844 | 4.234666 | 0.232000 |
| 3 | +FID-FIA existence refinement | 2.179712 | 4.469651 | 0.165000 |
| 4 | +structure-aware decoupled KLA | 2.219340 | 4.187835 | 0.216000 |
| 4 | +FID-FIA existence refinement | 2.183045 | 4.345498 | 0.173000 |
| 5 | +structure-aware decoupled KLA | 2.161802 | 4.235340 | 0.205000 |
| 5 | +FID-FIA existence refinement | 2.114116 | 4.622423 | 0.151000 |
| 6 | +structure-aware decoupled KLA | 2.089218 | 3.987445 | 0.196000 |
| 6 | +FID-FIA existence refinement | 2.029017 | 4.305154 | 0.122000 |
| 7 | +structure-aware decoupled KLA | 2.146442 | 4.201511 | 0.197000 |
| 7 | +FID-FIA existence refinement | 2.113560 | 4.869390 | 0.141000 |
| 8 | +structure-aware decoupled KLA | 2.133572 | 4.248916 | 0.185000 |
| 8 | +FID-FIA existence refinement | 2.031402 | 4.348616 | 0.124000 |
