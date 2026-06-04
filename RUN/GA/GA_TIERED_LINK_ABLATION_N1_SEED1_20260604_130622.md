# GA Tiered Link Ablation (2026-06-04 13:06:22)

Comparison order: +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- trialSeeds: 2
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
- useHistorySmoothedExistenceConfidence: 1
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.000
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
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
- useHistorySmoothedExistenceConfidence: 1
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.000
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.050
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
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | +structure-aware decoupled KLA | 1.892554 | 1.597960 | 0.231250 |
| 1 | 2 | +FID-FIA existence refinement | 1.731975 | 1.488560 | 0.065000 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| +structure-aware decoupled KLA | 1.892554 | 1.597960 | 0.231250 |
| +FID-FIA existence refinement | 1.731975 | 1.488560 | 0.065000 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | OSPA | 1.892554 +/- 0.000000 | [1.892554, 1.892554] | 1 |
| +FID-FIA existence refinement | OSPA | 1.731975 +/- 0.000000 | [1.731975, 1.731975] | 1 |
| +structure-aware decoupled KLA | RMSE | 1.597960 +/- 0.000000 | [1.597960, 1.597960] | 1 |
| +FID-FIA existence refinement | RMSE | 1.488560 +/- 0.000000 | [1.488560, 1.488560] | 1 |
| +structure-aware decoupled KLA | Cardinality | 0.231250 +/- 0.000000 | [0.231250, 0.231250] | 1 |
| +FID-FIA existence refinement | Cardinality | 0.065000 +/- 0.000000 | [0.065000, 0.065000] | 1 |

## Paired Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | OSPA | 0.160579 +/- 0.000000 | [0.160579, 0.160579] | 8.48% | 1/1 | 1 |
| +FID-FIA existence refinement | RMSE | 0.109400 +/- 0.000000 | [0.109400, 0.109400] | 6.85% | 1/1 | 1 |
| +FID-FIA existence refinement | Cardinality | 0.166250 +/- 0.000000 | [0.166250, 0.166250] | 71.89% | 1/1 | 1 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| +structure-aware decoupled KLA | 60.159180 +/- 0.000000 | 0.601592 | 1.000x | 1 |
| +FID-FIA existence refinement | 195.960220 +/- 0.000000 | 1.959602 | 3.257x | 1 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +FID-FIA existence refinement | 135.801040 +/- 0.000000 | 225.74% | 1/1 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | +structure-aware decoupled KLA | 60.159180 | 0.601592 | 1.000x |
| 1 | 2 | +FID-FIA existence refinement | 195.960220 | 1.959602 | 3.257x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| +structure-aware decoupled KLA | 2.436326 | 1.703675 | 0.606250 |
| +FID-FIA existence refinement | 2.126870 | 1.731337 | 0.220000 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | E-OSPA | 2.436326 +/- 0.000000 | [2.436326, 2.436326] | 1 |
| +FID-FIA existence refinement | E-OSPA | 2.126870 +/- 0.000000 | [2.126870, 2.126870] | 1 |
| +structure-aware decoupled KLA | RMSE | 1.703675 +/- 0.000000 | [1.703675, 1.703675] | 1 |
| +FID-FIA existence refinement | RMSE | 1.731337 +/- 0.000000 | [1.731337, 1.731337] | 1 |
| +structure-aware decoupled KLA | CardErr | 0.606250 +/- 0.000000 | [0.606250, 0.606250] | 1 |
| +FID-FIA existence refinement | CardErr | 0.220000 +/- 0.000000 | [0.220000, 0.220000] | 1 |

## Paired Local-Metric Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | E-OSPA | 0.309457 +/- 0.000000 | [0.309457, 0.309457] | 12.70% | 1/1 | 1 |
| +FID-FIA existence refinement | RMSE | -0.027661 +/- 0.000000 | [-0.027661, -0.027661] | -1.62% | 0/1 | 1 |
| +FID-FIA existence refinement | CardErr | 0.386250 +/- 0.000000 | [0.386250, 0.386250] | 63.71% | 1/1 | 1 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | +structure-aware decoupled KLA | 2.093015 | 1.548320 | 0.430000 |
| 1 | +FID-FIA existence refinement | 1.905992 | 1.579362 | 0.160000 |
| 2 | +structure-aware decoupled KLA | 2.400899 | 1.696338 | 0.680000 |
| 2 | +FID-FIA existence refinement | 2.077985 | 1.689979 | 0.230000 |
| 3 | +structure-aware decoupled KLA | 2.417228 | 1.567360 | 0.750000 |
| 3 | +FID-FIA existence refinement | 2.019505 | 1.634264 | 0.230000 |
| 4 | +structure-aware decoupled KLA | 2.273062 | 1.597758 | 0.580000 |
| 4 | +FID-FIA existence refinement | 1.986712 | 1.618511 | 0.210000 |
| 5 | +structure-aware decoupled KLA | 2.554417 | 1.806018 | 0.600000 |
| 5 | +FID-FIA existence refinement | 2.256873 | 1.848997 | 0.230000 |
| 6 | +structure-aware decoupled KLA | 2.477667 | 1.786731 | 0.500000 |
| 6 | +FID-FIA existence refinement | 2.231725 | 1.820485 | 0.230000 |
| 7 | +structure-aware decoupled KLA | 2.690523 | 1.851787 | 0.690000 |
| 7 | +FID-FIA existence refinement | 2.258200 | 1.830843 | 0.230000 |
| 8 | +structure-aware decoupled KLA | 2.583799 | 1.775091 | 0.620000 |
| 8 | +FID-FIA existence refinement | 2.277966 | 1.828253 | 0.240000 |
