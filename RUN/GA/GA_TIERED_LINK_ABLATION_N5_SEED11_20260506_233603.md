# GA Tiered Link Ablation (2026-05-06 23:36:03)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: structureAwareDecoupledKla

## Arm Configs
### fixed weights
- enabled: 0
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250

### +structure-aware decoupled KLA
- enabled: 1
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 2: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 3: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 4: [0.1 0.5 0 0.2 0.5 0.1 0.1 0.1]
- Trial 5: [0 0.1 0.5 0.1 0.1 0.2 0.1 0.5]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 12 | fixed weights | 2.235333 | 2.348212 | 0.482500 |
| 1 | 12 | +structure-aware decoupled KLA | 1.797235 | 1.885002 | 0.225000 |
| 2 | 13 | fixed weights | 2.380325 | 2.213801 | 0.656250 |
| 2 | 13 | +structure-aware decoupled KLA | 1.786939 | 1.575583 | 0.281250 |
| 3 | 14 | fixed weights | 2.363292 | 2.665071 | 0.552500 |
| 3 | 14 | +structure-aware decoupled KLA | 1.850337 | 1.788203 | 0.222500 |
| 4 | 15 | fixed weights | 2.147298 | 2.230920 | 0.382500 |
| 4 | 15 | +structure-aware decoupled KLA | 1.798971 | 1.592700 | 0.247500 |
| 5 | 16 | fixed weights | 2.048554 | 2.956503 | 0.333750 |
| 5 | 16 | +structure-aware decoupled KLA | 1.765436 | 2.136723 | 0.210000 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.234960 | 2.482901 | 0.481500 |
| +structure-aware decoupled KLA | 1.799783 | 1.795642 | 0.237250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.234960 +/- 0.141450 | [2.059355, 2.410565] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.799783 +/- 0.031255 | [1.760981, 1.838586] | 5 |
| fixed weights | RMSE | 2.482901 +/- 0.320755 | [2.084696, 2.881107] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.795642 +/- 0.231294 | [1.508498, 2.082786] | 5 |
| fixed weights | Cardinality | 0.481500 +/- 0.129625 | [0.320575, 0.642425] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.237250 +/- 0.028068 | [0.202405, 0.272095] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.435177 +/- 0.124251 | [0.280924, 0.589430] | 19.47% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.687259 +/- 0.164712 | [0.482774, 0.891744] | 27.68% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.244250 +/- 0.113004 | [0.103959, 0.384541] | 50.73% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.809376 | 0.500000 | 1.606183 | 1.347500 |
| +structure-aware decoupled KLA | 2.380050 | 0.500000 | 1.580965 | 0.700250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.809376 +/- 0.101830 | [2.682958, 2.935794] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.380050 +/- 0.098939 | [2.257221, 2.502879] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.606183 +/- 0.079226 | [1.507827, 1.704539] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.580965 +/- 0.066187 | [1.498796, 1.663134] | 5 |
| fixed weights | CardErr | 1.347500 +/- 0.164609 | [1.143144, 1.551856] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.700250 +/- 0.071874 | [0.611020, 0.789480] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.429325 +/- 0.064107 | [0.349739, 0.508912] | 15.28% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | RMSE | 0.025218 +/- 0.014664 | [0.007014, 0.043423] | 1.57% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | CardErr | 0.647250 +/- 0.132772 | [0.482418, 0.812082] | 48.03% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.672151 | 0.500000 | 1.546066 | 1.090000 |
| 1 | +structure-aware decoupled KLA | 2.300713 | 0.500000 | 1.508750 | 0.672000 |
| 2 | fixed weights | 2.709157 | 0.500000 | 1.544048 | 1.046000 |
| 2 | +structure-aware decoupled KLA | 2.417000 | 0.500000 | 1.580313 | 0.756000 |
| 3 | fixed weights | 2.590826 | 0.500000 | 1.545617 | 1.008000 |
| 3 | +structure-aware decoupled KLA | 2.358253 | 0.500000 | 1.532769 | 0.752000 |
| 4 | fixed weights | 2.925962 | 0.500000 | 1.612146 | 1.402000 |
| 4 | +structure-aware decoupled KLA | 2.438183 | 0.500000 | 1.589868 | 0.824000 |
| 5 | fixed weights | 2.759220 | 0.500000 | 1.639050 | 1.332000 |
| 5 | +structure-aware decoupled KLA | 2.435773 | 0.500000 | 1.635512 | 0.798000 |
| 6 | fixed weights | 2.844265 | 0.500000 | 1.667184 | 1.290000 |
| 6 | +structure-aware decoupled KLA | 2.342740 | 0.500000 | 1.579546 | 0.540000 |
| 7 | fixed weights | 2.764006 | 0.500000 | 1.628361 | 1.256000 |
| 7 | +structure-aware decoupled KLA | 2.383606 | 0.500000 | 1.627634 | 0.676000 |
| 8 | fixed weights | 3.209419 | 0.500000 | 1.666992 | 2.356000 |
| 8 | +structure-aware decoupled KLA | 2.364135 | 0.499998 | 1.593326 | 0.584000 |
