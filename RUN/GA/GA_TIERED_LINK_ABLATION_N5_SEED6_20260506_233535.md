# GA Tiered Link Ablation (2026-05-06 23:35:35)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 6 (fixed=1)
- trialSeeds: [7 8 9 10 11]
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
- Trial 1: [0 0.2 0.1 0.5 0.1 0.5 0.1 0.1]
- Trial 2: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 3: [0.5 0.1 0.2 0.1 0.5 0.1 0.1 0]
- Trial 4: [0.5 0.1 0 0.1 0.1 0.5 0.2 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.5 0.1 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 7 | fixed weights | 2.541049 | 2.575987 | 0.747500 |
| 1 | 7 | +structure-aware decoupled KLA | 1.823612 | 1.828964 | 0.205000 |
| 2 | 8 | fixed weights | 2.386825 | 2.412256 | 0.563750 |
| 2 | 8 | +structure-aware decoupled KLA | 1.762642 | 1.481461 | 0.185000 |
| 3 | 9 | fixed weights | 2.456704 | 2.596131 | 0.663750 |
| 3 | 9 | +structure-aware decoupled KLA | 1.756928 | 1.546963 | 0.172500 |
| 4 | 10 | fixed weights | 2.571992 | 3.155163 | 0.736250 |
| 4 | 10 | +structure-aware decoupled KLA | 1.823264 | 2.505816 | 0.215000 |
| 5 | 11 | fixed weights | 2.175224 | 2.634355 | 0.416250 |
| 5 | 11 | +structure-aware decoupled KLA | 1.831138 | 1.611837 | 0.212500 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.426359 | 2.674778 | 0.625500 |
| +structure-aware decoupled KLA | 1.799517 | 1.795008 | 0.198000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.426359 +/- 0.158047 | [2.230149, 2.622569] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.799517 +/- 0.036462 | [1.754250, 1.844783] | 5 |
| fixed weights | RMSE | 2.674778 +/- 0.281632 | [2.325142, 3.024415] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.795008 +/- 0.418268 | [1.275744, 2.314273] | 5 |
| fixed weights | Cardinality | 0.625500 +/- 0.138032 | [0.454138, 0.796862] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.198000 +/- 0.018490 | [0.175045, 0.220955] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.626842 +/- 0.164576 | [0.422527, 0.831158] | 25.83% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.879770 +/- 0.174923 | [0.662609, 1.096932] | 32.89% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.427500 +/- 0.140106 | [0.253563, 0.601437] | 68.35% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.884983 | 0.500000 | 1.622667 | 1.562000 |
| +structure-aware decoupled KLA | 2.381163 | 0.500000 | 1.613118 | 0.698500 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.884983 +/- 0.116558 | [2.740281, 3.029685] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.381163 +/- 0.111846 | [2.242310, 2.520016] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.622667 +/- 0.075367 | [1.529102, 1.716232] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.613118 +/- 0.084517 | [1.508193, 1.718043] | 5 |
| fixed weights | CardErr | 1.562000 +/- 0.222734 | [1.285483, 1.838517] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.698500 +/- 0.099209 | [0.575335, 0.821665] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.503820 +/- 0.082750 | [0.401088, 0.606552] | 17.46% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | RMSE | 0.009549 +/- 0.044942 | [-0.046246, 0.065343] | 0.59% | 3/5 | 1 |
| +structure-aware decoupled KLA | CardErr | 0.863500 +/- 0.196684 | [0.619323, 1.107677] | 55.28% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.928414 | 0.500000 | 1.582825 | 1.488000 |
| 1 | +structure-aware decoupled KLA | 2.418283 | 0.500000 | 1.571219 | 0.782000 |
| 2 | fixed weights | 2.702078 | 0.500000 | 1.537237 | 1.150000 |
| 2 | +structure-aware decoupled KLA | 2.343667 | 0.500000 | 1.562058 | 0.712000 |
| 3 | fixed weights | 2.671070 | 0.500000 | 1.516722 | 1.158000 |
| 3 | +structure-aware decoupled KLA | 2.259115 | 0.500000 | 1.502414 | 0.680000 |
| 4 | fixed weights | 2.774375 | 0.500000 | 1.615440 | 1.366000 |
| 4 | +structure-aware decoupled KLA | 2.409330 | 0.500000 | 1.596272 | 0.798000 |
| 5 | fixed weights | 3.103272 | 0.500000 | 1.636035 | 1.922000 |
| 5 | +structure-aware decoupled KLA | 2.501598 | 0.500000 | 1.690026 | 0.796000 |
| 6 | fixed weights | 3.002259 | 0.500000 | 1.737740 | 1.724000 |
| 6 | +structure-aware decoupled KLA | 2.386975 | 0.500000 | 1.670172 | 0.560000 |
| 7 | fixed weights | 2.844626 | 0.500000 | 1.664797 | 1.464000 |
| 7 | +structure-aware decoupled KLA | 2.412332 | 0.500000 | 1.645451 | 0.744000 |
| 8 | fixed weights | 3.053768 | 0.500000 | 1.690539 | 2.224000 |
| 8 | +structure-aware decoupled KLA | 2.318003 | 0.499999 | 1.667332 | 0.516000 |
