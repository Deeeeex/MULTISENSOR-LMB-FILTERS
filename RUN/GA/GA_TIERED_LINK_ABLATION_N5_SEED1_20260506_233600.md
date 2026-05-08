# GA Tiered Link Ablation (2026-05-06 23:36:00)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6]
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
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.590531 | 2.268101 | 0.868750 |
| 1 | 2 | +structure-aware decoupled KLA | 1.892404 | 1.611468 | 0.241250 |
| 2 | 3 | fixed weights | 2.623249 | 3.319849 | 0.761250 |
| 2 | 3 | +structure-aware decoupled KLA | 1.928044 | 1.541570 | 0.267500 |
| 3 | 4 | fixed weights | 2.681097 | 2.507659 | 0.871250 |
| 3 | 4 | +structure-aware decoupled KLA | 1.835859 | 1.487550 | 0.237500 |
| 4 | 5 | fixed weights | 2.398507 | 2.920391 | 0.625000 |
| 4 | 5 | +structure-aware decoupled KLA | 1.764445 | 2.213187 | 0.191250 |
| 5 | 6 | fixed weights | 2.826942 | 2.497009 | 1.267500 |
| 5 | 6 | +structure-aware decoupled KLA | 1.890466 | 1.894265 | 0.283750 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.624065 | 2.702602 | 0.878750 |
| +structure-aware decoupled KLA | 1.862244 | 1.749608 | 0.244250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.624065 +/- 0.155252 | [2.431324, 2.816805] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.862244 +/- 0.063829 | [1.783003, 1.941485] | 5 |
| fixed weights | RMSE | 2.702602 +/- 0.417579 | [2.184192, 3.221011] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.749608 +/- 0.302848 | [1.373633, 2.125583] | 5 |
| fixed weights | Cardinality | 0.878750 +/- 0.239519 | [0.581395, 1.176105] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.244250 +/- 0.035229 | [0.200514, 0.287986] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.761821 +/- 0.124779 | [0.606912, 0.916730] | 29.03% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.952994 +/- 0.489008 | [0.345907, 1.560081] | 35.26% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.634500 +/- 0.213387 | [0.369587, 0.899413] | 72.20% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.945058 | 0.500000 | 1.622083 | 1.762250 |
| +structure-aware decoupled KLA | 2.381696 | 0.499999 | 1.602228 | 0.710250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.945058 +/- 0.101250 | [2.819359, 3.070757] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.381696 +/- 0.073946 | [2.289894, 2.473497] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.499999 +/- 0.000002 | [0.499997, 0.500001] | 5 |
| fixed weights | RMSE | 1.622083 +/- 0.047195 | [1.563492, 1.680673] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.602228 +/- 0.059596 | [1.528241, 1.676214] | 5 |
| fixed weights | CardErr | 1.762250 +/- 0.223078 | [1.485307, 2.039193] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.710250 +/- 0.083859 | [0.606142, 0.814358] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.563362 +/- 0.037356 | [0.516987, 0.609738] | 19.13% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | H-OSPA | 0.000001 +/- 0.000002 | [-0.000001, 0.000003] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | RMSE | 0.019855 +/- 0.039355 | [-0.029003, 0.068713] | 1.22% | 4/5 | 0.375 |
| +structure-aware decoupled KLA | CardErr | 1.052000 +/- 0.144546 | [0.872552, 1.231448] | 59.70% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.761763 | 0.500000 | 1.533529 | 1.304000 |
| 1 | +structure-aware decoupled KLA | 2.343192 | 0.500000 | 1.549339 | 0.684000 |
| 2 | fixed weights | 2.545475 | 0.500000 | 1.572886 | 0.938000 |
| 2 | +structure-aware decoupled KLA | 2.317623 | 0.500000 | 1.534088 | 0.690000 |
| 3 | fixed weights | 2.838552 | 0.500000 | 1.536159 | 1.406000 |
| 3 | +structure-aware decoupled KLA | 2.373107 | 0.500000 | 1.545717 | 0.734000 |
| 4 | fixed weights | 2.794773 | 0.500000 | 1.623058 | 1.542000 |
| 4 | +structure-aware decoupled KLA | 2.360557 | 0.500000 | 1.577893 | 0.778000 |
| 5 | fixed weights | 3.259823 | 0.500000 | 1.713498 | 2.320000 |
| 5 | +structure-aware decoupled KLA | 2.516018 | 0.500000 | 1.676317 | 0.876000 |
| 6 | fixed weights | 2.899061 | 0.500000 | 1.650463 | 1.534000 |
| 6 | +structure-aware decoupled KLA | 2.271183 | 0.500000 | 1.604269 | 0.526000 |
| 7 | fixed weights | 3.269107 | 0.500000 | 1.653931 | 2.368000 |
| 7 | +structure-aware decoupled KLA | 2.546423 | 0.500000 | 1.683179 | 0.832000 |
| 8 | fixed weights | 3.191911 | 0.500000 | 1.693135 | 2.686000 |
| 8 | +structure-aware decoupled KLA | 2.325462 | 0.499990 | 1.647022 | 0.562000 |
