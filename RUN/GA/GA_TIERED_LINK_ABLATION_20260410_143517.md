# GA Tiered Link Ablation (2026-04-10 14:35:17)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
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

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.624065 | 2.702602 | 0.878750 |
| +structure-aware decoupled KLA | 1.862244 | 1.749608 | 0.244250 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.945058 | 0.500000 | 1.622083 | 1.762250 |
| +structure-aware decoupled KLA | 2.381696 | 0.499999 | 1.602228 | 0.710250 |

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
