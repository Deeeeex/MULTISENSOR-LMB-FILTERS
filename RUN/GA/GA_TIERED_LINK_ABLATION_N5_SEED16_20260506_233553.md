# GA Tiered Link Ablation (2026-05-06 23:35:53)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 16 (fixed=1)
- trialSeeds: [17 18 19 20 21]
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
- Trial 1: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 2: [0 0.1 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 3: [0.2 0 0.5 0.5 0.1 0.1 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.1 0 0.5 0.5 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.1 0.5 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 17 | fixed weights | 2.600917 | 2.376823 | 0.802500 |
| 1 | 17 | +structure-aware decoupled KLA | 1.863439 | 1.430953 | 0.268750 |
| 2 | 18 | fixed weights | 2.530244 | 2.232345 | 0.762500 |
| 2 | 18 | +structure-aware decoupled KLA | 1.873030 | 1.746324 | 0.237500 |
| 3 | 19 | fixed weights | 2.440243 | 2.295666 | 0.741250 |
| 3 | 19 | +structure-aware decoupled KLA | 1.703659 | 1.650134 | 0.208750 |
| 4 | 20 | fixed weights | 2.498598 | 2.662959 | 0.777500 |
| 4 | 20 | +structure-aware decoupled KLA | 1.851685 | 1.651547 | 0.253750 |
| 5 | 21 | fixed weights | 2.462637 | 2.808408 | 0.576250 |
| 5 | 21 | +structure-aware decoupled KLA | 1.829428 | 1.815996 | 0.266250 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.506528 | 2.475240 | 0.732000 |
| +structure-aware decoupled KLA | 1.824248 | 1.658991 | 0.247000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.506528 +/- 0.062958 | [2.428368, 2.584688] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.824248 +/- 0.069349 | [1.738154, 1.910342] | 5 |
| fixed weights | RMSE | 2.475240 +/- 0.248581 | [2.166636, 2.783845] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.658991 +/- 0.145269 | [1.478644, 1.839338] | 5 |
| fixed weights | Cardinality | 0.732000 +/- 0.089881 | [0.620416, 0.843584] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.247000 +/- 0.024711 | [0.216322, 0.277678] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.682280 +/- 0.050702 | [0.619335, 0.745225] | 27.22% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.816250 +/- 0.236705 | [0.522388, 1.110111] | 32.98% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.485000 +/- 0.097928 | [0.363426, 0.606574] | 66.26% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.943300 | 0.500000 | 1.631986 | 1.603000 |
| +structure-aware decoupled KLA | 2.434683 | 0.500000 | 1.599937 | 0.741500 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.943300 +/- 0.105438 | [2.812403, 3.074198] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.434683 +/- 0.085233 | [2.328870, 2.540497] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.631986 +/- 0.065246 | [1.550985, 1.712986] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.599937 +/- 0.038341 | [1.552338, 1.647537] | 5 |
| fixed weights | CardErr | 1.603000 +/- 0.087948 | [1.493816, 1.712184] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.741500 +/- 0.063630 | [0.662506, 0.820494] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.508617 +/- 0.077386 | [0.412545, 0.604689] | 17.28% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | RMSE | 0.032048 +/- 0.034203 | [-0.010413, 0.074510] | 1.96% | 4/5 | 0.375 |
| +structure-aware decoupled KLA | CardErr | 0.861500 +/- 0.113782 | [0.720243, 1.002757] | 53.74% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.853760 | 0.500000 | 1.561902 | 1.382000 |
| 1 | +structure-aware decoupled KLA | 2.424764 | 0.500000 | 1.546232 | 0.776000 |
| 2 | fixed weights | 2.743049 | 0.500000 | 1.599421 | 1.182000 |
| 2 | +structure-aware decoupled KLA | 2.394380 | 0.500000 | 1.542949 | 0.752000 |
| 3 | fixed weights | 2.869709 | 0.500000 | 1.549348 | 1.356000 |
| 3 | +structure-aware decoupled KLA | 2.447432 | 0.500000 | 1.560267 | 0.824000 |
| 4 | fixed weights | 3.013113 | 0.500000 | 1.563764 | 1.908000 |
| 4 | +structure-aware decoupled KLA | 2.500704 | 0.500000 | 1.584394 | 0.902000 |
| 5 | fixed weights | 2.927131 | 0.500000 | 1.802351 | 1.510000 |
| 5 | +structure-aware decoupled KLA | 2.521199 | 0.500000 | 1.665426 | 0.800000 |
| 6 | fixed weights | 2.907753 | 0.500000 | 1.631867 | 1.484000 |
| 6 | +structure-aware decoupled KLA | 2.340783 | 0.500000 | 1.611814 | 0.550000 |
| 7 | fixed weights | 3.047597 | 0.500000 | 1.680301 | 1.720000 |
| 7 | +structure-aware decoupled KLA | 2.492258 | 0.500000 | 1.650829 | 0.762000 |
| 8 | fixed weights | 3.184291 | 0.500000 | 1.666934 | 2.282000 |
| 8 | +structure-aware decoupled KLA | 2.355948 | 0.499997 | 1.637587 | 0.566000 |
