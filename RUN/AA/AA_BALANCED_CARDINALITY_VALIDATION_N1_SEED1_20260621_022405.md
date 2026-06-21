# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:25:12

Comparison order: Cardinality-critical AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- trialSeeds: 2
- lmbParallelUpdateMode: AA
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- targetFormationLifeSpan: 24
- existenceThreshold: 0.080000
- maximumNumberOfGmComponents: 1
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Cardinality-critical AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 1
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 1.000
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Cardinality-critical AA | 2.959493 | 3.312314 | 0.111250 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Cardinality-critical AA | 2.959493 | 3.312314 | 0.111250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cardinality-critical AA | OSPA | 2.959493 +/- 0.000000 | [2.959493, 2.959493] | 1 |
| Cardinality-critical AA | Loc. disag. | 3.312314 +/- 0.000000 | [3.312314, 3.312314] | 1 |
| Cardinality-critical AA | Card. disp. | 0.111250 +/- 0.000000 | [0.111250, 0.111250] | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Cardinality-critical AA | 3.193426 | 4.004249 | 0.183750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cardinality-critical AA | E-OSPA | 3.193426 +/- 0.000000 | [3.193426, 3.193426] | 1 |
| Cardinality-critical AA | RMSE | 4.004249 +/- 0.000000 | [4.004249, 4.004249] | 1 |
| Cardinality-critical AA | CardErr | 0.183750 +/- 0.000000 | [0.183750, 0.183750] | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Cardinality-critical AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Cardinality-critical AA | 59.567919 +/- 0.000000 | 2.481997 | 1.000x | 1 |
