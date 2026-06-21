# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 14:55:29

Comparison order: Balanced spatial-KLA AA

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
- existenceThreshold: 0.180000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Balanced spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.750
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced spatial-KLA AA | 1.699147 | 1.489474 | 0.018750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.699147 | 1.489474 | 0.018750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.699147 +/- 0.000000 | [1.699147, 1.699147] | 1 |
| Balanced spatial-KLA AA | Loc. disag. | 1.489474 +/- 0.000000 | [1.489474, 1.489474] | 1 |
| Balanced spatial-KLA AA | Card. disp. | 0.018750 +/- 0.000000 | [0.018750, 0.018750] | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 2.021241 | 4.137743 | 0.066250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 2.021241 +/- 0.000000 | [2.021241, 2.021241] | 1 |
| Balanced spatial-KLA AA | RMSE | 4.137743 +/- 0.000000 | [4.137743, 4.137743] | 1 |
| Balanced spatial-KLA AA | CardErr | 0.066250 +/- 0.000000 | [0.066250, 0.066250] | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 55.813963 +/- 0.000000 | 2.325582 | 1.000x | 1 |
