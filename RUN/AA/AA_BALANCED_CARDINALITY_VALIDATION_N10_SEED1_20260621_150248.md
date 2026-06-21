# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 15:12:33

Comparison order: Balanced spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 10
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11]
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
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| 2 | 3 | Balanced spatial-KLA AA | 1.667430 | 1.435498 | 0.035000 |
| 3 | 4 | Balanced spatial-KLA AA | 1.665450 | 1.448322 | 0.031250 |
| 4 | 5 | Balanced spatial-KLA AA | 1.603341 | 1.300640 | 0.042500 |
| 5 | 6 | Balanced spatial-KLA AA | 1.610089 | 1.457096 | 0.027500 |
| 6 | 7 | Balanced spatial-KLA AA | 1.598491 | 1.528223 | 0.035000 |
| 7 | 8 | Balanced spatial-KLA AA | 1.747386 | 1.519075 | 0.038750 |
| 8 | 9 | Balanced spatial-KLA AA | 1.623301 | 1.359868 | 0.030000 |
| 9 | 10 | Balanced spatial-KLA AA | 1.664538 | 1.534739 | 0.051250 |
| 10 | 11 | Balanced spatial-KLA AA | 1.768428 | 1.504540 | 0.040000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.663109 | 1.456257 | 0.035000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.663109 +/- 0.058448 | [1.621301, 1.704918] | 10 |
| Balanced spatial-KLA AA | Loc. disag. | 1.456257 +/- 0.075977 | [1.401910, 1.510604] | 10 |
| Balanced spatial-KLA AA | Card. disp. | 0.035000 +/- 0.008937 | [0.028608, 0.041392] | 10 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 1.998637 | 3.716553 | 0.084000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 1.998637 +/- 0.063423 | [1.953270, 2.044004] | 10 |
| Balanced spatial-KLA AA | RMSE | 3.716553 +/- 0.283574 | [3.513710, 3.919396] | 10 |
| Balanced spatial-KLA AA | CardErr | 0.084000 +/- 0.010620 | [0.076404, 0.091596] | 10 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 51.016771 +/- 0.993668 | 2.125699 | 1.000x | 10 |
