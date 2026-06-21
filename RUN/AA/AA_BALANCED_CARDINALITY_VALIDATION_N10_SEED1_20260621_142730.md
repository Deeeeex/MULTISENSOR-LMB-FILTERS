# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 14:35:43

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
- existenceThreshold: 0.100000
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
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
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
| 1 | 2 | Balanced spatial-KLA AA | 1.690392 | 1.482117 | 0.018750 |
| 2 | 3 | Balanced spatial-KLA AA | 1.673607 | 1.438198 | 0.035000 |
| 3 | 4 | Balanced spatial-KLA AA | 1.647088 | 1.436270 | 0.028750 |
| 4 | 5 | Balanced spatial-KLA AA | 1.639079 | 1.658551 | 0.035000 |
| 5 | 6 | Balanced spatial-KLA AA | 1.594765 | 1.468791 | 0.021250 |
| 6 | 7 | Balanced spatial-KLA AA | 1.591745 | 1.456651 | 0.025000 |
| 7 | 8 | Balanced spatial-KLA AA | 1.788193 | 1.544961 | 0.037500 |
| 8 | 9 | Balanced spatial-KLA AA | 1.631585 | 1.383789 | 0.020000 |
| 9 | 10 | Balanced spatial-KLA AA | 1.655918 | 1.639283 | 0.036250 |
| 10 | 11 | Balanced spatial-KLA AA | 1.766125 | 1.510803 | 0.037500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.667850 | 1.501941 | 0.029500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.667850 +/- 0.065431 | [1.621046, 1.714653] | 10 |
| Balanced spatial-KLA AA | Loc. disag. | 1.501941 +/- 0.088943 | [1.438320, 1.565563] | 10 |
| Balanced spatial-KLA AA | Card. disp. | 0.029500 +/- 0.007665 | [0.024017, 0.034983] | 10 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 1.989437 | 3.737840 | 0.077250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 1.989437 +/- 0.064088 | [1.943595, 2.035279] | 10 |
| Balanced spatial-KLA AA | RMSE | 3.737840 +/- 0.281873 | [3.536214, 3.939466] | 10 |
| Balanced spatial-KLA AA | CardErr | 0.077250 +/- 0.009201 | [0.070669, 0.083831] | 10 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 43.013623 +/- 3.081345 | 1.792234 | 1.000x | 10 |
