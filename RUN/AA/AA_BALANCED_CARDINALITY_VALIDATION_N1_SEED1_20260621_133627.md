# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 13:39:51

Comparison order: Balanced spatial-KLA AA -> Cardinality spatial-KLA AA

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
- existenceThreshold: 0.030000
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

### Cardinality spatial-KLA AA
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
- useFidFiaExistence: 1
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced spatial-KLA AA | 1.719874 | 1.534164 | 0.022500 |
| 1 | 2 | Cardinality spatial-KLA AA | 2.778168 | 22.290024 | 0.321250 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.719874 | 1.534164 | 0.022500 |
| Cardinality spatial-KLA AA | 2.778168 | 22.290024 | 0.321250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.719874 +/- 0.000000 | [1.719874, 1.719874] | 1 |
| Cardinality spatial-KLA AA | OSPA | 2.778168 +/- 0.000000 | [2.778168, 2.778168] | 1 |
| Balanced spatial-KLA AA | Loc. disag. | 1.534164 +/- 0.000000 | [1.534164, 1.534164] | 1 |
| Cardinality spatial-KLA AA | Loc. disag. | 22.290024 +/- 0.000000 | [22.290024, 22.290024] | 1 |
| Balanced spatial-KLA AA | Card. disp. | 0.022500 +/- 0.000000 | [0.022500, 0.022500] | 1 |
| Cardinality spatial-KLA AA | Card. disp. | 0.321250 +/- 0.000000 | [0.321250, 0.321250] | 1 |

## Paired Improvements Relative to Balanced spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality spatial-KLA AA | OSPA | -1.058293 +/- 0.000000 | [-1.058293, -1.058293] | -61.53% | 0/1 | 1 |
| Cardinality spatial-KLA AA | Loc. disag. | -20.755861 +/- 0.000000 | [-20.755861, -20.755861] | -1352.91% | 0/1 | 1 |
| Cardinality spatial-KLA AA | Card. disp. | -0.298750 +/- 0.000000 | [-0.298750, -0.298750] | -1327.78% | 0/1 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 2.042636 | 4.209988 | 0.067500 |
| Cardinality spatial-KLA AA | 2.788991 | 5.697451 | 0.606250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 2.042636 +/- 0.000000 | [2.042636, 2.042636] | 1 |
| Cardinality spatial-KLA AA | E-OSPA | 2.788991 +/- 0.000000 | [2.788991, 2.788991] | 1 |
| Balanced spatial-KLA AA | RMSE | 4.209988 +/- 0.000000 | [4.209988, 4.209988] | 1 |
| Cardinality spatial-KLA AA | RMSE | 5.697451 +/- 0.000000 | [5.697451, 5.697451] | 1 |
| Balanced spatial-KLA AA | CardErr | 0.067500 +/- 0.000000 | [0.067500, 0.067500] | 1 |
| Cardinality spatial-KLA AA | CardErr | 0.606250 +/- 0.000000 | [0.606250, 0.606250] | 1 |

## Paired Local-Metric Improvements Relative to Balanced spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality spatial-KLA AA | E-OSPA | -0.746355 +/- 0.000000 | [-0.746355, -0.746355] | -36.54% | 0/1 | 1 |
| Cardinality spatial-KLA AA | RMSE | -1.487463 +/- 0.000000 | [-1.487463, -1.487463] | -35.33% | 0/1 | 1 |
| Cardinality spatial-KLA AA | CardErr | -0.538750 +/- 0.000000 | [-0.538750, -0.538750] | -798.15% | 0/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 58.942939 +/- 0.000000 | 2.455956 | 1.000x | 1 |
| Cardinality spatial-KLA AA | 127.592573 +/- 0.000000 | 5.316357 | 2.165x | 1 |
