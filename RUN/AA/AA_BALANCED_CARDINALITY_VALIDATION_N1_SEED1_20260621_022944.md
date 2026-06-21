# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:32:38

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
- existenceMinWeight: 0.050

### Cardinality spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
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
| 1 | 2 | Balanced spatial-KLA AA | 1.796201 | 1.706193 | 0.098750 |
| 1 | 2 | Cardinality spatial-KLA AA | 1.792949 | 1.644492 | 0.091250 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.796201 | 1.706193 | 0.098750 |
| Cardinality spatial-KLA AA | 1.792949 | 1.644492 | 0.091250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.796201 +/- 0.000000 | [1.796201, 1.796201] | 1 |
| Cardinality spatial-KLA AA | OSPA | 1.792949 +/- 0.000000 | [1.792949, 1.792949] | 1 |
| Balanced spatial-KLA AA | Loc. disag. | 1.706193 +/- 0.000000 | [1.706193, 1.706193] | 1 |
| Cardinality spatial-KLA AA | Loc. disag. | 1.644492 +/- 0.000000 | [1.644492, 1.644492] | 1 |
| Balanced spatial-KLA AA | Card. disp. | 0.098750 +/- 0.000000 | [0.098750, 0.098750] | 1 |
| Cardinality spatial-KLA AA | Card. disp. | 0.091250 +/- 0.000000 | [0.091250, 0.091250] | 1 |

## Paired Improvements Relative to Balanced spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality spatial-KLA AA | OSPA | 0.003251 +/- 0.000000 | [0.003251, 0.003251] | 0.18% | 1/1 | 1 |
| Cardinality spatial-KLA AA | Loc. disag. | 0.061701 +/- 0.000000 | [0.061701, 0.061701] | 3.62% | 1/1 | 1 |
| Cardinality spatial-KLA AA | Card. disp. | 0.007500 +/- 0.000000 | [0.007500, 0.007500] | 7.59% | 1/1 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 2.085242 | 3.022775 | 0.161250 |
| Cardinality spatial-KLA AA | 2.082760 | 3.263681 | 0.151250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 2.085242 +/- 0.000000 | [2.085242, 2.085242] | 1 |
| Cardinality spatial-KLA AA | E-OSPA | 2.082760 +/- 0.000000 | [2.082760, 2.082760] | 1 |
| Balanced spatial-KLA AA | RMSE | 3.022775 +/- 0.000000 | [3.022775, 3.022775] | 1 |
| Cardinality spatial-KLA AA | RMSE | 3.263681 +/- 0.000000 | [3.263681, 3.263681] | 1 |
| Balanced spatial-KLA AA | CardErr | 0.161250 +/- 0.000000 | [0.161250, 0.161250] | 1 |
| Cardinality spatial-KLA AA | CardErr | 0.151250 +/- 0.000000 | [0.151250, 0.151250] | 1 |

## Paired Local-Metric Improvements Relative to Balanced spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality spatial-KLA AA | E-OSPA | 0.002482 +/- 0.000000 | [0.002482, 0.002482] | 0.12% | 1/1 | 1 |
| Cardinality spatial-KLA AA | RMSE | -0.240906 +/- 0.000000 | [-0.240906, -0.240906] | -7.97% | 0/1 | 1 |
| Cardinality spatial-KLA AA | CardErr | 0.010000 +/- 0.000000 | [0.010000, 0.010000] | 6.20% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 59.720248 +/- 0.000000 | 2.488344 | 1.000x | 1 |
| Cardinality spatial-KLA AA | 98.576561 +/- 0.000000 | 4.107357 | 1.651x | 1 |
