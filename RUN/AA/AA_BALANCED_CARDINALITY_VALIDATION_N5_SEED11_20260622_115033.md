# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 12:00:03

Comparison order: Tuned spatial-KLA AA -> Output-history lifecycle spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
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
### Tuned spatial-KLA AA
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
- aaKlaSpatialExistencePower: 0.000
- aaKlaSpatialExistenceMinScore: 0.000
- labelPruningThreshold: NaN
- labelPruningMinTrajectoryLength: NaN
- labelPruningProtectionMode: trajectory-age
- labelPruningMaxOutputGap: NaN
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Output-history lifecycle spatial-KLA AA
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
- aaKlaSpatialExistencePower: 0.000
- aaKlaSpatialExistenceMinScore: 0.000
- labelPruningThreshold: 0.010000
- labelPruningMinTrajectoryLength: NaN
- labelPruningProtectionMode: last-output
- labelPruningMaxOutputGap: 1
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 2: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 3: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 4: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 5: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 1 | 12 | Output-history lifecycle spatial-KLA AA | 1.673167 | 1.464721 | 0.036250 |
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 2 | 13 | Output-history lifecycle spatial-KLA AA | 1.809159 | 1.614062 | 0.051250 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 3 | 14 | Output-history lifecycle spatial-KLA AA | 1.670999 | 1.560121 | 0.022500 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 4 | 15 | Output-history lifecycle spatial-KLA AA | 1.719879 | 1.450262 | 0.038750 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 5 | 16 | Output-history lifecycle spatial-KLA AA | 1.641084 | 1.559100 | 0.022500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 |
| Output-history lifecycle spatial-KLA AA | 1.702857 | 1.529653 | 0.034250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.702915 +/- 0.065821 | [1.621201, 1.784630] | 5 |
| Output-history lifecycle spatial-KLA AA | OSPA | 1.702857 +/- 0.065769 | [1.621208, 1.784507] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.529716 +/- 0.069688 | [1.443201, 1.616231] | 5 |
| Output-history lifecycle spatial-KLA AA | Loc. disag. | 1.529653 +/- 0.069713 | [1.443107, 1.616199] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |
| Output-history lifecycle spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Output-history lifecycle spatial-KLA AA | OSPA | 0.000058 +/- 0.000070 | [-0.000029, 0.000145] | 0.00% | 4/5 | 0.375 |
| Output-history lifecycle spatial-KLA AA | Loc. disag. | 0.000063 +/- 0.000077 | [-0.000033, 0.000158] | 0.00% | 4/5 | 0.375 |
| Output-history lifecycle spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.032799 | 3.588145 | 0.086250 |
| Output-history lifecycle spatial-KLA AA | 2.032737 | 3.588080 | 0.086250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.032799 +/- 0.040742 | [1.982219, 2.083379] | 5 |
| Output-history lifecycle spatial-KLA AA | E-OSPA | 2.032737 +/- 0.040701 | [1.982208, 2.083265] | 5 |
| Tuned spatial-KLA AA | RMSE | 3.588145 +/- 0.379057 | [3.117559, 4.058731] | 5 |
| Output-history lifecycle spatial-KLA AA | RMSE | 3.588080 +/- 0.379064 | [3.117486, 4.058675] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |
| Output-history lifecycle spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Output-history lifecycle spatial-KLA AA | E-OSPA | 0.000062 +/- 0.000067 | [-0.000020, 0.000145] | 0.00% | 4/5 | 0.375 |
| Output-history lifecycle spatial-KLA AA | RMSE | 0.000065 +/- 0.000061 | [-0.000011, 0.000140] | 0.00% | 4/5 | 0.375 |
| Output-history lifecycle spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 49.743403 +/- 0.772949 | 2.072642 | 1.000x | 5 |
| Output-history lifecycle spatial-KLA AA | 50.342920 +/- 0.738582 | 2.097622 | 1.012x | 5 |
