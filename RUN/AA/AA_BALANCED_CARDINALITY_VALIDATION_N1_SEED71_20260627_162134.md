# AA Balanced/Cardinality Validation

Generated at: 2026-06-27 16:23:32

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 71 (fixed=1)
- trialSeeds: 72
- lmbParallelUpdateMode: AA
- scenarioLabel: maneuver-crossing-assignment-smoke
- targetScenarioMode: maneuver-crossing-assignment
- neighborMapMode: 4plus4
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- sensorFovEnabled: 1
- sensorFovHalfAngleDeg: 60.000
- sensorFovRange: 60000.000
- sensorMotionType: CV
- sensorMotionProcessNoiseStd: 0.000000
- targetFormationLifeSpan: 24
- targetFormationCount: 10
- targetFormationStartTime: 1
- targetFormationStaggeredBirths: 0
- targetFormationBirthInterval: 8
- crossingWindow: [9 17]
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
- labelSupportMinEffectiveCount: NaN
- useCrossLocalLabelConsensusProjection: 0
- crossLocalConsensusProjectionMode: barycenter
- crossLocalConsensusCutoff: NaN
- crossLocalConsensusIterations: 1
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Neighborhood label-barycenter spatial-KLA AA
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
- labelSupportMinEffectiveCount: NaN
- useCrossLocalLabelConsensusProjection: 1
- crossLocalConsensusProjectionMode: neighborhood-barycenter
- crossLocalConsensusCutoff: NaN
- crossLocalConsensusIterations: 3
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Neighborhood reference-only label-consensus spatial-KLA AA
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
- labelSupportMinEffectiveCount: NaN
- useCrossLocalLabelConsensusProjection: 1
- crossLocalConsensusProjectionMode: neighborhood-reference-only
- crossLocalConsensusCutoff: NaN
- crossLocalConsensusIterations: 3
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.618293 | 6.253516 | 0.229167 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 0.887164 | 0.956680 | 0.187500 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.687278 | 4.025160 | 0.187500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.618293 | 6.253516 | 0.229167 |
| Neighborhood label-barycenter spatial-KLA AA | 0.887164 | 0.956680 | 0.187500 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.687278 | 4.025160 | 0.187500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.618293 +/- 0.000000 | [2.618293, 2.618293] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.887164 +/- 0.000000 | [0.887164, 0.887164] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.687278 +/- 0.000000 | [1.687278, 1.687278] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 6.253516 +/- 0.000000 | [6.253516, 6.253516] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.956680 +/- 0.000000 | [0.956680, 0.956680] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 4.025160 +/- 0.000000 | [4.025160, 4.025160] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.229167 +/- 0.000000 | [0.229167, 0.229167] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.187500 +/- 0.000000 | [0.187500, 0.187500] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.187500 +/- 0.000000 | [0.187500, 0.187500] | 1 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.731130 +/- 0.000000 | [1.731130, 1.731130] | 66.12% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.931016 +/- 0.000000 | [0.931016, 0.931016] | 35.56% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 5.296836 +/- 0.000000 | [5.296836, 5.296836] | 84.70% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 2.228356 +/- 0.000000 | [2.228356, 2.228356] | 35.63% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.041667 +/- 0.000000 | [0.041667, 0.041667] | 18.18% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.041667 +/- 0.000000 | [0.041667, 0.041667] | 18.18% | 1/1 | 1 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.928595 | 2.714397 | 0.729167 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 2.733518 | 4.241043 | 0.729167 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.842239 | 2.594949 | 0.729167 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.928595 | 2.714397 | 0.729167 |
| Neighborhood label-barycenter spatial-KLA AA | 2.733518 | 4.241043 | 0.729167 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.842239 | 2.594949 | 0.729167 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.928595 +/- 0.000000 | [2.928595, 2.928595] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.733518 +/- 0.000000 | [2.733518, 2.733518] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.842239 +/- 0.000000 | [2.842239, 2.842239] | 1 |
| Tuned spatial-KLA AA | RMSE | 2.714397 +/- 0.000000 | [2.714397, 2.714397] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 4.241043 +/- 0.000000 | [4.241043, 4.241043] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.594949 +/- 0.000000 | [2.594949, 2.594949] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.729167 +/- 0.000000 | [0.729167, 0.729167] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.729167 +/- 0.000000 | [0.729167, 0.729167] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.729167 +/- 0.000000 | [0.729167, 0.729167] | 1 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.195077 +/- 0.000000 | [0.195077, 0.195077] | 6.66% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.086356 +/- 0.000000 | [0.086356, 0.086356] | 2.95% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | -1.526646 +/- 0.000000 | [-1.526646, -1.526646] | -56.24% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.119448 +/- 0.000000 | [0.119448, 0.119448] | 4.40% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | -0.000000 +/- 0.000000 | [-0.000000, -0.000000] | -0.00% | 0/1 | NaN |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | -0.000000 +/- 0.000000 | [-0.000000, -0.000000] | -0.00% | 0/1 | NaN |

## Scenario Window Metrics
Windowed metrics are restricted to the fixed stress interval `[9 17]` (indices `[9 10 11 12 13 14 15 16 17]`). They are intended for maneuver/crossing diagnostics and should not be mixed with whole-run formation metrics.

### Per-Trial Scenario-Window Network Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.315274 | 2.169030 | 0.125000 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 0.564400 | 0.353552 | 0.111111 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.490487 | 1.356088 | 0.111111 |

### Scenario-Window Network Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.315274 +/- 0.000000 | [2.315274, 2.315274] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.564400 +/- 0.000000 | [0.564400, 0.564400] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.490487 +/- 0.000000 | [1.490487, 1.490487] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 2.169030 +/- 0.000000 | [2.169030, 2.169030] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.353552 +/- 0.000000 | [0.353552, 0.353552] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.356088 +/- 0.000000 | [1.356088, 1.356088] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.125000 +/- 0.000000 | [0.125000, 0.125000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.111111 +/- 0.000000 | [0.111111, 0.111111] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.111111 +/- 0.000000 | [0.111111, 0.111111] | 1 |

### Scenario-Window Network Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.750874 +/- 0.000000 | [1.750874, 1.750874] | 75.62% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.824787 +/- 0.000000 | [0.824787, 0.824787] | 35.62% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.815478 +/- 0.000000 | [1.815478, 1.815478] | 83.70% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.812942 +/- 0.000000 | [0.812942, 0.812942] | 37.48% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.013889 +/- 0.000000 | [0.013889, 0.013889] | 11.11% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.013889 +/- 0.000000 | [0.013889, 0.013889] | 11.11% | 1/1 | 1 |

### Per-Trial Scenario-Window Local Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.670122 | 2.366493 | 0.125000 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 2.430426 | 2.050042 | 0.111111 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.573311 | 2.307712 | 0.111111 |

### Scenario-Window Local Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.670122 +/- 0.000000 | [2.670122, 2.670122] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.430426 +/- 0.000000 | [2.430426, 2.430426] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.573311 +/- 0.000000 | [2.573311, 2.573311] | 1 |
| Tuned spatial-KLA AA | RMSE | 2.366493 +/- 0.000000 | [2.366493, 2.366493] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 2.050042 +/- 0.000000 | [2.050042, 2.050042] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.307712 +/- 0.000000 | [2.307712, 2.307712] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.125000 +/- 0.000000 | [0.125000, 0.125000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.111111 +/- 0.000000 | [0.111111, 0.111111] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.111111 +/- 0.000000 | [0.111111, 0.111111] | 1 |

### Scenario-Window Local Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.239696 +/- 0.000000 | [0.239696, 0.239696] | 8.98% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.096811 +/- 0.000000 | [0.096811, 0.096811] | 3.63% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.316451 +/- 0.000000 | [0.316451, 0.316451] | 13.37% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.058781 +/- 0.000000 | [0.058781, 0.058781] | 2.48% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.013889 +/- 0.000000 | [0.013889, 0.013889] | 11.11% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.013889 +/- 0.000000 | [0.013889, 0.013889] | 11.11% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 16.129222 +/- 0.000000 | 0.672051 | 1.000x | 1 |
| Neighborhood label-barycenter spatial-KLA AA | 39.543660 +/- 0.000000 | 1.647652 | 2.452x | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 36.169080 +/- 0.000000 | 1.507045 | 2.242x | 1 |
