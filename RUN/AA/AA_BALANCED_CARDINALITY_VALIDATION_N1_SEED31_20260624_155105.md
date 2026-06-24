# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 15:52:56

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 31 (fixed=1)
- trialSeeds: 32
- lmbParallelUpdateMode: AA
- scenarioLabel: topology-ring-formation
- neighborMapMode: ring
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- sensorFovEnabled: 1
- sensorFovHalfAngleDeg: 60.000
- sensorFovRange: 60000.000
- sensorMotionType: CV
- sensorMotionProcessNoiseStd: 0.000000
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
- Trial 1: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 32 | Tuned spatial-KLA AA | 2.771921 | 2.682580 | 0.083750 |
| 1 | 32 | Neighborhood label-barycenter spatial-KLA AA | 1.348601 | 1.197544 | 0.030000 |
| 1 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.263993 | 2.099705 | 0.030000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.771921 | 2.682580 | 0.083750 |
| Neighborhood label-barycenter spatial-KLA AA | 1.348601 | 1.197544 | 0.030000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.263993 | 2.099705 | 0.030000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.771921 +/- 0.000000 | [2.771921, 2.771921] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.348601 +/- 0.000000 | [1.348601, 1.348601] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 2.263993 +/- 0.000000 | [2.263993, 2.263993] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 2.682580 +/- 0.000000 | [2.682580, 2.682580] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.197544 +/- 0.000000 | [1.197544, 1.197544] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 2.099705 +/- 0.000000 | [2.099705, 2.099705] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.083750 +/- 0.000000 | [0.083750, 0.083750] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.030000 +/- 0.000000 | [0.030000, 0.030000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.030000 +/- 0.000000 | [0.030000, 0.030000] | 1 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.423320 +/- 0.000000 | [1.423320, 1.423320] | 51.35% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.507928 +/- 0.000000 | [0.507928, 0.507928] | 18.32% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.485036 +/- 0.000000 | [1.485036, 1.485036] | 55.36% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.582875 +/- 0.000000 | [0.582875, 0.582875] | 21.73% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.053750 +/- 0.000000 | [0.053750, 0.053750] | 64.18% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.053750 +/- 0.000000 | [0.053750, 0.053750] | 64.18% | 1/1 | 1 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 32 | Tuned spatial-KLA AA | 2.580668 | 4.568272 | 0.116250 |
| 1 | 32 | Neighborhood label-barycenter spatial-KLA AA | 2.010548 | 4.035532 | 0.065000 |
| 1 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.410974 | 4.511156 | 0.065000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.580668 | 4.568272 | 0.116250 |
| Neighborhood label-barycenter spatial-KLA AA | 2.010548 | 4.035532 | 0.065000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.410974 | 4.511156 | 0.065000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.580668 +/- 0.000000 | [2.580668, 2.580668] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.010548 +/- 0.000000 | [2.010548, 2.010548] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.410974 +/- 0.000000 | [2.410974, 2.410974] | 1 |
| Tuned spatial-KLA AA | RMSE | 4.568272 +/- 0.000000 | [4.568272, 4.568272] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 4.035532 +/- 0.000000 | [4.035532, 4.035532] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.511156 +/- 0.000000 | [4.511156, 4.511156] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.116250 +/- 0.000000 | [0.116250, 0.116250] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.065000 +/- 0.000000 | [0.065000, 0.065000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.065000 +/- 0.000000 | [0.065000, 0.065000] | 1 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.570120 +/- 0.000000 | [0.570120, 0.570120] | 22.09% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.169694 +/- 0.000000 | [0.169694, 0.169694] | 6.58% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.532740 +/- 0.000000 | [0.532740, 0.532740] | 11.66% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.057116 +/- 0.000000 | [0.057116, 0.057116] | 1.25% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.051250 +/- 0.000000 | [0.051250, 0.051250] | 44.09% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.051250 +/- 0.000000 | [0.051250, 0.051250] | 44.09% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 25.365188 +/- 0.000000 | 1.056883 | 1.000x | 1 |
| Neighborhood label-barycenter spatial-KLA AA | 36.110156 +/- 0.000000 | 1.504590 | 1.424x | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 33.593759 +/- 0.000000 | 1.399740 | 1.324x | 1 |
