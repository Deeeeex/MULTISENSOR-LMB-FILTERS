# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 15:59:47

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 41 (fixed=1)
- trialSeeds: 42
- lmbParallelUpdateMode: AA
- scenarioLabel: partial-fov35-formation
- neighborMapMode: 4plus4
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- sensorFovEnabled: 1
- sensorFovHalfAngleDeg: 35.000
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
- Trial 1: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 42 | Tuned spatial-KLA AA | 2.327551 | 1.182761 | 0.457500 |
| 1 | 42 | Neighborhood label-barycenter spatial-KLA AA | 1.516556 | 0.126668 | 0.470000 |
| 1 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.884422 | 0.553296 | 0.470000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.327551 | 1.182761 | 0.457500 |
| Neighborhood label-barycenter spatial-KLA AA | 1.516556 | 0.126668 | 0.470000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.884422 | 0.553296 | 0.470000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.327551 +/- 0.000000 | [2.327551, 2.327551] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.516556 +/- 0.000000 | [1.516556, 1.516556] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.884422 +/- 0.000000 | [1.884422, 1.884422] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 1.182761 +/- 0.000000 | [1.182761, 1.182761] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.126668 +/- 0.000000 | [0.126668, 0.126668] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.553296 +/- 0.000000 | [0.553296, 0.553296] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.457500 +/- 0.000000 | [0.457500, 0.457500] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.470000 +/- 0.000000 | [0.470000, 0.470000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.470000 +/- 0.000000 | [0.470000, 0.470000] | 1 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.810995 +/- 0.000000 | [0.810995, 0.810995] | 34.84% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.443129 +/- 0.000000 | [0.443129, 0.443129] | 19.04% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.056093 +/- 0.000000 | [1.056093, 1.056093] | 89.29% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.629466 +/- 0.000000 | [0.629466, 0.629466] | 53.22% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | -0.012500 +/- 0.000000 | [-0.012500, -0.012500] | -2.73% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | -0.012500 +/- 0.000000 | [-0.012500, -0.012500] | -2.73% | 0/1 | 1 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 42 | Tuned spatial-KLA AA | 2.922353 | 4.461712 | 0.612500 |
| 1 | 42 | Neighborhood label-barycenter spatial-KLA AA | 2.734257 | 4.281871 | 0.600000 |
| 1 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.874641 | 4.460423 | 0.600000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.922353 | 4.461712 | 0.612500 |
| Neighborhood label-barycenter spatial-KLA AA | 2.734257 | 4.281871 | 0.600000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.874641 | 4.460423 | 0.600000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.922353 +/- 0.000000 | [2.922353, 2.922353] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.734257 +/- 0.000000 | [2.734257, 2.734257] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.874641 +/- 0.000000 | [2.874641, 2.874641] | 1 |
| Tuned spatial-KLA AA | RMSE | 4.461712 +/- 0.000000 | [4.461712, 4.461712] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 4.281871 +/- 0.000000 | [4.281871, 4.281871] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.460423 +/- 0.000000 | [4.460423, 4.460423] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.612500 +/- 0.000000 | [0.612500, 0.612500] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.600000 +/- 0.000000 | [0.600000, 0.600000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.600000 +/- 0.000000 | [0.600000, 0.600000] | 1 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.188095 +/- 0.000000 | [0.188095, 0.188095] | 6.44% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.047712 +/- 0.000000 | [0.047712, 0.047712] | 1.63% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.179842 +/- 0.000000 | [0.179842, 0.179842] | 4.03% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.001289 +/- 0.000000 | [0.001289, 0.001289] | 0.03% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.012500 +/- 0.000000 | [0.012500, 0.012500] | 2.04% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.012500 +/- 0.000000 | [0.012500, 0.012500] | 2.04% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 37.361862 +/- 0.000000 | 1.556744 | 1.000x | 1 |
| Neighborhood label-barycenter spatial-KLA AA | 56.633480 +/- 0.000000 | 2.359728 | 1.516x | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 53.409599 +/- 0.000000 | 2.225400 | 1.430x | 1 |
