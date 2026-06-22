# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 17:45:21

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

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
- Trial 1: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 2: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 3: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 4: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 5: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 1 | 12 | Neighborhood label-barycenter spatial-KLA AA | 0.300460 | 0.227285 | 0.015000 |
| 1 | 12 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.063672 | 0.949262 | 0.015000 |
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 2 | 13 | Neighborhood label-barycenter spatial-KLA AA | 0.351073 | 0.224834 | 0.030000 |
| 2 | 13 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.103466 | 0.914666 | 0.030000 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 3 | 14 | Neighborhood label-barycenter spatial-KLA AA | 0.282517 | 0.244711 | 0.005000 |
| 3 | 14 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.998267 | 0.951321 | 0.005000 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 4 | 15 | Neighborhood label-barycenter spatial-KLA AA | 0.318286 | 0.206477 | 0.025000 |
| 4 | 15 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.026745 | 0.838960 | 0.025000 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 5 | 16 | Neighborhood label-barycenter spatial-KLA AA | 0.274923 | 0.230215 | 0.010000 |
| 5 | 16 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.977335 | 0.862316 | 0.010000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 |
| Neighborhood label-barycenter spatial-KLA AA | 0.305452 | 0.226704 | 0.017000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.033897 | 0.903305 | 0.017000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.702915 +/- 0.065821 | [1.621201, 1.784630] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.305452 +/- 0.030533 | [0.267547, 0.343357] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.033897 +/- 0.050612 | [0.971064, 1.096730] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.529716 +/- 0.069688 | [1.443201, 1.616231] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.226704 +/- 0.013689 | [0.209710, 0.243698] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.903305 +/- 0.050910 | [0.840102, 0.966507] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.017000 +/- 0.010368 | [0.004128, 0.029872] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.017000 +/- 0.010368 | [0.004128, 0.029872] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.397463 +/- 0.036670 | [1.351939, 1.442987] | 82.06% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.669018 +/- 0.037138 | [0.622913, 0.715123] | 39.29% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.303012 +/- 0.063354 | [1.224360, 1.381664] | 85.18% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.626411 +/- 0.076009 | [0.532048, 0.720774] | 40.95% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.017250 +/- 0.004089 | [0.012174, 0.022326] | 50.36% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.017250 +/- 0.004089 | [0.012174, 0.022326] | 50.36% | 5/5 | 0.0625 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.032799 | 3.588145 | 0.086250 |
| Neighborhood label-barycenter spatial-KLA AA | 1.691451 | 3.450998 | 0.073000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.918293 | 3.716909 | 0.073000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.032799 +/- 0.040742 | [1.982219, 2.083379] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 1.691451 +/- 0.036645 | [1.645956, 1.736945] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 1.918293 +/- 0.051150 | [1.854792, 1.981794] | 5 |
| Tuned spatial-KLA AA | RMSE | 3.588145 +/- 0.379057 | [3.117559, 4.058731] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.450998 +/- 0.326458 | [3.045712, 3.856284] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.716909 +/- 0.336927 | [3.298625, 4.135192] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.073000 +/- 0.012550 | [0.057420, 0.088580] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.073000 +/- 0.012550 | [0.057420, 0.088580] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.341349 +/- 0.028459 | [0.306018, 0.376679] | 16.79% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.114506 +/- 0.025747 | [0.082542, 0.146471] | 5.63% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.137147 +/- 0.089562 | [0.025959, 0.248335] | 3.82% | 4/5 | 0.375 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | -0.128764 +/- 0.136177 | [-0.297823, 0.040295] | -3.59% | 1/5 | 0.375 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.013250 +/- 0.003812 | [0.008518, 0.017982] | 15.36% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.013250 +/- 0.003812 | [0.008518, 0.017982] | 15.36% | 5/5 | 0.0625 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 65.951485 +/- 11.659697 | 2.747979 | 1.000x | 5 |
| Neighborhood label-barycenter spatial-KLA AA | 102.857426 +/- 4.938271 | 4.285726 | 1.588x | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 101.949488 +/- 20.394801 | 4.247895 | 1.589x | 5 |
