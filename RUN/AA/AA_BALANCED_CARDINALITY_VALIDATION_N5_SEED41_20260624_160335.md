# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 16:16:55

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 5
- baseSeed: 41 (fixed=1)
- trialSeeds: [42 43 44 45 46]
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
- Trial 2: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 3: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 4: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 5: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 42 | Tuned spatial-KLA AA | 2.327551 | 1.182761 | 0.457500 |
| 1 | 42 | Neighborhood label-barycenter spatial-KLA AA | 1.516556 | 0.126668 | 0.470000 |
| 1 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.884422 | 0.553296 | 0.470000 |
| 2 | 43 | Tuned spatial-KLA AA | 2.340305 | 1.914697 | 0.303750 |
| 2 | 43 | Neighborhood label-barycenter spatial-KLA AA | 0.968059 | 0.419551 | 0.235000 |
| 2 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.436879 | 1.041874 | 0.235000 |
| 3 | 44 | Tuned spatial-KLA AA | 2.196818 | 1.153008 | 0.450000 |
| 3 | 44 | Neighborhood label-barycenter spatial-KLA AA | 1.354366 | 0.186124 | 0.435000 |
| 3 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.760526 | 0.819918 | 0.435000 |
| 4 | 45 | Tuned spatial-KLA AA | 2.430865 | 1.899630 | 0.386250 |
| 4 | 45 | Neighborhood label-barycenter spatial-KLA AA | 1.418611 | 0.205305 | 0.425000 |
| 4 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.854608 | 0.951898 | 0.425000 |
| 5 | 46 | Tuned spatial-KLA AA | 2.193409 | 0.928314 | 0.402500 |
| 5 | 46 | Neighborhood label-barycenter spatial-KLA AA | 1.451071 | 0.127716 | 0.395000 |
| 5 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.817462 | 0.566195 | 0.395000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.297790 | 1.415682 | 0.400000 |
| Neighborhood label-barycenter spatial-KLA AA | 1.341733 | 0.213073 | 0.392000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.750779 | 0.786636 | 0.392000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.297790 +/- 0.101849 | [2.171348, 2.424231] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.341733 +/- 0.216922 | [1.072432, 1.611033] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.750779 +/- 0.181463 | [1.525499, 1.976060] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.415682 +/- 0.459347 | [0.845419, 1.985945] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.213073 +/- 0.120594 | [0.063360, 0.362785] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.786636 +/- 0.221702 | [0.511401, 1.061872] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.400000 +/- 0.061777 | [0.323306, 0.476694] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.392000 +/- 0.091761 | [0.278082, 0.505918] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.392000 +/- 0.091761 | [0.278082, 0.505918] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.956057 +/- 0.252980 | [0.641991, 1.270123] | 41.61% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.547010 +/- 0.212250 | [0.283510, 0.810511] | 23.81% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.202609 +/- 0.376168 | [0.735611, 1.669608] | 84.95% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.629046 +/- 0.282766 | [0.278001, 0.980091] | 44.43% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.008000 +/- 0.039830 | [-0.041447, 0.057447] | 2.00% | 3/5 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.008000 +/- 0.039830 | [-0.041447, 0.057447] | 2.00% | 3/5 | 1 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 42 | Tuned spatial-KLA AA | 2.922353 | 4.461712 | 0.612500 |
| 1 | 42 | Neighborhood label-barycenter spatial-KLA AA | 2.734257 | 4.281871 | 0.600000 |
| 1 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.874641 | 4.460423 | 0.600000 |
| 2 | 43 | Tuned spatial-KLA AA | 2.579910 | 4.251270 | 0.426250 |
| 2 | 43 | Neighborhood label-barycenter spatial-KLA AA | 2.292473 | 3.888111 | 0.355000 |
| 2 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.422548 | 3.998444 | 0.355000 |
| 3 | 44 | Tuned spatial-KLA AA | 2.826258 | 4.227633 | 0.575000 |
| 3 | 44 | Neighborhood label-barycenter spatial-KLA AA | 2.641442 | 3.909874 | 0.575000 |
| 3 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.776937 | 4.182596 | 0.575000 |
| 4 | 45 | Tuned spatial-KLA AA | 2.839057 | 3.899957 | 0.556250 |
| 4 | 45 | Neighborhood label-barycenter spatial-KLA AA | 2.659610 | 3.279527 | 0.605000 |
| 4 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.834351 | 3.613296 | 0.605000 |
| 5 | 46 | Tuned spatial-KLA AA | 3.020138 | 3.760747 | 0.662500 |
| 5 | 46 | Neighborhood label-barycenter spatial-KLA AA | 2.882929 | 3.277132 | 0.665000 |
| 5 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.994885 | 3.469197 | 0.665000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.837543 | 4.120264 | 0.566500 |
| Neighborhood label-barycenter spatial-KLA AA | 2.642142 | 3.727303 | 0.560000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.780672 | 3.944791 | 0.560000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.837543 +/- 0.163583 | [2.634460, 3.040626] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.642142 +/- 0.217392 | [2.372257, 2.912027] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.780672 +/- 0.215561 | [2.513061, 3.048284] | 5 |
| Tuned spatial-KLA AA | RMSE | 4.120264 +/- 0.284194 | [3.767447, 4.473081] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.727303 +/- 0.438718 | [3.182650, 4.271956] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.944791 +/- 0.406628 | [3.439977, 4.449605] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.566500 +/- 0.088288 | [0.456893, 0.676107] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.560000 +/- 0.119269 | [0.411932, 0.708068] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.560000 +/- 0.119269 | [0.411932, 0.708068] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.195401 +/- 0.055401 | [0.126623, 0.264179] | 6.89% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.056871 +/- 0.059077 | [-0.016471, 0.130213] | 2.00% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.392961 +/- 0.167275 | [0.185295, 0.600627] | 9.54% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.175473 +/- 0.140689 | [0.000812, 0.350133] | 4.26% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.006500 +/- 0.043024 | [-0.046913, 0.059913] | 1.15% | 2/5 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.006500 +/- 0.043024 | [-0.046913, 0.059913] | 1.15% | 2/5 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 37.291680 +/- 0.481907 | 1.553820 | 1.000x | 5 |
| Neighborhood label-barycenter spatial-KLA AA | 56.809697 +/- 1.803807 | 2.367071 | 1.523x | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 53.479854 +/- 1.619396 | 2.228327 | 1.434x | 5 |
