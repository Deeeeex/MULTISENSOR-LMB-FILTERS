# AA Balanced/Cardinality Validation

Generated at: 2026-06-27 16:36:42

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 5
- baseSeed: 71 (fixed=1)
- trialSeeds: [72 73 74 75 76]
- lmbParallelUpdateMode: AA
- scenarioLabel: maneuver-crossing-assignment
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
- Trial 2: [0.1 0.5 0.2 0 0.1 0.5 0.1 0.1]
- Trial 3: [0.1 0.1 0.1 0.5 0.2 0.5 0.1 0]
- Trial 4: [0.1 0.5 0.5 0 0.1 0.1 0.1 0.2]
- Trial 5: [0 0.1 0.5 0.2 0.5 0.1 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.618293 | 6.253516 | 0.229167 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 0.887164 | 0.956680 | 0.187500 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.687278 | 4.025160 | 0.187500 |
| 2 | 73 | Tuned spatial-KLA AA | 2.858435 | 6.415267 | 0.260417 |
| 2 | 73 | Neighborhood label-barycenter spatial-KLA AA | 0.766138 | 0.879954 | 0.125000 |
| 2 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.485505 | 2.801469 | 0.125000 |
| 3 | 74 | Tuned spatial-KLA AA | 3.010729 | 5.642628 | 0.234375 |
| 3 | 74 | Neighborhood label-barycenter spatial-KLA AA | 0.763732 | 0.749671 | 0.104167 |
| 3 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.585668 | 3.462325 | 0.104167 |
| 4 | 75 | Tuned spatial-KLA AA | 2.569193 | 7.613947 | 0.187500 |
| 4 | 75 | Neighborhood label-barycenter spatial-KLA AA | 0.900431 | 1.286971 | 0.125000 |
| 4 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.499586 | 4.299093 | 0.125000 |
| 5 | 76 | Tuned spatial-KLA AA | 2.800611 | 6.246042 | 0.312500 |
| 5 | 76 | Neighborhood label-barycenter spatial-KLA AA | 0.892002 | 1.047329 | 0.125000 |
| 5 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.528029 | 3.837303 | 0.125000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.771452 | 6.434280 | 0.244792 |
| Neighborhood label-barycenter spatial-KLA AA | 0.841893 | 0.984121 | 0.133333 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.557213 | 3.685070 | 0.133333 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.771452 +/- 0.180303 | [2.547613, 2.995291] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.841893 +/- 0.070418 | [0.754471, 0.929315] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.557213 +/- 0.082221 | [1.455139, 1.659287] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 6.434280 +/- 0.722288 | [5.537585, 7.330975] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.984121 +/- 0.201423 | [0.734062, 1.234180] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 3.685070 +/- 0.580135 | [2.964853, 4.405287] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.244792 +/- 0.045999 | [0.187686, 0.301898] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.133333 +/- 0.031595 | [0.094109, 0.172558] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.133333 +/- 0.031595 | [0.094109, 0.172558] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.929559 +/- 0.242424 | [1.628597, 2.230520] | 69.62% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.214239 +/- 0.208666 | [0.955187, 1.473291] | 43.81% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 5.450159 +/- 0.541587 | [4.777798, 6.122521] | 84.71% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 2.749210 +/- 0.666770 | [1.921438, 3.576982] | 42.73% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.111458 +/- 0.059109 | [0.038076, 0.184841] | 45.53% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.111458 +/- 0.059109 | [0.038076, 0.184841] | 45.53% | 5/5 | 0.0625 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.928595 | 2.714397 | 0.729167 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 2.733518 | 4.241043 | 0.729167 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.842239 | 2.594949 | 0.729167 |
| 2 | 73 | Tuned spatial-KLA AA | 2.993535 | 2.900235 | 0.781250 |
| 2 | 73 | Neighborhood label-barycenter spatial-KLA AA | 2.709872 | 3.791992 | 0.666667 |
| 2 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.840275 | 2.557887 | 0.666667 |
| 3 | 74 | Tuned spatial-KLA AA | 3.147152 | 3.008433 | 0.807292 |
| 3 | 74 | Neighborhood label-barycenter spatial-KLA AA | 2.871305 | 3.313081 | 0.687500 |
| 3 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 3.004818 | 2.801605 | 0.687500 |
| 4 | 75 | Tuned spatial-KLA AA | 2.791107 | 2.451977 | 0.770833 |
| 4 | 75 | Neighborhood label-barycenter spatial-KLA AA | 2.614649 | 4.448474 | 0.708333 |
| 4 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.663911 | 2.275757 | 0.708333 |
| 5 | 76 | Tuned spatial-KLA AA | 2.924623 | 2.597579 | 0.906250 |
| 5 | 76 | Neighborhood label-barycenter spatial-KLA AA | 2.741726 | 4.013487 | 0.791667 |
| 5 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.741259 | 2.353364 | 0.791667 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.957002 | 2.734524 | 0.798958 |
| Neighborhood label-barycenter spatial-KLA AA | 2.734214 | 3.961615 | 0.716667 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.818501 | 2.516712 | 0.716667 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.957002 +/- 0.129294 | [2.796488, 3.117517] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.734214 +/- 0.091841 | [2.620197, 2.848231] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.818501 +/- 0.128078 | [2.659496, 2.977505] | 5 |
| Tuned spatial-KLA AA | RMSE | 2.734524 +/- 0.224432 | [2.455900, 3.013148] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.961615 +/- 0.437941 | [3.417927, 4.505304] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.516712 +/- 0.208400 | [2.257991, 2.775434] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.798958 +/- 0.066250 | [0.716711, 0.881206] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.716667 +/- 0.047962 | [0.657124, 0.776210] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.716667 +/- 0.047962 | [0.657124, 0.776210] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.222789 +/- 0.052504 | [0.157607, 0.287970] | 7.53% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.138502 +/- 0.035680 | [0.094207, 0.182797] | 4.68% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | -1.227091 +/- 0.648213 | [-2.031825, -0.422357] | -44.87% | 0/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.217812 +/- 0.083265 | [0.114441, 0.321183] | 7.97% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.082292 +/- 0.051612 | [0.018217, 0.146367] | 10.30% | 4/5 | 0.125 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.082292 +/- 0.051612 | [0.018217, 0.146367] | 10.30% | 4/5 | 0.125 |

## Scenario Window Metrics
Windowed metrics are restricted to the fixed stress interval `[9 17]` (indices `[9 10 11 12 13 14 15 16 17]`). They are intended for maneuver/crossing diagnostics and should not be mixed with whole-run formation metrics.

### Per-Trial Scenario-Window Network Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.315274 | 2.169030 | 0.125000 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 0.564400 | 0.353552 | 0.111111 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.490487 | 1.356088 | 0.111111 |
| 2 | 73 | Tuned spatial-KLA AA | 2.656563 | 2.367707 | 0.277778 |
| 2 | 73 | Neighborhood label-barycenter spatial-KLA AA | 0.681872 | 0.339509 | 0.222222 |
| 2 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.447850 | 1.197854 | 0.222222 |
| 3 | 74 | Tuned spatial-KLA AA | 2.953511 | 2.791968 | 0.361111 |
| 3 | 74 | Neighborhood label-barycenter spatial-KLA AA | 0.622923 | 0.354262 | 0.166667 |
| 3 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.355223 | 1.190479 | 0.166667 |
| 4 | 75 | Tuned spatial-KLA AA | 2.426520 | 2.162755 | 0.208333 |
| 4 | 75 | Neighborhood label-barycenter spatial-KLA AA | 0.758069 | 0.366028 | 0.222222 |
| 4 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.272933 | 0.991854 | 0.222222 |
| 5 | 76 | Tuned spatial-KLA AA | 2.586284 | 2.442452 | 0.277778 |
| 5 | 76 | Neighborhood label-barycenter spatial-KLA AA | 0.581885 | 0.357993 | 0.055556 |
| 5 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.388442 | 1.193057 | 0.055556 |

### Scenario-Window Network Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.587630 +/- 0.244314 | [2.284324, 2.890937] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.641830 +/- 0.079154 | [0.543563, 0.740097] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.390987 +/- 0.084196 | [1.286460, 1.495514] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 2.386782 +/- 0.257509 | [2.067094, 2.706470] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.354269 +/- 0.009624 | [0.342320, 0.366217] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.185866 +/- 0.129260 | [1.025394, 1.346339] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.250000 +/- 0.088388 | [0.140269, 0.359731] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.155556 +/- 0.072436 | [0.065629, 0.245482] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.155556 +/- 0.072436 | [0.065629, 0.245482] | 5 |

### Scenario-Window Network Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.945800 +/- 0.258481 | [1.624906, 2.266695] | 75.20% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.196643 +/- 0.274589 | [0.855751, 1.537536] | 46.24% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 2.032514 +/- 0.259614 | [1.710212, 2.354815] | 85.16% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.200916 +/- 0.280713 | [0.852421, 1.549411] | 50.32% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.094444 +/- 0.107314 | [-0.038782, 0.227670] | 37.78% | 4/5 | 0.375 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.094444 +/- 0.107314 | [-0.038782, 0.227670] | 37.78% | 4/5 | 0.375 |

### Per-Trial Scenario-Window Local Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 72 | Tuned spatial-KLA AA | 2.670122 | 2.366493 | 0.125000 |
| 1 | 72 | Neighborhood label-barycenter spatial-KLA AA | 2.430426 | 2.050042 | 0.111111 |
| 1 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.573311 | 2.307712 | 0.111111 |
| 2 | 73 | Tuned spatial-KLA AA | 2.671412 | 2.322119 | 0.305556 |
| 2 | 73 | Neighborhood label-barycenter spatial-KLA AA | 2.357810 | 1.833464 | 0.222222 |
| 2 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.414094 | 2.073094 | 0.222222 |
| 3 | 74 | Tuned spatial-KLA AA | 2.851501 | 2.488051 | 0.361111 |
| 3 | 74 | Neighborhood label-barycenter spatial-KLA AA | 2.450678 | 1.868867 | 0.166667 |
| 3 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.621002 | 2.287659 | 0.166667 |
| 4 | 75 | Tuned spatial-KLA AA | 2.540016 | 2.180247 | 0.263889 |
| 4 | 75 | Neighborhood label-barycenter spatial-KLA AA | 2.219290 | 1.750697 | 0.222222 |
| 4 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.346569 | 1.999929 | 0.222222 |
| 5 | 76 | Tuned spatial-KLA AA | 2.670616 | 2.206088 | 0.444444 |
| 5 | 76 | Neighborhood label-barycenter spatial-KLA AA | 2.324639 | 1.806159 | 0.388889 |
| 5 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.502548 | 2.073469 | 0.388889 |

### Scenario-Window Local Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.680733 +/- 0.110978 | [2.542958, 2.818509] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.356569 +/- 0.092436 | [2.241812, 2.471325] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.491505 +/- 0.112453 | [2.351899, 2.631111] | 5 |
| Tuned spatial-KLA AA | RMSE | 2.312600 +/- 0.125140 | [2.157242, 2.467957] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 1.861846 +/- 0.113719 | [1.720668, 2.003023] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.148373 +/- 0.139734 | [1.974898, 2.321848] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.300000 +/- 0.118910 | [0.152377, 0.447623] | 5 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.222222 +/- 0.103935 | [0.093191, 0.351254] | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.222222 +/- 0.103935 | [0.093191, 0.351254] | 5 |

### Scenario-Window Local Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.324165 +/- 0.058332 | [0.251747, 0.396582] | 12.09% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.189228 +/- 0.061940 | [0.112333, 0.266124] | 7.06% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.450754 +/- 0.112778 | [0.310745, 0.590764] | 19.49% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.164227 +/- 0.072242 | [0.074541, 0.253913] | 7.10% | 5/5 | 0.0625 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.077778 +/- 0.069860 | [-0.008951, 0.164506] | 25.93% | 5/5 | 0.0625 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.077778 +/- 0.069860 | [-0.008951, 0.164506] | 25.93% | 5/5 | 0.0625 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 17.000366 +/- 0.829764 | 0.708349 | 1.000x | 5 |
| Neighborhood label-barycenter spatial-KLA AA | 41.776416 +/- 2.891881 | 1.740684 | 2.456x | 5 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 38.241181 +/- 2.556659 | 1.593383 | 2.249x | 5 |
