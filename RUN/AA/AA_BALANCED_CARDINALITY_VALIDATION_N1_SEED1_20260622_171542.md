# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 17:20:23

Comparison order: Cross-local label-consensus spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

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
### Cross-local label-consensus spatial-KLA AA
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
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 1 | 2 | Neighborhood label-barycenter spatial-KLA AA | 0.312781 | 0.224735 | 0.015000 |
| 1 | 2 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.112777 | 0.944124 | 0.015000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| Neighborhood label-barycenter spatial-KLA AA | 0.312781 | 0.224735 | 0.015000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.112777 | 0.944124 | 0.015000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.312781 +/- 0.000000 | [0.312781, 0.312781] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.112777 +/- 0.000000 | [1.112777, 1.112777] | 1 |
| Cross-local label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.224735 +/- 0.000000 | [0.224735, 0.224735] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.944124 +/- 0.000000 | [0.944124, 0.944124] | 1 |
| Cross-local label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.015000 +/- 0.000000 | [0.015000, 0.015000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.015000 +/- 0.000000 | [0.015000, 0.015000] | 1 |

## Paired Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | -0.312781 +/- 0.000000 | [-0.312781, -0.312781] | NaN% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | -1.112777 +/- 0.000000 | [-1.112777, -1.112777] | NaN% | 0/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | -0.224735 +/- 0.000000 | [-0.224735, -0.224735] | NaN% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | -0.944124 +/- 0.000000 | [-0.944124, -0.944124] | NaN% | 0/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | -0.015000 +/- 0.000000 | [-0.015000, -0.015000] | NaN% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | -0.015000 +/- 0.000000 | [-0.015000, -0.015000] | NaN% | 0/1 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Cross-local label-consensus spatial-KLA AA | 1.634304 | 3.778896 | 0.060000 |
| Neighborhood label-barycenter spatial-KLA AA | 1.678544 | 3.814444 | 0.065000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.938542 | 4.053640 | 0.065000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | E-OSPA | 1.634304 +/- 0.000000 | [1.634304, 1.634304] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 1.678544 +/- 0.000000 | [1.678544, 1.678544] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 1.938542 +/- 0.000000 | [1.938542, 1.938542] | 1 |
| Cross-local label-consensus spatial-KLA AA | RMSE | 3.778896 +/- 0.000000 | [3.778896, 3.778896] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.814444 +/- 0.000000 | [3.814444, 3.814444] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.053640 +/- 0.000000 | [4.053640, 4.053640] | 1 |
| Cross-local label-consensus spatial-KLA AA | CardErr | 0.060000 +/- 0.000000 | [0.060000, 0.060000] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.065000 +/- 0.000000 | [0.065000, 0.065000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.065000 +/- 0.000000 | [0.065000, 0.065000] | 1 |

## Paired Local-Metric Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | -0.044240 +/- 0.000000 | [-0.044240, -0.044240] | -2.71% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | -0.304237 +/- 0.000000 | [-0.304237, -0.304237] | -18.62% | 0/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | -0.035548 +/- 0.000000 | [-0.035548, -0.035548] | -0.94% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | -0.274744 +/- 0.000000 | [-0.274744, -0.274744] | -7.27% | 0/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | -0.005000 +/- 0.000000 | [-0.005000, -0.005000] | -8.33% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | -0.005000 +/- 0.000000 | [-0.005000, -0.005000] | -8.33% | 0/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Cross-local label-consensus spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Cross-local label-consensus spatial-KLA AA | 66.710909 +/- 0.000000 | 2.779621 | 1.000x | 1 |
| Neighborhood label-barycenter spatial-KLA AA | 98.451734 +/- 0.000000 | 4.102156 | 1.476x | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 91.984120 +/- 0.000000 | 3.832672 | 1.379x | 1 |
