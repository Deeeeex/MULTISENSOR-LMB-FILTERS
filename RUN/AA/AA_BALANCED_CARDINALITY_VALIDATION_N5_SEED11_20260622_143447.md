# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 14:48:15

Comparison order: Cross-local label-consensus spatial-KLA AA -> Reference-only label-consensus spatial-KLA AA

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
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Reference-only label-consensus spatial-KLA AA
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
- crossLocalConsensusProjectionMode: reference-only
- crossLocalConsensusCutoff: NaN
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
| 1 | 12 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 1 | 12 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 13 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 13 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 14 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 14 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 15 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 15 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 16 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 16 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |
| Reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |
| Cross-local label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |
| Reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |
| Cross-local label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |
| Reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 5 |

## Paired Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/5 | NaN |
| Reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/5 | NaN |
| Reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/5 | NaN |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Cross-local label-consensus spatial-KLA AA | 1.660378 | 3.290166 | 0.068000 |
| Reference-only label-consensus spatial-KLA AA | 1.797598 | 3.468548 | 0.068000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | E-OSPA | 1.660378 +/- 0.042014 | [1.608219, 1.712538] | 5 |
| Reference-only label-consensus spatial-KLA AA | E-OSPA | 1.797598 +/- 0.067379 | [1.713949, 1.881246] | 5 |
| Cross-local label-consensus spatial-KLA AA | RMSE | 3.290166 +/- 0.425366 | [2.762089, 3.818243] | 5 |
| Reference-only label-consensus spatial-KLA AA | RMSE | 3.468548 +/- 0.438062 | [2.924710, 4.012387] | 5 |
| Cross-local label-consensus spatial-KLA AA | CardErr | 0.068000 +/- 0.010954 | [0.054400, 0.081600] | 5 |
| Reference-only label-consensus spatial-KLA AA | CardErr | 0.068000 +/- 0.010954 | [0.054400, 0.081600] | 5 |

## Paired Local-Metric Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Reference-only label-consensus spatial-KLA AA | E-OSPA | -0.137219 +/- 0.040106 | [-0.187010, -0.087429] | -8.26% | 0/5 | 0.0625 |
| Reference-only label-consensus spatial-KLA AA | RMSE | -0.178383 +/- 0.113887 | [-0.319769, -0.036996] | -5.42% | 0/5 | 0.0625 |
| Reference-only label-consensus spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Cross-local label-consensus spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Cross-local label-consensus spatial-KLA AA | 70.696447 +/- 12.370556 | 2.945685 | 1.000x | 5 |
| Reference-only label-consensus spatial-KLA AA | 72.318146 +/- 12.870647 | 3.013256 | 1.029x | 5 |
