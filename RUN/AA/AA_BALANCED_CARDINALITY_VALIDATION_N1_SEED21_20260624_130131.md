# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 13:04:07

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 21 (fixed=1)
- trialSeeds: 22
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
- pDropLevels: [0.2 0.35 0.5 0.7]
- pDropLevelCounts: [1 3 2 2]

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
- Trial 1: [0.7 0.5 0.7 0.5 0.35 0.2 0.35 0.35]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 22 | Tuned spatial-KLA AA | 2.101216 | 2.492763 | 0.093750 |
| 1 | 22 | Neighborhood label-barycenter spatial-KLA AA | 0.466321 | 0.348431 | 0.045000 |
| 1 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.222720 | 1.658173 | 0.045000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.101216 | 2.492763 | 0.093750 |
| Neighborhood label-barycenter spatial-KLA AA | 0.466321 | 0.348431 | 0.045000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.222720 | 1.658173 | 0.045000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.101216 +/- 0.000000 | [2.101216, 2.101216] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.466321 +/- 0.000000 | [0.466321, 0.466321] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.222720 +/- 0.000000 | [1.222720, 1.222720] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 2.492763 +/- 0.000000 | [2.492763, 2.492763] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.348431 +/- 0.000000 | [0.348431, 0.348431] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.658173 +/- 0.000000 | [1.658173, 1.658173] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.093750 +/- 0.000000 | [0.093750, 0.093750] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.045000 +/- 0.000000 | [0.045000, 0.045000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.045000 +/- 0.000000 | [0.045000, 0.045000] | 1 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.634895 +/- 0.000000 | [1.634895, 1.634895] | 77.81% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.878497 +/- 0.000000 | [0.878497, 0.878497] | 41.81% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 2.144333 +/- 0.000000 | [2.144333, 2.144333] | 86.02% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.834591 +/- 0.000000 | [0.834591, 0.834591] | 33.48% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.048750 +/- 0.000000 | [0.048750, 0.048750] | 52.00% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.048750 +/- 0.000000 | [0.048750, 0.048750] | 52.00% | 1/1 | 1 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 22 | Tuned spatial-KLA AA | 2.438530 | 3.951127 | 0.173750 |
| 1 | 22 | Neighborhood label-barycenter spatial-KLA AA | 2.066908 | 4.011443 | 0.135000 |
| 1 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.276603 | 3.956631 | 0.135000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.438530 | 3.951127 | 0.173750 |
| Neighborhood label-barycenter spatial-KLA AA | 2.066908 | 4.011443 | 0.135000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.276603 | 3.956631 | 0.135000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.438530 +/- 0.000000 | [2.438530, 2.438530] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.066908 +/- 0.000000 | [2.066908, 2.066908] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.276603 +/- 0.000000 | [2.276603, 2.276603] | 1 |
| Tuned spatial-KLA AA | RMSE | 3.951127 +/- 0.000000 | [3.951127, 3.951127] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 4.011443 +/- 0.000000 | [4.011443, 4.011443] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.956631 +/- 0.000000 | [3.956631, 3.956631] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.173750 +/- 0.000000 | [0.173750, 0.173750] | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.135000 +/- 0.000000 | [0.135000, 0.135000] | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.135000 +/- 0.000000 | [0.135000, 0.135000] | 1 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.371622 +/- 0.000000 | [0.371622, 0.371622] | 15.24% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.161927 +/- 0.000000 | [0.161927, 0.161927] | 6.64% | 1/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | -0.060317 +/- 0.000000 | [-0.060317, -0.060317] | -1.53% | 0/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | -0.005504 +/- 0.000000 | [-0.005504, -0.005504] | -0.14% | 0/1 | 1 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.038750 +/- 0.000000 | [0.038750, 0.038750] | 22.30% | 1/1 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.038750 +/- 0.000000 | [0.038750, 0.038750] | 22.30% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 31.506451 +/- 0.000000 | 1.312769 | 1.000x | 1 |
| Neighborhood label-barycenter spatial-KLA AA | 56.099282 +/- 0.000000 | 2.337470 | 1.781x | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 52.320411 +/- 0.000000 | 2.180017 | 1.661x | 1 |
