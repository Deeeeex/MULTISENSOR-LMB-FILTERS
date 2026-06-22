# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 17:05:11

Comparison order: Cross-local label-consensus spatial-KLA AA -> Reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
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
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.5 0.2]
- Trial 3: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0 0.1 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 5: [0.1 0.1 0.1 0.5 0.2 0 0.1 0.5]
- Trial 6: [0.1 0.2 0.1 0.5 0.1 0.5 0.1 0]
- Trial 7: [0.1 0 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 8: [0 0.1 0.5 0.1 0.1 0.2 0.5 0.1]
- Trial 9: [0.5 0.1 0.1 0 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0 0.5 0.1 0.2 0.1 0.1]
- Trial 11: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 12: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 13: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 14: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 15: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]
- Trial 16: [0.1 0.2 0.1 0.5 0 0.1 0.5 0.1]
- Trial 17: [0.1 0.5 0.1 0.2 0.1 0.5 0.1 0]
- Trial 18: [0.1 0.5 0.1 0.5 0.2 0.1 0.1 0]
- Trial 19: [0.1 0.1 0.1 0.1 0.5 0.5 0 0.2]
- Trial 20: [0.5 0.5 0 0.1 0.1 0.1 0.1 0.2]
- Trial 21: [0.5 0.2 0.5 0.1 0.1 0 0.1 0.1]
- Trial 22: [0.1 0.1 0.1 0.5 0 0.2 0.1 0.5]
- Trial 23: [0.5 0.2 0.1 0.1 0.1 0.1 0 0.5]
- Trial 24: [0.1 0.5 0.2 0 0.5 0.1 0.1 0.1]
- Trial 25: [0.1 0.2 0.1 0.5 0.5 0 0.1 0.1]
- Trial 26: [0.1 0.1 0.5 0.1 0 0.5 0.1 0.2]
- Trial 27: [0.1 0.5 0.1 0.1 0 0.5 0.1 0.2]
- Trial 28: [0 0.5 0.1 0.2 0.1 0.5 0.1 0.1]
- Trial 29: [0.1 0.1 0 0.1 0.5 0.1 0.2 0.5]
- Trial 30: [0.1 0.1 0.5 0 0.1 0.5 0.2 0.1]
- Trial 31: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]
- Trial 32: [0.5 0.5 0.1 0.1 0.2 0.1 0 0.1]
- Trial 33: [0.1 0.1 0.1 0 0.5 0.1 0.2 0.5]
- Trial 34: [0.2 0.1 0.1 0 0.5 0.5 0.1 0.1]
- Trial 35: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 36: [0.5 0 0.1 0.5 0.2 0.1 0.1 0.1]
- Trial 37: [0.2 0.5 0.1 0.1 0.1 0.5 0 0.1]
- Trial 38: [0.1 0.5 0 0.1 0.5 0.1 0.1 0.2]
- Trial 39: [0.2 0.5 0 0.1 0.1 0.1 0.1 0.5]
- Trial 40: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 41: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 42: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 43: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 44: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 45: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 46: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 47: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 48: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 49: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 50: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 1 | 2 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 3 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 3 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 4 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 4 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 5 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 5 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 6 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 6 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 7 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 7 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 8 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 8 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 9 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 9 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 10 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 10 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 11 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 11 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 12 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 12 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 13 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 13 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 14 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 14 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 15 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 15 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 16 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 16 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 17 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 17 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 18 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 18 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 19 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 19 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 20 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 20 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 21 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 21 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 22 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 22 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 23 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 23 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 24 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 24 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 25 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 25 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 26 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 26 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 27 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 27 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 28 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 28 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 29 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 29 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 30 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 30 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 31 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 31 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 32 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 32 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 33 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 33 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 34 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 34 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 35 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 35 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 36 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 36 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 37 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 37 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 38 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 38 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 39 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 39 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 40 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 40 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 41 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 41 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 42 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 42 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 43 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 43 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 44 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 44 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 45 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 45 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 46 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 46 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 47 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 47 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 48 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 48 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 49 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 49 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 50 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 50 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 51 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 51 | Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| Reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Cross-local label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Cross-local label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |

## Paired Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Cross-local label-consensus spatial-KLA AA | 1.645476 | 3.343985 | 0.071200 |
| Reference-only label-consensus spatial-KLA AA | 1.781249 | 3.463664 | 0.071200 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cross-local label-consensus spatial-KLA AA | E-OSPA | 1.645476 +/- 0.065546 | [1.627308, 1.663644] | 50 |
| Reference-only label-consensus spatial-KLA AA | E-OSPA | 1.781249 +/- 0.069513 | [1.761981, 1.800516] | 50 |
| Cross-local label-consensus spatial-KLA AA | RMSE | 3.343985 +/- 0.410326 | [3.230249, 3.457722] | 50 |
| Reference-only label-consensus spatial-KLA AA | RMSE | 3.463664 +/- 0.401105 | [3.352483, 3.574845] | 50 |
| Cross-local label-consensus spatial-KLA AA | CardErr | 0.071200 +/- 0.018029 | [0.066202, 0.076198] | 50 |
| Reference-only label-consensus spatial-KLA AA | CardErr | 0.071200 +/- 0.018029 | [0.066202, 0.076198] | 50 |

## Paired Local-Metric Improvements Relative to Cross-local label-consensus spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Reference-only label-consensus spatial-KLA AA | E-OSPA | -0.135773 +/- 0.033016 | [-0.144924, -0.126621] | -8.25% | 0/50 | 1.776e-15 |
| Reference-only label-consensus spatial-KLA AA | RMSE | -0.119678 +/- 0.088228 | [-0.144134, -0.095223] | -3.58% | 4/50 | 4.462e-10 |
| Reference-only label-consensus spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/50 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Cross-local label-consensus spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Cross-local label-consensus spatial-KLA AA | 72.206077 +/- 16.406475 | 3.008587 | 1.000x | 50 |
| Reference-only label-consensus spatial-KLA AA | 70.858710 +/- 14.577054 | 2.952446 | 0.987x | 50 |
