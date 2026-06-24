# AA Balanced/Cardinality Validation

Generated at: 2026-06-25 04:49:27

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 51 (fixed=1)
- trialSeeds: [52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81 82 83 84 85 86 87 88 89 90 91 92 93 94 95 96 97 98 99 100 101]
- lmbParallelUpdateMode: AA
- scenarioLabel: full-topology-formation
- neighborMapMode: full
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
- Trial 1: [0.5 0.1 0.1 0.5 0 0.2 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 3: [0 0.5 0.5 0.1 0.1 0.1 0.1 0.2]
- Trial 4: [0.5 0 0.5 0.1 0.2 0.1 0.1 0.1]
- Trial 5: [0.1 0.5 0.1 0.1 0.5 0.2 0 0.1]
- Trial 6: [0.2 0.1 0 0.5 0.1 0.5 0.1 0.1]
- Trial 7: [0.1 0.2 0.1 0.5 0 0.5 0.1 0.1]
- Trial 8: [0.5 0.1 0.1 0.1 0.5 0.2 0 0.1]
- Trial 9: [0.2 0.5 0.1 0 0.1 0.5 0.1 0.1]
- Trial 10: [0 0.1 0.1 0.1 0.5 0.2 0.1 0.5]
- Trial 11: [0.1 0.1 0 0.2 0.1 0.5 0.1 0.5]
- Trial 12: [0.1 0.1 0 0.5 0.2 0.1 0.5 0.1]
- Trial 13: [0.1 0.1 0.2 0 0.1 0.1 0.5 0.5]
- Trial 14: [0.1 0.1 0 0.2 0.5 0.1 0.1 0.5]
- Trial 15: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 16: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 17: [0.1 0.2 0.1 0 0.1 0.1 0.5 0.5]
- Trial 18: [0 0.5 0.1 0.5 0.1 0.2 0.1 0.1]
- Trial 19: [0.1 0.5 0.1 0.1 0.5 0.1 0 0.2]
- Trial 20: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 21: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 22: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 23: [0.5 0.1 0.5 0.2 0.1 0 0.1 0.1]
- Trial 24: [0.1 0.1 0.1 0.5 0.2 0 0.5 0.1]
- Trial 25: [0 0.2 0.1 0.5 0.1 0.1 0.1 0.5]
- Trial 26: [0.1 0 0.5 0.5 0.2 0.1 0.1 0.1]
- Trial 27: [0.1 0.1 0.5 0.1 0 0.2 0.1 0.5]
- Trial 28: [0.1 0.1 0.2 0.1 0.1 0.5 0.5 0]
- Trial 29: [0.5 0.2 0.5 0 0.1 0.1 0.1 0.1]
- Trial 30: [0.1 0.1 0.5 0.1 0.5 0.1 0.2 0]
- Trial 31: [0.1 0.1 0.1 0.5 0.1 0.2 0 0.5]
- Trial 32: [0.1 0.2 0.5 0.1 0 0.5 0.1 0.1]
- Trial 33: [0.2 0.5 0.1 0.1 0.1 0.5 0.1 0]
- Trial 34: [0.1 0.5 0.5 0 0.1 0.1 0.1 0.2]
- Trial 35: [0.2 0.5 0.5 0.1 0.1 0.1 0 0.1]
- Trial 36: [0.1 0.1 0.2 0.1 0.5 0.5 0.1 0]
- Trial 37: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 38: [0.1 0 0.1 0.1 0.5 0.1 0.2 0.5]
- Trial 39: [0.5 0.1 0.1 0.1 0.1 0.2 0 0.5]
- Trial 40: [0.1 0.2 0.5 0.1 0.5 0.1 0 0.1]
- Trial 41: [0.1 0.5 0.5 0.2 0.1 0 0.1 0.1]
- Trial 42: [0.1 0.1 0.5 0.1 0 0.2 0.5 0.1]
- Trial 43: [0.1 0 0.1 0.5 0.5 0.2 0.1 0.1]
- Trial 44: [0.1 0.1 0.5 0.1 0.2 0.1 0.5 0]
- Trial 45: [0 0.1 0.5 0.1 0.5 0.1 0.2 0.1]
- Trial 46: [0.5 0.2 0.5 0.1 0.1 0.1 0.1 0]
- Trial 47: [0.1 0.2 0 0.5 0.1 0.5 0.1 0.1]
- Trial 48: [0.5 0 0.1 0.1 0.1 0.5 0.2 0.1]
- Trial 49: [0.1 0.1 0.2 0.5 0.5 0.1 0.1 0]
- Trial 50: [0.1 0.1 0 0.1 0.5 0.1 0.5 0.2]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 52 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 1 | 52 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 1 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 53 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 53 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 54 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 54 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 55 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 55 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 56 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 56 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 57 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 57 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 58 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 58 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 59 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 59 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 60 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 60 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 61 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 61 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 62 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 62 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 63 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 63 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 64 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 64 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 65 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 65 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 66 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 66 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 67 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 67 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 68 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 68 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 69 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 69 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 70 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 70 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 71 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 71 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 72 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 72 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 73 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 73 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 74 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 74 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 75 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 75 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 76 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 76 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 77 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 77 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 78 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 78 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 79 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 79 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 80 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 80 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 81 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 81 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 82 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 82 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 83 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 83 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 84 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 84 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 85 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 85 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 86 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 86 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 87 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 87 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 88 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 88 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 89 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 89 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 90 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 90 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 91 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 91 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 92 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 92 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 93 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 93 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 94 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 94 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 95 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 95 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 96 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 96 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 97 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 97 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 98 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 98 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 99 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 99 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 100 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 100 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 101 | Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 101 | Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| Neighborhood label-barycenter spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | NaN% | 0/50 | NaN |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 52 | Tuned spatial-KLA AA | 1.639683 | 2.917511 | 0.110000 |
| 1 | 52 | Neighborhood label-barycenter spatial-KLA AA | 1.639683 | 2.917511 | 0.110000 |
| 1 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.639683 | 2.917511 | 0.110000 |
| 2 | 53 | Tuned spatial-KLA AA | 1.563583 | 3.334475 | 0.060000 |
| 2 | 53 | Neighborhood label-barycenter spatial-KLA AA | 1.563583 | 3.334475 | 0.060000 |
| 2 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.563583 | 3.334475 | 0.060000 |
| 3 | 54 | Tuned spatial-KLA AA | 1.519980 | 3.347230 | 0.050000 |
| 3 | 54 | Neighborhood label-barycenter spatial-KLA AA | 1.519980 | 3.347230 | 0.050000 |
| 3 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.519980 | 3.347230 | 0.050000 |
| 4 | 55 | Tuned spatial-KLA AA | 1.691020 | 3.325344 | 0.080000 |
| 4 | 55 | Neighborhood label-barycenter spatial-KLA AA | 1.691020 | 3.325344 | 0.080000 |
| 4 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.691020 | 3.325344 | 0.080000 |
| 5 | 56 | Tuned spatial-KLA AA | 1.601551 | 3.613233 | 0.070000 |
| 5 | 56 | Neighborhood label-barycenter spatial-KLA AA | 1.601551 | 3.613233 | 0.070000 |
| 5 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.601551 | 3.613233 | 0.070000 |
| 6 | 57 | Tuned spatial-KLA AA | 1.713467 | 3.672170 | 0.090000 |
| 6 | 57 | Neighborhood label-barycenter spatial-KLA AA | 1.713467 | 3.672170 | 0.090000 |
| 6 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.713467 | 3.672170 | 0.090000 |
| 7 | 58 | Tuned spatial-KLA AA | 1.937032 | 3.276384 | 0.110000 |
| 7 | 58 | Neighborhood label-barycenter spatial-KLA AA | 1.937032 | 3.276384 | 0.110000 |
| 7 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.937032 | 3.276384 | 0.110000 |
| 8 | 59 | Tuned spatial-KLA AA | 1.587024 | 3.303573 | 0.060000 |
| 8 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.587024 | 3.303573 | 0.060000 |
| 8 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.587024 | 3.303573 | 0.060000 |
| 9 | 60 | Tuned spatial-KLA AA | 1.784668 | 3.504199 | 0.080000 |
| 9 | 60 | Neighborhood label-barycenter spatial-KLA AA | 1.784668 | 3.504199 | 0.080000 |
| 9 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.784668 | 3.504199 | 0.080000 |
| 10 | 61 | Tuned spatial-KLA AA | 1.705530 | 3.882825 | 0.050000 |
| 10 | 61 | Neighborhood label-barycenter spatial-KLA AA | 1.705530 | 3.882825 | 0.050000 |
| 10 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.705530 | 3.882825 | 0.050000 |
| 11 | 62 | Tuned spatial-KLA AA | 1.534280 | 3.008248 | 0.080000 |
| 11 | 62 | Neighborhood label-barycenter spatial-KLA AA | 1.534280 | 3.008248 | 0.080000 |
| 11 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.534280 | 3.008248 | 0.080000 |
| 12 | 63 | Tuned spatial-KLA AA | 1.540119 | 3.265306 | 0.070000 |
| 12 | 63 | Neighborhood label-barycenter spatial-KLA AA | 1.540119 | 3.265306 | 0.070000 |
| 12 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.540119 | 3.265306 | 0.070000 |
| 13 | 64 | Tuned spatial-KLA AA | 1.605026 | 3.784164 | 0.070000 |
| 13 | 64 | Neighborhood label-barycenter spatial-KLA AA | 1.605026 | 3.784164 | 0.070000 |
| 13 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.605026 | 3.784164 | 0.070000 |
| 14 | 65 | Tuned spatial-KLA AA | 1.679272 | 3.635148 | 0.070000 |
| 14 | 65 | Neighborhood label-barycenter spatial-KLA AA | 1.679272 | 3.635148 | 0.070000 |
| 14 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.679272 | 3.635148 | 0.070000 |
| 15 | 66 | Tuned spatial-KLA AA | 1.605691 | 3.331472 | 0.060000 |
| 15 | 66 | Neighborhood label-barycenter spatial-KLA AA | 1.605691 | 3.331472 | 0.060000 |
| 15 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.605691 | 3.331472 | 0.060000 |
| 16 | 67 | Tuned spatial-KLA AA | 1.572679 | 2.865174 | 0.090000 |
| 16 | 67 | Neighborhood label-barycenter spatial-KLA AA | 1.572679 | 2.865174 | 0.090000 |
| 16 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.572679 | 2.865174 | 0.090000 |
| 17 | 68 | Tuned spatial-KLA AA | 1.753953 | 3.130320 | 0.110000 |
| 17 | 68 | Neighborhood label-barycenter spatial-KLA AA | 1.753953 | 3.130320 | 0.110000 |
| 17 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.753953 | 3.130320 | 0.110000 |
| 18 | 69 | Tuned spatial-KLA AA | 1.438042 | 2.795490 | 0.050000 |
| 18 | 69 | Neighborhood label-barycenter spatial-KLA AA | 1.438042 | 2.795490 | 0.050000 |
| 18 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.438042 | 2.795490 | 0.050000 |
| 19 | 70 | Tuned spatial-KLA AA | 1.523628 | 3.306327 | 0.060000 |
| 19 | 70 | Neighborhood label-barycenter spatial-KLA AA | 1.523628 | 3.306327 | 0.060000 |
| 19 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.523628 | 3.306327 | 0.060000 |
| 20 | 71 | Tuned spatial-KLA AA | 1.796033 | 3.540403 | 0.100000 |
| 20 | 71 | Neighborhood label-barycenter spatial-KLA AA | 1.796033 | 3.540403 | 0.100000 |
| 20 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.796033 | 3.540403 | 0.100000 |
| 21 | 72 | Tuned spatial-KLA AA | 1.665603 | 2.920398 | 0.090000 |
| 21 | 72 | Neighborhood label-barycenter spatial-KLA AA | 1.665603 | 2.920398 | 0.090000 |
| 21 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.665603 | 2.920398 | 0.090000 |
| 22 | 73 | Tuned spatial-KLA AA | 1.683543 | 3.442836 | 0.080000 |
| 22 | 73 | Neighborhood label-barycenter spatial-KLA AA | 1.683543 | 3.442836 | 0.080000 |
| 22 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.683543 | 3.442836 | 0.080000 |
| 23 | 74 | Tuned spatial-KLA AA | 1.547926 | 3.303309 | 0.060000 |
| 23 | 74 | Neighborhood label-barycenter spatial-KLA AA | 1.547926 | 3.303309 | 0.060000 |
| 23 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.547926 | 3.303309 | 0.060000 |
| 24 | 75 | Tuned spatial-KLA AA | 1.635907 | 3.384916 | 0.060000 |
| 24 | 75 | Neighborhood label-barycenter spatial-KLA AA | 1.635907 | 3.384916 | 0.060000 |
| 24 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.635907 | 3.384916 | 0.060000 |
| 25 | 76 | Tuned spatial-KLA AA | 1.614400 | 3.823302 | 0.050000 |
| 25 | 76 | Neighborhood label-barycenter spatial-KLA AA | 1.614400 | 3.823302 | 0.050000 |
| 25 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.614400 | 3.823302 | 0.050000 |
| 26 | 77 | Tuned spatial-KLA AA | 1.588951 | 3.000068 | 0.080000 |
| 26 | 77 | Neighborhood label-barycenter spatial-KLA AA | 1.588951 | 3.000068 | 0.080000 |
| 26 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.588951 | 3.000068 | 0.080000 |
| 27 | 78 | Tuned spatial-KLA AA | 1.680790 | 2.808252 | 0.100000 |
| 27 | 78 | Neighborhood label-barycenter spatial-KLA AA | 1.680790 | 2.808252 | 0.100000 |
| 27 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.680790 | 2.808252 | 0.100000 |
| 28 | 79 | Tuned spatial-KLA AA | 1.691563 | 2.948667 | 0.100000 |
| 28 | 79 | Neighborhood label-barycenter spatial-KLA AA | 1.691563 | 2.948667 | 0.100000 |
| 28 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.691563 | 2.948667 | 0.100000 |
| 29 | 80 | Tuned spatial-KLA AA | 1.648053 | 3.253732 | 0.080000 |
| 29 | 80 | Neighborhood label-barycenter spatial-KLA AA | 1.648053 | 3.253732 | 0.080000 |
| 29 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.648053 | 3.253732 | 0.080000 |
| 30 | 81 | Tuned spatial-KLA AA | 1.581305 | 3.764086 | 0.040000 |
| 30 | 81 | Neighborhood label-barycenter spatial-KLA AA | 1.581305 | 3.764086 | 0.040000 |
| 30 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.581305 | 3.764086 | 0.040000 |
| 31 | 82 | Tuned spatial-KLA AA | 1.705336 | 3.399938 | 0.080000 |
| 31 | 82 | Neighborhood label-barycenter spatial-KLA AA | 1.705336 | 3.399938 | 0.080000 |
| 31 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.705336 | 3.399938 | 0.080000 |
| 32 | 83 | Tuned spatial-KLA AA | 1.647375 | 2.815215 | 0.090000 |
| 32 | 83 | Neighborhood label-barycenter spatial-KLA AA | 1.647375 | 2.815215 | 0.090000 |
| 32 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.647375 | 2.815215 | 0.090000 |
| 33 | 84 | Tuned spatial-KLA AA | 1.629474 | 2.702435 | 0.090000 |
| 33 | 84 | Neighborhood label-barycenter spatial-KLA AA | 1.629474 | 2.702435 | 0.090000 |
| 33 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.629474 | 2.702435 | 0.090000 |
| 34 | 85 | Tuned spatial-KLA AA | 1.636372 | 3.012995 | 0.070000 |
| 34 | 85 | Neighborhood label-barycenter spatial-KLA AA | 1.636372 | 3.012995 | 0.070000 |
| 34 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.636372 | 3.012995 | 0.070000 |
| 35 | 86 | Tuned spatial-KLA AA | 1.563542 | 3.114065 | 0.060000 |
| 35 | 86 | Neighborhood label-barycenter spatial-KLA AA | 1.563542 | 3.114065 | 0.060000 |
| 35 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.563542 | 3.114065 | 0.060000 |
| 36 | 87 | Tuned spatial-KLA AA | 1.635621 | 3.377876 | 0.070000 |
| 36 | 87 | Neighborhood label-barycenter spatial-KLA AA | 1.635621 | 3.377876 | 0.070000 |
| 36 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.635621 | 3.377876 | 0.070000 |
| 37 | 88 | Tuned spatial-KLA AA | 1.606079 | 2.938889 | 0.080000 |
| 37 | 88 | Neighborhood label-barycenter spatial-KLA AA | 1.606079 | 2.938889 | 0.080000 |
| 37 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.606079 | 2.938889 | 0.080000 |
| 38 | 89 | Tuned spatial-KLA AA | 1.676431 | 3.261364 | 0.070000 |
| 38 | 89 | Neighborhood label-barycenter spatial-KLA AA | 1.676431 | 3.261364 | 0.070000 |
| 38 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.676431 | 3.261364 | 0.070000 |
| 39 | 90 | Tuned spatial-KLA AA | 1.617242 | 2.940357 | 0.070000 |
| 39 | 90 | Neighborhood label-barycenter spatial-KLA AA | 1.617242 | 2.940357 | 0.070000 |
| 39 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.617242 | 2.940357 | 0.070000 |
| 40 | 91 | Tuned spatial-KLA AA | 1.736126 | 3.890579 | 0.060000 |
| 40 | 91 | Neighborhood label-barycenter spatial-KLA AA | 1.736126 | 3.890579 | 0.060000 |
| 40 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.736126 | 3.890579 | 0.060000 |
| 41 | 92 | Tuned spatial-KLA AA | 1.499329 | 3.424493 | 0.050000 |
| 41 | 92 | Neighborhood label-barycenter spatial-KLA AA | 1.499329 | 3.424493 | 0.050000 |
| 41 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.499329 | 3.424493 | 0.050000 |
| 42 | 93 | Tuned spatial-KLA AA | 1.559711 | 3.038731 | 0.070000 |
| 42 | 93 | Neighborhood label-barycenter spatial-KLA AA | 1.559711 | 3.038731 | 0.070000 |
| 42 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.559711 | 3.038731 | 0.070000 |
| 43 | 94 | Tuned spatial-KLA AA | 1.603536 | 3.326140 | 0.090000 |
| 43 | 94 | Neighborhood label-barycenter spatial-KLA AA | 1.603536 | 3.326140 | 0.090000 |
| 43 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.603536 | 3.326140 | 0.090000 |
| 44 | 95 | Tuned spatial-KLA AA | 1.711350 | 3.482861 | 0.080000 |
| 44 | 95 | Neighborhood label-barycenter spatial-KLA AA | 1.711350 | 3.482861 | 0.080000 |
| 44 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.711350 | 3.482861 | 0.080000 |
| 45 | 96 | Tuned spatial-KLA AA | 1.617357 | 3.449116 | 0.060000 |
| 45 | 96 | Neighborhood label-barycenter spatial-KLA AA | 1.617357 | 3.449116 | 0.060000 |
| 45 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.617357 | 3.449116 | 0.060000 |
| 46 | 97 | Tuned spatial-KLA AA | 1.669576 | 2.547591 | 0.080000 |
| 46 | 97 | Neighborhood label-barycenter spatial-KLA AA | 1.669576 | 2.547591 | 0.080000 |
| 46 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.669576 | 2.547591 | 0.080000 |
| 47 | 98 | Tuned spatial-KLA AA | 1.548387 | 3.573493 | 0.060000 |
| 47 | 98 | Neighborhood label-barycenter spatial-KLA AA | 1.548387 | 3.573493 | 0.060000 |
| 47 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.548387 | 3.573493 | 0.060000 |
| 48 | 99 | Tuned spatial-KLA AA | 1.608280 | 3.519174 | 0.040000 |
| 48 | 99 | Neighborhood label-barycenter spatial-KLA AA | 1.608280 | 3.519174 | 0.040000 |
| 48 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.608280 | 3.519174 | 0.040000 |
| 49 | 100 | Tuned spatial-KLA AA | 1.695895 | 3.487107 | 0.060000 |
| 49 | 100 | Neighborhood label-barycenter spatial-KLA AA | 1.695895 | 3.487107 | 0.060000 |
| 49 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.695895 | 3.487107 | 0.060000 |
| 50 | 101 | Tuned spatial-KLA AA | 1.619216 | 3.060886 | 0.080000 |
| 50 | 101 | Neighborhood label-barycenter spatial-KLA AA | 1.619216 | 3.060886 | 0.080000 |
| 50 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.619216 | 3.060886 | 0.080000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 1.634331 | 3.277037 | 0.073400 |
| Neighborhood label-barycenter spatial-KLA AA | 1.634331 | 3.277037 | 0.073400 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.634331 | 3.277037 | 0.073400 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 1.634331 +/- 0.085398 | [1.610660, 1.658002] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 1.634331 +/- 0.085398 | [1.610660, 1.658002] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 1.634331 +/- 0.085398 | [1.610660, 1.658002] | 50 |
| Tuned spatial-KLA AA | RMSE | 3.277037 +/- 0.322032 | [3.187775, 3.366300] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.277037 +/- 0.322032 | [3.187775, 3.366300] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.277037 +/- 0.322032 | [3.187775, 3.366300] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.073400 +/- 0.017683 | [0.068498, 0.078302] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.073400 +/- 0.017683 | [0.068498, 0.078302] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.073400 +/- 0.017683 | [0.068498, 0.078302] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | -0.000000 +/- 0.000000 | [-0.000000, 0.000000] | -0.00% | 17/50 | 0.6291 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/50 | NaN |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.000000 +/- 0.000000 | [-0.000000, 0.000000] | 0.00% | 13/50 | 1 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/50 | NaN |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/50 | NaN |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/50 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 66.502786 +/- 8.506067 | 2.770949 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 129.507081 +/- 10.072274 | 5.396128 | 1.962x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 122.815012 +/- 14.755503 | 5.117292 | 1.864x | 50 |
