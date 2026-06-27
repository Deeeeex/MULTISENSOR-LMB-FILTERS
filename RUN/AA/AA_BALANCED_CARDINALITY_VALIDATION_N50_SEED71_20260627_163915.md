# AA Balanced/Cardinality Validation

Generated at: 2026-06-27 18:19:58

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 71 (fixed=1)
- trialSeeds: [72 73 74 75 76 77 78 79 80 81 82 83 84 85 86 87 88 89 90 91 92 93 94 95 96 97 98 99 100 101 102 103 104 105 106 107 108 109 110 111 112 113 114 115 116 117 118 119 120 121]
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
- Trial 6: [0.2 0 0.1 0.1 0.5 0.5 0.1 0.1]
- Trial 7: [0.5 0.1 0.5 0.1 0.1 0 0.2 0.1]
- Trial 8: [0.1 0.5 0.1 0.2 0.1 0 0.1 0.5]
- Trial 9: [0.5 0.1 0.1 0.1 0 0.5 0.1 0.2]
- Trial 10: [0.1 0 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 11: [0.5 0.1 0 0.5 0.1 0.1 0.2 0.1]
- Trial 12: [0 0.2 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 13: [0.5 0.1 0 0.1 0.5 0.2 0.1 0.1]
- Trial 14: [0.1 0 0.1 0.5 0.1 0.5 0.2 0.1]
- Trial 15: [0.5 0.1 0.1 0.2 0 0.1 0.1 0.5]
- Trial 16: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]
- Trial 17: [0.5 0.1 0 0.1 0.5 0.1 0.1 0.2]
- Trial 18: [0.5 0.1 0.1 0.1 0.2 0.5 0 0.1]
- Trial 19: [0 0.2 0.5 0.1 0.1 0.5 0.1 0.1]
- Trial 20: [0.2 0.1 0.5 0.1 0.5 0 0.1 0.1]
- Trial 21: [0.2 0.1 0.1 0.5 0 0.1 0.1 0.5]
- Trial 22: [0.5 0.1 0.2 0.1 0.1 0.1 0 0.5]
- Trial 23: [0.1 0.5 0.5 0.1 0.1 0.1 0 0.2]
- Trial 24: [0.1 0.2 0.1 0.5 0.5 0 0.1 0.1]
- Trial 25: [0.1 0.1 0.1 0 0.5 0.5 0.1 0.2]
- Trial 26: [0.1 0.5 0.2 0.5 0.1 0 0.1 0.1]
- Trial 27: [0.1 0.5 0 0.5 0.2 0.1 0.1 0.1]
- Trial 28: [0.5 0.2 0.1 0.1 0.5 0.1 0.1 0]
- Trial 29: [0 0.2 0.1 0.5 0.1 0.1 0.1 0.5]
- Trial 30: [0.1 0.1 0.1 0.2 0.5 0.5 0 0.1]
- Trial 31: [0.5 0.1 0.2 0.1 0 0.5 0.1 0.1]
- Trial 32: [0.1 0.1 0 0.5 0.2 0.1 0.5 0.1]
- Trial 33: [0.5 0.2 0.5 0.1 0.1 0.1 0 0.1]
- Trial 34: [0.5 0.1 0.1 0.5 0.1 0 0.2 0.1]
- Trial 35: [0 0.1 0.2 0.1 0.1 0.1 0.5 0.5]
- Trial 36: [0.5 0.1 0.1 0.1 0 0.5 0.2 0.1]
- Trial 37: [0 0.1 0.1 0.1 0.1 0.2 0.5 0.5]
- Trial 38: [0.1 0.5 0.1 0.5 0.1 0.1 0 0.2]
- Trial 39: [0.1 0.2 0.5 0.1 0 0.5 0.1 0.1]
- Trial 40: [0.1 0.1 0 0.2 0.1 0.5 0.1 0.5]
- Trial 41: [0.2 0.5 0.1 0 0.1 0.1 0.1 0.5]
- Trial 42: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 43: [0.1 0 0.1 0.1 0.1 0.5 0.2 0.5]
- Trial 44: [0.5 0.2 0.1 0.5 0.1 0.1 0.1 0]
- Trial 45: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 46: [0.1 0.1 0.1 0.2 0.5 0.5 0 0.1]
- Trial 47: [0.1 0.5 0.1 0.2 0.1 0 0.5 0.1]
- Trial 48: [0.5 0.1 0.5 0.1 0 0.1 0.2 0.1]
- Trial 49: [0.5 0.5 0.1 0.1 0.1 0.2 0 0.1]
- Trial 50: [0.1 0.1 0.1 0.5 0 0.2 0.5 0.1]

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
| 6 | 77 | Tuned spatial-KLA AA | 2.848247 | 7.059592 | 0.515625 |
| 6 | 77 | Neighborhood label-barycenter spatial-KLA AA | 1.051381 | 1.142309 | 0.333333 |
| 6 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.513561 | 3.466099 | 0.333333 |
| 7 | 78 | Tuned spatial-KLA AA | 2.402959 | 4.885601 | 0.166667 |
| 7 | 78 | Neighborhood label-barycenter spatial-KLA AA | 0.670024 | 0.786487 | 0.041667 |
| 7 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.386515 | 2.893368 | 0.041667 |
| 8 | 79 | Tuned spatial-KLA AA | 2.771482 | 6.270676 | 0.250000 |
| 8 | 79 | Neighborhood label-barycenter spatial-KLA AA | 0.802404 | 0.936266 | 0.104167 |
| 8 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.551273 | 3.689955 | 0.104167 |
| 9 | 80 | Tuned spatial-KLA AA | 2.327689 | 5.914107 | 0.208333 |
| 9 | 80 | Neighborhood label-barycenter spatial-KLA AA | 0.715848 | 0.685252 | 0.125000 |
| 9 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.325635 | 1.995237 | 0.125000 |
| 10 | 81 | Tuned spatial-KLA AA | 2.561552 | 6.308002 | 0.171875 |
| 10 | 81 | Neighborhood label-barycenter spatial-KLA AA | 0.798741 | 1.045785 | 0.083333 |
| 10 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.350682 | 2.577024 | 0.083333 |
| 11 | 82 | Tuned spatial-KLA AA | 2.933160 | 6.220182 | 0.520833 |
| 11 | 82 | Neighborhood label-barycenter spatial-KLA AA | 0.948936 | 0.984384 | 0.208333 |
| 11 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.770889 | 2.581093 | 0.208333 |
| 12 | 83 | Tuned spatial-KLA AA | 2.708354 | 6.671611 | 0.213542 |
| 12 | 83 | Neighborhood label-barycenter spatial-KLA AA | 0.834042 | 1.068657 | 0.125000 |
| 12 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.526620 | 4.368822 | 0.125000 |
| 13 | 84 | Tuned spatial-KLA AA | 2.730381 | 5.650548 | 0.312500 |
| 13 | 84 | Neighborhood label-barycenter spatial-KLA AA | 0.913699 | 0.862675 | 0.166667 |
| 13 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.405143 | 1.949245 | 0.166667 |
| 14 | 85 | Tuned spatial-KLA AA | 2.887570 | 7.202757 | 0.343750 |
| 14 | 85 | Neighborhood label-barycenter spatial-KLA AA | 0.973878 | 1.199937 | 0.229167 |
| 14 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.857657 | 4.132609 | 0.229167 |
| 15 | 86 | Tuned spatial-KLA AA | 2.542550 | 6.805882 | 0.244792 |
| 15 | 86 | Neighborhood label-barycenter spatial-KLA AA | 0.994529 | 1.111866 | 0.250000 |
| 15 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.656492 | 3.849401 | 0.250000 |
| 16 | 87 | Tuned spatial-KLA AA | 2.377414 | 5.561450 | 0.177083 |
| 16 | 87 | Neighborhood label-barycenter spatial-KLA AA | 0.702902 | 0.943535 | 0.125000 |
| 16 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.340231 | 3.735578 | 0.125000 |
| 17 | 88 | Tuned spatial-KLA AA | 2.653982 | 6.733101 | 0.239583 |
| 17 | 88 | Neighborhood label-barycenter spatial-KLA AA | 0.941021 | 1.162725 | 0.166667 |
| 17 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.686991 | 4.678020 | 0.166667 |
| 18 | 89 | Tuned spatial-KLA AA | 2.738880 | 6.287777 | 0.322917 |
| 18 | 89 | Neighborhood label-barycenter spatial-KLA AA | 0.939795 | 0.858701 | 0.208333 |
| 18 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.632851 | 2.883009 | 0.208333 |
| 19 | 90 | Tuned spatial-KLA AA | 2.822093 | 5.542699 | 0.343750 |
| 19 | 90 | Neighborhood label-barycenter spatial-KLA AA | 1.017488 | 0.834935 | 0.291667 |
| 19 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.917458 | 2.618074 | 0.291667 |
| 20 | 91 | Tuned spatial-KLA AA | 2.733818 | 7.413609 | 0.265625 |
| 20 | 91 | Neighborhood label-barycenter spatial-KLA AA | 1.073012 | 1.228306 | 0.229167 |
| 20 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.597155 | 4.448395 | 0.229167 |
| 21 | 92 | Tuned spatial-KLA AA | 2.706909 | 5.314065 | 0.208333 |
| 21 | 92 | Neighborhood label-barycenter spatial-KLA AA | 0.902412 | 0.901583 | 0.125000 |
| 21 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.643697 | 2.790702 | 0.125000 |
| 22 | 93 | Tuned spatial-KLA AA | 2.802294 | 6.198751 | 0.322917 |
| 22 | 93 | Neighborhood label-barycenter spatial-KLA AA | 0.862886 | 0.964796 | 0.125000 |
| 22 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.495328 | 2.439345 | 0.125000 |
| 23 | 94 | Tuned spatial-KLA AA | 2.665562 | 6.721916 | 0.276042 |
| 23 | 94 | Neighborhood label-barycenter spatial-KLA AA | 0.921147 | 0.939199 | 0.250000 |
| 23 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.532507 | 3.492728 | 0.250000 |
| 24 | 95 | Tuned spatial-KLA AA | 2.891538 | 5.744619 | 0.375000 |
| 24 | 95 | Neighborhood label-barycenter spatial-KLA AA | 0.956801 | 0.850977 | 0.229167 |
| 24 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.617563 | 3.274142 | 0.229167 |
| 25 | 96 | Tuned spatial-KLA AA | 2.771683 | 5.841466 | 0.416667 |
| 25 | 96 | Neighborhood label-barycenter spatial-KLA AA | 0.894178 | 1.109168 | 0.208333 |
| 25 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.574183 | 3.564265 | 0.208333 |
| 26 | 97 | Tuned spatial-KLA AA | 2.610133 | 7.076089 | 0.161458 |
| 26 | 97 | Neighborhood label-barycenter spatial-KLA AA | 0.826688 | 1.102216 | 0.083333 |
| 26 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.538285 | 4.011643 | 0.083333 |
| 27 | 98 | Tuned spatial-KLA AA | 2.659639 | 5.603949 | 0.322917 |
| 27 | 98 | Neighborhood label-barycenter spatial-KLA AA | 0.909882 | 1.256937 | 0.166667 |
| 27 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.526287 | 3.811884 | 0.166667 |
| 28 | 99 | Tuned spatial-KLA AA | 2.824573 | 6.200256 | 0.322917 |
| 28 | 99 | Neighborhood label-barycenter spatial-KLA AA | 0.967065 | 1.296574 | 0.187500 |
| 28 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.672122 | 4.066926 | 0.187500 |
| 29 | 100 | Tuned spatial-KLA AA | 2.808688 | 6.515448 | 0.281250 |
| 29 | 100 | Neighborhood label-barycenter spatial-KLA AA | 0.921309 | 1.144126 | 0.166667 |
| 29 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.633623 | 4.724417 | 0.166667 |
| 30 | 101 | Tuned spatial-KLA AA | 2.607074 | 5.292992 | 0.385417 |
| 30 | 101 | Neighborhood label-barycenter spatial-KLA AA | 0.935799 | 0.865473 | 0.270833 |
| 30 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.457275 | 2.024208 | 0.270833 |
| 31 | 102 | Tuned spatial-KLA AA | 2.459002 | 5.135015 | 0.161458 |
| 31 | 102 | Neighborhood label-barycenter spatial-KLA AA | 0.693502 | 0.844161 | 0.104167 |
| 31 | 102 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.528230 | 2.266124 | 0.104167 |
| 32 | 103 | Tuned spatial-KLA AA | 2.740438 | 5.692585 | 0.208333 |
| 32 | 103 | Neighborhood label-barycenter spatial-KLA AA | 0.788937 | 1.121266 | 0.020833 |
| 32 | 103 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.267048 | 3.094010 | 0.020833 |
| 33 | 104 | Tuned spatial-KLA AA | 2.896720 | 5.959971 | 0.494792 |
| 33 | 104 | Neighborhood label-barycenter spatial-KLA AA | 1.235724 | 1.053750 | 0.500000 |
| 33 | 104 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.871033 | 3.642207 | 0.500000 |
| 34 | 105 | Tuned spatial-KLA AA | 2.302544 | 5.092385 | 0.135417 |
| 34 | 105 | Neighborhood label-barycenter spatial-KLA AA | 0.672912 | 0.838053 | 0.062500 |
| 34 | 105 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.541137 | 3.803714 | 0.062500 |
| 35 | 106 | Tuned spatial-KLA AA | 2.492235 | 4.557924 | 0.192708 |
| 35 | 106 | Neighborhood label-barycenter spatial-KLA AA | 0.693541 | 0.670201 | 0.083333 |
| 35 | 106 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.539649 | 2.622311 | 0.083333 |
| 36 | 107 | Tuned spatial-KLA AA | 2.686594 | 5.551858 | 0.307292 |
| 36 | 107 | Neighborhood label-barycenter spatial-KLA AA | 0.930716 | 1.098458 | 0.229167 |
| 36 | 107 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.736106 | 3.963225 | 0.229167 |
| 37 | 108 | Tuned spatial-KLA AA | 2.825748 | 5.995090 | 0.244792 |
| 37 | 108 | Neighborhood label-barycenter spatial-KLA AA | 0.855658 | 0.818141 | 0.229167 |
| 37 | 108 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.668580 | 2.977409 | 0.229167 |
| 38 | 109 | Tuned spatial-KLA AA | 2.458868 | 3.757409 | 0.213542 |
| 38 | 109 | Neighborhood label-barycenter spatial-KLA AA | 0.596456 | 0.454184 | 0.083333 |
| 38 | 109 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.416875 | 1.542383 | 0.083333 |
| 39 | 110 | Tuned spatial-KLA AA | 2.715807 | 5.475306 | 0.312500 |
| 39 | 110 | Neighborhood label-barycenter spatial-KLA AA | 0.987666 | 0.816351 | 0.229167 |
| 39 | 110 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.687278 | 2.101373 | 0.229167 |
| 40 | 111 | Tuned spatial-KLA AA | 2.905186 | 7.147333 | 0.338542 |
| 40 | 111 | Neighborhood label-barycenter spatial-KLA AA | 0.991689 | 0.969704 | 0.187500 |
| 40 | 111 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.759483 | 3.761298 | 0.187500 |
| 41 | 112 | Tuned spatial-KLA AA | 2.515490 | 6.245835 | 0.182292 |
| 41 | 112 | Neighborhood label-barycenter spatial-KLA AA | 0.784190 | 0.972620 | 0.083333 |
| 41 | 112 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.536221 | 3.851649 | 0.083333 |
| 42 | 113 | Tuned spatial-KLA AA | 2.473936 | 6.580920 | 0.281250 |
| 42 | 113 | Neighborhood label-barycenter spatial-KLA AA | 0.996151 | 1.098896 | 0.250000 |
| 42 | 113 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.455252 | 3.629166 | 0.250000 |
| 43 | 114 | Tuned spatial-KLA AA | 2.656696 | 4.412649 | 0.223958 |
| 43 | 114 | Neighborhood label-barycenter spatial-KLA AA | 0.713847 | 0.573235 | 0.145833 |
| 43 | 114 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.260236 | 2.201406 | 0.145833 |
| 44 | 115 | Tuned spatial-KLA AA | 2.702660 | 6.442743 | 0.322917 |
| 44 | 115 | Neighborhood label-barycenter spatial-KLA AA | 0.942123 | 1.100867 | 0.166667 |
| 44 | 115 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.631201 | 3.276520 | 0.166667 |
| 45 | 116 | Tuned spatial-KLA AA | 2.352922 | 5.310785 | 0.250000 |
| 45 | 116 | Neighborhood label-barycenter spatial-KLA AA | 0.862936 | 0.732451 | 0.187500 |
| 45 | 116 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.299873 | 1.593839 | 0.187500 |
| 46 | 117 | Tuned spatial-KLA AA | 2.719701 | 7.525355 | 0.338542 |
| 46 | 117 | Neighborhood label-barycenter spatial-KLA AA | 1.022668 | 1.175002 | 0.250000 |
| 46 | 117 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.628241 | 4.358781 | 0.250000 |
| 47 | 118 | Tuned spatial-KLA AA | 2.543397 | 5.233371 | 0.182292 |
| 47 | 118 | Neighborhood label-barycenter spatial-KLA AA | 0.793089 | 0.957855 | 0.104167 |
| 47 | 118 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.566819 | 3.216341 | 0.104167 |
| 48 | 119 | Tuned spatial-KLA AA | 2.775170 | 7.584974 | 0.239583 |
| 48 | 119 | Neighborhood label-barycenter spatial-KLA AA | 0.969413 | 1.167162 | 0.145833 |
| 48 | 119 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.723007 | 3.388112 | 0.145833 |
| 49 | 120 | Tuned spatial-KLA AA | 2.643120 | 5.284034 | 0.239583 |
| 49 | 120 | Neighborhood label-barycenter spatial-KLA AA | 0.927478 | 1.217220 | 0.145833 |
| 49 | 120 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.714040 | 3.558040 | 0.145833 |
| 50 | 121 | Tuned spatial-KLA AA | 2.620679 | 5.605883 | 0.239583 |
| 50 | 121 | Neighborhood label-barycenter spatial-KLA AA | 0.869273 | 1.098149 | 0.145833 |
| 50 | 121 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.498421 | 3.498054 | 0.145833 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.674648 | 6.035999 | 0.274062 |
| Neighborhood label-barycenter spatial-KLA AA | 0.880266 | 0.979624 | 0.170417 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.566496 | 3.296750 | 0.170417 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.674648 +/- 0.168935 | [2.627821, 2.721474] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.880266 +/- 0.122657 | [0.846267, 0.914265] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.566496 +/- 0.150141 | [1.524879, 1.608113] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 6.035999 +/- 0.841977 | [5.802615, 6.269384] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.979624 +/- 0.187082 | [0.927768, 1.031480] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 3.296750 +/- 0.811904 | [3.071702, 3.521798] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.274062 +/- 0.088950 | [0.249407, 0.298718] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.170417 +/- 0.082874 | [0.147445, 0.193388] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.170417 +/- 0.082874 | [0.147445, 0.193388] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.794382 +/- 0.142693 | [1.754829, 1.833934] | 67.09% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.108152 +/- 0.150578 | [1.066413, 1.149890] | 41.43% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 5.056375 +/- 0.722484 | [4.856113, 5.256638] | 83.77% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 2.739249 +/- 0.711991 | [2.541895, 2.936603] | 45.38% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.103646 +/- 0.058881 | [0.087325, 0.119967] | 37.82% | 48/50 | 2.267e-12 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.103646 +/- 0.058881 | [0.087325, 0.119967] | 37.82% | 48/50 | 2.267e-12 |

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
| 6 | 77 | Tuned spatial-KLA AA | 2.863090 | 2.645571 | 1.036458 |
| 6 | 77 | Neighborhood label-barycenter spatial-KLA AA | 2.601406 | 4.158519 | 0.833333 |
| 6 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.628011 | 2.310005 | 0.833333 |
| 7 | 78 | Tuned spatial-KLA AA | 2.828982 | 2.478539 | 0.770833 |
| 7 | 78 | Neighborhood label-barycenter spatial-KLA AA | 2.667672 | 3.648853 | 0.750000 |
| 7 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.747899 | 2.385097 | 0.750000 |
| 8 | 79 | Tuned spatial-KLA AA | 2.957111 | 2.799032 | 0.760417 |
| 8 | 79 | Neighborhood label-barycenter spatial-KLA AA | 2.731825 | 3.710541 | 0.645833 |
| 8 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.812585 | 2.578284 | 0.645833 |
| 9 | 80 | Tuned spatial-KLA AA | 2.791202 | 2.389744 | 0.760417 |
| 9 | 80 | Neighborhood label-barycenter spatial-KLA AA | 2.638219 | 3.717963 | 0.708333 |
| 9 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.812617 | 2.394677 | 0.708333 |
| 10 | 81 | Tuned spatial-KLA AA | 2.826768 | 2.513346 | 0.692708 |
| 10 | 81 | Neighborhood label-barycenter spatial-KLA AA | 2.676237 | 3.588851 | 0.625000 |
| 10 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.731960 | 2.441713 | 0.625000 |
| 11 | 82 | Tuned spatial-KLA AA | 3.114740 | 2.641085 | 1.479167 |
| 11 | 82 | Neighborhood label-barycenter spatial-KLA AA | 3.162202 | 3.719304 | 1.541667 |
| 11 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 3.175722 | 2.639585 | 1.541667 |
| 12 | 83 | Tuned spatial-KLA AA | 2.850880 | 2.505538 | 0.703125 |
| 12 | 83 | Neighborhood label-barycenter spatial-KLA AA | 2.544335 | 3.899110 | 0.666667 |
| 12 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.731902 | 2.282531 | 0.666667 |
| 13 | 84 | Tuned spatial-KLA AA | 2.955646 | 2.512243 | 1.020833 |
| 13 | 84 | Neighborhood label-barycenter spatial-KLA AA | 2.809027 | 3.388383 | 0.958333 |
| 13 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.864529 | 2.438849 | 0.958333 |
| 14 | 85 | Tuned spatial-KLA AA | 2.844495 | 2.513264 | 0.916667 |
| 14 | 85 | Neighborhood label-barycenter spatial-KLA AA | 2.562634 | 3.690120 | 0.812500 |
| 14 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.713371 | 2.425392 | 0.812500 |
| 15 | 86 | Tuned spatial-KLA AA | 2.720798 | 2.599352 | 0.755208 |
| 15 | 86 | Neighborhood label-barycenter spatial-KLA AA | 2.616567 | 3.969317 | 0.750000 |
| 15 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.648875 | 2.566278 | 0.750000 |
| 16 | 87 | Tuned spatial-KLA AA | 2.585097 | 2.312774 | 0.635417 |
| 16 | 87 | Neighborhood label-barycenter spatial-KLA AA | 2.316600 | 3.693344 | 0.583333 |
| 16 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.434287 | 2.173843 | 0.583333 |
| 17 | 88 | Tuned spatial-KLA AA | 2.798066 | 2.636254 | 0.781250 |
| 17 | 88 | Neighborhood label-barycenter spatial-KLA AA | 2.623902 | 4.273523 | 0.750000 |
| 17 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.713050 | 2.436699 | 0.750000 |
| 18 | 89 | Tuned spatial-KLA AA | 2.896975 | 2.534068 | 0.895833 |
| 18 | 89 | Neighborhood label-barycenter spatial-KLA AA | 2.675428 | 3.715950 | 0.833333 |
| 18 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.772271 | 2.345834 | 0.833333 |
| 19 | 90 | Tuned spatial-KLA AA | 2.921111 | 2.679905 | 0.937500 |
| 19 | 90 | Neighborhood label-barycenter spatial-KLA AA | 2.811541 | 2.951642 | 0.875000 |
| 19 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.919203 | 2.502251 | 0.875000 |
| 20 | 91 | Tuned spatial-KLA AA | 2.974584 | 2.546830 | 0.921875 |
| 20 | 91 | Neighborhood label-barycenter spatial-KLA AA | 2.838647 | 4.453625 | 0.895833 |
| 20 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.829426 | 2.331808 | 0.895833 |
| 21 | 92 | Tuned spatial-KLA AA | 2.729531 | 2.417001 | 0.718750 |
| 21 | 92 | Neighborhood label-barycenter spatial-KLA AA | 2.541274 | 3.578456 | 0.708333 |
| 21 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.633829 | 2.208009 | 0.708333 |
| 22 | 93 | Tuned spatial-KLA AA | 2.913307 | 2.579140 | 0.927083 |
| 22 | 93 | Neighborhood label-barycenter spatial-KLA AA | 2.660687 | 3.743550 | 0.750000 |
| 22 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.738341 | 2.317189 | 0.750000 |
| 23 | 94 | Tuned spatial-KLA AA | 2.830790 | 2.487099 | 0.848958 |
| 23 | 94 | Neighborhood label-barycenter spatial-KLA AA | 2.584787 | 3.686597 | 0.791667 |
| 23 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.640839 | 2.245097 | 0.791667 |
| 24 | 95 | Tuned spatial-KLA AA | 3.044817 | 2.767275 | 1.020833 |
| 24 | 95 | Neighborhood label-barycenter spatial-KLA AA | 2.780831 | 4.034853 | 0.895833 |
| 24 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.842324 | 2.520258 | 0.895833 |
| 25 | 96 | Tuned spatial-KLA AA | 2.780742 | 2.425816 | 0.947917 |
| 25 | 96 | Neighborhood label-barycenter spatial-KLA AA | 2.626852 | 3.684180 | 0.750000 |
| 25 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.672694 | 2.320207 | 0.750000 |
| 26 | 97 | Tuned spatial-KLA AA | 2.666530 | 2.347593 | 0.828125 |
| 26 | 97 | Neighborhood label-barycenter spatial-KLA AA | 2.443754 | 3.946624 | 0.833333 |
| 26 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.556822 | 2.186942 | 0.833333 |
| 27 | 98 | Tuned spatial-KLA AA | 2.830116 | 2.566976 | 0.822917 |
| 27 | 98 | Neighborhood label-barycenter spatial-KLA AA | 2.640335 | 3.900458 | 0.750000 |
| 27 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.732854 | 2.510755 | 0.750000 |
| 28 | 99 | Tuned spatial-KLA AA | 2.872300 | 2.641094 | 0.895833 |
| 28 | 99 | Neighborhood label-barycenter spatial-KLA AA | 2.659986 | 3.856987 | 0.812500 |
| 28 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.759849 | 2.486096 | 0.812500 |
| 29 | 100 | Tuned spatial-KLA AA | 2.987879 | 2.765563 | 0.854167 |
| 29 | 100 | Neighborhood label-barycenter spatial-KLA AA | 2.827622 | 4.225937 | 0.833333 |
| 29 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.882680 | 2.593134 | 0.833333 |
| 30 | 101 | Tuned spatial-KLA AA | 2.823264 | 2.551869 | 0.875000 |
| 30 | 101 | Neighborhood label-barycenter spatial-KLA AA | 2.623995 | 3.506740 | 0.729167 |
| 30 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.706376 | 2.376129 | 0.729167 |
| 31 | 102 | Tuned spatial-KLA AA | 2.644791 | 2.343491 | 0.588542 |
| 31 | 102 | Neighborhood label-barycenter spatial-KLA AA | 2.357913 | 2.869858 | 0.520833 |
| 31 | 102 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.552065 | 2.241688 | 0.520833 |
| 32 | 103 | Tuned spatial-KLA AA | 2.920580 | 2.730972 | 0.916667 |
| 32 | 103 | Neighborhood label-barycenter spatial-KLA AA | 2.791550 | 3.376430 | 0.937500 |
| 32 | 103 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.787857 | 2.485607 | 0.937500 |
| 33 | 104 | Tuned spatial-KLA AA | 3.146037 | 2.700871 | 1.161458 |
| 33 | 104 | Neighborhood label-barycenter spatial-KLA AA | 3.040018 | 3.901341 | 1.166667 |
| 33 | 104 | Neighborhood reference-only label-consensus spatial-KLA AA | 3.111281 | 2.595281 | 1.166667 |
| 34 | 105 | Tuned spatial-KLA AA | 2.570562 | 2.296234 | 0.572917 |
| 34 | 105 | Neighborhood label-barycenter spatial-KLA AA | 2.263124 | 3.411662 | 0.520833 |
| 34 | 105 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.501502 | 2.194878 | 0.520833 |
| 35 | 106 | Tuned spatial-KLA AA | 2.641814 | 2.447035 | 0.640625 |
| 35 | 106 | Neighborhood label-barycenter spatial-KLA AA | 2.393758 | 2.738406 | 0.541667 |
| 35 | 106 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.530252 | 2.310256 | 0.541667 |
| 36 | 107 | Tuned spatial-KLA AA | 2.810944 | 2.443834 | 0.817708 |
| 36 | 107 | Neighborhood label-barycenter spatial-KLA AA | 2.625297 | 3.675583 | 0.812500 |
| 36 | 107 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.702874 | 2.267916 | 0.812500 |
| 37 | 108 | Tuned spatial-KLA AA | 2.913888 | 2.665818 | 0.765625 |
| 37 | 108 | Neighborhood label-barycenter spatial-KLA AA | 2.652596 | 3.537584 | 0.729167 |
| 37 | 108 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.826456 | 2.520992 | 0.729167 |
| 38 | 109 | Tuned spatial-KLA AA | 2.872428 | 2.442459 | 0.723958 |
| 38 | 109 | Neighborhood label-barycenter spatial-KLA AA | 2.628456 | 2.605297 | 0.666667 |
| 38 | 109 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.714900 | 2.241509 | 0.666667 |
| 39 | 110 | Tuned spatial-KLA AA | 2.971621 | 2.752155 | 0.833333 |
| 39 | 110 | Neighborhood label-barycenter spatial-KLA AA | 2.795447 | 3.247877 | 0.770833 |
| 39 | 110 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.955353 | 2.700888 | 0.770833 |
| 40 | 111 | Tuned spatial-KLA AA | 2.983704 | 2.771689 | 1.015625 |
| 40 | 111 | Neighborhood label-barycenter spatial-KLA AA | 2.742352 | 4.063196 | 0.854167 |
| 40 | 111 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.847924 | 2.621123 | 0.854167 |
| 41 | 112 | Tuned spatial-KLA AA | 2.759534 | 2.406664 | 0.734375 |
| 41 | 112 | Neighborhood label-barycenter spatial-KLA AA | 2.541576 | 3.692163 | 0.666667 |
| 41 | 112 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.628443 | 2.243521 | 0.666667 |
| 42 | 113 | Tuned spatial-KLA AA | 2.703438 | 2.422907 | 0.770833 |
| 42 | 113 | Neighborhood label-barycenter spatial-KLA AA | 2.641427 | 4.060569 | 0.750000 |
| 42 | 113 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.590247 | 2.211632 | 0.750000 |
| 43 | 114 | Tuned spatial-KLA AA | 2.742110 | 2.500978 | 0.734375 |
| 43 | 114 | Neighborhood label-barycenter spatial-KLA AA | 2.425444 | 2.840342 | 0.645833 |
| 43 | 114 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.536591 | 2.201227 | 0.645833 |
| 44 | 115 | Tuned spatial-KLA AA | 2.933264 | 2.497479 | 0.927083 |
| 44 | 115 | Neighborhood label-barycenter spatial-KLA AA | 2.802445 | 4.461555 | 0.875000 |
| 44 | 115 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.873258 | 2.352904 | 0.875000 |
| 45 | 116 | Tuned spatial-KLA AA | 2.696842 | 2.287250 | 0.781250 |
| 45 | 116 | Neighborhood label-barycenter spatial-KLA AA | 2.570155 | 3.113014 | 0.729167 |
| 45 | 116 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.597422 | 2.232709 | 0.729167 |
| 46 | 117 | Tuned spatial-KLA AA | 2.864654 | 2.607804 | 0.932292 |
| 46 | 117 | Neighborhood label-barycenter spatial-KLA AA | 2.802708 | 4.497645 | 0.833333 |
| 46 | 117 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.747407 | 2.417678 | 0.833333 |
| 47 | 118 | Tuned spatial-KLA AA | 2.777918 | 2.464200 | 0.755208 |
| 47 | 118 | Neighborhood label-barycenter spatial-KLA AA | 2.506558 | 3.529092 | 0.729167 |
| 47 | 118 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.694792 | 2.401092 | 0.729167 |
| 48 | 119 | Tuned spatial-KLA AA | 2.961562 | 2.654918 | 0.885417 |
| 48 | 119 | Neighborhood label-barycenter spatial-KLA AA | 2.892971 | 4.304725 | 0.895833 |
| 48 | 119 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.969452 | 2.654233 | 0.895833 |
| 49 | 120 | Tuned spatial-KLA AA | 2.954181 | 2.691476 | 0.770833 |
| 49 | 120 | Neighborhood label-barycenter spatial-KLA AA | 2.780067 | 3.428478 | 0.687500 |
| 49 | 120 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.857060 | 2.548319 | 0.687500 |
| 50 | 121 | Tuned spatial-KLA AA | 2.809170 | 2.442098 | 0.781250 |
| 50 | 121 | Neighborhood label-barycenter spatial-KLA AA | 2.588516 | 3.747812 | 0.687500 |
| 50 | 121 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.705582 | 2.285599 | 0.687500 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.857458 | 2.561998 | 0.838229 |
| Neighborhood label-barycenter spatial-KLA AA | 2.663596 | 3.713083 | 0.773750 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.750511 | 2.406586 | 0.773750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.857458 +/- 0.131240 | [2.821080, 2.893835] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.663596 +/- 0.167803 | [2.617084, 2.710109] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.750511 +/- 0.148688 | [2.709297, 2.791725] | 50 |
| Tuned spatial-KLA AA | RMSE | 2.561998 +/- 0.157077 | [2.518459, 2.605538] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.713083 +/- 0.448737 | [3.588699, 3.837466] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.406586 +/- 0.155941 | [2.363361, 2.449810] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.838229 +/- 0.150575 | [0.796492, 0.879966] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.773750 +/- 0.160825 | [0.729172, 0.818328] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.773750 +/- 0.160825 | [0.729172, 0.818328] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.193861 +/- 0.074840 | [0.173117, 0.214606] | 6.78% | 49/50 | 9.059e-14 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.106947 +/- 0.056770 | [0.091211, 0.122682] | 3.74% | 47/50 | 3.708e-11 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | -1.151084 +/- 0.445726 | [-1.274633, -1.027535] | -44.93% | 0/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.155413 +/- 0.081294 | [0.132879, 0.177947] | 6.07% | 49/50 | 9.059e-14 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.064479 +/- 0.055559 | [0.049079, 0.079879] | 7.69% | 44/50 | 7.597e-09 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.064479 +/- 0.055559 | [0.049079, 0.079879] | 7.69% | 44/50 | 7.597e-09 |

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
| 6 | 77 | Tuned spatial-KLA AA | 2.735871 | 2.563908 | 0.611111 |
| 6 | 77 | Neighborhood label-barycenter spatial-KLA AA | 0.709661 | 0.344027 | 0.277778 |
| 6 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.304140 | 1.041340 | 0.277778 |
| 7 | 78 | Tuned spatial-KLA AA | 2.151607 | 2.167959 | 0.138889 |
| 7 | 78 | Neighborhood label-barycenter spatial-KLA AA | 0.432236 | 0.402862 | 0.000000 |
| 7 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.132839 | 1.444331 | 0.000000 |
| 8 | 79 | Tuned spatial-KLA AA | 2.659805 | 2.465496 | 0.208333 |
| 8 | 79 | Neighborhood label-barycenter spatial-KLA AA | 0.564626 | 0.336094 | 0.111111 |
| 8 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.426169 | 1.195277 | 0.111111 |
| 9 | 80 | Tuned spatial-KLA AA | 2.225402 | 2.005065 | 0.208333 |
| 9 | 80 | Neighborhood label-barycenter spatial-KLA AA | 0.498938 | 0.245406 | 0.111111 |
| 9 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.141378 | 1.057878 | 0.111111 |
| 10 | 81 | Tuned spatial-KLA AA | 2.493158 | 2.281505 | 0.222222 |
| 10 | 81 | Neighborhood label-barycenter spatial-KLA AA | 0.490312 | 0.258900 | 0.111111 |
| 10 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.036631 | 0.895570 | 0.111111 |
| 11 | 82 | Tuned spatial-KLA AA | 2.598680 | 2.290199 | 0.444444 |
| 11 | 82 | Neighborhood label-barycenter spatial-KLA AA | 0.473157 | 0.364230 | 0.000000 |
| 11 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.563549 | 1.593290 | 0.000000 |
| 12 | 83 | Tuned spatial-KLA AA | 2.708559 | 2.812013 | 0.208333 |
| 12 | 83 | Neighborhood label-barycenter spatial-KLA AA | 0.671467 | 0.400159 | 0.166667 |
| 12 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.454564 | 1.236508 | 0.166667 |
| 13 | 84 | Tuned spatial-KLA AA | 2.579174 | 2.200052 | 0.319444 |
| 13 | 84 | Neighborhood label-barycenter spatial-KLA AA | 0.698302 | 0.495151 | 0.111111 |
| 13 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.325435 | 1.261324 | 0.111111 |
| 14 | 85 | Tuned spatial-KLA AA | 2.741725 | 2.467464 | 0.250000 |
| 14 | 85 | Neighborhood label-barycenter spatial-KLA AA | 0.505227 | 0.349823 | 0.055556 |
| 14 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.739446 | 1.535940 | 0.055556 |
| 15 | 86 | Tuned spatial-KLA AA | 2.303171 | 2.542025 | 0.263889 |
| 15 | 86 | Neighborhood label-barycenter spatial-KLA AA | 0.856125 | 0.471200 | 0.277778 |
| 15 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.553928 | 1.229567 | 0.277778 |
| 16 | 87 | Tuned spatial-KLA AA | 2.195425 | 1.971872 | 0.152778 |
| 16 | 87 | Neighborhood label-barycenter spatial-KLA AA | 0.412965 | 0.229269 | 0.111111 |
| 16 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.042600 | 0.894499 | 0.111111 |
| 17 | 88 | Tuned spatial-KLA AA | 2.372636 | 2.530754 | 0.263889 |
| 17 | 88 | Neighborhood label-barycenter spatial-KLA AA | 0.785953 | 0.562530 | 0.222222 |
| 17 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.412953 | 1.961886 | 0.222222 |
| 18 | 89 | Tuned spatial-KLA AA | 2.569901 | 2.445672 | 0.250000 |
| 18 | 89 | Neighborhood label-barycenter spatial-KLA AA | 0.665234 | 0.340666 | 0.166667 |
| 18 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.626206 | 1.434081 | 0.166667 |
| 19 | 90 | Tuned spatial-KLA AA | 2.690395 | 2.374705 | 0.305556 |
| 19 | 90 | Neighborhood label-barycenter spatial-KLA AA | 0.737621 | 0.373777 | 0.222222 |
| 19 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.832679 | 1.497331 | 0.222222 |
| 20 | 91 | Tuned spatial-KLA AA | 2.626146 | 2.743629 | 0.388889 |
| 20 | 91 | Neighborhood label-barycenter spatial-KLA AA | 1.056116 | 0.527508 | 0.388889 |
| 20 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.555197 | 1.334879 | 0.388889 |
| 21 | 92 | Tuned spatial-KLA AA | 2.508678 | 2.576922 | 0.180556 |
| 21 | 92 | Neighborhood label-barycenter spatial-KLA AA | 0.777753 | 0.601114 | 0.111111 |
| 21 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.524702 | 1.331516 | 0.111111 |
| 22 | 93 | Tuned spatial-KLA AA | 2.684946 | 2.669996 | 0.236111 |
| 22 | 93 | Neighborhood label-barycenter spatial-KLA AA | 0.614489 | 0.463732 | 0.000000 |
| 22 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.575481 | 1.497115 | 0.000000 |
| 23 | 94 | Tuned spatial-KLA AA | 2.210391 | 1.938079 | 0.194444 |
| 23 | 94 | Neighborhood label-barycenter spatial-KLA AA | 0.478520 | 0.255708 | 0.111111 |
| 23 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.074126 | 0.938825 | 0.111111 |
| 24 | 95 | Tuned spatial-KLA AA | 2.694022 | 2.698416 | 0.402778 |
| 24 | 95 | Neighborhood label-barycenter spatial-KLA AA | 0.861148 | 0.371438 | 0.333333 |
| 24 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.630296 | 1.320673 | 0.333333 |
| 25 | 96 | Tuned spatial-KLA AA | 2.438888 | 2.197519 | 0.333333 |
| 25 | 96 | Neighborhood label-barycenter spatial-KLA AA | 0.548233 | 0.335458 | 0.111111 |
| 25 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.181852 | 1.106844 | 0.111111 |
| 26 | 97 | Tuned spatial-KLA AA | 2.384662 | 2.102074 | 0.166667 |
| 26 | 97 | Neighborhood label-barycenter spatial-KLA AA | 0.625729 | 0.315962 | 0.166667 |
| 26 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.523983 | 1.304582 | 0.166667 |
| 27 | 98 | Tuned spatial-KLA AA | 2.487780 | 2.392207 | 0.277778 |
| 27 | 98 | Neighborhood label-barycenter spatial-KLA AA | 0.601459 | 0.347744 | 0.166667 |
| 27 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.602274 | 1.372617 | 0.166667 |
| 28 | 99 | Tuned spatial-KLA AA | 2.648225 | 2.760599 | 0.402778 |
| 28 | 99 | Neighborhood label-barycenter spatial-KLA AA | 0.877709 | 0.422922 | 0.277778 |
| 28 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.568603 | 1.270478 | 0.277778 |
| 29 | 100 | Tuned spatial-KLA AA | 2.666526 | 2.311848 | 0.305556 |
| 29 | 100 | Neighborhood label-barycenter spatial-KLA AA | 0.665386 | 0.351013 | 0.166667 |
| 29 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.515023 | 1.335758 | 0.166667 |
| 30 | 101 | Tuned spatial-KLA AA | 2.471306 | 2.315830 | 0.416667 |
| 30 | 101 | Neighborhood label-barycenter spatial-KLA AA | 0.830058 | 0.368138 | 0.333333 |
| 30 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.553561 | 1.320068 | 0.333333 |
| 31 | 102 | Tuned spatial-KLA AA | 2.424047 | 2.177480 | 0.236111 |
| 31 | 102 | Neighborhood label-barycenter spatial-KLA AA | 0.548004 | 0.324776 | 0.111111 |
| 31 | 102 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.555860 | 1.397535 | 0.111111 |
| 32 | 103 | Tuned spatial-KLA AA | 2.467050 | 2.229523 | 0.055556 |
| 32 | 103 | Neighborhood label-barycenter spatial-KLA AA | 0.421254 | 0.277929 | 0.055556 |
| 32 | 103 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.088215 | 0.881726 | 0.055556 |
| 33 | 104 | Tuned spatial-KLA AA | 2.880358 | 2.821459 | 0.500000 |
| 33 | 104 | Neighborhood label-barycenter spatial-KLA AA | 1.124343 | 0.554520 | 0.500000 |
| 33 | 104 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.913334 | 1.521965 | 0.500000 |
| 34 | 105 | Tuned spatial-KLA AA | 2.230528 | 2.046331 | 0.166667 |
| 34 | 105 | Neighborhood label-barycenter spatial-KLA AA | 0.695320 | 0.387367 | 0.111111 |
| 34 | 105 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.545622 | 1.322945 | 0.111111 |
| 35 | 106 | Tuned spatial-KLA AA | 2.274588 | 2.132090 | 0.152778 |
| 35 | 106 | Neighborhood label-barycenter spatial-KLA AA | 0.494515 | 0.310981 | 0.055556 |
| 35 | 106 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.318505 | 1.171603 | 0.055556 |
| 36 | 107 | Tuned spatial-KLA AA | 2.434010 | 2.124120 | 0.208333 |
| 36 | 107 | Neighborhood label-barycenter spatial-KLA AA | 0.587857 | 0.346259 | 0.111111 |
| 36 | 107 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.565792 | 1.392390 | 0.111111 |
| 37 | 108 | Tuned spatial-KLA AA | 2.730184 | 2.367811 | 0.111111 |
| 37 | 108 | Neighborhood label-barycenter spatial-KLA AA | 0.569076 | 0.300038 | 0.111111 |
| 37 | 108 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.444136 | 1.139841 | 0.111111 |
| 38 | 109 | Tuned spatial-KLA AA | 2.412082 | 2.221832 | 0.222222 |
| 38 | 109 | Neighborhood label-barycenter spatial-KLA AA | 0.500071 | 0.323418 | 0.055556 |
| 38 | 109 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.404596 | 1.265381 | 0.055556 |
| 39 | 110 | Tuned spatial-KLA AA | 2.585655 | 2.763925 | 0.430556 |
| 39 | 110 | Neighborhood label-barycenter spatial-KLA AA | 0.945368 | 0.496007 | 0.388889 |
| 39 | 110 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.637529 | 1.470572 | 0.388889 |
| 40 | 111 | Tuned spatial-KLA AA | 2.738147 | 2.512685 | 0.361111 |
| 40 | 111 | Neighborhood label-barycenter spatial-KLA AA | 0.751632 | 0.419637 | 0.222222 |
| 40 | 111 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.562411 | 1.266439 | 0.222222 |
| 41 | 112 | Tuned spatial-KLA AA | 2.275133 | 2.050592 | 0.152778 |
| 41 | 112 | Neighborhood label-barycenter spatial-KLA AA | 0.394137 | 0.290874 | 0.000000 |
| 41 | 112 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.372151 | 1.286691 | 0.000000 |
| 42 | 113 | Tuned spatial-KLA AA | 2.258016 | 2.169369 | 0.236111 |
| 42 | 113 | Neighborhood label-barycenter spatial-KLA AA | 0.714715 | 0.320662 | 0.222222 |
| 42 | 113 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.339165 | 1.137016 | 0.222222 |
| 43 | 114 | Tuned spatial-KLA AA | 2.609098 | 2.315097 | 0.208333 |
| 43 | 114 | Neighborhood label-barycenter spatial-KLA AA | 0.558534 | 0.269380 | 0.111111 |
| 43 | 114 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.850557 | 0.753650 | 0.111111 |
| 44 | 115 | Tuned spatial-KLA AA | 2.641571 | 2.405399 | 0.361111 |
| 44 | 115 | Neighborhood label-barycenter spatial-KLA AA | 0.788427 | 0.410433 | 0.222222 |
| 44 | 115 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.693573 | 1.524379 | 0.222222 |
| 45 | 116 | Tuned spatial-KLA AA | 2.173375 | 2.019913 | 0.291667 |
| 45 | 116 | Neighborhood label-barycenter spatial-KLA AA | 0.760331 | 0.342366 | 0.277778 |
| 45 | 116 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.352934 | 1.091592 | 0.277778 |
| 46 | 117 | Tuned spatial-KLA AA | 2.437967 | 2.579509 | 0.402778 |
| 46 | 117 | Neighborhood label-barycenter spatial-KLA AA | 0.799117 | 0.338525 | 0.333333 |
| 46 | 117 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.492978 | 1.128431 | 0.333333 |
| 47 | 118 | Tuned spatial-KLA AA | 2.189300 | 2.037682 | 0.041667 |
| 47 | 118 | Neighborhood label-barycenter spatial-KLA AA | 0.354199 | 0.264722 | 0.000000 |
| 47 | 118 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.319591 | 1.151411 | 0.000000 |
| 48 | 119 | Tuned spatial-KLA AA | 2.538515 | 3.109560 | 0.263889 |
| 48 | 119 | Neighborhood label-barycenter spatial-KLA AA | 0.766455 | 0.441572 | 0.222222 |
| 48 | 119 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.644885 | 1.409690 | 0.222222 |
| 49 | 120 | Tuned spatial-KLA AA | 2.378794 | 2.357071 | 0.263889 |
| 49 | 120 | Neighborhood label-barycenter spatial-KLA AA | 0.725164 | 0.377967 | 0.222222 |
| 49 | 120 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.536609 | 1.309941 | 0.222222 |
| 50 | 121 | Tuned spatial-KLA AA | 2.371095 | 1.961808 | 0.277778 |
| 50 | 121 | Neighborhood label-barycenter spatial-KLA AA | 0.681828 | 0.283186 | 0.222222 |
| 50 | 121 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.552202 | 1.225444 | 0.222222 |

### Scenario-Window Network Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.496695 +/- 0.196179 | [2.442317, 2.551073] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.656558 +/- 0.165860 | [0.610584, 0.702533] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.440864 +/- 0.211963 | [1.382111, 1.499617] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 2.363259 +/- 0.270961 | [2.288153, 2.438366] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.367736 +/- 0.083947 | [0.344467, 0.391005] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.263803 +/- 0.212883 | [1.204795, 1.322811] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.266944 +/- 0.112284 | [0.235821, 0.298068] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.168889 +/- 0.112217 | [0.137784, 0.199994] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.168889 +/- 0.112217 | [0.137784, 0.199994] | 50 |

### Scenario-Window Network Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.840136 +/- 0.200111 | [1.784669, 1.895604] | 73.70% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.055831 +/- 0.217744 | [0.995475, 1.116187] | 42.29% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.995524 +/- 0.227184 | [1.932551, 2.058496] | 84.44% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.099456 +/- 0.278556 | [1.022245, 1.176668] | 46.52% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.098056 +/- 0.090072 | [0.073089, 0.123022] | 36.73% | 43/50 | 5.889e-11 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.098056 +/- 0.090072 | [0.073089, 0.123022] | 36.73% | 43/50 | 5.889e-11 |

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
| 6 | 77 | Tuned spatial-KLA AA | 2.633782 | 2.193922 | 0.722222 |
| 6 | 77 | Neighborhood label-barycenter spatial-KLA AA | 2.178598 | 1.711947 | 0.388889 |
| 6 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.309450 | 1.909365 | 0.388889 |
| 7 | 78 | Tuned spatial-KLA AA | 2.542237 | 2.181592 | 0.277778 |
| 7 | 78 | Neighborhood label-barycenter spatial-KLA AA | 2.424936 | 2.219612 | 0.222222 |
| 7 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.429654 | 2.126570 | 0.222222 |
| 8 | 79 | Tuned spatial-KLA AA | 2.722360 | 2.366522 | 0.236111 |
| 8 | 79 | Neighborhood label-barycenter spatial-KLA AA | 2.249485 | 1.792688 | 0.111111 |
| 8 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.499481 | 2.117891 | 0.111111 |
| 9 | 80 | Tuned spatial-KLA AA | 2.686085 | 2.303251 | 0.208333 |
| 9 | 80 | Neighborhood label-barycenter spatial-KLA AA | 2.411873 | 1.963660 | 0.111111 |
| 9 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.706080 | 2.319034 | 0.111111 |
| 10 | 81 | Tuned spatial-KLA AA | 2.680782 | 2.355798 | 0.250000 |
| 10 | 81 | Neighborhood label-barycenter spatial-KLA AA | 2.292261 | 1.886710 | 0.111111 |
| 10 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.474013 | 2.213029 | 0.111111 |
| 11 | 82 | Tuned spatial-KLA AA | 2.811479 | 2.176893 | 0.972222 |
| 11 | 82 | Neighborhood label-barycenter spatial-KLA AA | 2.784896 | 1.951031 | 1.000000 |
| 11 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.909963 | 2.205459 | 1.000000 |
| 12 | 83 | Tuned spatial-KLA AA | 2.957468 | 2.549210 | 0.291667 |
| 12 | 83 | Neighborhood label-barycenter spatial-KLA AA | 2.640288 | 2.120634 | 0.277778 |
| 12 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.857669 | 2.410489 | 0.277778 |
| 13 | 84 | Tuned spatial-KLA AA | 2.647451 | 2.258906 | 0.319444 |
| 13 | 84 | Neighborhood label-barycenter spatial-KLA AA | 2.315557 | 1.890673 | 0.111111 |
| 13 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.457235 | 2.172769 | 0.111111 |
| 14 | 85 | Tuned spatial-KLA AA | 2.654447 | 2.307828 | 0.250000 |
| 14 | 85 | Neighborhood label-barycenter spatial-KLA AA | 2.229952 | 1.747586 | 0.055556 |
| 14 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.485848 | 2.224556 | 0.055556 |
| 15 | 86 | Tuned spatial-KLA AA | 2.316577 | 1.952125 | 0.291667 |
| 15 | 86 | Neighborhood label-barycenter spatial-KLA AA | 2.201627 | 1.828192 | 0.277778 |
| 15 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.211507 | 1.856140 | 0.277778 |
| 16 | 87 | Tuned spatial-KLA AA | 2.327292 | 2.079527 | 0.180556 |
| 16 | 87 | Neighborhood label-barycenter spatial-KLA AA | 1.870943 | 1.596648 | 0.111111 |
| 16 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.067220 | 1.833955 | 0.111111 |
| 17 | 88 | Tuned spatial-KLA AA | 2.361970 | 2.057575 | 0.319444 |
| 17 | 88 | Neighborhood label-barycenter spatial-KLA AA | 2.164606 | 2.011934 | 0.333333 |
| 17 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.259292 | 1.915174 | 0.333333 |
| 18 | 89 | Tuned spatial-KLA AA | 2.680470 | 2.296969 | 0.277778 |
| 18 | 89 | Neighborhood label-barycenter spatial-KLA AA | 2.257660 | 1.782569 | 0.166667 |
| 18 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.601911 | 2.203235 | 0.166667 |
| 19 | 90 | Tuned spatial-KLA AA | 2.711742 | 2.276349 | 0.361111 |
| 19 | 90 | Neighborhood label-barycenter spatial-KLA AA | 2.439570 | 1.883966 | 0.222222 |
| 19 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.662461 | 2.189912 | 0.222222 |
| 20 | 91 | Tuned spatial-KLA AA | 2.841935 | 2.373851 | 0.527778 |
| 20 | 91 | Neighborhood label-barycenter spatial-KLA AA | 2.619780 | 2.088340 | 0.500000 |
| 20 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.702068 | 2.270295 | 0.500000 |
| 21 | 92 | Tuned spatial-KLA AA | 2.442314 | 2.153435 | 0.236111 |
| 21 | 92 | Neighborhood label-barycenter spatial-KLA AA | 2.181224 | 1.984342 | 0.222222 |
| 21 | 92 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.315620 | 2.004617 | 0.222222 |
| 22 | 93 | Tuned spatial-KLA AA | 2.698469 | 2.514378 | 0.291667 |
| 22 | 93 | Neighborhood label-barycenter spatial-KLA AA | 2.481785 | 1.988904 | 0.111111 |
| 22 | 93 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.539415 | 2.182876 | 0.111111 |
| 23 | 94 | Tuned spatial-KLA AA | 2.340683 | 2.026913 | 0.222222 |
| 23 | 94 | Neighborhood label-barycenter spatial-KLA AA | 1.952842 | 1.603116 | 0.111111 |
| 23 | 94 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.097211 | 1.841595 | 0.111111 |
| 24 | 95 | Tuned spatial-KLA AA | 2.736680 | 2.260886 | 0.458333 |
| 24 | 95 | Neighborhood label-barycenter spatial-KLA AA | 2.365141 | 1.767105 | 0.333333 |
| 24 | 95 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.520128 | 2.115111 | 0.333333 |
| 25 | 96 | Tuned spatial-KLA AA | 2.441273 | 2.008899 | 0.416667 |
| 25 | 96 | Neighborhood label-barycenter spatial-KLA AA | 2.224181 | 1.839238 | 0.222222 |
| 25 | 96 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.321584 | 1.942735 | 0.222222 |
| 26 | 97 | Tuned spatial-KLA AA | 2.272649 | 2.003256 | 0.250000 |
| 26 | 97 | Neighborhood label-barycenter spatial-KLA AA | 1.881653 | 1.415549 | 0.277778 |
| 26 | 97 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.162619 | 1.878286 | 0.277778 |
| 27 | 98 | Tuned spatial-KLA AA | 2.499692 | 2.161312 | 0.333333 |
| 27 | 98 | Neighborhood label-barycenter spatial-KLA AA | 2.245587 | 1.751743 | 0.277778 |
| 27 | 98 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.381716 | 2.068810 | 0.277778 |
| 28 | 99 | Tuned spatial-KLA AA | 2.724101 | 2.293434 | 0.569444 |
| 28 | 99 | Neighborhood label-barycenter spatial-KLA AA | 2.629744 | 2.115097 | 0.500000 |
| 28 | 99 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.771417 | 2.392382 | 0.500000 |
| 29 | 100 | Tuned spatial-KLA AA | 2.656104 | 2.189404 | 0.333333 |
| 29 | 100 | Neighborhood label-barycenter spatial-KLA AA | 2.285152 | 1.726491 | 0.277778 |
| 29 | 100 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.503368 | 2.071022 | 0.277778 |
| 30 | 101 | Tuned spatial-KLA AA | 2.593054 | 2.327450 | 0.444444 |
| 30 | 101 | Neighborhood label-barycenter spatial-KLA AA | 2.304079 | 1.903629 | 0.333333 |
| 30 | 101 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.458375 | 2.158004 | 0.333333 |
| 31 | 102 | Tuned spatial-KLA AA | 2.521801 | 2.169421 | 0.347222 |
| 31 | 102 | Neighborhood label-barycenter spatial-KLA AA | 2.210480 | 1.746313 | 0.222222 |
| 31 | 102 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.438507 | 2.173496 | 0.222222 |
| 32 | 103 | Tuned spatial-KLA AA | 2.601771 | 2.321810 | 0.083333 |
| 32 | 103 | Neighborhood label-barycenter spatial-KLA AA | 2.215670 | 1.821312 | 0.055556 |
| 32 | 103 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.305630 | 2.038959 | 0.055556 |
| 33 | 104 | Tuned spatial-KLA AA | 3.101752 | 2.666875 | 0.555556 |
| 33 | 104 | Neighborhood label-barycenter spatial-KLA AA | 2.904627 | 2.429799 | 0.500000 |
| 33 | 104 | Neighborhood reference-only label-consensus spatial-KLA AA | 3.086445 | 2.578357 | 0.500000 |
| 34 | 105 | Tuned spatial-KLA AA | 2.358538 | 2.070023 | 0.166667 |
| 34 | 105 | Neighborhood label-barycenter spatial-KLA AA | 2.140102 | 1.731179 | 0.111111 |
| 34 | 105 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.331456 | 2.025713 | 0.111111 |
| 35 | 106 | Tuned spatial-KLA AA | 2.209736 | 1.921692 | 0.152778 |
| 35 | 106 | Neighborhood label-barycenter spatial-KLA AA | 1.830504 | 1.432316 | 0.055556 |
| 35 | 106 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.024620 | 1.818887 | 0.055556 |
| 36 | 107 | Tuned spatial-KLA AA | 2.488631 | 2.184443 | 0.263889 |
| 36 | 107 | Neighborhood label-barycenter spatial-KLA AA | 2.158117 | 1.759076 | 0.222222 |
| 36 | 107 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.364347 | 2.111469 | 0.222222 |
| 37 | 108 | Tuned spatial-KLA AA | 2.657231 | 2.278633 | 0.138889 |
| 37 | 108 | Neighborhood label-barycenter spatial-KLA AA | 2.298746 | 1.696157 | 0.111111 |
| 37 | 108 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.507473 | 2.077591 | 0.111111 |
| 38 | 109 | Tuned spatial-KLA AA | 2.721166 | 2.316706 | 0.305556 |
| 38 | 109 | Neighborhood label-barycenter spatial-KLA AA | 2.513566 | 1.958302 | 0.277778 |
| 38 | 109 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.685354 | 2.241079 | 0.277778 |
| 39 | 110 | Tuned spatial-KLA AA | 2.565666 | 2.220770 | 0.486111 |
| 39 | 110 | Neighborhood label-barycenter spatial-KLA AA | 2.380126 | 1.998332 | 0.388889 |
| 39 | 110 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.549750 | 2.204391 | 0.388889 |
| 40 | 111 | Tuned spatial-KLA AA | 2.599444 | 2.134733 | 0.361111 |
| 40 | 111 | Neighborhood label-barycenter spatial-KLA AA | 2.198794 | 1.609908 | 0.222222 |
| 40 | 111 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.413068 | 2.036340 | 0.222222 |
| 41 | 112 | Tuned spatial-KLA AA | 2.534562 | 2.205219 | 0.208333 |
| 41 | 112 | Neighborhood label-barycenter spatial-KLA AA | 2.214931 | 1.798489 | 0.111111 |
| 41 | 112 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.406664 | 2.113312 | 0.111111 |
| 42 | 113 | Tuned spatial-KLA AA | 2.464390 | 2.037889 | 0.319444 |
| 42 | 113 | Neighborhood label-barycenter spatial-KLA AA | 2.411034 | 1.775493 | 0.333333 |
| 42 | 113 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.414477 | 1.944136 | 0.333333 |
| 43 | 114 | Tuned spatial-KLA AA | 2.557966 | 2.182930 | 0.208333 |
| 43 | 114 | Neighborhood label-barycenter spatial-KLA AA | 2.129322 | 1.652003 | 0.111111 |
| 43 | 114 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.184376 | 1.878521 | 0.111111 |
| 44 | 115 | Tuned spatial-KLA AA | 2.784799 | 2.428906 | 0.444444 |
| 44 | 115 | Neighborhood label-barycenter spatial-KLA AA | 2.522721 | 2.024440 | 0.333333 |
| 44 | 115 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.699774 | 2.383558 | 0.333333 |
| 45 | 116 | Tuned spatial-KLA AA | 2.485108 | 2.079657 | 0.319444 |
| 45 | 116 | Neighborhood label-barycenter spatial-KLA AA | 2.217600 | 1.789521 | 0.277778 |
| 45 | 116 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.372459 | 1.957619 | 0.277778 |
| 46 | 117 | Tuned spatial-KLA AA | 2.564047 | 2.245732 | 0.513889 |
| 46 | 117 | Neighborhood label-barycenter spatial-KLA AA | 2.403991 | 1.826561 | 0.333333 |
| 46 | 117 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.409519 | 1.980912 | 0.333333 |
| 47 | 118 | Tuned spatial-KLA AA | 2.343009 | 2.044555 | 0.041667 |
| 47 | 118 | Neighborhood label-barycenter spatial-KLA AA | 1.889790 | 1.572972 | 0.000000 |
| 47 | 118 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.229727 | 1.937569 | 0.000000 |
| 48 | 119 | Tuned spatial-KLA AA | 2.786627 | 2.350765 | 0.402778 |
| 48 | 119 | Neighborhood label-barycenter spatial-KLA AA | 2.720221 | 2.343292 | 0.444444 |
| 48 | 119 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.834585 | 2.309067 | 0.444444 |
| 49 | 120 | Tuned spatial-KLA AA | 2.795617 | 2.306331 | 0.430556 |
| 49 | 120 | Neighborhood label-barycenter spatial-KLA AA | 2.558523 | 2.034624 | 0.333333 |
| 49 | 120 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.705746 | 2.285610 | 0.333333 |
| 50 | 121 | Tuned spatial-KLA AA | 2.480989 | 2.046795 | 0.305556 |
| 50 | 121 | Neighborhood label-barycenter spatial-KLA AA | 2.268260 | 1.631982 | 0.222222 |
| 50 | 121 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.403136 | 1.993489 | 0.222222 |

### Scenario-Window Local Summary
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.600152 +/- 0.180120 | [2.550225, 2.650079] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.312068 +/- 0.225080 | [2.249679, 2.374457] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.470599 +/- 0.218945 | [2.409910, 2.531287] | 50 |
| Tuned spatial-KLA AA | RMSE | 2.228931 +/- 0.156038 | [2.185680, 2.272183] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 1.850254 +/- 0.202540 | [1.794113, 1.906395] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 2.109105 +/- 0.171073 | [2.061686, 2.156524] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.331944 +/- 0.161002 | [0.287317, 0.376572] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.248889 +/- 0.164667 | [0.203246, 0.294532] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.248889 +/- 0.164667 | [0.203246, 0.294532] | 50 |

### Scenario-Window Local Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.288084 +/- 0.113445 | [0.256639, 0.319530] | 11.08% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.129553 +/- 0.094178 | [0.103449, 0.155658] | 4.98% | 46/50 | 4.462e-10 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.378677 +/- 0.153036 | [0.336258, 0.421097] | 16.99% | 49/50 | 9.059e-14 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.119826 +/- 0.090282 | [0.094802, 0.144851] | 5.38% | 46/50 | 4.462e-10 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.083056 +/- 0.072223 | [0.063036, 0.103075] | 25.02% | 45/50 | 4.21e-09 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.083056 +/- 0.072223 | [0.063036, 0.103075] | 25.02% | 45/50 | 4.21e-09 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 17.230369 +/- 0.905049 | 0.717932 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 40.372458 +/- 2.230690 | 1.682186 | 2.344x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 36.810690 +/- 1.512714 | 1.533779 | 2.139x | 50 |
