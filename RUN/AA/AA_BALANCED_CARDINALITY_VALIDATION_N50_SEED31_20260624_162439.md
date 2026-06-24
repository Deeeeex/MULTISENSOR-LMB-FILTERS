# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 18:07:21

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 31 (fixed=1)
- trialSeeds: [32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81]
- lmbParallelUpdateMode: AA
- scenarioLabel: topology-ring-formation
- neighborMapMode: ring
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
- Trial 1: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]
- Trial 2: [0.5 0.5 0.1 0.1 0.2 0.1 0 0.1]
- Trial 3: [0.1 0.1 0.1 0 0.5 0.1 0.2 0.5]
- Trial 4: [0.2 0.1 0.1 0 0.5 0.5 0.1 0.1]
- Trial 5: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 6: [0.5 0 0.1 0.5 0.2 0.1 0.1 0.1]
- Trial 7: [0.2 0.5 0.1 0.1 0.1 0.5 0 0.1]
- Trial 8: [0.1 0.5 0 0.1 0.5 0.1 0.1 0.2]
- Trial 9: [0.2 0.5 0 0.1 0.1 0.1 0.1 0.5]
- Trial 10: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 11: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 12: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 13: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 14: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 15: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 16: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 17: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 18: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 19: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 20: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]
- Trial 21: [0.5 0.1 0.1 0.5 0 0.2 0.1 0.1]
- Trial 22: [0.5 0 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 23: [0 0.5 0.5 0.1 0.1 0.1 0.1 0.2]
- Trial 24: [0.5 0 0.5 0.1 0.2 0.1 0.1 0.1]
- Trial 25: [0.1 0.5 0.1 0.1 0.5 0.2 0 0.1]
- Trial 26: [0.2 0.1 0 0.5 0.1 0.5 0.1 0.1]
- Trial 27: [0.1 0.2 0.1 0.5 0 0.5 0.1 0.1]
- Trial 28: [0.5 0.1 0.1 0.1 0.5 0.2 0 0.1]
- Trial 29: [0.2 0.5 0.1 0 0.1 0.5 0.1 0.1]
- Trial 30: [0 0.1 0.1 0.1 0.5 0.2 0.1 0.5]
- Trial 31: [0.1 0.1 0 0.2 0.1 0.5 0.1 0.5]
- Trial 32: [0.1 0.1 0 0.5 0.2 0.1 0.5 0.1]
- Trial 33: [0.1 0.1 0.2 0 0.1 0.1 0.5 0.5]
- Trial 34: [0.1 0.1 0 0.2 0.5 0.1 0.1 0.5]
- Trial 35: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 36: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 37: [0.1 0.2 0.1 0 0.1 0.1 0.5 0.5]
- Trial 38: [0 0.5 0.1 0.5 0.1 0.2 0.1 0.1]
- Trial 39: [0.1 0.5 0.1 0.1 0.5 0.1 0 0.2]
- Trial 40: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 41: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 42: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 43: [0.5 0.1 0.5 0.2 0.1 0 0.1 0.1]
- Trial 44: [0.1 0.1 0.1 0.5 0.2 0 0.5 0.1]
- Trial 45: [0 0.2 0.1 0.5 0.1 0.1 0.1 0.5]
- Trial 46: [0.1 0 0.5 0.5 0.2 0.1 0.1 0.1]
- Trial 47: [0.1 0.1 0.5 0.1 0 0.2 0.1 0.5]
- Trial 48: [0.1 0.1 0.2 0.1 0.1 0.5 0.5 0]
- Trial 49: [0.5 0.2 0.5 0 0.1 0.1 0.1 0.1]
- Trial 50: [0.1 0.1 0.5 0.1 0.5 0.1 0.2 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 32 | Tuned spatial-KLA AA | 2.771921 | 2.682580 | 0.083750 |
| 1 | 32 | Neighborhood label-barycenter spatial-KLA AA | 1.348601 | 1.197544 | 0.030000 |
| 1 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.263993 | 2.099705 | 0.030000 |
| 2 | 33 | Tuned spatial-KLA AA | 2.783937 | 2.969863 | 0.076250 |
| 2 | 33 | Neighborhood label-barycenter spatial-KLA AA | 1.484601 | 1.287325 | 0.035000 |
| 2 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.268028 | 2.092580 | 0.035000 |
| 3 | 34 | Tuned spatial-KLA AA | 2.767141 | 3.104279 | 0.076250 |
| 3 | 34 | Neighborhood label-barycenter spatial-KLA AA | 1.460093 | 1.368100 | 0.040000 |
| 3 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.356705 | 2.308617 | 0.040000 |
| 4 | 35 | Tuned spatial-KLA AA | 2.670840 | 2.558529 | 0.061250 |
| 4 | 35 | Neighborhood label-barycenter spatial-KLA AA | 1.239156 | 1.126368 | 0.020000 |
| 4 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.196907 | 2.144919 | 0.020000 |
| 5 | 36 | Tuned spatial-KLA AA | 2.784698 | 2.949485 | 0.068750 |
| 5 | 36 | Neighborhood label-barycenter spatial-KLA AA | 1.519421 | 1.259628 | 0.045000 |
| 5 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.401154 | 2.142118 | 0.045000 |
| 6 | 37 | Tuned spatial-KLA AA | 2.838680 | 3.044874 | 0.080000 |
| 6 | 37 | Neighborhood label-barycenter spatial-KLA AA | 1.453096 | 1.283433 | 0.052500 |
| 6 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.372438 | 2.429983 | 0.052500 |
| 7 | 38 | Tuned spatial-KLA AA | 2.696120 | 2.556268 | 0.093750 |
| 7 | 38 | Neighborhood label-barycenter spatial-KLA AA | 1.390076 | 1.140377 | 0.066250 |
| 7 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.206022 | 1.990206 | 0.066250 |
| 8 | 39 | Tuned spatial-KLA AA | 2.785448 | 2.884551 | 0.088750 |
| 8 | 39 | Neighborhood label-barycenter spatial-KLA AA | 1.399625 | 1.310687 | 0.038750 |
| 8 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.329613 | 2.469408 | 0.038750 |
| 9 | 40 | Tuned spatial-KLA AA | 2.952848 | 2.937485 | 0.123750 |
| 9 | 40 | Neighborhood label-barycenter spatial-KLA AA | 1.524626 | 1.288489 | 0.058750 |
| 9 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.366667 | 2.184756 | 0.058750 |
| 10 | 41 | Tuned spatial-KLA AA | 2.688758 | 2.835730 | 0.075000 |
| 10 | 41 | Neighborhood label-barycenter spatial-KLA AA | 1.379775 | 1.251113 | 0.021250 |
| 10 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.244038 | 2.087195 | 0.021250 |
| 11 | 42 | Tuned spatial-KLA AA | 2.789036 | 3.111542 | 0.076250 |
| 11 | 42 | Neighborhood label-barycenter spatial-KLA AA | 1.486153 | 1.335148 | 0.051250 |
| 11 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.383468 | 2.319496 | 0.051250 |
| 12 | 43 | Tuned spatial-KLA AA | 2.772830 | 2.979898 | 0.100000 |
| 12 | 43 | Neighborhood label-barycenter spatial-KLA AA | 1.559339 | 1.333606 | 0.066250 |
| 12 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.304776 | 2.242533 | 0.066250 |
| 13 | 44 | Tuned spatial-KLA AA | 2.711666 | 2.558265 | 0.072500 |
| 13 | 44 | Neighborhood label-barycenter spatial-KLA AA | 1.398178 | 1.154049 | 0.048750 |
| 13 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.371378 | 2.154950 | 0.048750 |
| 14 | 45 | Tuned spatial-KLA AA | 2.785249 | 2.672177 | 0.103750 |
| 14 | 45 | Neighborhood label-barycenter spatial-KLA AA | 1.423404 | 1.227186 | 0.057500 |
| 14 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.340848 | 2.257143 | 0.057500 |
| 15 | 46 | Tuned spatial-KLA AA | 2.808379 | 3.429787 | 0.098750 |
| 15 | 46 | Neighborhood label-barycenter spatial-KLA AA | 1.581979 | 1.547389 | 0.062500 |
| 15 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.287909 | 2.377685 | 0.062500 |
| 16 | 47 | Tuned spatial-KLA AA | 2.738632 | 2.522361 | 0.081250 |
| 16 | 47 | Neighborhood label-barycenter spatial-KLA AA | 1.350010 | 1.177269 | 0.042500 |
| 16 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.331965 | 2.117939 | 0.042500 |
| 17 | 48 | Tuned spatial-KLA AA | 2.943990 | 2.801990 | 0.092500 |
| 17 | 48 | Neighborhood label-barycenter spatial-KLA AA | 1.478880 | 1.299772 | 0.036250 |
| 17 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.413246 | 2.314110 | 0.036250 |
| 18 | 49 | Tuned spatial-KLA AA | 2.841332 | 2.648094 | 0.100000 |
| 18 | 49 | Neighborhood label-barycenter spatial-KLA AA | 1.439603 | 1.142364 | 0.048750 |
| 18 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.362139 | 2.106599 | 0.048750 |
| 19 | 50 | Tuned spatial-KLA AA | 2.894063 | 3.107516 | 0.098750 |
| 19 | 50 | Neighborhood label-barycenter spatial-KLA AA | 1.617232 | 1.347862 | 0.070000 |
| 19 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.459602 | 2.192704 | 0.070000 |
| 20 | 51 | Tuned spatial-KLA AA | 2.739420 | 2.591956 | 0.076250 |
| 20 | 51 | Neighborhood label-barycenter spatial-KLA AA | 1.368158 | 1.220531 | 0.042500 |
| 20 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.279972 | 2.064504 | 0.042500 |
| 21 | 52 | Tuned spatial-KLA AA | 2.820434 | 2.901712 | 0.110000 |
| 21 | 52 | Neighborhood label-barycenter spatial-KLA AA | 1.577413 | 1.292184 | 0.082500 |
| 21 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.437548 | 2.195907 | 0.082500 |
| 22 | 53 | Tuned spatial-KLA AA | 2.703932 | 2.709013 | 0.086250 |
| 22 | 53 | Neighborhood label-barycenter spatial-KLA AA | 1.398129 | 1.243555 | 0.043750 |
| 22 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.165952 | 2.014372 | 0.043750 |
| 23 | 54 | Tuned spatial-KLA AA | 2.721099 | 2.694765 | 0.090000 |
| 23 | 54 | Neighborhood label-barycenter spatial-KLA AA | 1.336894 | 1.147846 | 0.047500 |
| 23 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.220727 | 2.060347 | 0.047500 |
| 24 | 55 | Tuned spatial-KLA AA | 2.881570 | 2.981060 | 0.078750 |
| 24 | 55 | Neighborhood label-barycenter spatial-KLA AA | 1.508114 | 1.338332 | 0.041250 |
| 24 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.402978 | 2.167048 | 0.041250 |
| 25 | 56 | Tuned spatial-KLA AA | 2.833957 | 2.983444 | 0.088750 |
| 25 | 56 | Neighborhood label-barycenter spatial-KLA AA | 1.495998 | 1.398546 | 0.047500 |
| 25 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.422768 | 2.421688 | 0.047500 |
| 26 | 57 | Tuned spatial-KLA AA | 2.898688 | 2.937811 | 0.140000 |
| 26 | 57 | Neighborhood label-barycenter spatial-KLA AA | 1.611175 | 1.303510 | 0.088750 |
| 26 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.485172 | 2.508742 | 0.088750 |
| 27 | 58 | Tuned spatial-KLA AA | 2.913388 | 2.806124 | 0.098750 |
| 27 | 58 | Neighborhood label-barycenter spatial-KLA AA | 1.430180 | 1.218171 | 0.038750 |
| 27 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.328957 | 2.135270 | 0.038750 |
| 28 | 59 | Tuned spatial-KLA AA | 2.785065 | 2.728708 | 0.077500 |
| 28 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.409252 | 1.224242 | 0.040000 |
| 28 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.266730 | 2.355759 | 0.040000 |
| 29 | 60 | Tuned spatial-KLA AA | 2.772269 | 2.628585 | 0.083750 |
| 29 | 60 | Neighborhood label-barycenter spatial-KLA AA | 1.351420 | 1.113260 | 0.045000 |
| 29 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.258714 | 1.988553 | 0.045000 |
| 30 | 61 | Tuned spatial-KLA AA | 2.691286 | 3.086733 | 0.067500 |
| 30 | 61 | Neighborhood label-barycenter spatial-KLA AA | 1.513895 | 1.397801 | 0.037500 |
| 30 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.306676 | 2.047042 | 0.037500 |
| 31 | 62 | Tuned spatial-KLA AA | 2.827875 | 2.615341 | 0.111250 |
| 31 | 62 | Neighborhood label-barycenter spatial-KLA AA | 1.455347 | 1.151655 | 0.065000 |
| 31 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.414077 | 2.234937 | 0.065000 |
| 32 | 63 | Tuned spatial-KLA AA | 2.835480 | 2.881944 | 0.073750 |
| 32 | 63 | Neighborhood label-barycenter spatial-KLA AA | 1.518340 | 1.278950 | 0.050000 |
| 32 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.488093 | 2.300602 | 0.050000 |
| 33 | 64 | Tuned spatial-KLA AA | 2.770338 | 2.876738 | 0.100000 |
| 33 | 64 | Neighborhood label-barycenter spatial-KLA AA | 1.450113 | 1.272596 | 0.053750 |
| 33 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.232154 | 2.259146 | 0.053750 |
| 34 | 65 | Tuned spatial-KLA AA | 3.018888 | 3.110754 | 0.121250 |
| 34 | 65 | Neighborhood label-barycenter spatial-KLA AA | 1.674613 | 1.581280 | 0.085000 |
| 34 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.487405 | 2.358981 | 0.085000 |
| 35 | 66 | Tuned spatial-KLA AA | 2.791749 | 3.654542 | 0.076250 |
| 35 | 66 | Neighborhood label-barycenter spatial-KLA AA | 1.653884 | 1.540383 | 0.047500 |
| 35 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.313856 | 2.066764 | 0.047500 |
| 36 | 67 | Tuned spatial-KLA AA | 2.839185 | 3.891412 | 0.073750 |
| 36 | 67 | Neighborhood label-barycenter spatial-KLA AA | 1.604534 | 1.744153 | 0.041250 |
| 36 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.386545 | 2.536060 | 0.041250 |
| 37 | 68 | Tuned spatial-KLA AA | 2.966245 | 3.340906 | 0.121250 |
| 37 | 68 | Neighborhood label-barycenter spatial-KLA AA | 1.688304 | 1.465400 | 0.083750 |
| 37 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.528309 | 2.383906 | 0.083750 |
| 38 | 69 | Tuned spatial-KLA AA | 2.719752 | 2.676849 | 0.090000 |
| 38 | 69 | Neighborhood label-barycenter spatial-KLA AA | 1.491296 | 1.231716 | 0.056250 |
| 38 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.316611 | 2.128137 | 0.056250 |
| 39 | 70 | Tuned spatial-KLA AA | 2.813249 | 2.588185 | 0.067500 |
| 39 | 70 | Neighborhood label-barycenter spatial-KLA AA | 1.443144 | 1.168237 | 0.042500 |
| 39 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.403982 | 2.124079 | 0.042500 |
| 40 | 71 | Tuned spatial-KLA AA | 2.947080 | 3.060098 | 0.102500 |
| 40 | 71 | Neighborhood label-barycenter spatial-KLA AA | 1.617416 | 1.434370 | 0.065000 |
| 40 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.482008 | 2.452890 | 0.065000 |
| 41 | 72 | Tuned spatial-KLA AA | 2.768482 | 2.826470 | 0.061250 |
| 41 | 72 | Neighborhood label-barycenter spatial-KLA AA | 1.414537 | 1.207396 | 0.036250 |
| 41 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.245770 | 2.164587 | 0.036250 |
| 42 | 73 | Tuned spatial-KLA AA | 2.880958 | 2.596556 | 0.093750 |
| 42 | 73 | Neighborhood label-barycenter spatial-KLA AA | 1.431307 | 1.147560 | 0.045000 |
| 42 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.374763 | 2.140463 | 0.045000 |
| 43 | 74 | Tuned spatial-KLA AA | 2.910667 | 3.042199 | 0.066250 |
| 43 | 74 | Neighborhood label-barycenter spatial-KLA AA | 1.373905 | 1.388362 | 0.028750 |
| 43 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.410563 | 2.515269 | 0.028750 |
| 44 | 75 | Tuned spatial-KLA AA | 2.789823 | 2.840159 | 0.093750 |
| 44 | 75 | Neighborhood label-barycenter spatial-KLA AA | 1.437940 | 1.318287 | 0.058750 |
| 44 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.332556 | 2.336825 | 0.058750 |
| 45 | 76 | Tuned spatial-KLA AA | 2.893154 | 3.153595 | 0.106250 |
| 45 | 76 | Neighborhood label-barycenter spatial-KLA AA | 1.519577 | 1.296197 | 0.060000 |
| 45 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.375332 | 2.312070 | 0.060000 |
| 46 | 77 | Tuned spatial-KLA AA | 2.870361 | 3.070072 | 0.088750 |
| 46 | 77 | Neighborhood label-barycenter spatial-KLA AA | 1.514303 | 1.249691 | 0.056250 |
| 46 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.409351 | 2.241180 | 0.056250 |
| 47 | 78 | Tuned spatial-KLA AA | 2.811975 | 2.671365 | 0.100000 |
| 47 | 78 | Neighborhood label-barycenter spatial-KLA AA | 1.446525 | 1.120439 | 0.073750 |
| 47 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.327924 | 2.089920 | 0.073750 |
| 48 | 79 | Tuned spatial-KLA AA | 2.691542 | 3.028866 | 0.067500 |
| 48 | 79 | Neighborhood label-barycenter spatial-KLA AA | 1.468772 | 1.287406 | 0.037500 |
| 48 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.167967 | 1.999974 | 0.037500 |
| 49 | 80 | Tuned spatial-KLA AA | 2.869949 | 3.062541 | 0.128750 |
| 49 | 80 | Neighborhood label-barycenter spatial-KLA AA | 1.527531 | 1.292647 | 0.057500 |
| 49 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.299310 | 2.367692 | 0.057500 |
| 50 | 81 | Tuned spatial-KLA AA | 2.918948 | 3.492191 | 0.060000 |
| 50 | 81 | Neighborhood label-barycenter spatial-KLA AA | 1.607603 | 1.456989 | 0.022500 |
| 50 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.372685 | 2.249512 | 0.022500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.814447 | 2.917719 | 0.089050 |
| Neighborhood label-barycenter spatial-KLA AA | 1.477469 | 1.292188 | 0.050250 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.344127 | 2.225097 | 0.050250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.814447 +/- 0.082856 | [2.791481, 2.837414] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.477469 +/- 0.096191 | [1.450806, 1.504132] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 2.344127 +/- 0.087009 | [2.320009, 2.368245] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 2.917719 +/- 0.290764 | [2.837124, 2.998315] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.292188 +/- 0.130147 | [1.256113, 1.328263] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 2.225097 +/- 0.148876 | [2.183831, 2.266364] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.089050 +/- 0.018536 | [0.083912, 0.094188] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.050250 +/- 0.016063 | [0.045797, 0.054703] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.050250 +/- 0.016063 | [0.045797, 0.054703] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.336978 +/- 0.078981 | [1.315086, 1.358870] | 47.50% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.470320 +/- 0.058882 | [0.453999, 0.486642] | 16.71% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.625531 +/- 0.180628 | [1.575464, 1.675599] | 55.71% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.692622 +/- 0.255919 | [0.621685, 0.763559] | 23.74% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.038800 +/- 0.011149 | [0.035710, 0.041890] | 43.57% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.038800 +/- 0.011149 | [0.035710, 0.041890] | 43.57% | 50/50 | 1.776e-15 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 32 | Tuned spatial-KLA AA | 2.580668 | 4.568272 | 0.116250 |
| 1 | 32 | Neighborhood label-barycenter spatial-KLA AA | 2.010548 | 4.035532 | 0.065000 |
| 1 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.410974 | 4.511156 | 0.065000 |
| 2 | 33 | Tuned spatial-KLA AA | 2.515239 | 4.181518 | 0.126250 |
| 2 | 33 | Neighborhood label-barycenter spatial-KLA AA | 1.960509 | 3.554518 | 0.090000 |
| 2 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.295833 | 3.829228 | 0.090000 |
| 3 | 34 | Tuned spatial-KLA AA | 2.604247 | 5.000523 | 0.126250 |
| 3 | 34 | Neighborhood label-barycenter spatial-KLA AA | 2.032675 | 4.387011 | 0.090000 |
| 3 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.448202 | 4.716316 | 0.090000 |
| 4 | 35 | Tuned spatial-KLA AA | 2.477691 | 4.488773 | 0.111250 |
| 4 | 35 | Neighborhood label-barycenter spatial-KLA AA | 1.886131 | 3.962488 | 0.080000 |
| 4 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.318712 | 4.345406 | 0.080000 |
| 5 | 36 | Tuned spatial-KLA AA | 2.520962 | 4.239493 | 0.111250 |
| 5 | 36 | Neighborhood label-barycenter spatial-KLA AA | 2.006823 | 3.602879 | 0.087500 |
| 5 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.373872 | 3.899913 | 0.087500 |
| 6 | 37 | Tuned spatial-KLA AA | 2.608508 | 4.098639 | 0.127500 |
| 6 | 37 | Neighborhood label-barycenter spatial-KLA AA | 2.006571 | 3.550173 | 0.105000 |
| 6 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.456091 | 3.888801 | 0.105000 |
| 7 | 38 | Tuned spatial-KLA AA | 2.522922 | 4.125540 | 0.141250 |
| 7 | 38 | Neighborhood label-barycenter spatial-KLA AA | 1.955500 | 3.588248 | 0.116250 |
| 7 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.361944 | 3.944029 | 0.116250 |
| 8 | 39 | Tuned spatial-KLA AA | 2.573970 | 4.538927 | 0.146250 |
| 8 | 39 | Neighborhood label-barycenter spatial-KLA AA | 1.998677 | 3.797572 | 0.108750 |
| 8 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.404074 | 4.176181 | 0.108750 |
| 9 | 40 | Tuned spatial-KLA AA | 2.706995 | 4.549273 | 0.186250 |
| 9 | 40 | Neighborhood label-barycenter spatial-KLA AA | 2.174955 | 4.065157 | 0.131250 |
| 9 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.496338 | 4.335961 | 0.131250 |
| 10 | 41 | Tuned spatial-KLA AA | 2.458880 | 4.472769 | 0.110000 |
| 10 | 41 | Neighborhood label-barycenter spatial-KLA AA | 1.956105 | 3.988057 | 0.056250 |
| 10 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.285731 | 4.311238 | 0.056250 |
| 11 | 42 | Tuned spatial-KLA AA | 2.578120 | 4.671757 | 0.116250 |
| 11 | 42 | Neighborhood label-barycenter spatial-KLA AA | 2.062385 | 3.971209 | 0.101250 |
| 11 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.439016 | 4.453471 | 0.101250 |
| 12 | 43 | Tuned spatial-KLA AA | 2.497035 | 4.376562 | 0.135000 |
| 12 | 43 | Neighborhood label-barycenter spatial-KLA AA | 2.008027 | 3.979511 | 0.101250 |
| 12 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.333573 | 4.288374 | 0.101250 |
| 13 | 44 | Tuned spatial-KLA AA | 2.553763 | 3.870639 | 0.125000 |
| 13 | 44 | Neighborhood label-barycenter spatial-KLA AA | 2.009299 | 3.431695 | 0.108750 |
| 13 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.433608 | 3.783356 | 0.108750 |
| 14 | 45 | Tuned spatial-KLA AA | 2.659173 | 3.864193 | 0.161250 |
| 14 | 45 | Neighborhood label-barycenter spatial-KLA AA | 2.098164 | 3.280711 | 0.127500 |
| 14 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.511885 | 3.685540 | 0.127500 |
| 15 | 46 | Tuned spatial-KLA AA | 2.577684 | 4.215910 | 0.141250 |
| 15 | 46 | Neighborhood label-barycenter spatial-KLA AA | 2.099774 | 3.591379 | 0.115000 |
| 15 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.397928 | 3.797751 | 0.115000 |
| 16 | 47 | Tuned spatial-KLA AA | 2.497409 | 3.989225 | 0.128750 |
| 16 | 47 | Neighborhood label-barycenter spatial-KLA AA | 1.908787 | 3.380078 | 0.102500 |
| 16 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.340616 | 3.807223 | 0.102500 |
| 17 | 48 | Tuned spatial-KLA AA | 2.650721 | 4.414858 | 0.150000 |
| 17 | 48 | Neighborhood label-barycenter spatial-KLA AA | 2.026931 | 3.923598 | 0.106250 |
| 17 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.459992 | 4.331721 | 0.106250 |
| 18 | 49 | Tuned spatial-KLA AA | 2.581523 | 4.178650 | 0.140000 |
| 18 | 49 | Neighborhood label-barycenter spatial-KLA AA | 1.901370 | 3.685289 | 0.093750 |
| 18 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.386011 | 4.144827 | 0.093750 |
| 19 | 50 | Tuned spatial-KLA AA | 2.621845 | 4.691490 | 0.141250 |
| 19 | 50 | Neighborhood label-barycenter spatial-KLA AA | 2.132021 | 4.030349 | 0.117500 |
| 19 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.448566 | 4.278106 | 0.117500 |
| 20 | 51 | Tuned spatial-KLA AA | 2.541263 | 4.042856 | 0.128750 |
| 20 | 51 | Neighborhood label-barycenter spatial-KLA AA | 1.927985 | 3.515083 | 0.102500 |
| 20 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.366303 | 3.908361 | 0.102500 |
| 21 | 52 | Tuned spatial-KLA AA | 2.564330 | 4.175151 | 0.167500 |
| 21 | 52 | Neighborhood label-barycenter spatial-KLA AA | 2.026027 | 3.518204 | 0.145000 |
| 21 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.416218 | 3.913559 | 0.145000 |
| 22 | 53 | Tuned spatial-KLA AA | 2.471470 | 4.320133 | 0.128750 |
| 22 | 53 | Neighborhood label-barycenter spatial-KLA AA | 1.963480 | 3.922736 | 0.091250 |
| 22 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.272572 | 4.110285 | 0.091250 |
| 23 | 54 | Tuned spatial-KLA AA | 2.506599 | 4.407005 | 0.132500 |
| 23 | 54 | Neighborhood label-barycenter spatial-KLA AA | 1.909139 | 3.849666 | 0.092500 |
| 23 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.305600 | 4.277203 | 0.092500 |
| 24 | 55 | Tuned spatial-KLA AA | 2.630857 | 4.419665 | 0.136250 |
| 24 | 55 | Neighborhood label-barycenter spatial-KLA AA | 2.087712 | 3.911426 | 0.103750 |
| 24 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.469666 | 4.273248 | 0.103750 |
| 25 | 56 | Tuned spatial-KLA AA | 2.553616 | 4.701485 | 0.121250 |
| 25 | 56 | Neighborhood label-barycenter spatial-KLA AA | 1.973945 | 4.145430 | 0.080000 |
| 25 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.400978 | 4.444783 | 0.080000 |
| 26 | 57 | Tuned spatial-KLA AA | 2.683424 | 4.755077 | 0.192500 |
| 26 | 57 | Neighborhood label-barycenter spatial-KLA AA | 2.130213 | 4.172253 | 0.146250 |
| 26 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.519594 | 4.670439 | 0.146250 |
| 27 | 58 | Tuned spatial-KLA AA | 2.696647 | 4.194980 | 0.151250 |
| 27 | 58 | Neighborhood label-barycenter spatial-KLA AA | 2.075533 | 3.674943 | 0.098750 |
| 27 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.478059 | 4.006015 | 0.098750 |
| 28 | 59 | Tuned spatial-KLA AA | 2.566181 | 3.876499 | 0.137500 |
| 28 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.968031 | 3.316738 | 0.105000 |
| 28 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.358437 | 3.652795 | 0.105000 |
| 29 | 60 | Tuned spatial-KLA AA | 2.633828 | 4.241272 | 0.143750 |
| 29 | 60 | Neighborhood label-barycenter spatial-KLA AA | 2.096129 | 3.651948 | 0.112500 |
| 29 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.455590 | 3.968663 | 0.112500 |
| 30 | 61 | Tuned spatial-KLA AA | 2.572851 | 4.933007 | 0.112500 |
| 30 | 61 | Neighborhood label-barycenter spatial-KLA AA | 2.119736 | 4.361326 | 0.087500 |
| 30 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.443741 | 4.485808 | 0.087500 |
| 31 | 62 | Tuned spatial-KLA AA | 2.558026 | 4.247209 | 0.158750 |
| 31 | 62 | Neighborhood label-barycenter spatial-KLA AA | 1.940217 | 3.762342 | 0.115000 |
| 31 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.404183 | 4.209147 | 0.115000 |
| 32 | 63 | Tuned spatial-KLA AA | 2.582292 | 4.269396 | 0.116250 |
| 32 | 63 | Neighborhood label-barycenter spatial-KLA AA | 1.998957 | 3.584911 | 0.102500 |
| 32 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.442549 | 4.003392 | 0.102500 |
| 33 | 64 | Tuned spatial-KLA AA | 2.539513 | 4.582686 | 0.155000 |
| 33 | 64 | Neighborhood label-barycenter spatial-KLA AA | 1.980306 | 4.023949 | 0.113750 |
| 33 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.342781 | 4.384140 | 0.113750 |
| 34 | 65 | Tuned spatial-KLA AA | 2.754923 | 4.652565 | 0.171250 |
| 34 | 65 | Neighborhood label-barycenter spatial-KLA AA | 2.183189 | 4.032075 | 0.140000 |
| 34 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.547787 | 4.338095 | 0.140000 |
| 35 | 66 | Tuned spatial-KLA AA | 2.560182 | 4.577900 | 0.126250 |
| 35 | 66 | Neighborhood label-barycenter spatial-KLA AA | 2.125983 | 4.004259 | 0.102500 |
| 35 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.394594 | 4.029897 | 0.102500 |
| 36 | 67 | Tuned spatial-KLA AA | 2.556643 | 4.480857 | 0.121250 |
| 36 | 67 | Neighborhood label-barycenter spatial-KLA AA | 2.073397 | 3.856761 | 0.096250 |
| 36 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.401781 | 3.998499 | 0.096250 |
| 37 | 68 | Tuned spatial-KLA AA | 2.747947 | 4.545104 | 0.186250 |
| 37 | 68 | Neighborhood label-barycenter spatial-KLA AA | 2.306414 | 4.093518 | 0.158750 |
| 37 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.624716 | 4.261085 | 0.158750 |
| 38 | 69 | Tuned spatial-KLA AA | 2.467942 | 4.205584 | 0.122500 |
| 38 | 69 | Neighborhood label-barycenter spatial-KLA AA | 1.886202 | 3.605394 | 0.088750 |
| 38 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.304905 | 3.957498 | 0.088750 |
| 39 | 70 | Tuned spatial-KLA AA | 2.535176 | 4.217843 | 0.117500 |
| 39 | 70 | Neighborhood label-barycenter spatial-KLA AA | 1.924061 | 3.750419 | 0.095000 |
| 39 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.395274 | 4.164725 | 0.095000 |
| 40 | 71 | Tuned spatial-KLA AA | 2.687836 | 4.463627 | 0.162500 |
| 40 | 71 | Neighborhood label-barycenter spatial-KLA AA | 2.142112 | 3.913390 | 0.132500 |
| 40 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.506828 | 4.297002 | 0.132500 |
| 41 | 72 | Tuned spatial-KLA AA | 2.570063 | 4.408403 | 0.118750 |
| 41 | 72 | Neighborhood label-barycenter spatial-KLA AA | 1.973046 | 3.721348 | 0.098750 |
| 41 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.380491 | 4.168274 | 0.098750 |
| 42 | 73 | Tuned spatial-KLA AA | 2.619526 | 4.167284 | 0.133750 |
| 42 | 73 | Neighborhood label-barycenter spatial-KLA AA | 1.982814 | 3.678059 | 0.087500 |
| 42 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.433026 | 4.078213 | 0.087500 |
| 43 | 74 | Tuned spatial-KLA AA | 2.582162 | 4.494696 | 0.106250 |
| 43 | 74 | Neighborhood label-barycenter spatial-KLA AA | 1.932611 | 3.802358 | 0.071250 |
| 43 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.375882 | 4.253762 | 0.071250 |
| 44 | 75 | Tuned spatial-KLA AA | 2.633063 | 4.444132 | 0.146250 |
| 44 | 75 | Neighborhood label-barycenter spatial-KLA AA | 2.094203 | 3.850394 | 0.116250 |
| 44 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.473233 | 4.227728 | 0.116250 |
| 45 | 76 | Tuned spatial-KLA AA | 2.622925 | 4.389897 | 0.146250 |
| 45 | 76 | Neighborhood label-barycenter spatial-KLA AA | 1.999843 | 3.866667 | 0.110000 |
| 45 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.450824 | 4.377172 | 0.110000 |
| 46 | 77 | Tuned spatial-KLA AA | 2.594988 | 4.166010 | 0.143750 |
| 46 | 77 | Neighborhood label-barycenter spatial-KLA AA | 1.990051 | 3.488606 | 0.113750 |
| 46 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.421942 | 3.921114 | 0.113750 |
| 47 | 78 | Tuned spatial-KLA AA | 2.617275 | 4.108426 | 0.152500 |
| 47 | 78 | Neighborhood label-barycenter spatial-KLA AA | 2.029734 | 3.492716 | 0.131250 |
| 47 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.443316 | 3.996407 | 0.131250 |
| 48 | 79 | Tuned spatial-KLA AA | 2.555200 | 4.278442 | 0.122500 |
| 48 | 79 | Neighborhood label-barycenter spatial-KLA AA | 2.137604 | 3.609214 | 0.110000 |
| 48 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.376253 | 3.871081 | 0.110000 |
| 49 | 80 | Tuned spatial-KLA AA | 2.597037 | 4.286383 | 0.168750 |
| 49 | 80 | Neighborhood label-barycenter spatial-KLA AA | 2.098610 | 3.652883 | 0.102500 |
| 49 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.361340 | 3.998700 | 0.102500 |
| 50 | 81 | Tuned spatial-KLA AA | 2.606494 | 5.032206 | 0.097500 |
| 50 | 81 | Neighborhood label-barycenter spatial-KLA AA | 2.068839 | 4.315331 | 0.062500 |
| 50 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.392385 | 4.536258 | 0.062500 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.583953 | 4.372456 | 0.137350 |
| Neighborhood label-barycenter spatial-KLA AA | 2.027627 | 3.798996 | 0.104350 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.411276 | 4.146239 | 0.104350 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.583953 +/- 0.068143 | [2.565064, 2.602841] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.027627 +/- 0.088464 | [2.003106, 2.052149] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.411276 +/- 0.070189 | [2.391821, 2.430732] | 50 |
| Tuned spatial-KLA AA | RMSE | 4.372456 +/- 0.269369 | [4.297791, 4.447121] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.798996 +/- 0.264201 | [3.725763, 3.872229] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.146239 +/- 0.252358 | [4.076289, 4.216189] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.137350 +/- 0.021665 | [0.131345, 0.143355] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.104350 +/- 0.020960 | [0.098540, 0.110160] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.104350 +/- 0.020960 | [0.098540, 0.110160] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.556325 +/- 0.057567 | [0.540369, 0.572282] | 21.53% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.172676 +/- 0.026266 | [0.165396, 0.179957] | 6.68% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.573460 +/- 0.082351 | [0.550633, 0.596286] | 13.12% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.226217 +/- 0.128049 | [0.190724, 0.261710] | 5.17% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.033000 +/- 0.011468 | [0.029821, 0.036179] | 24.03% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.033000 +/- 0.011468 | [0.029821, 0.036179] | 24.03% | 50/50 | 1.776e-15 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 28.196607 +/- 3.257764 | 1.174859 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 40.379173 +/- 5.471138 | 1.682466 | 1.431x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 36.753796 +/- 4.312098 | 1.531408 | 1.309x | 50 |
