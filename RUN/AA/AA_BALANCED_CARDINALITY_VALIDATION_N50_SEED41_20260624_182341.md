# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 20:49:34

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 41 (fixed=1)
- trialSeeds: [42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81 82 83 84 85 86 87 88 89 90 91]
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
- Trial 6: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 7: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 8: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 9: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 10: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]
- Trial 11: [0.5 0.1 0.1 0.5 0 0.2 0.1 0.1]
- Trial 12: [0.5 0 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 13: [0 0.5 0.5 0.1 0.1 0.1 0.1 0.2]
- Trial 14: [0.5 0 0.5 0.1 0.2 0.1 0.1 0.1]
- Trial 15: [0.1 0.5 0.1 0.1 0.5 0.2 0 0.1]
- Trial 16: [0.2 0.1 0 0.5 0.1 0.5 0.1 0.1]
- Trial 17: [0.1 0.2 0.1 0.5 0 0.5 0.1 0.1]
- Trial 18: [0.5 0.1 0.1 0.1 0.5 0.2 0 0.1]
- Trial 19: [0.2 0.5 0.1 0 0.1 0.5 0.1 0.1]
- Trial 20: [0 0.1 0.1 0.1 0.5 0.2 0.1 0.5]
- Trial 21: [0.1 0.1 0 0.2 0.1 0.5 0.1 0.5]
- Trial 22: [0.1 0.1 0 0.5 0.2 0.1 0.5 0.1]
- Trial 23: [0.1 0.1 0.2 0 0.1 0.1 0.5 0.5]
- Trial 24: [0.1 0.1 0 0.2 0.5 0.1 0.1 0.5]
- Trial 25: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 26: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 27: [0.1 0.2 0.1 0 0.1 0.1 0.5 0.5]
- Trial 28: [0 0.5 0.1 0.5 0.1 0.2 0.1 0.1]
- Trial 29: [0.1 0.5 0.1 0.1 0.5 0.1 0 0.2]
- Trial 30: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 31: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 32: [0.1 0.1 0.1 0.5 0.5 0.2 0 0.1]
- Trial 33: [0.5 0.1 0.5 0.2 0.1 0 0.1 0.1]
- Trial 34: [0.1 0.1 0.1 0.5 0.2 0 0.5 0.1]
- Trial 35: [0 0.2 0.1 0.5 0.1 0.1 0.1 0.5]
- Trial 36: [0.1 0 0.5 0.5 0.2 0.1 0.1 0.1]
- Trial 37: [0.1 0.1 0.5 0.1 0 0.2 0.1 0.5]
- Trial 38: [0.1 0.1 0.2 0.1 0.1 0.5 0.5 0]
- Trial 39: [0.5 0.2 0.5 0 0.1 0.1 0.1 0.1]
- Trial 40: [0.1 0.1 0.5 0.1 0.5 0.1 0.2 0]
- Trial 41: [0.1 0.1 0.1 0.5 0.1 0.2 0 0.5]
- Trial 42: [0.1 0.2 0.5 0.1 0 0.5 0.1 0.1]
- Trial 43: [0.2 0.5 0.1 0.1 0.1 0.5 0.1 0]
- Trial 44: [0.1 0.5 0.5 0 0.1 0.1 0.1 0.2]
- Trial 45: [0.2 0.5 0.5 0.1 0.1 0.1 0 0.1]
- Trial 46: [0.1 0.1 0.2 0.1 0.5 0.5 0.1 0]
- Trial 47: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 48: [0.1 0 0.1 0.1 0.5 0.1 0.2 0.5]
- Trial 49: [0.5 0.1 0.1 0.1 0.1 0.2 0 0.5]
- Trial 50: [0.1 0.2 0.5 0.1 0.5 0.1 0 0.1]

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
| 6 | 47 | Tuned spatial-KLA AA | 2.286839 | 1.155414 | 0.427500 |
| 6 | 47 | Neighborhood label-barycenter spatial-KLA AA | 1.440852 | 0.147892 | 0.435000 |
| 6 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.873789 | 0.639268 | 0.435000 |
| 7 | 48 | Tuned spatial-KLA AA | 2.336721 | 1.058926 | 0.447500 |
| 7 | 48 | Neighborhood label-barycenter spatial-KLA AA | 1.453970 | 0.152695 | 0.430000 |
| 7 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.859543 | 0.634913 | 0.430000 |
| 8 | 49 | Tuned spatial-KLA AA | 2.238609 | 0.977115 | 0.442500 |
| 8 | 49 | Neighborhood label-barycenter spatial-KLA AA | 1.414787 | 0.125830 | 0.430000 |
| 8 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.824411 | 0.585138 | 0.430000 |
| 9 | 50 | Tuned spatial-KLA AA | 2.265014 | 1.119599 | 0.450000 |
| 9 | 50 | Neighborhood label-barycenter spatial-KLA AA | 1.451030 | 0.147337 | 0.435000 |
| 9 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.816761 | 0.578491 | 0.435000 |
| 10 | 51 | Tuned spatial-KLA AA | 2.372734 | 2.296573 | 0.207500 |
| 10 | 51 | Neighborhood label-barycenter spatial-KLA AA | 0.593902 | 0.309021 | 0.110000 |
| 10 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.005714 | 0.827878 | 0.110000 |
| 11 | 52 | Tuned spatial-KLA AA | 2.597917 | 2.749847 | 0.333750 |
| 11 | 52 | Neighborhood label-barycenter spatial-KLA AA | 1.060497 | 0.412406 | 0.280000 |
| 11 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.508020 | 1.424475 | 0.280000 |
| 12 | 53 | Tuned spatial-KLA AA | 2.268460 | 1.373504 | 0.418750 |
| 12 | 53 | Neighborhood label-barycenter spatial-KLA AA | 1.408000 | 0.218712 | 0.365000 |
| 12 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.694161 | 0.927446 | 0.365000 |
| 13 | 54 | Tuned spatial-KLA AA | 2.230992 | 1.312539 | 0.440000 |
| 13 | 54 | Neighborhood label-barycenter spatial-KLA AA | 1.373218 | 0.144122 | 0.440000 |
| 13 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.790754 | 0.588641 | 0.440000 |
| 14 | 55 | Tuned spatial-KLA AA | 2.440112 | 1.878826 | 0.390000 |
| 14 | 55 | Neighborhood label-barycenter spatial-KLA AA | 1.144419 | 0.357978 | 0.345000 |
| 14 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.607971 | 0.980087 | 0.345000 |
| 15 | 56 | Tuned spatial-KLA AA | 2.223071 | 1.108084 | 0.402500 |
| 15 | 56 | Neighborhood label-barycenter spatial-KLA AA | 1.359618 | 0.140011 | 0.380000 |
| 15 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.810055 | 0.625456 | 0.380000 |
| 16 | 57 | Tuned spatial-KLA AA | 2.222874 | 1.055245 | 0.422500 |
| 16 | 57 | Neighborhood label-barycenter spatial-KLA AA | 1.370331 | 0.137943 | 0.415000 |
| 16 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.780634 | 0.586717 | 0.415000 |
| 17 | 58 | Tuned spatial-KLA AA | 2.393116 | 2.526957 | 0.368750 |
| 17 | 58 | Neighborhood label-barycenter spatial-KLA AA | 1.224370 | 0.369633 | 0.315000 |
| 17 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.651124 | 1.042252 | 0.315000 |
| 18 | 59 | Tuned spatial-KLA AA | 2.169314 | 0.969843 | 0.472500 |
| 18 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.350819 | 0.139250 | 0.460000 |
| 18 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.803866 | 0.636169 | 0.460000 |
| 19 | 60 | Tuned spatial-KLA AA | 2.331506 | 1.825472 | 0.400000 |
| 19 | 60 | Neighborhood label-barycenter spatial-KLA AA | 1.389907 | 0.210272 | 0.440000 |
| 19 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.819434 | 0.930126 | 0.440000 |
| 20 | 61 | Tuned spatial-KLA AA | 2.151471 | 0.896362 | 0.446250 |
| 20 | 61 | Neighborhood label-barycenter spatial-KLA AA | 1.364395 | 0.119484 | 0.435000 |
| 20 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.745857 | 0.515807 | 0.435000 |
| 21 | 62 | Tuned spatial-KLA AA | 2.243428 | 1.415766 | 0.413750 |
| 21 | 62 | Neighborhood label-barycenter spatial-KLA AA | 1.393881 | 0.247219 | 0.405000 |
| 21 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.821053 | 1.106230 | 0.405000 |
| 22 | 63 | Tuned spatial-KLA AA | 2.240747 | 1.017796 | 0.470000 |
| 22 | 63 | Neighborhood label-barycenter spatial-KLA AA | 1.374423 | 0.146704 | 0.460000 |
| 22 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.773136 | 0.607924 | 0.460000 |
| 23 | 64 | Tuned spatial-KLA AA | 2.328323 | 1.414532 | 0.483750 |
| 23 | 64 | Neighborhood label-barycenter spatial-KLA AA | 1.463285 | 0.191997 | 0.460000 |
| 23 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.757041 | 0.778261 | 0.460000 |
| 24 | 65 | Tuned spatial-KLA AA | 2.245845 | 1.258956 | 0.425000 |
| 24 | 65 | Neighborhood label-barycenter spatial-KLA AA | 1.365204 | 0.185341 | 0.405000 |
| 24 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.708046 | 0.788169 | 0.405000 |
| 25 | 66 | Tuned spatial-KLA AA | 2.366938 | 2.198371 | 0.286250 |
| 25 | 66 | Neighborhood label-barycenter spatial-KLA AA | 0.893223 | 0.310277 | 0.255000 |
| 25 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.309392 | 0.914942 | 0.255000 |
| 26 | 67 | Tuned spatial-KLA AA | 2.220703 | 0.979023 | 0.475000 |
| 26 | 67 | Neighborhood label-barycenter spatial-KLA AA | 1.411207 | 0.127418 | 0.460000 |
| 26 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.784703 | 0.520194 | 0.460000 |
| 27 | 68 | Tuned spatial-KLA AA | 2.487066 | 2.193417 | 0.438750 |
| 27 | 68 | Neighborhood label-barycenter spatial-KLA AA | 1.511575 | 0.260996 | 0.460000 |
| 27 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.853145 | 1.089382 | 0.460000 |
| 28 | 69 | Tuned spatial-KLA AA | 2.419227 | 2.020277 | 0.433750 |
| 28 | 69 | Neighborhood label-barycenter spatial-KLA AA | 1.447495 | 0.229994 | 0.455000 |
| 28 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.913230 | 0.989658 | 0.455000 |
| 29 | 70 | Tuned spatial-KLA AA | 2.238931 | 1.117869 | 0.418750 |
| 29 | 70 | Neighborhood label-barycenter spatial-KLA AA | 1.347340 | 0.201114 | 0.395000 |
| 29 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.749479 | 0.846642 | 0.395000 |
| 30 | 71 | Tuned spatial-KLA AA | 2.308731 | 1.343479 | 0.383750 |
| 30 | 71 | Neighborhood label-barycenter spatial-KLA AA | 1.416599 | 0.156061 | 0.410000 |
| 30 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.814972 | 0.607837 | 0.410000 |
| 31 | 72 | Tuned spatial-KLA AA | 2.311829 | 1.413814 | 0.471250 |
| 31 | 72 | Neighborhood label-barycenter spatial-KLA AA | 1.393305 | 0.125965 | 0.480000 |
| 31 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.758493 | 0.544547 | 0.480000 |
| 32 | 73 | Tuned spatial-KLA AA | 2.475632 | 2.269017 | 0.336250 |
| 32 | 73 | Neighborhood label-barycenter spatial-KLA AA | 1.046174 | 0.385753 | 0.270000 |
| 32 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.554681 | 1.400967 | 0.270000 |
| 33 | 74 | Tuned spatial-KLA AA | 2.547657 | 2.840269 | 0.327500 |
| 33 | 74 | Neighborhood label-barycenter spatial-KLA AA | 1.002788 | 0.513358 | 0.265000 |
| 33 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.536201 | 1.675153 | 0.265000 |
| 34 | 75 | Tuned spatial-KLA AA | 2.256376 | 1.016831 | 0.478750 |
| 34 | 75 | Neighborhood label-barycenter spatial-KLA AA | 1.364056 | 0.147837 | 0.455000 |
| 34 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.774935 | 0.656710 | 0.455000 |
| 35 | 76 | Tuned spatial-KLA AA | 2.467440 | 2.047182 | 0.385000 |
| 35 | 76 | Neighborhood label-barycenter spatial-KLA AA | 1.264847 | 0.367015 | 0.340000 |
| 35 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.612854 | 0.887821 | 0.340000 |
| 36 | 77 | Tuned spatial-KLA AA | 2.397748 | 1.598994 | 0.351250 |
| 36 | 77 | Neighborhood label-barycenter spatial-KLA AA | 1.411107 | 0.134216 | 0.360000 |
| 36 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.815735 | 0.585420 | 0.360000 |
| 37 | 78 | Tuned spatial-KLA AA | 2.229736 | 1.019133 | 0.446250 |
| 37 | 78 | Neighborhood label-barycenter spatial-KLA AA | 1.374936 | 0.146343 | 0.430000 |
| 37 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.795705 | 0.616884 | 0.430000 |
| 38 | 79 | Tuned spatial-KLA AA | 2.395516 | 2.238556 | 0.390000 |
| 38 | 79 | Neighborhood label-barycenter spatial-KLA AA | 1.187172 | 0.400862 | 0.350000 |
| 38 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.631449 | 1.338594 | 0.350000 |
| 39 | 80 | Tuned spatial-KLA AA | 2.131922 | 0.908001 | 0.433750 |
| 39 | 80 | Neighborhood label-barycenter spatial-KLA AA | 1.364338 | 0.120133 | 0.415000 |
| 39 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.733784 | 0.531804 | 0.415000 |
| 40 | 81 | Tuned spatial-KLA AA | 2.287057 | 1.037186 | 0.443750 |
| 40 | 81 | Neighborhood label-barycenter spatial-KLA AA | 1.433630 | 0.141280 | 0.425000 |
| 40 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.877691 | 0.613721 | 0.425000 |
| 41 | 82 | Tuned spatial-KLA AA | 2.341050 | 1.787242 | 0.323750 |
| 41 | 82 | Neighborhood label-barycenter spatial-KLA AA | 1.122227 | 0.224747 | 0.240000 |
| 41 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.452893 | 0.722711 | 0.240000 |
| 42 | 83 | Tuned spatial-KLA AA | 2.265728 | 1.298667 | 0.430000 |
| 42 | 83 | Neighborhood label-barycenter spatial-KLA AA | 1.355090 | 0.184193 | 0.400000 |
| 42 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.790879 | 0.806586 | 0.400000 |
| 43 | 84 | Tuned spatial-KLA AA | 2.213817 | 1.331769 | 0.416250 |
| 43 | 84 | Neighborhood label-barycenter spatial-KLA AA | 1.372446 | 0.215516 | 0.410000 |
| 43 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.769723 | 0.903447 | 0.410000 |
| 44 | 85 | Tuned spatial-KLA AA | 2.349916 | 1.079408 | 0.483750 |
| 44 | 85 | Neighborhood label-barycenter spatial-KLA AA | 1.414309 | 0.147135 | 0.450000 |
| 44 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.784551 | 0.551802 | 0.450000 |
| 45 | 86 | Tuned spatial-KLA AA | 2.493018 | 2.175171 | 0.347500 |
| 45 | 86 | Neighborhood label-barycenter spatial-KLA AA | 1.172007 | 0.313727 | 0.285000 |
| 45 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.555829 | 1.001896 | 0.285000 |
| 46 | 87 | Tuned spatial-KLA AA | 2.259654 | 1.078765 | 0.428750 |
| 46 | 87 | Neighborhood label-barycenter spatial-KLA AA | 1.367022 | 0.155996 | 0.420000 |
| 46 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.825718 | 0.672825 | 0.420000 |
| 47 | 88 | Tuned spatial-KLA AA | 2.492957 | 1.602822 | 0.427500 |
| 47 | 88 | Neighborhood label-barycenter spatial-KLA AA | 1.484299 | 0.217024 | 0.430000 |
| 47 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.825060 | 0.755042 | 0.430000 |
| 48 | 89 | Tuned spatial-KLA AA | 2.233604 | 1.026190 | 0.475000 |
| 48 | 89 | Neighborhood label-barycenter spatial-KLA AA | 1.344443 | 0.127221 | 0.450000 |
| 48 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.694156 | 0.528175 | 0.450000 |
| 49 | 90 | Tuned spatial-KLA AA | 2.408360 | 2.117424 | 0.356250 |
| 49 | 90 | Neighborhood label-barycenter spatial-KLA AA | 1.218834 | 0.421150 | 0.280000 |
| 49 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.574050 | 0.797905 | 0.280000 |
| 50 | 91 | Tuned spatial-KLA AA | 2.283632 | 1.335358 | 0.450000 |
| 50 | 91 | Neighborhood label-barycenter spatial-KLA AA | 1.381133 | 0.217362 | 0.430000 |
| 50 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.804760 | 0.902048 | 0.430000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.319206 | 1.511280 | 0.410025 |
| Neighborhood label-barycenter spatial-KLA AA | 1.316143 | 0.219238 | 0.388500 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.719987 | 0.803987 | 0.388500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.319206 +/- 0.107241 | [2.289480, 2.348931] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.316143 +/- 0.177138 | [1.267043, 1.365244] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.719987 +/- 0.165728 | [1.674049, 1.765924] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 1.511280 +/- 0.534973 | [1.362993, 1.659567] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.219238 +/- 0.102494 | [0.190828, 0.247648] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.803987 +/- 0.262963 | [0.731097, 0.876876] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.410025 +/- 0.057384 | [0.394119, 0.425931] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.388500 +/- 0.078690 | [0.366688, 0.410312] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.388500 +/- 0.078690 | [0.366688, 0.410312] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.003062 +/- 0.240668 | [0.936353, 1.069772] | 43.25% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.599219 +/- 0.227802 | [0.536076, 0.662363] | 25.84% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.292042 +/- 0.445471 | [1.168564, 1.415520] | 85.49% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.707293 +/- 0.347444 | [0.610987, 0.803600] | 46.80% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.021525 +/- 0.030274 | [0.013133, 0.029917] | 5.25% | 39/50 | 3.846e-05 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.021525 +/- 0.030274 | [0.013133, 0.029917] | 5.25% | 39/50 | 3.846e-05 |

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
| 6 | 47 | Tuned spatial-KLA AA | 2.827477 | 4.239676 | 0.595000 |
| 6 | 47 | Neighborhood label-barycenter spatial-KLA AA | 2.622687 | 3.944054 | 0.605000 |
| 6 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.785717 | 4.164914 | 0.605000 |
| 7 | 48 | Tuned spatial-KLA AA | 2.916964 | 4.358147 | 0.602500 |
| 7 | 48 | Neighborhood label-barycenter spatial-KLA AA | 2.697637 | 4.035574 | 0.590000 |
| 7 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.834146 | 4.228686 | 0.590000 |
| 8 | 49 | Tuned spatial-KLA AA | 2.895157 | 4.363707 | 0.597500 |
| 8 | 49 | Neighborhood label-barycenter spatial-KLA AA | 2.711469 | 4.396453 | 0.580000 |
| 8 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.828530 | 4.460687 | 0.580000 |
| 9 | 50 | Tuned spatial-KLA AA | 2.956623 | 4.734505 | 0.592500 |
| 9 | 50 | Neighborhood label-barycenter spatial-KLA AA | 2.769983 | 4.534431 | 0.585000 |
| 9 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.895735 | 4.734600 | 0.585000 |
| 10 | 51 | Tuned spatial-KLA AA | 2.754652 | 4.523877 | 0.350000 |
| 10 | 51 | Neighborhood label-barycenter spatial-KLA AA | 2.473233 | 4.185192 | 0.270000 |
| 10 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.586452 | 4.262505 | 0.270000 |
| 11 | 52 | Tuned spatial-KLA AA | 2.802255 | 4.425309 | 0.463750 |
| 11 | 52 | Neighborhood label-barycenter spatial-KLA AA | 2.600693 | 3.773709 | 0.430000 |
| 11 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.635780 | 3.965591 | 0.430000 |
| 12 | 53 | Tuned spatial-KLA AA | 2.778579 | 4.659076 | 0.623750 |
| 12 | 53 | Neighborhood label-barycenter spatial-KLA AA | 2.622510 | 4.398633 | 0.645000 |
| 12 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.720179 | 4.630950 | 0.645000 |
| 13 | 54 | Tuned spatial-KLA AA | 2.702330 | 4.220374 | 0.567500 |
| 13 | 54 | Neighborhood label-barycenter spatial-KLA AA | 2.492086 | 3.935015 | 0.570000 |
| 13 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.641939 | 4.052632 | 0.570000 |
| 14 | 55 | Tuned spatial-KLA AA | 2.806566 | 4.626030 | 0.522500 |
| 14 | 55 | Neighborhood label-barycenter spatial-KLA AA | 2.602689 | 4.405611 | 0.495000 |
| 14 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.678423 | 4.495579 | 0.495000 |
| 15 | 56 | Tuned spatial-KLA AA | 2.834982 | 4.697158 | 0.582500 |
| 15 | 56 | Neighborhood label-barycenter spatial-KLA AA | 2.648557 | 4.413357 | 0.600000 |
| 15 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.807222 | 4.706223 | 0.600000 |
| 16 | 57 | Tuned spatial-KLA AA | 2.788765 | 4.554772 | 0.587500 |
| 16 | 57 | Neighborhood label-barycenter spatial-KLA AA | 2.579751 | 4.252171 | 0.595000 |
| 16 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.708540 | 4.368800 | 0.595000 |
| 17 | 58 | Tuned spatial-KLA AA | 2.715060 | 4.214457 | 0.528750 |
| 17 | 58 | Neighborhood label-barycenter spatial-KLA AA | 2.529348 | 3.728028 | 0.485000 |
| 17 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.613773 | 3.734774 | 0.485000 |
| 18 | 59 | Tuned spatial-KLA AA | 2.628456 | 4.126238 | 0.557500 |
| 18 | 59 | Neighborhood label-barycenter spatial-KLA AA | 2.418456 | 3.855965 | 0.550000 |
| 18 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.580587 | 4.067127 | 0.550000 |
| 19 | 60 | Tuned spatial-KLA AA | 2.801993 | 4.641699 | 0.527500 |
| 19 | 60 | Neighborhood label-barycenter spatial-KLA AA | 2.652054 | 4.280916 | 0.570000 |
| 19 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.790497 | 4.518919 | 0.570000 |
| 20 | 61 | Tuned spatial-KLA AA | 2.803698 | 4.829073 | 0.583750 |
| 20 | 61 | Neighborhood label-barycenter spatial-KLA AA | 2.619777 | 4.634122 | 0.585000 |
| 20 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.772177 | 4.791580 | 0.585000 |
| 21 | 62 | Tuned spatial-KLA AA | 2.949650 | 4.384305 | 0.606250 |
| 21 | 62 | Neighborhood label-barycenter spatial-KLA AA | 2.763989 | 4.177606 | 0.595000 |
| 21 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.895281 | 4.554837 | 0.595000 |
| 22 | 63 | Tuned spatial-KLA AA | 2.687048 | 3.978395 | 0.557500 |
| 22 | 63 | Neighborhood label-barycenter spatial-KLA AA | 2.480958 | 3.720799 | 0.550000 |
| 22 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.616869 | 3.880426 | 0.550000 |
| 23 | 64 | Tuned spatial-KLA AA | 2.761391 | 4.100549 | 0.591250 |
| 23 | 64 | Neighborhood label-barycenter spatial-KLA AA | 2.568083 | 4.075081 | 0.570000 |
| 23 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.662156 | 4.309700 | 0.570000 |
| 24 | 65 | Tuned spatial-KLA AA | 2.892351 | 4.730777 | 0.590000 |
| 24 | 65 | Neighborhood label-barycenter spatial-KLA AA | 2.677157 | 4.462950 | 0.575000 |
| 24 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.790428 | 4.648551 | 0.575000 |
| 25 | 66 | Tuned spatial-KLA AA | 2.637772 | 4.136321 | 0.393750 |
| 25 | 66 | Neighborhood label-barycenter spatial-KLA AA | 2.319594 | 3.525649 | 0.395000 |
| 25 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.463553 | 3.747754 | 0.395000 |
| 26 | 67 | Tuned spatial-KLA AA | 2.698584 | 3.831144 | 0.565000 |
| 26 | 67 | Neighborhood label-barycenter spatial-KLA AA | 2.511844 | 3.609849 | 0.560000 |
| 26 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.645240 | 3.869959 | 0.560000 |
| 27 | 68 | Tuned spatial-KLA AA | 2.913716 | 4.034236 | 0.603750 |
| 27 | 68 | Neighborhood label-barycenter spatial-KLA AA | 2.743597 | 3.876574 | 0.620000 |
| 27 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.870530 | 4.209937 | 0.620000 |
| 28 | 69 | Tuned spatial-KLA AA | 2.758597 | 4.137608 | 0.558750 |
| 28 | 69 | Neighborhood label-barycenter spatial-KLA AA | 2.510754 | 3.711084 | 0.585000 |
| 28 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.692784 | 4.023439 | 0.585000 |
| 29 | 70 | Tuned spatial-KLA AA | 2.789916 | 4.383660 | 0.603750 |
| 29 | 70 | Neighborhood label-barycenter spatial-KLA AA | 2.598634 | 4.458301 | 0.605000 |
| 29 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.726053 | 4.472647 | 0.605000 |
| 30 | 71 | Tuned spatial-KLA AA | 2.908814 | 4.323581 | 0.576250 |
| 30 | 71 | Neighborhood label-barycenter spatial-KLA AA | 2.748975 | 4.012532 | 0.600000 |
| 30 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.856720 | 4.134932 | 0.600000 |
| 31 | 72 | Tuned spatial-KLA AA | 2.778813 | 4.482424 | 0.546250 |
| 31 | 72 | Neighborhood label-barycenter spatial-KLA AA | 2.584089 | 4.328128 | 0.560000 |
| 31 | 72 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.713828 | 4.468979 | 0.560000 |
| 32 | 73 | Tuned spatial-KLA AA | 2.788470 | 4.000065 | 0.546250 |
| 32 | 73 | Neighborhood label-barycenter spatial-KLA AA | 2.601283 | 3.751331 | 0.510000 |
| 32 | 73 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.698610 | 4.177589 | 0.510000 |
| 33 | 74 | Tuned spatial-KLA AA | 2.688515 | 4.220131 | 0.402500 |
| 33 | 74 | Neighborhood label-barycenter spatial-KLA AA | 2.416820 | 3.596270 | 0.365000 |
| 33 | 74 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.533337 | 3.994366 | 0.365000 |
| 34 | 75 | Tuned spatial-KLA AA | 2.757137 | 4.239728 | 0.581250 |
| 34 | 75 | Neighborhood label-barycenter spatial-KLA AA | 2.548497 | 3.912904 | 0.575000 |
| 34 | 75 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.688810 | 4.123239 | 0.575000 |
| 35 | 76 | Tuned spatial-KLA AA | 2.822687 | 4.513305 | 0.527500 |
| 35 | 76 | Neighborhood label-barycenter spatial-KLA AA | 2.695544 | 4.552314 | 0.500000 |
| 35 | 76 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.706083 | 4.377281 | 0.500000 |
| 36 | 77 | Tuned spatial-KLA AA | 2.953510 | 4.821631 | 0.623750 |
| 36 | 77 | Neighborhood label-barycenter spatial-KLA AA | 2.845398 | 4.411044 | 0.680000 |
| 36 | 77 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.941559 | 4.518745 | 0.680000 |
| 37 | 78 | Tuned spatial-KLA AA | 2.842341 | 4.159835 | 0.601250 |
| 37 | 78 | Neighborhood label-barycenter spatial-KLA AA | 2.618137 | 4.128170 | 0.590000 |
| 37 | 78 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.769664 | 4.316435 | 0.590000 |
| 38 | 79 | Tuned spatial-KLA AA | 2.734541 | 4.304046 | 0.482500 |
| 38 | 79 | Neighborhood label-barycenter spatial-KLA AA | 2.607759 | 4.033129 | 0.460000 |
| 38 | 79 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.659267 | 4.203013 | 0.460000 |
| 39 | 80 | Tuned spatial-KLA AA | 2.742820 | 3.436958 | 0.603750 |
| 39 | 80 | Neighborhood label-barycenter spatial-KLA AA | 2.573591 | 3.437165 | 0.605000 |
| 39 | 80 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.690875 | 3.583975 | 0.605000 |
| 40 | 81 | Tuned spatial-KLA AA | 2.784811 | 4.731847 | 0.568750 |
| 40 | 81 | Neighborhood label-barycenter spatial-KLA AA | 2.586261 | 4.462622 | 0.575000 |
| 40 | 81 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.764938 | 4.687813 | 0.575000 |
| 41 | 82 | Tuned spatial-KLA AA | 2.921803 | 4.264085 | 0.683750 |
| 41 | 82 | Neighborhood label-barycenter spatial-KLA AA | 2.890581 | 3.786746 | 0.780000 |
| 41 | 82 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.989655 | 4.114122 | 0.780000 |
| 42 | 83 | Tuned spatial-KLA AA | 2.901160 | 3.634726 | 0.632500 |
| 42 | 83 | Neighborhood label-barycenter spatial-KLA AA | 2.707087 | 3.361060 | 0.620000 |
| 42 | 83 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.842225 | 3.649389 | 0.620000 |
| 43 | 84 | Tuned spatial-KLA AA | 2.825264 | 3.396891 | 0.603750 |
| 43 | 84 | Neighborhood label-barycenter spatial-KLA AA | 2.616406 | 3.049542 | 0.600000 |
| 43 | 84 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.735066 | 3.310079 | 0.600000 |
| 44 | 85 | Tuned spatial-KLA AA | 2.869805 | 3.756771 | 0.598750 |
| 44 | 85 | Neighborhood label-barycenter spatial-KLA AA | 2.697744 | 3.427244 | 0.590000 |
| 44 | 85 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.792490 | 3.573008 | 0.590000 |
| 45 | 86 | Tuned spatial-KLA AA | 2.790861 | 4.041389 | 0.527500 |
| 45 | 86 | Neighborhood label-barycenter spatial-KLA AA | 2.570758 | 3.626602 | 0.465000 |
| 45 | 86 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.625987 | 3.688858 | 0.465000 |
| 46 | 87 | Tuned spatial-KLA AA | 2.899774 | 4.216690 | 0.586250 |
| 46 | 87 | Neighborhood label-barycenter spatial-KLA AA | 2.686303 | 3.915228 | 0.590000 |
| 46 | 87 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.846666 | 4.117668 | 0.590000 |
| 47 | 88 | Tuned spatial-KLA AA | 2.889150 | 4.083784 | 0.605000 |
| 47 | 88 | Neighborhood label-barycenter spatial-KLA AA | 2.708761 | 4.198144 | 0.620000 |
| 47 | 88 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.814386 | 4.310426 | 0.620000 |
| 48 | 89 | Tuned spatial-KLA AA | 2.841857 | 3.844416 | 0.587500 |
| 48 | 89 | Neighborhood label-barycenter spatial-KLA AA | 2.625132 | 3.662922 | 0.570000 |
| 48 | 89 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.748711 | 3.815265 | 0.570000 |
| 49 | 90 | Tuned spatial-KLA AA | 2.788831 | 3.893797 | 0.546250 |
| 49 | 90 | Neighborhood label-barycenter spatial-KLA AA | 2.591739 | 3.643363 | 0.480000 |
| 49 | 90 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.631959 | 3.574218 | 0.480000 |
| 50 | 91 | Tuned spatial-KLA AA | 2.741002 | 4.334091 | 0.565000 |
| 50 | 91 | Neighborhood label-barycenter spatial-KLA AA | 2.533846 | 4.122504 | 0.560000 |
| 50 | 91 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.685503 | 4.344812 | 0.560000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.811245 | 4.244652 | 0.563625 |
| Neighborhood label-barycenter spatial-KLA AA | 2.617619 | 3.968932 | 0.558100 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.737646 | 4.154180 | 0.558100 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.811245 +/- 0.092216 | [2.785684, 2.836806] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.617619 +/- 0.121978 | [2.583809, 2.651430] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.737646 +/- 0.121392 | [2.703998, 2.771294] | 50 |
| Tuned spatial-KLA AA | RMSE | 4.244652 +/- 0.339065 | [4.150668, 4.338636] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.968932 +/- 0.385154 | [3.862173, 4.075691] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.154180 +/- 0.368343 | [4.052080, 4.256279] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.563625 +/- 0.064515 | [0.545742, 0.581508] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.558100 +/- 0.086384 | [0.534156, 0.582044] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.558100 +/- 0.086384 | [0.534156, 0.582044] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.193626 +/- 0.045058 | [0.181137, 0.206116] | 6.89% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.073600 +/- 0.048167 | [0.060248, 0.086951] | 2.62% | 49/50 | 9.059e-14 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.275720 +/- 0.172740 | [0.227839, 0.323601] | 6.50% | 45/50 | 4.21e-09 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.090472 +/- 0.169776 | [0.043413, 0.137532] | 2.13% | 36/50 | 0.002602 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.005525 +/- 0.031274 | [-0.003144, 0.014194] | 0.98% | 28/50 | 0.3916 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.005525 +/- 0.031274 | [-0.003144, 0.014194] | 0.98% | 28/50 | 0.3916 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 41.003932 +/- 6.687450 | 1.708497 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 62.406831 +/- 9.130601 | 2.600285 | 1.525x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 58.146258 +/- 6.796848 | 2.422761 | 1.429x | 50 |
