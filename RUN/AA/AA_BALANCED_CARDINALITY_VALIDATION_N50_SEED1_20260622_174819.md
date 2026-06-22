# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 21:45:51

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

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
| 1 | 2 | Tuned spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| 1 | 2 | Neighborhood label-barycenter spatial-KLA AA | 0.312781 | 0.224735 | 0.015000 |
| 1 | 2 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.112777 | 0.944124 | 0.015000 |
| 2 | 3 | Tuned spatial-KLA AA | 1.667430 | 1.435498 | 0.035000 |
| 2 | 3 | Neighborhood label-barycenter spatial-KLA AA | 0.336056 | 0.213231 | 0.030000 |
| 2 | 3 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.092095 | 0.901099 | 0.030000 |
| 3 | 4 | Tuned spatial-KLA AA | 1.665450 | 1.448322 | 0.031250 |
| 3 | 4 | Neighborhood label-barycenter spatial-KLA AA | 0.303042 | 0.211637 | 0.015000 |
| 3 | 4 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.065871 | 0.905686 | 0.015000 |
| 4 | 5 | Tuned spatial-KLA AA | 1.603341 | 1.300640 | 0.042500 |
| 4 | 5 | Neighborhood label-barycenter spatial-KLA AA | 0.314093 | 0.186753 | 0.025000 |
| 4 | 5 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.984527 | 0.780380 | 0.025000 |
| 5 | 6 | Tuned spatial-KLA AA | 1.610089 | 1.457096 | 0.027500 |
| 5 | 6 | Neighborhood label-barycenter spatial-KLA AA | 0.305617 | 0.211747 | 0.020000 |
| 5 | 6 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.988366 | 0.821915 | 0.020000 |
| 6 | 7 | Tuned spatial-KLA AA | 1.598491 | 1.528223 | 0.035000 |
| 6 | 7 | Neighborhood label-barycenter spatial-KLA AA | 0.270643 | 0.220300 | 0.010000 |
| 6 | 7 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.935545 | 0.974308 | 0.010000 |
| 7 | 8 | Tuned spatial-KLA AA | 1.747386 | 1.519075 | 0.038750 |
| 7 | 8 | Neighborhood label-barycenter spatial-KLA AA | 0.361354 | 0.225017 | 0.030000 |
| 7 | 8 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.159180 | 0.989045 | 0.030000 |
| 8 | 9 | Tuned spatial-KLA AA | 1.623301 | 1.359868 | 0.030000 |
| 8 | 9 | Neighborhood label-barycenter spatial-KLA AA | 0.287008 | 0.198063 | 0.020000 |
| 8 | 9 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.952946 | 0.819764 | 0.020000 |
| 9 | 10 | Tuned spatial-KLA AA | 1.664538 | 1.534739 | 0.051250 |
| 9 | 10 | Neighborhood label-barycenter spatial-KLA AA | 0.333803 | 0.280065 | 0.025000 |
| 9 | 10 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.013737 | 1.075693 | 0.025000 |
| 10 | 11 | Tuned spatial-KLA AA | 1.768428 | 1.504540 | 0.040000 |
| 10 | 11 | Neighborhood label-barycenter spatial-KLA AA | 0.294339 | 0.209217 | 0.015000 |
| 10 | 11 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.013631 | 0.847745 | 0.015000 |
| 11 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 11 | 12 | Neighborhood label-barycenter spatial-KLA AA | 0.300460 | 0.227285 | 0.015000 |
| 11 | 12 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.063672 | 0.949262 | 0.015000 |
| 12 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 12 | 13 | Neighborhood label-barycenter spatial-KLA AA | 0.351073 | 0.224834 | 0.030000 |
| 12 | 13 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.103466 | 0.914666 | 0.030000 |
| 13 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 13 | 14 | Neighborhood label-barycenter spatial-KLA AA | 0.282517 | 0.244711 | 0.005000 |
| 13 | 14 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.998267 | 0.951321 | 0.005000 |
| 14 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 14 | 15 | Neighborhood label-barycenter spatial-KLA AA | 0.318286 | 0.206477 | 0.025000 |
| 14 | 15 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.026745 | 0.838960 | 0.025000 |
| 15 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 15 | 16 | Neighborhood label-barycenter spatial-KLA AA | 0.274923 | 0.230215 | 0.010000 |
| 15 | 16 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.977335 | 0.862316 | 0.010000 |
| 16 | 17 | Tuned spatial-KLA AA | 1.685404 | 1.388587 | 0.046250 |
| 16 | 17 | Neighborhood label-barycenter spatial-KLA AA | 0.292001 | 0.187499 | 0.025000 |
| 16 | 17 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.969933 | 0.795157 | 0.025000 |
| 17 | 18 | Tuned spatial-KLA AA | 1.654544 | 1.461745 | 0.020000 |
| 17 | 18 | Neighborhood label-barycenter spatial-KLA AA | 0.286964 | 0.213391 | 0.015000 |
| 17 | 18 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.023742 | 0.893535 | 0.015000 |
| 18 | 19 | Tuned spatial-KLA AA | 1.676684 | 1.472405 | 0.030000 |
| 18 | 19 | Neighborhood label-barycenter spatial-KLA AA | 0.302170 | 0.200867 | 0.025000 |
| 18 | 19 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.949360 | 0.799718 | 0.025000 |
| 19 | 20 | Tuned spatial-KLA AA | 1.709979 | 1.671988 | 0.037500 |
| 19 | 20 | Neighborhood label-barycenter spatial-KLA AA | 0.342610 | 0.254243 | 0.025000 |
| 19 | 20 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.092770 | 1.160585 | 0.025000 |
| 20 | 21 | Tuned spatial-KLA AA | 1.734930 | 1.489287 | 0.042500 |
| 20 | 21 | Neighborhood label-barycenter spatial-KLA AA | 0.315184 | 0.216701 | 0.020000 |
| 20 | 21 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.073299 | 0.905585 | 0.020000 |
| 21 | 22 | Tuned spatial-KLA AA | 1.636799 | 1.428640 | 0.030000 |
| 21 | 22 | Neighborhood label-barycenter spatial-KLA AA | 0.227032 | 0.193920 | 0.005000 |
| 21 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.902487 | 0.799631 | 0.005000 |
| 22 | 23 | Tuned spatial-KLA AA | 1.780009 | 1.492509 | 0.043750 |
| 22 | 23 | Neighborhood label-barycenter spatial-KLA AA | 0.286690 | 0.218064 | 0.010000 |
| 22 | 23 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.017645 | 0.862388 | 0.010000 |
| 23 | 24 | Tuned spatial-KLA AA | 1.702572 | 1.505093 | 0.026250 |
| 23 | 24 | Neighborhood label-barycenter spatial-KLA AA | 0.285476 | 0.222277 | 0.015000 |
| 23 | 24 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.027825 | 0.904061 | 0.015000 |
| 24 | 25 | Tuned spatial-KLA AA | 1.749069 | 1.504870 | 0.041250 |
| 24 | 25 | Neighborhood label-barycenter spatial-KLA AA | 0.339566 | 0.227367 | 0.030000 |
| 24 | 25 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.107811 | 0.945851 | 0.030000 |
| 25 | 26 | Tuned spatial-KLA AA | 1.776746 | 1.504229 | 0.066250 |
| 25 | 26 | Neighborhood label-barycenter spatial-KLA AA | 0.414702 | 0.223985 | 0.055000 |
| 25 | 26 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.127039 | 0.909551 | 0.055000 |
| 26 | 27 | Tuned spatial-KLA AA | 1.690300 | 1.398991 | 0.041250 |
| 26 | 27 | Neighborhood label-barycenter spatial-KLA AA | 0.342507 | 0.200608 | 0.030000 |
| 26 | 27 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.045952 | 0.838322 | 0.030000 |
| 27 | 28 | Tuned spatial-KLA AA | 1.785713 | 1.624857 | 0.031250 |
| 27 | 28 | Neighborhood label-barycenter spatial-KLA AA | 0.316006 | 0.250131 | 0.010000 |
| 27 | 28 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.099147 | 1.033175 | 0.010000 |
| 28 | 29 | Tuned spatial-KLA AA | 1.726645 | 1.416342 | 0.060000 |
| 28 | 29 | Neighborhood label-barycenter spatial-KLA AA | 0.361264 | 0.193530 | 0.040000 |
| 28 | 29 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.999678 | 0.781193 | 0.040000 |
| 29 | 30 | Tuned spatial-KLA AA | 1.647364 | 1.406921 | 0.033750 |
| 29 | 30 | Neighborhood label-barycenter spatial-KLA AA | 0.277150 | 0.190888 | 0.020000 |
| 29 | 30 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.969186 | 0.824703 | 0.020000 |
| 30 | 31 | Tuned spatial-KLA AA | 1.688300 | 1.424875 | 0.048750 |
| 30 | 31 | Neighborhood label-barycenter spatial-KLA AA | 0.356833 | 0.217255 | 0.040000 |
| 30 | 31 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.094361 | 0.928129 | 0.040000 |
| 31 | 32 | Tuned spatial-KLA AA | 1.743321 | 1.528194 | 0.021250 |
| 31 | 32 | Neighborhood label-barycenter spatial-KLA AA | 0.295142 | 0.230247 | 0.010000 |
| 31 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.093726 | 0.934688 | 0.010000 |
| 32 | 33 | Tuned spatial-KLA AA | 1.613191 | 1.396453 | 0.023750 |
| 32 | 33 | Neighborhood label-barycenter spatial-KLA AA | 0.283944 | 0.201861 | 0.015000 |
| 32 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.003319 | 0.851229 | 0.015000 |
| 33 | 34 | Tuned spatial-KLA AA | 1.695320 | 1.748860 | 0.025000 |
| 33 | 34 | Neighborhood label-barycenter spatial-KLA AA | 0.327244 | 0.288415 | 0.010000 |
| 33 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.065679 | 1.133232 | 0.010000 |
| 34 | 35 | Tuned spatial-KLA AA | 1.610147 | 1.512261 | 0.025000 |
| 34 | 35 | Neighborhood label-barycenter spatial-KLA AA | 0.271866 | 0.208537 | 0.010000 |
| 34 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.978653 | 0.835367 | 0.010000 |
| 35 | 36 | Tuned spatial-KLA AA | 1.616460 | 1.423190 | 0.021250 |
| 35 | 36 | Neighborhood label-barycenter spatial-KLA AA | 0.292115 | 0.213652 | 0.015000 |
| 35 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.026707 | 0.896379 | 0.015000 |
| 36 | 37 | Tuned spatial-KLA AA | 1.780506 | 1.528165 | 0.031250 |
| 36 | 37 | Neighborhood label-barycenter spatial-KLA AA | 0.305279 | 0.221607 | 0.020000 |
| 36 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.080172 | 0.931478 | 0.020000 |
| 37 | 38 | Tuned spatial-KLA AA | 1.660233 | 1.444823 | 0.037500 |
| 37 | 38 | Neighborhood label-barycenter spatial-KLA AA | 0.319515 | 0.213886 | 0.025000 |
| 37 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.041895 | 0.888306 | 0.025000 |
| 38 | 39 | Tuned spatial-KLA AA | 1.611358 | 1.515457 | 0.022500 |
| 38 | 39 | Neighborhood label-barycenter spatial-KLA AA | 0.292327 | 0.228453 | 0.015000 |
| 38 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.015322 | 0.891146 | 0.015000 |
| 39 | 40 | Tuned spatial-KLA AA | 1.667447 | 1.373777 | 0.036250 |
| 39 | 40 | Neighborhood label-barycenter spatial-KLA AA | 0.329642 | 0.200610 | 0.025000 |
| 39 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.069647 | 0.862358 | 0.025000 |
| 40 | 41 | Tuned spatial-KLA AA | 1.625285 | 1.397315 | 0.021250 |
| 40 | 41 | Neighborhood label-barycenter spatial-KLA AA | 0.293846 | 0.211119 | 0.015000 |
| 40 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.054516 | 0.893096 | 0.015000 |
| 41 | 42 | Tuned spatial-KLA AA | 1.598724 | 1.434675 | 0.017500 |
| 41 | 42 | Neighborhood label-barycenter spatial-KLA AA | 0.261609 | 0.211758 | 0.010000 |
| 41 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.999599 | 0.879733 | 0.010000 |
| 42 | 43 | Tuned spatial-KLA AA | 1.622933 | 1.336654 | 0.036250 |
| 42 | 43 | Neighborhood label-barycenter spatial-KLA AA | 0.275573 | 0.185154 | 0.020000 |
| 42 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.908214 | 0.740328 | 0.020000 |
| 43 | 44 | Tuned spatial-KLA AA | 1.627533 | 1.397986 | 0.048750 |
| 43 | 44 | Neighborhood label-barycenter spatial-KLA AA | 0.324753 | 0.204578 | 0.030000 |
| 43 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.987918 | 0.819876 | 0.030000 |
| 44 | 45 | Tuned spatial-KLA AA | 1.726464 | 1.468741 | 0.040000 |
| 44 | 45 | Neighborhood label-barycenter spatial-KLA AA | 0.283924 | 0.218091 | 0.015000 |
| 44 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.015649 | 0.897175 | 0.015000 |
| 45 | 46 | Tuned spatial-KLA AA | 1.650691 | 1.398505 | 0.046250 |
| 45 | 46 | Neighborhood label-barycenter spatial-KLA AA | 0.339945 | 0.197026 | 0.040000 |
| 45 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.010610 | 0.832949 | 0.040000 |
| 46 | 47 | Tuned spatial-KLA AA | 1.670539 | 1.445587 | 0.031250 |
| 46 | 47 | Neighborhood label-barycenter spatial-KLA AA | 0.295779 | 0.216475 | 0.020000 |
| 46 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.023508 | 0.889120 | 0.020000 |
| 47 | 48 | Tuned spatial-KLA AA | 1.729929 | 1.502969 | 0.032500 |
| 47 | 48 | Neighborhood label-barycenter spatial-KLA AA | 0.318924 | 0.225833 | 0.020000 |
| 47 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.055592 | 0.916342 | 0.020000 |
| 48 | 49 | Tuned spatial-KLA AA | 1.643617 | 1.395513 | 0.031250 |
| 48 | 49 | Neighborhood label-barycenter spatial-KLA AA | 0.304284 | 0.206848 | 0.020000 |
| 48 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.008484 | 0.822953 | 0.020000 |
| 49 | 50 | Tuned spatial-KLA AA | 1.781736 | 1.523159 | 0.070000 |
| 49 | 50 | Neighborhood label-barycenter spatial-KLA AA | 0.407054 | 0.210125 | 0.055000 |
| 49 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.088530 | 0.861900 | 0.055000 |
| 50 | 51 | Tuned spatial-KLA AA | 1.664178 | 1.467082 | 0.028750 |
| 50 | 51 | Neighborhood label-barycenter spatial-KLA AA | 0.275983 | 0.199318 | 0.015000 |
| 50 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.986961 | 0.843718 | 0.015000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 |
| Neighborhood label-barycenter spatial-KLA AA | 0.309818 | 0.216372 | 0.021200 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.030062 | 0.891585 | 0.021200 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.682607 +/- 0.058035 | [1.666520, 1.698693] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.309818 +/- 0.034773 | [0.300179, 0.319457] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.030062 +/- 0.056808 | [1.014315, 1.045808] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 1.472837 +/- 0.082422 | [1.449991, 1.495683] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.216372 +/- 0.020581 | [0.210667, 0.222077] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.891585 +/- 0.083325 | [0.868488, 0.914681] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.035350 +/- 0.011802 | [0.032079, 0.038621] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.021200 +/- 0.010999 | [0.018151, 0.024249] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.021200 +/- 0.010999 | [0.018151, 0.024249] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.372789 +/- 0.049094 | [1.359181, 1.386397] | 81.59% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.652545 +/- 0.049022 | [0.638957, 0.666133] | 38.78% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.256465 +/- 0.066080 | [1.238149, 1.274781] | 85.31% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.581252 +/- 0.053491 | [0.566425, 0.596079] | 39.46% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.014150 +/- 0.006750 | [0.012279, 0.016021] | 40.03% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.014150 +/- 0.006750 | [0.012279, 0.016021] | 40.03% | 50/50 | 1.776e-15 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.029641 | 3.682880 | 0.088000 |
| Neighborhood label-barycenter spatial-KLA AA | 1.681483 | 3.449035 | 0.077200 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.911286 | 3.662955 | 0.077200 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.029641 +/- 0.061460 | [2.012605, 2.046677] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 1.681483 +/- 0.064539 | [1.663594, 1.699373] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 1.911286 +/- 0.064429 | [1.893427, 1.929145] | 50 |
| Tuned spatial-KLA AA | RMSE | 3.682880 +/- 0.308425 | [3.597389, 3.768371] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.449035 +/- 0.331430 | [3.357167, 3.540903] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.662955 +/- 0.337343 | [3.569448, 3.756462] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.088000 +/- 0.015584 | [0.083680, 0.092320] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.077200 +/- 0.016230 | [0.072701, 0.081699] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.077200 +/- 0.016230 | [0.072701, 0.081699] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.348157 +/- 0.023714 | [0.341584, 0.354731] | 17.15% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.118355 +/- 0.022214 | [0.112198, 0.124512] | 5.83% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.233845 +/- 0.120104 | [0.200554, 0.267136] | 6.35% | 48/50 | 2.267e-12 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.019925 +/- 0.131662 | [-0.016570, 0.056420] | 0.54% | 27/50 | 0.6718 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.010800 +/- 0.006709 | [0.008940, 0.012660] | 12.27% | 46/50 | 6.981e-11 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.010800 +/- 0.006709 | [0.008940, 0.012660] | 12.27% | 46/50 | 6.981e-11 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 62.245046 +/- 6.444045 | 2.593544 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 102.346812 +/- 11.261774 | 4.264450 | 1.647x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 95.576241 +/- 11.190371 | 3.982343 | 1.542x | 50 |
