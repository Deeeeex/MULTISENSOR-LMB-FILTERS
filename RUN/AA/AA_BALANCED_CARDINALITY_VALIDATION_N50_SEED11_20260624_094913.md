# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 12:22:45

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61]
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
- Trial 1: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 2: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 3: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 4: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 5: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]
- Trial 6: [0.1 0.2 0.1 0.5 0 0.1 0.5 0.1]
- Trial 7: [0.1 0.5 0.1 0.2 0.1 0.5 0.1 0]
- Trial 8: [0.1 0.5 0.1 0.5 0.2 0.1 0.1 0]
- Trial 9: [0.1 0.1 0.1 0.1 0.5 0.5 0 0.2]
- Trial 10: [0.5 0.5 0 0.1 0.1 0.1 0.1 0.2]
- Trial 11: [0.5 0.2 0.5 0.1 0.1 0 0.1 0.1]
- Trial 12: [0.1 0.1 0.1 0.5 0 0.2 0.1 0.5]
- Trial 13: [0.5 0.2 0.1 0.1 0.1 0.1 0 0.5]
- Trial 14: [0.1 0.5 0.2 0 0.5 0.1 0.1 0.1]
- Trial 15: [0.1 0.2 0.1 0.5 0.5 0 0.1 0.1]
- Trial 16: [0.1 0.1 0.5 0.1 0 0.5 0.1 0.2]
- Trial 17: [0.1 0.5 0.1 0.1 0 0.5 0.1 0.2]
- Trial 18: [0 0.5 0.1 0.2 0.1 0.5 0.1 0.1]
- Trial 19: [0.1 0.1 0 0.1 0.5 0.1 0.2 0.5]
- Trial 20: [0.1 0.1 0.5 0 0.1 0.5 0.2 0.1]
- Trial 21: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]
- Trial 22: [0.5 0.5 0.1 0.1 0.2 0.1 0 0.1]
- Trial 23: [0.1 0.1 0.1 0 0.5 0.1 0.2 0.5]
- Trial 24: [0.2 0.1 0.1 0 0.5 0.5 0.1 0.1]
- Trial 25: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 26: [0.5 0 0.1 0.5 0.2 0.1 0.1 0.1]
- Trial 27: [0.2 0.5 0.1 0.1 0.1 0.5 0 0.1]
- Trial 28: [0.1 0.5 0 0.1 0.5 0.1 0.1 0.2]
- Trial 29: [0.2 0.5 0 0.1 0.1 0.1 0.1 0.5]
- Trial 30: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 31: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 32: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 33: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 34: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 35: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 36: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 37: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 38: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 39: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 40: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]
- Trial 41: [0.5 0.1 0.1 0.5 0 0.2 0.1 0.1]
- Trial 42: [0.5 0 0.1 0.1 0.1 0.1 0.2 0.5]
- Trial 43: [0 0.5 0.5 0.1 0.1 0.1 0.1 0.2]
- Trial 44: [0.5 0 0.5 0.1 0.2 0.1 0.1 0.1]
- Trial 45: [0.1 0.5 0.1 0.1 0.5 0.2 0 0.1]
- Trial 46: [0.2 0.1 0 0.5 0.1 0.5 0.1 0.1]
- Trial 47: [0.1 0.2 0.1 0.5 0 0.5 0.1 0.1]
- Trial 48: [0.5 0.1 0.1 0.1 0.5 0.2 0 0.1]
- Trial 49: [0.2 0.5 0.1 0 0.1 0.5 0.1 0.1]
- Trial 50: [0 0.1 0.1 0.1 0.5 0.2 0.1 0.5]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 1 | 12 | Neighborhood label-barycenter spatial-KLA AA | 0.300460 | 0.227285 | 0.015000 |
| 1 | 12 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.063672 | 0.949262 | 0.015000 |
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 2 | 13 | Neighborhood label-barycenter spatial-KLA AA | 0.351073 | 0.224834 | 0.030000 |
| 2 | 13 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.103466 | 0.914666 | 0.030000 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 3 | 14 | Neighborhood label-barycenter spatial-KLA AA | 0.282517 | 0.244711 | 0.005000 |
| 3 | 14 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.998267 | 0.951321 | 0.005000 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 4 | 15 | Neighborhood label-barycenter spatial-KLA AA | 0.318286 | 0.206477 | 0.025000 |
| 4 | 15 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.026745 | 0.838960 | 0.025000 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 5 | 16 | Neighborhood label-barycenter spatial-KLA AA | 0.274923 | 0.230215 | 0.010000 |
| 5 | 16 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.977335 | 0.862316 | 0.010000 |
| 6 | 17 | Tuned spatial-KLA AA | 1.685404 | 1.388587 | 0.046250 |
| 6 | 17 | Neighborhood label-barycenter spatial-KLA AA | 0.292001 | 0.187499 | 0.025000 |
| 6 | 17 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.969933 | 0.795157 | 0.025000 |
| 7 | 18 | Tuned spatial-KLA AA | 1.654544 | 1.461745 | 0.020000 |
| 7 | 18 | Neighborhood label-barycenter spatial-KLA AA | 0.286964 | 0.213391 | 0.015000 |
| 7 | 18 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.023742 | 0.893535 | 0.015000 |
| 8 | 19 | Tuned spatial-KLA AA | 1.676684 | 1.472405 | 0.030000 |
| 8 | 19 | Neighborhood label-barycenter spatial-KLA AA | 0.302170 | 0.200867 | 0.025000 |
| 8 | 19 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.949360 | 0.799718 | 0.025000 |
| 9 | 20 | Tuned spatial-KLA AA | 1.709979 | 1.671988 | 0.037500 |
| 9 | 20 | Neighborhood label-barycenter spatial-KLA AA | 0.342610 | 0.254243 | 0.025000 |
| 9 | 20 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.092770 | 1.160585 | 0.025000 |
| 10 | 21 | Tuned spatial-KLA AA | 1.734930 | 1.489287 | 0.042500 |
| 10 | 21 | Neighborhood label-barycenter spatial-KLA AA | 0.315184 | 0.216701 | 0.020000 |
| 10 | 21 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.073299 | 0.905585 | 0.020000 |
| 11 | 22 | Tuned spatial-KLA AA | 1.636799 | 1.428640 | 0.030000 |
| 11 | 22 | Neighborhood label-barycenter spatial-KLA AA | 0.227032 | 0.193920 | 0.005000 |
| 11 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.902487 | 0.799631 | 0.005000 |
| 12 | 23 | Tuned spatial-KLA AA | 1.780009 | 1.492509 | 0.043750 |
| 12 | 23 | Neighborhood label-barycenter spatial-KLA AA | 0.286690 | 0.218064 | 0.010000 |
| 12 | 23 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.017645 | 0.862388 | 0.010000 |
| 13 | 24 | Tuned spatial-KLA AA | 1.702572 | 1.505093 | 0.026250 |
| 13 | 24 | Neighborhood label-barycenter spatial-KLA AA | 0.285476 | 0.222277 | 0.015000 |
| 13 | 24 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.027825 | 0.904061 | 0.015000 |
| 14 | 25 | Tuned spatial-KLA AA | 1.749069 | 1.504870 | 0.041250 |
| 14 | 25 | Neighborhood label-barycenter spatial-KLA AA | 0.339566 | 0.227367 | 0.030000 |
| 14 | 25 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.107811 | 0.945851 | 0.030000 |
| 15 | 26 | Tuned spatial-KLA AA | 1.776746 | 1.504229 | 0.066250 |
| 15 | 26 | Neighborhood label-barycenter spatial-KLA AA | 0.414702 | 0.223985 | 0.055000 |
| 15 | 26 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.127039 | 0.909551 | 0.055000 |
| 16 | 27 | Tuned spatial-KLA AA | 1.690300 | 1.398991 | 0.041250 |
| 16 | 27 | Neighborhood label-barycenter spatial-KLA AA | 0.342507 | 0.200608 | 0.030000 |
| 16 | 27 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.045952 | 0.838322 | 0.030000 |
| 17 | 28 | Tuned spatial-KLA AA | 1.785713 | 1.624857 | 0.031250 |
| 17 | 28 | Neighborhood label-barycenter spatial-KLA AA | 0.316006 | 0.250131 | 0.010000 |
| 17 | 28 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.099147 | 1.033175 | 0.010000 |
| 18 | 29 | Tuned spatial-KLA AA | 1.726645 | 1.416342 | 0.060000 |
| 18 | 29 | Neighborhood label-barycenter spatial-KLA AA | 0.361264 | 0.193530 | 0.040000 |
| 18 | 29 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.999678 | 0.781193 | 0.040000 |
| 19 | 30 | Tuned spatial-KLA AA | 1.647364 | 1.406921 | 0.033750 |
| 19 | 30 | Neighborhood label-barycenter spatial-KLA AA | 0.277150 | 0.190888 | 0.020000 |
| 19 | 30 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.969186 | 0.824703 | 0.020000 |
| 20 | 31 | Tuned spatial-KLA AA | 1.688300 | 1.424875 | 0.048750 |
| 20 | 31 | Neighborhood label-barycenter spatial-KLA AA | 0.356833 | 0.217255 | 0.040000 |
| 20 | 31 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.094361 | 0.928129 | 0.040000 |
| 21 | 32 | Tuned spatial-KLA AA | 1.743321 | 1.528194 | 0.021250 |
| 21 | 32 | Neighborhood label-barycenter spatial-KLA AA | 0.295142 | 0.230247 | 0.010000 |
| 21 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.093726 | 0.934688 | 0.010000 |
| 22 | 33 | Tuned spatial-KLA AA | 1.613191 | 1.396453 | 0.023750 |
| 22 | 33 | Neighborhood label-barycenter spatial-KLA AA | 0.283944 | 0.201861 | 0.015000 |
| 22 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.003319 | 0.851229 | 0.015000 |
| 23 | 34 | Tuned spatial-KLA AA | 1.695320 | 1.748860 | 0.025000 |
| 23 | 34 | Neighborhood label-barycenter spatial-KLA AA | 0.327244 | 0.288415 | 0.010000 |
| 23 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.065679 | 1.133232 | 0.010000 |
| 24 | 35 | Tuned spatial-KLA AA | 1.610147 | 1.512261 | 0.025000 |
| 24 | 35 | Neighborhood label-barycenter spatial-KLA AA | 0.271866 | 0.208537 | 0.010000 |
| 24 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.978653 | 0.835367 | 0.010000 |
| 25 | 36 | Tuned spatial-KLA AA | 1.616460 | 1.423190 | 0.021250 |
| 25 | 36 | Neighborhood label-barycenter spatial-KLA AA | 0.292115 | 0.213652 | 0.015000 |
| 25 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.026707 | 0.896379 | 0.015000 |
| 26 | 37 | Tuned spatial-KLA AA | 1.780506 | 1.528165 | 0.031250 |
| 26 | 37 | Neighborhood label-barycenter spatial-KLA AA | 0.305279 | 0.221607 | 0.020000 |
| 26 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.080172 | 0.931478 | 0.020000 |
| 27 | 38 | Tuned spatial-KLA AA | 1.660233 | 1.444823 | 0.037500 |
| 27 | 38 | Neighborhood label-barycenter spatial-KLA AA | 0.319515 | 0.213886 | 0.025000 |
| 27 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.041895 | 0.888306 | 0.025000 |
| 28 | 39 | Tuned spatial-KLA AA | 1.611358 | 1.515457 | 0.022500 |
| 28 | 39 | Neighborhood label-barycenter spatial-KLA AA | 0.292327 | 0.228453 | 0.015000 |
| 28 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.015322 | 0.891146 | 0.015000 |
| 29 | 40 | Tuned spatial-KLA AA | 1.667447 | 1.373777 | 0.036250 |
| 29 | 40 | Neighborhood label-barycenter spatial-KLA AA | 0.329642 | 0.200610 | 0.025000 |
| 29 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.069647 | 0.862358 | 0.025000 |
| 30 | 41 | Tuned spatial-KLA AA | 1.625285 | 1.397315 | 0.021250 |
| 30 | 41 | Neighborhood label-barycenter spatial-KLA AA | 0.293846 | 0.211119 | 0.015000 |
| 30 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.054516 | 0.893096 | 0.015000 |
| 31 | 42 | Tuned spatial-KLA AA | 1.598724 | 1.434675 | 0.017500 |
| 31 | 42 | Neighborhood label-barycenter spatial-KLA AA | 0.261609 | 0.211758 | 0.010000 |
| 31 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.999599 | 0.879733 | 0.010000 |
| 32 | 43 | Tuned spatial-KLA AA | 1.622933 | 1.336654 | 0.036250 |
| 32 | 43 | Neighborhood label-barycenter spatial-KLA AA | 0.275573 | 0.185154 | 0.020000 |
| 32 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.908214 | 0.740328 | 0.020000 |
| 33 | 44 | Tuned spatial-KLA AA | 1.627533 | 1.397986 | 0.048750 |
| 33 | 44 | Neighborhood label-barycenter spatial-KLA AA | 0.324753 | 0.204578 | 0.030000 |
| 33 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.987918 | 0.819876 | 0.030000 |
| 34 | 45 | Tuned spatial-KLA AA | 1.726464 | 1.468741 | 0.040000 |
| 34 | 45 | Neighborhood label-barycenter spatial-KLA AA | 0.283924 | 0.218091 | 0.015000 |
| 34 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.015649 | 0.897175 | 0.015000 |
| 35 | 46 | Tuned spatial-KLA AA | 1.650691 | 1.398505 | 0.046250 |
| 35 | 46 | Neighborhood label-barycenter spatial-KLA AA | 0.339945 | 0.197026 | 0.040000 |
| 35 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.010610 | 0.832949 | 0.040000 |
| 36 | 47 | Tuned spatial-KLA AA | 1.670539 | 1.445587 | 0.031250 |
| 36 | 47 | Neighborhood label-barycenter spatial-KLA AA | 0.295779 | 0.216475 | 0.020000 |
| 36 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.023508 | 0.889120 | 0.020000 |
| 37 | 48 | Tuned spatial-KLA AA | 1.729929 | 1.502969 | 0.032500 |
| 37 | 48 | Neighborhood label-barycenter spatial-KLA AA | 0.318924 | 0.225833 | 0.020000 |
| 37 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.055592 | 0.916342 | 0.020000 |
| 38 | 49 | Tuned spatial-KLA AA | 1.643617 | 1.395513 | 0.031250 |
| 38 | 49 | Neighborhood label-barycenter spatial-KLA AA | 0.304284 | 0.206848 | 0.020000 |
| 38 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.008484 | 0.822953 | 0.020000 |
| 39 | 50 | Tuned spatial-KLA AA | 1.781736 | 1.523159 | 0.070000 |
| 39 | 50 | Neighborhood label-barycenter spatial-KLA AA | 0.407054 | 0.210125 | 0.055000 |
| 39 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.088530 | 0.861900 | 0.055000 |
| 40 | 51 | Tuned spatial-KLA AA | 1.664178 | 1.467082 | 0.028750 |
| 40 | 51 | Neighborhood label-barycenter spatial-KLA AA | 0.275983 | 0.199318 | 0.015000 |
| 40 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.986961 | 0.843718 | 0.015000 |
| 41 | 52 | Tuned spatial-KLA AA | 1.724701 | 1.454953 | 0.052500 |
| 41 | 52 | Neighborhood label-barycenter spatial-KLA AA | 0.329544 | 0.194830 | 0.040000 |
| 41 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.949885 | 0.764822 | 0.040000 |
| 42 | 53 | Tuned spatial-KLA AA | 1.603330 | 1.446482 | 0.026250 |
| 42 | 53 | Neighborhood label-barycenter spatial-KLA AA | 0.269221 | 0.209952 | 0.015000 |
| 42 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.963471 | 0.857802 | 0.015000 |
| 43 | 54 | Tuned spatial-KLA AA | 1.632418 | 1.421881 | 0.031250 |
| 43 | 54 | Neighborhood label-barycenter spatial-KLA AA | 0.280450 | 0.203259 | 0.020000 |
| 43 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.988056 | 0.864955 | 0.020000 |
| 44 | 55 | Tuned spatial-KLA AA | 1.722974 | 1.620954 | 0.030000 |
| 44 | 55 | Neighborhood label-barycenter spatial-KLA AA | 0.304095 | 0.246511 | 0.015000 |
| 44 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.100424 | 0.949297 | 0.015000 |
| 45 | 56 | Tuned spatial-KLA AA | 1.683268 | 1.425747 | 0.036250 |
| 45 | 56 | Neighborhood label-barycenter spatial-KLA AA | 0.331251 | 0.206087 | 0.030000 |
| 45 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.063830 | 0.879068 | 0.030000 |
| 46 | 57 | Tuned spatial-KLA AA | 1.818498 | 1.547557 | 0.053750 |
| 46 | 57 | Neighborhood label-barycenter spatial-KLA AA | 0.317064 | 0.222953 | 0.020000 |
| 46 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.100128 | 0.940273 | 0.020000 |
| 47 | 58 | Tuned spatial-KLA AA | 1.761633 | 1.632547 | 0.035000 |
| 47 | 58 | Neighborhood label-barycenter spatial-KLA AA | 0.280528 | 0.212938 | 0.015000 |
| 47 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.986379 | 0.841182 | 0.015000 |
| 48 | 59 | Tuned spatial-KLA AA | 1.597575 | 1.389651 | 0.015000 |
| 48 | 59 | Neighborhood label-barycenter spatial-KLA AA | 0.245967 | 0.205073 | 0.005000 |
| 48 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.988588 | 0.860652 | 0.005000 |
| 49 | 60 | Tuned spatial-KLA AA | 1.606069 | 1.334397 | 0.036250 |
| 49 | 60 | Neighborhood label-barycenter spatial-KLA AA | 0.278175 | 0.193042 | 0.020000 |
| 49 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.934002 | 0.779471 | 0.020000 |
| 50 | 61 | Tuned spatial-KLA AA | 1.689259 | 1.409483 | 0.033750 |
| 50 | 61 | Neighborhood label-barycenter spatial-KLA AA | 0.298628 | 0.211444 | 0.015000 |
| 50 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.060000 | 0.881502 | 0.015000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.686779 | 1.475259 | 0.035350 |
| Neighborhood label-barycenter spatial-KLA AA | 0.306142 | 0.214879 | 0.021000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.026384 | 0.882770 | 0.021000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.686779 +/- 0.060553 | [1.669995, 1.703564] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.306142 +/- 0.035506 | [0.296300, 0.315984] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.026384 +/- 0.054765 | [1.011203, 1.041564] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 1.475259 +/- 0.086524 | [1.451275, 1.499242] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.214879 +/- 0.018927 | [0.209632, 0.220125] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.882770 +/- 0.078074 | [0.861129, 0.904411] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.035350 +/- 0.012193 | [0.031970, 0.038730] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.021000 +/- 0.011384 | [0.017845, 0.024155] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.021000 +/- 0.011384 | [0.017845, 0.024155] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.380638 +/- 0.049905 | [1.366805, 1.394471] | 81.85% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.660396 +/- 0.049763 | [0.646602, 0.674189] | 39.15% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.260380 +/- 0.070971 | [1.240708, 1.280052] | 85.43% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.592489 +/- 0.058597 | [0.576246, 0.608731] | 40.16% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.014350 +/- 0.006538 | [0.012538, 0.016162] | 40.59% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.014350 +/- 0.006538 | [0.012538, 0.016162] | 40.59% | 50/50 | 1.776e-15 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 12 | Tuned spatial-KLA AA | 2.037356 | 4.168666 | 0.071250 |
| 1 | 12 | Neighborhood label-barycenter spatial-KLA AA | 1.731185 | 3.950050 | 0.055000 |
| 1 | 12 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.957852 | 4.168047 | 0.055000 |
| 2 | 13 | Tuned spatial-KLA AA | 2.102036 | 3.283166 | 0.103750 |
| 2 | 13 | Neighborhood label-barycenter spatial-KLA AA | 1.726449 | 3.164303 | 0.090000 |
| 2 | 13 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.983320 | 3.411609 | 0.090000 |
| 3 | 14 | Tuned spatial-KLA AA | 2.010781 | 3.416971 | 0.092500 |
| 3 | 14 | Neighborhood label-barycenter spatial-KLA AA | 1.653909 | 3.419193 | 0.075000 |
| 3 | 14 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.878979 | 3.772731 | 0.075000 |
| 4 | 15 | Tuned spatial-KLA AA | 2.007164 | 3.301470 | 0.083750 |
| 4 | 15 | Neighborhood label-barycenter spatial-KLA AA | 1.657575 | 3.163567 | 0.075000 |
| 4 | 15 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.863341 | 3.357335 | 0.075000 |
| 5 | 16 | Tuned spatial-KLA AA | 2.006659 | 3.770452 | 0.080000 |
| 5 | 16 | Neighborhood label-barycenter spatial-KLA AA | 1.688135 | 3.557877 | 0.070000 |
| 5 | 16 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.907970 | 3.874823 | 0.070000 |
| 6 | 17 | Tuned spatial-KLA AA | 1.974042 | 3.275452 | 0.086250 |
| 6 | 17 | Neighborhood label-barycenter spatial-KLA AA | 1.596602 | 2.946270 | 0.065000 |
| 6 | 17 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.842916 | 3.155441 | 0.065000 |
| 7 | 18 | Tuned spatial-KLA AA | 1.965767 | 3.887499 | 0.077500 |
| 7 | 18 | Neighborhood label-barycenter spatial-KLA AA | 1.593961 | 3.562587 | 0.075000 |
| 7 | 18 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.848689 | 3.819671 | 0.075000 |
| 8 | 19 | Tuned spatial-KLA AA | 2.004215 | 3.667468 | 0.080000 |
| 8 | 19 | Neighborhood label-barycenter spatial-KLA AA | 1.601516 | 3.187414 | 0.075000 |
| 8 | 19 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.855304 | 3.408146 | 0.075000 |
| 9 | 20 | Tuned spatial-KLA AA | 2.056388 | 3.434770 | 0.092500 |
| 9 | 20 | Neighborhood label-barycenter spatial-KLA AA | 1.703758 | 3.211109 | 0.085000 |
| 9 | 20 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.963282 | 3.449559 | 0.085000 |
| 10 | 21 | Tuned spatial-KLA AA | 2.133225 | 3.397427 | 0.132500 |
| 10 | 21 | Neighborhood label-barycenter spatial-KLA AA | 1.793694 | 2.944004 | 0.130000 |
| 10 | 21 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.015768 | 3.108549 | 0.130000 |
| 11 | 22 | Tuned spatial-KLA AA | 1.980259 | 3.740146 | 0.087500 |
| 11 | 22 | Neighborhood label-barycenter spatial-KLA AA | 1.627348 | 3.658043 | 0.065000 |
| 11 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.822183 | 3.830580 | 0.065000 |
| 12 | 23 | Tuned spatial-KLA AA | 2.146965 | 3.831830 | 0.093750 |
| 12 | 23 | Neighborhood label-barycenter spatial-KLA AA | 1.770040 | 3.710336 | 0.080000 |
| 12 | 23 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.015681 | 3.913467 | 0.080000 |
| 13 | 24 | Tuned spatial-KLA AA | 2.102909 | 3.855136 | 0.078750 |
| 13 | 24 | Neighborhood label-barycenter spatial-KLA AA | 1.735544 | 3.669039 | 0.065000 |
| 13 | 24 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.988074 | 3.904073 | 0.065000 |
| 14 | 25 | Tuned spatial-KLA AA | 2.030394 | 3.938113 | 0.076250 |
| 14 | 25 | Neighborhood label-barycenter spatial-KLA AA | 1.669367 | 3.570059 | 0.070000 |
| 14 | 25 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.923006 | 3.775667 | 0.070000 |
| 15 | 26 | Tuned spatial-KLA AA | 2.028451 | 2.852841 | 0.118750 |
| 15 | 26 | Neighborhood label-barycenter spatial-KLA AA | 1.639793 | 2.460449 | 0.105000 |
| 15 | 26 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.871978 | 2.675957 | 0.105000 |
| 16 | 27 | Tuned spatial-KLA AA | 2.116906 | 3.753047 | 0.118750 |
| 16 | 27 | Neighborhood label-barycenter spatial-KLA AA | 1.790024 | 3.534027 | 0.110000 |
| 16 | 27 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.015442 | 3.720846 | 0.110000 |
| 17 | 28 | Tuned spatial-KLA AA | 2.080020 | 3.991589 | 0.081250 |
| 17 | 28 | Neighborhood label-barycenter spatial-KLA AA | 1.690858 | 3.890202 | 0.060000 |
| 17 | 28 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.942572 | 4.134321 | 0.060000 |
| 18 | 29 | Tuned spatial-KLA AA | 2.011484 | 3.548443 | 0.107500 |
| 18 | 29 | Neighborhood label-barycenter spatial-KLA AA | 1.662875 | 3.179408 | 0.090000 |
| 18 | 29 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.890722 | 3.374495 | 0.090000 |
| 19 | 30 | Tuned spatial-KLA AA | 1.976144 | 3.736853 | 0.068750 |
| 19 | 30 | Neighborhood label-barycenter spatial-KLA AA | 1.608312 | 3.516589 | 0.050000 |
| 19 | 30 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.836204 | 3.736876 | 0.050000 |
| 20 | 31 | Tuned spatial-KLA AA | 1.997924 | 3.628737 | 0.096250 |
| 20 | 31 | Neighborhood label-barycenter spatial-KLA AA | 1.651807 | 3.334364 | 0.090000 |
| 20 | 31 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.890660 | 3.558989 | 0.090000 |
| 21 | 32 | Tuned spatial-KLA AA | 2.103052 | 4.246511 | 0.066250 |
| 21 | 32 | Neighborhood label-barycenter spatial-KLA AA | 1.726636 | 3.875291 | 0.060000 |
| 21 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.982396 | 4.156063 | 0.060000 |
| 22 | 33 | Tuned spatial-KLA AA | 1.993714 | 3.542226 | 0.086250 |
| 22 | 33 | Neighborhood label-barycenter spatial-KLA AA | 1.638089 | 3.457866 | 0.075000 |
| 22 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.898048 | 3.681642 | 0.075000 |
| 23 | 34 | Tuned spatial-KLA AA | 2.135636 | 4.449378 | 0.075000 |
| 23 | 34 | Neighborhood label-barycenter spatial-KLA AA | 1.809639 | 4.211310 | 0.060000 |
| 23 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.023469 | 4.375365 | 0.060000 |
| 24 | 35 | Tuned spatial-KLA AA | 1.979734 | 4.104894 | 0.077500 |
| 24 | 35 | Neighborhood label-barycenter spatial-KLA AA | 1.647289 | 3.792912 | 0.070000 |
| 24 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.852947 | 4.027931 | 0.070000 |
| 25 | 36 | Tuned spatial-KLA AA | 1.972823 | 3.461380 | 0.073750 |
| 25 | 36 | Neighborhood label-barycenter spatial-KLA AA | 1.637422 | 3.230576 | 0.075000 |
| 25 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.871800 | 3.396608 | 0.075000 |
| 26 | 37 | Tuned spatial-KLA AA | 2.090258 | 3.735021 | 0.081250 |
| 26 | 37 | Neighborhood label-barycenter spatial-KLA AA | 1.716222 | 3.616961 | 0.070000 |
| 26 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.982921 | 3.859923 | 0.070000 |
| 27 | 38 | Tuned spatial-KLA AA | 2.025133 | 3.764122 | 0.092500 |
| 27 | 38 | Neighborhood label-barycenter spatial-KLA AA | 1.684163 | 3.583605 | 0.075000 |
| 27 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.914688 | 3.832544 | 0.075000 |
| 28 | 39 | Tuned spatial-KLA AA | 2.002165 | 3.738346 | 0.092500 |
| 28 | 39 | Neighborhood label-barycenter spatial-KLA AA | 1.661104 | 3.425726 | 0.085000 |
| 28 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.873484 | 3.566311 | 0.085000 |
| 29 | 40 | Tuned spatial-KLA AA | 2.110014 | 3.894615 | 0.113750 |
| 29 | 40 | Neighborhood label-barycenter spatial-KLA AA | 1.793600 | 3.604491 | 0.115000 |
| 29 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.031268 | 3.765029 | 0.115000 |
| 30 | 41 | Tuned spatial-KLA AA | 1.951218 | 3.884326 | 0.056250 |
| 30 | 41 | Neighborhood label-barycenter spatial-KLA AA | 1.632412 | 3.642643 | 0.055000 |
| 30 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.868518 | 3.846857 | 0.055000 |
| 31 | 42 | Tuned spatial-KLA AA | 2.028833 | 3.984521 | 0.070000 |
| 31 | 42 | Neighborhood label-barycenter spatial-KLA AA | 1.710223 | 3.887043 | 0.060000 |
| 31 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.899226 | 4.068589 | 0.060000 |
| 32 | 43 | Tuned spatial-KLA AA | 1.930116 | 3.688322 | 0.078750 |
| 32 | 43 | Neighborhood label-barycenter spatial-KLA AA | 1.581147 | 3.577550 | 0.060000 |
| 32 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.772116 | 3.732821 | 0.060000 |
| 33 | 44 | Tuned spatial-KLA AA | 2.017967 | 3.223458 | 0.098750 |
| 33 | 44 | Neighborhood label-barycenter spatial-KLA AA | 1.720061 | 3.093393 | 0.080000 |
| 33 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.881188 | 3.226339 | 0.080000 |
| 34 | 45 | Tuned spatial-KLA AA | 2.163661 | 3.293489 | 0.102500 |
| 34 | 45 | Neighborhood label-barycenter spatial-KLA AA | 1.820328 | 2.914857 | 0.085000 |
| 34 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.020655 | 3.104211 | 0.085000 |
| 35 | 46 | Tuned spatial-KLA AA | 2.047680 | 3.280164 | 0.113750 |
| 35 | 46 | Neighborhood label-barycenter spatial-KLA AA | 1.726518 | 2.875873 | 0.110000 |
| 35 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.949268 | 3.083164 | 0.110000 |
| 36 | 47 | Tuned spatial-KLA AA | 1.999704 | 3.358546 | 0.093750 |
| 36 | 47 | Neighborhood label-barycenter spatial-KLA AA | 1.631602 | 3.098063 | 0.080000 |
| 36 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.846483 | 3.294741 | 0.080000 |
| 37 | 48 | Tuned spatial-KLA AA | 2.047167 | 3.698988 | 0.087500 |
| 37 | 48 | Neighborhood label-barycenter spatial-KLA AA | 1.664675 | 3.301132 | 0.080000 |
| 37 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.916715 | 3.584314 | 0.080000 |
| 38 | 49 | Tuned spatial-KLA AA | 1.984189 | 3.638821 | 0.076250 |
| 38 | 49 | Neighborhood label-barycenter spatial-KLA AA | 1.637001 | 3.438381 | 0.070000 |
| 38 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.881424 | 3.639848 | 0.070000 |
| 39 | 50 | Tuned spatial-KLA AA | 2.099682 | 3.893739 | 0.112500 |
| 39 | 50 | Neighborhood label-barycenter spatial-KLA AA | 1.769597 | 3.636349 | 0.095000 |
| 39 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.973576 | 3.777072 | 0.095000 |
| 40 | 51 | Tuned spatial-KLA AA | 2.043834 | 3.617527 | 0.083750 |
| 40 | 51 | Neighborhood label-barycenter spatial-KLA AA | 1.708913 | 3.343269 | 0.075000 |
| 40 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.923273 | 3.542099 | 0.075000 |
| 41 | 52 | Tuned spatial-KLA AA | 2.044668 | 3.754093 | 0.125000 |
| 41 | 52 | Neighborhood label-barycenter spatial-KLA AA | 1.707628 | 3.151048 | 0.130000 |
| 41 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.905998 | 3.330029 | 0.130000 |
| 42 | 53 | Tuned spatial-KLA AA | 1.958314 | 3.771526 | 0.078750 |
| 42 | 53 | Neighborhood label-barycenter spatial-KLA AA | 1.626828 | 3.596977 | 0.065000 |
| 42 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.852292 | 3.795464 | 0.065000 |
| 43 | 54 | Tuned spatial-KLA AA | 1.976217 | 3.855971 | 0.076250 |
| 43 | 54 | Neighborhood label-barycenter spatial-KLA AA | 1.623420 | 3.608434 | 0.070000 |
| 43 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.873149 | 3.837728 | 0.070000 |
| 44 | 55 | Tuned spatial-KLA AA | 2.080423 | 3.709363 | 0.085000 |
| 44 | 55 | Neighborhood label-barycenter spatial-KLA AA | 1.745281 | 3.476814 | 0.075000 |
| 44 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.999090 | 3.815928 | 0.075000 |
| 45 | 56 | Tuned spatial-KLA AA | 1.986746 | 4.020252 | 0.071250 |
| 45 | 56 | Neighborhood label-barycenter spatial-KLA AA | 1.609573 | 3.704047 | 0.060000 |
| 45 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.863118 | 3.926286 | 0.060000 |
| 46 | 57 | Tuned spatial-KLA AA | 2.133446 | 3.829763 | 0.116250 |
| 46 | 57 | Neighborhood label-barycenter spatial-KLA AA | 1.704155 | 3.482786 | 0.090000 |
| 46 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.953737 | 3.722268 | 0.090000 |
| 47 | 58 | Tuned spatial-KLA AA | 2.128641 | 3.685998 | 0.092500 |
| 47 | 58 | Neighborhood label-barycenter spatial-KLA AA | 1.785967 | 3.513432 | 0.075000 |
| 47 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.011907 | 3.838684 | 0.075000 |
| 48 | 59 | Tuned spatial-KLA AA | 1.970986 | 3.537510 | 0.075000 |
| 48 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.630946 | 3.337613 | 0.065000 |
| 48 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.867411 | 3.553954 | 0.065000 |
| 49 | 60 | Tuned spatial-KLA AA | 2.126878 | 3.448088 | 0.108750 |
| 49 | 60 | Neighborhood label-barycenter spatial-KLA AA | 1.822903 | 3.531845 | 0.090000 |
| 49 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.021952 | 3.676131 | 0.090000 |
| 50 | 61 | Tuned spatial-KLA AA | 2.079856 | 4.188025 | 0.073750 |
| 50 | 61 | Neighborhood label-barycenter spatial-KLA AA | 1.728473 | 3.873538 | 0.065000 |
| 50 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.964192 | 4.062115 | 0.065000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.039637 | 3.695581 | 0.089250 |
| Neighborhood label-barycenter spatial-KLA AA | 1.689891 | 3.450254 | 0.078000 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.919805 | 3.665425 | 0.078000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.039637 +/- 0.061089 | [2.022704, 2.056570] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 1.689891 +/- 0.065497 | [1.671736, 1.708046] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 1.919805 +/- 0.065402 | [1.901677, 1.937934] | 50 |
| Tuned spatial-KLA AA | RMSE | 3.695581 +/- 0.300628 | [3.612252, 3.778911] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.450254 +/- 0.322640 | [3.360823, 3.539685] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 3.665425 +/- 0.330161 | [3.573909, 3.756941] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.089250 +/- 0.016993 | [0.084540, 0.093960] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.078000 +/- 0.018070 | [0.072991, 0.083009] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.078000 +/- 0.018070 | [0.072991, 0.083009] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.349746 +/- 0.026374 | [0.342435, 0.357056] | 17.15% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.119832 +/- 0.022165 | [0.113688, 0.125976] | 5.88% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.245327 +/- 0.126965 | [0.210134, 0.280520] | 6.64% | 48/50 | 2.267e-12 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.030157 +/- 0.139122 | [-0.008406, 0.068719] | 0.82% | 28/50 | 0.4799 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.011250 +/- 0.006739 | [0.009382, 0.013118] | 12.61% | 47/50 | 3.708e-11 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.011250 +/- 0.006739 | [0.009382, 0.013118] | 12.61% | 47/50 | 3.708e-11 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 40.434686 +/- 2.922141 | 1.684779 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 65.901396 +/- 3.438776 | 2.745892 | 1.634x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 61.797248 +/- 4.145194 | 2.574885 | 1.533x | 50 |
