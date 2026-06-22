# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 14:28:32

Comparison order: Tuned spatial-KLA AA -> Cross-local label-consensus spatial-KLA AA

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
- crossLocalConsensusCutoff: NaN
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

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
| 1 | 2 | Tuned spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| 1 | 2 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 2 | 3 | Tuned spatial-KLA AA | 1.667430 | 1.435498 | 0.035000 |
| 2 | 3 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 3 | 4 | Tuned spatial-KLA AA | 1.665450 | 1.448322 | 0.031250 |
| 3 | 4 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 4 | 5 | Tuned spatial-KLA AA | 1.603341 | 1.300640 | 0.042500 |
| 4 | 5 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 5 | 6 | Tuned spatial-KLA AA | 1.610089 | 1.457096 | 0.027500 |
| 5 | 6 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 6 | 7 | Tuned spatial-KLA AA | 1.598491 | 1.528223 | 0.035000 |
| 6 | 7 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 7 | 8 | Tuned spatial-KLA AA | 1.747386 | 1.519075 | 0.038750 |
| 7 | 8 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 8 | 9 | Tuned spatial-KLA AA | 1.623301 | 1.359868 | 0.030000 |
| 8 | 9 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 9 | 10 | Tuned spatial-KLA AA | 1.664538 | 1.534739 | 0.051250 |
| 9 | 10 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 10 | 11 | Tuned spatial-KLA AA | 1.768428 | 1.504540 | 0.040000 |
| 10 | 11 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 11 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 11 | 12 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 12 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 12 | 13 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 13 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 13 | 14 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 14 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 14 | 15 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 15 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 15 | 16 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 16 | 17 | Tuned spatial-KLA AA | 1.685404 | 1.388587 | 0.046250 |
| 16 | 17 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 17 | 18 | Tuned spatial-KLA AA | 1.654544 | 1.461745 | 0.020000 |
| 17 | 18 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 18 | 19 | Tuned spatial-KLA AA | 1.676684 | 1.472405 | 0.030000 |
| 18 | 19 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 19 | 20 | Tuned spatial-KLA AA | 1.709979 | 1.671988 | 0.037500 |
| 19 | 20 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 20 | 21 | Tuned spatial-KLA AA | 1.734930 | 1.489287 | 0.042500 |
| 20 | 21 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 21 | 22 | Tuned spatial-KLA AA | 1.636799 | 1.428640 | 0.030000 |
| 21 | 22 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 22 | 23 | Tuned spatial-KLA AA | 1.780009 | 1.492509 | 0.043750 |
| 22 | 23 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 23 | 24 | Tuned spatial-KLA AA | 1.702572 | 1.505093 | 0.026250 |
| 23 | 24 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 24 | 25 | Tuned spatial-KLA AA | 1.749069 | 1.504870 | 0.041250 |
| 24 | 25 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 25 | 26 | Tuned spatial-KLA AA | 1.776746 | 1.504229 | 0.066250 |
| 25 | 26 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 26 | 27 | Tuned spatial-KLA AA | 1.690300 | 1.398991 | 0.041250 |
| 26 | 27 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 27 | 28 | Tuned spatial-KLA AA | 1.785713 | 1.624857 | 0.031250 |
| 27 | 28 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 28 | 29 | Tuned spatial-KLA AA | 1.726645 | 1.416342 | 0.060000 |
| 28 | 29 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 29 | 30 | Tuned spatial-KLA AA | 1.647364 | 1.406921 | 0.033750 |
| 29 | 30 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 30 | 31 | Tuned spatial-KLA AA | 1.688300 | 1.424875 | 0.048750 |
| 30 | 31 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 31 | 32 | Tuned spatial-KLA AA | 1.743321 | 1.528194 | 0.021250 |
| 31 | 32 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 32 | 33 | Tuned spatial-KLA AA | 1.613191 | 1.396453 | 0.023750 |
| 32 | 33 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 33 | 34 | Tuned spatial-KLA AA | 1.695320 | 1.748860 | 0.025000 |
| 33 | 34 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 34 | 35 | Tuned spatial-KLA AA | 1.610147 | 1.512261 | 0.025000 |
| 34 | 35 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 35 | 36 | Tuned spatial-KLA AA | 1.616460 | 1.423190 | 0.021250 |
| 35 | 36 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 36 | 37 | Tuned spatial-KLA AA | 1.780506 | 1.528165 | 0.031250 |
| 36 | 37 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 37 | 38 | Tuned spatial-KLA AA | 1.660233 | 1.444823 | 0.037500 |
| 37 | 38 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 38 | 39 | Tuned spatial-KLA AA | 1.611358 | 1.515457 | 0.022500 |
| 38 | 39 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 39 | 40 | Tuned spatial-KLA AA | 1.667447 | 1.373777 | 0.036250 |
| 39 | 40 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 40 | 41 | Tuned spatial-KLA AA | 1.625285 | 1.397315 | 0.021250 |
| 40 | 41 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 41 | 42 | Tuned spatial-KLA AA | 1.598724 | 1.434675 | 0.017500 |
| 41 | 42 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 42 | 43 | Tuned spatial-KLA AA | 1.622933 | 1.336654 | 0.036250 |
| 42 | 43 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 43 | 44 | Tuned spatial-KLA AA | 1.627533 | 1.397986 | 0.048750 |
| 43 | 44 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 44 | 45 | Tuned spatial-KLA AA | 1.726464 | 1.468741 | 0.040000 |
| 44 | 45 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 45 | 46 | Tuned spatial-KLA AA | 1.650691 | 1.398505 | 0.046250 |
| 45 | 46 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 46 | 47 | Tuned spatial-KLA AA | 1.670539 | 1.445587 | 0.031250 |
| 46 | 47 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 47 | 48 | Tuned spatial-KLA AA | 1.729929 | 1.502969 | 0.032500 |
| 47 | 48 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 48 | 49 | Tuned spatial-KLA AA | 1.643617 | 1.395513 | 0.031250 |
| 48 | 49 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 49 | 50 | Tuned spatial-KLA AA | 1.781736 | 1.523159 | 0.070000 |
| 49 | 50 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |
| 50 | 51 | Tuned spatial-KLA AA | 1.664178 | 1.467082 | 0.028750 |
| 50 | 51 | Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.682607 | 1.472837 | 0.035350 |
| Cross-local label-consensus spatial-KLA AA | 0.000000 | 0.000000 | 0.000000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.682607 +/- 0.058035 | [1.666520, 1.698693] | 50 |
| Cross-local label-consensus spatial-KLA AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 1.472837 +/- 0.082422 | [1.449991, 1.495683] | 50 |
| Cross-local label-consensus spatial-KLA AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.035350 +/- 0.011802 | [0.032079, 0.038621] | 50 |
| Cross-local label-consensus spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cross-local label-consensus spatial-KLA AA | OSPA | 1.682607 +/- 0.058035 | [1.666520, 1.698693] | 100.00% | 50/50 | 1.776e-15 |
| Cross-local label-consensus spatial-KLA AA | Loc. disag. | 1.472837 +/- 0.082422 | [1.449991, 1.495683] | 100.00% | 50/50 | 1.776e-15 |
| Cross-local label-consensus spatial-KLA AA | Card. disp. | 0.035350 +/- 0.011802 | [0.032079, 0.038621] | 100.00% | 50/50 | 1.776e-15 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.029641 | 3.682880 | 0.088000 |
| Cross-local label-consensus spatial-KLA AA | 1.645476 | 3.343985 | 0.071200 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.029641 +/- 0.061460 | [2.012605, 2.046677] | 50 |
| Cross-local label-consensus spatial-KLA AA | E-OSPA | 1.645476 +/- 0.065546 | [1.627308, 1.663644] | 50 |
| Tuned spatial-KLA AA | RMSE | 3.682880 +/- 0.308425 | [3.597389, 3.768371] | 50 |
| Cross-local label-consensus spatial-KLA AA | RMSE | 3.343985 +/- 0.410326 | [3.230249, 3.457722] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.088000 +/- 0.015584 | [0.083680, 0.092320] | 50 |
| Cross-local label-consensus spatial-KLA AA | CardErr | 0.071200 +/- 0.018029 | [0.066202, 0.076198] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cross-local label-consensus spatial-KLA AA | E-OSPA | 0.384165 +/- 0.030365 | [0.375748, 0.392582] | 18.93% | 50/50 | 1.776e-15 |
| Cross-local label-consensus spatial-KLA AA | RMSE | 0.338895 +/- 0.227789 | [0.275755, 0.402035] | 9.20% | 47/50 | 3.708e-11 |
| Cross-local label-consensus spatial-KLA AA | CardErr | 0.016800 +/- 0.011726 | [0.013550, 0.020050] | 19.09% | 47/50 | 3.708e-11 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 61.803414 +/- 10.991752 | 2.575142 | 1.000x | 50 |
| Cross-local label-consensus spatial-KLA AA | 66.012240 +/- 12.579937 | 2.750510 | 1.073x | 50 |
