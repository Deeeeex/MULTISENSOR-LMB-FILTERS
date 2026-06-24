# AA Balanced/Cardinality Validation

Generated at: 2026-06-24 15:33:45

Comparison order: Tuned spatial-KLA AA -> Neighborhood label-barycenter spatial-KLA AA -> Neighborhood reference-only label-consensus spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 50
- baseSeed: 21 (fixed=1)
- trialSeeds: [22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71]
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
- Trial 2: [0.35 0.35 0.5 0.7 0.2 0.5 0.35 0.7]
- Trial 3: [0.7 0.5 0.35 0.35 0.35 0.5 0.2 0.7]
- Trial 4: [0.5 0.7 0.5 0.2 0.7 0.35 0.35 0.35]
- Trial 5: [0.35 0.5 0.5 0.7 0.7 0.2 0.35 0.35]
- Trial 6: [0.35 0.5 0.7 0.35 0.2 0.7 0.35 0.5]
- Trial 7: [0.35 0.7 0.35 0.5 0.2 0.7 0.35 0.5]
- Trial 8: [0.2 0.7 0.35 0.5 0.35 0.7 0.35 0.5]
- Trial 9: [0.35 0.35 0.2 0.5 0.7 0.35 0.5 0.7]
- Trial 10: [0.5 0.35 0.7 0.2 0.35 0.7 0.5 0.35]
- Trial 11: [0.35 0.7 0.7 0.35 0.2 0.5 0.5 0.35]
- Trial 12: [0.7 0.7 0.35 0.35 0.5 0.35 0.2 0.5]
- Trial 13: [0.5 0.35 0.35 0.2 0.7 0.35 0.5 0.7]
- Trial 14: [0.5 0.35 0.5 0.2 0.7 0.7 0.35 0.35]
- Trial 15: [0.5 0.5 0.7 0.35 0.2 0.7 0.35 0.35]
- Trial 16: [0.7 0.2 0.5 0.7 0.5 0.35 0.35 0.35]
- Trial 17: [0.5 0.7 0.35 0.35 0.5 0.7 0.2 0.35]
- Trial 18: [0.5 0.7 0.2 0.35 0.7 0.35 0.35 0.5]
- Trial 19: [0.5 0.7 0.2 0.5 0.35 0.35 0.35 0.7]
- Trial 20: [0.35 0.5 0.7 0.2 0.7 0.5 0.35 0.35]
- Trial 21: [0.35 0.2 0.7 0.35 0.5 0.5 0.35 0.7]
- Trial 22: [0.35 0.35 0.7 0.7 0.5 0.5 0.2 0.35]
- Trial 23: [0.5 0.35 0.7 0.2 0.35 0.5 0.35 0.7]
- Trial 24: [0.7 0.5 0.2 0.35 0.7 0.5 0.35 0.35]
- Trial 25: [0.35 0.5 0.5 0.7 0.2 0.7 0.35 0.35]
- Trial 26: [0.5 0.5 0.7 0.35 0.35 0.35 0.2 0.7]
- Trial 27: [0.5 0.35 0.2 0.5 0.35 0.7 0.7 0.35]
- Trial 28: [0.35 0.5 0.35 0.35 0.2 0.7 0.5 0.7]
- Trial 29: [0.5 0.7 0.35 0.5 0.2 0.35 0.7 0.35]
- Trial 30: [0.7 0.7 0.5 0.5 0.35 0.35 0.35 0.2]
- Trial 31: [0.7 0.5 0.35 0.7 0.2 0.5 0.35 0.35]
- Trial 32: [0.7 0.2 0.35 0.35 0.35 0.5 0.5 0.7]
- Trial 33: [0.2 0.7 0.7 0.35 0.35 0.5 0.35 0.5]
- Trial 34: [0.7 0.2 0.7 0.5 0.5 0.35 0.35 0.35]
- Trial 35: [0.35 0.7 0.35 0.35 0.7 0.5 0.2 0.5]
- Trial 36: [0.5 0.35 0.2 0.7 0.35 0.7 0.35 0.5]
- Trial 37: [0.5 0.5 0.35 0.7 0.2 0.7 0.35 0.35]
- Trial 38: [0.7 0.35 0.35 0.5 0.7 0.5 0.2 0.35]
- Trial 39: [0.5 0.7 0.35 0.2 0.35 0.7 0.5 0.35]
- Trial 40: [0.2 0.35 0.5 0.35 0.7 0.5 0.35 0.7]
- Trial 41: [0.5 0.35 0.2 0.5 0.35 0.7 0.35 0.7]
- Trial 42: [0.35 0.35 0.2 0.7 0.5 0.5 0.7 0.35]
- Trial 43: [0.5 0.35 0.5 0.2 0.35 0.35 0.7 0.7]
- Trial 44: [0.5 0.35 0.2 0.5 0.7 0.35 0.35 0.7]
- Trial 45: [0.7 0.35 0.7 0.5 0.35 0.5 0.2 0.35]
- Trial 46: [0.35 0.5 0.35 0.7 0.7 0.5 0.2 0.35]
- Trial 47: [0.5 0.5 0.35 0.2 0.35 0.35 0.7 0.7]
- Trial 48: [0.2 0.7 0.35 0.7 0.5 0.5 0.35 0.35]
- Trial 49: [0.35 0.7 0.5 0.35 0.7 0.35 0.2 0.5]
- Trial 50: [0.5 0.35 0.7 0.5 0.2 0.7 0.35 0.35]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 22 | Tuned spatial-KLA AA | 2.101216 | 2.492763 | 0.093750 |
| 1 | 22 | Neighborhood label-barycenter spatial-KLA AA | 0.466321 | 0.348431 | 0.045000 |
| 1 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.222720 | 1.658173 | 0.045000 |
| 2 | 23 | Tuned spatial-KLA AA | 2.155568 | 1.864367 | 0.076250 |
| 2 | 23 | Neighborhood label-barycenter spatial-KLA AA | 0.478807 | 0.254743 | 0.045000 |
| 2 | 23 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.228814 | 0.967695 | 0.045000 |
| 3 | 24 | Tuned spatial-KLA AA | 2.083470 | 2.007434 | 0.062500 |
| 3 | 24 | Neighborhood label-barycenter spatial-KLA AA | 0.403745 | 0.318575 | 0.030000 |
| 3 | 24 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.318900 | 1.246914 | 0.030000 |
| 4 | 25 | Tuned spatial-KLA AA | 2.231986 | 1.979444 | 0.070000 |
| 4 | 25 | Neighborhood label-barycenter spatial-KLA AA | 0.404362 | 0.280529 | 0.035000 |
| 4 | 25 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.275286 | 1.119510 | 0.035000 |
| 5 | 26 | Tuned spatial-KLA AA | 2.424864 | 2.751819 | 0.133750 |
| 5 | 26 | Neighborhood label-barycenter spatial-KLA AA | 0.631144 | 0.417502 | 0.070000 |
| 5 | 26 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.349113 | 1.361617 | 0.070000 |
| 6 | 27 | Tuned spatial-KLA AA | 2.095139 | 1.837118 | 0.073750 |
| 6 | 27 | Neighborhood label-barycenter spatial-KLA AA | 0.446294 | 0.264362 | 0.040000 |
| 6 | 27 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.242258 | 1.016108 | 0.040000 |
| 7 | 28 | Tuned spatial-KLA AA | 2.318778 | 1.925712 | 0.108750 |
| 7 | 28 | Neighborhood label-barycenter spatial-KLA AA | 0.532728 | 0.269273 | 0.065000 |
| 7 | 28 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.310791 | 1.074615 | 0.065000 |
| 8 | 29 | Tuned spatial-KLA AA | 2.341931 | 2.258677 | 0.103750 |
| 8 | 29 | Neighborhood label-barycenter spatial-KLA AA | 0.547082 | 0.316985 | 0.065000 |
| 8 | 29 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.331986 | 1.097737 | 0.065000 |
| 9 | 30 | Tuned spatial-KLA AA | 2.374769 | 2.161636 | 0.087500 |
| 9 | 30 | Neighborhood label-barycenter spatial-KLA AA | 0.410705 | 0.309389 | 0.025000 |
| 9 | 30 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.295195 | 1.288840 | 0.025000 |
| 10 | 31 | Tuned spatial-KLA AA | 2.111616 | 1.878215 | 0.071250 |
| 10 | 31 | Neighborhood label-barycenter spatial-KLA AA | 0.355005 | 0.262308 | 0.025000 |
| 10 | 31 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.223648 | 1.080381 | 0.025000 |
| 11 | 32 | Tuned spatial-KLA AA | 2.322000 | 2.112860 | 0.090000 |
| 11 | 32 | Neighborhood label-barycenter spatial-KLA AA | 0.417740 | 0.296620 | 0.035000 |
| 11 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.308136 | 1.217359 | 0.035000 |
| 12 | 33 | Tuned spatial-KLA AA | 2.168250 | 1.904059 | 0.081250 |
| 12 | 33 | Neighborhood label-barycenter spatial-KLA AA | 0.397751 | 0.269686 | 0.040000 |
| 12 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.231684 | 1.106065 | 0.040000 |
| 13 | 34 | Tuned spatial-KLA AA | 2.323900 | 2.378014 | 0.105000 |
| 13 | 34 | Neighborhood label-barycenter spatial-KLA AA | 0.501651 | 0.363038 | 0.045000 |
| 13 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.373139 | 1.389351 | 0.045000 |
| 14 | 35 | Tuned spatial-KLA AA | 2.094224 | 2.056148 | 0.047500 |
| 14 | 35 | Neighborhood label-barycenter spatial-KLA AA | 0.380199 | 0.273702 | 0.020000 |
| 14 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.236945 | 1.079816 | 0.020000 |
| 15 | 36 | Tuned spatial-KLA AA | 2.117035 | 1.946162 | 0.065000 |
| 15 | 36 | Neighborhood label-barycenter spatial-KLA AA | 0.399096 | 0.273377 | 0.025000 |
| 15 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.155345 | 0.974733 | 0.025000 |
| 16 | 37 | Tuned spatial-KLA AA | 2.417345 | 2.188355 | 0.123750 |
| 16 | 37 | Neighborhood label-barycenter spatial-KLA AA | 0.421340 | 0.281027 | 0.035000 |
| 16 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.193317 | 1.005285 | 0.035000 |
| 17 | 38 | Tuned spatial-KLA AA | 2.171419 | 2.187624 | 0.076250 |
| 17 | 38 | Neighborhood label-barycenter spatial-KLA AA | 0.427461 | 0.318028 | 0.035000 |
| 17 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.253190 | 1.242149 | 0.035000 |
| 18 | 39 | Tuned spatial-KLA AA | 2.182604 | 2.599653 | 0.077500 |
| 18 | 39 | Neighborhood label-barycenter spatial-KLA AA | 0.462976 | 0.392372 | 0.035000 |
| 18 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.262700 | 1.477005 | 0.035000 |
| 19 | 40 | Tuned spatial-KLA AA | 2.321645 | 2.164151 | 0.112500 |
| 19 | 40 | Neighborhood label-barycenter spatial-KLA AA | 0.516677 | 0.289176 | 0.060000 |
| 19 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.406087 | 1.285679 | 0.060000 |
| 20 | 41 | Tuned spatial-KLA AA | 2.179889 | 1.965700 | 0.095000 |
| 20 | 41 | Neighborhood label-barycenter spatial-KLA AA | 0.462573 | 0.287403 | 0.050000 |
| 20 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.294136 | 1.112550 | 0.050000 |
| 21 | 42 | Tuned spatial-KLA AA | 2.168924 | 1.895683 | 0.066250 |
| 21 | 42 | Neighborhood label-barycenter spatial-KLA AA | 0.469785 | 0.272035 | 0.045000 |
| 21 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.362851 | 1.174094 | 0.045000 |
| 22 | 43 | Tuned spatial-KLA AA | 1.930160 | 1.957211 | 0.062500 |
| 22 | 43 | Neighborhood label-barycenter spatial-KLA AA | 0.329997 | 0.266404 | 0.020000 |
| 22 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 0.992201 | 0.829658 | 0.020000 |
| 23 | 44 | Tuned spatial-KLA AA | 2.098633 | 1.920719 | 0.061250 |
| 23 | 44 | Neighborhood label-barycenter spatial-KLA AA | 0.390502 | 0.278088 | 0.025000 |
| 23 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.200390 | 1.150835 | 0.025000 |
| 24 | 45 | Tuned spatial-KLA AA | 2.345261 | 2.559987 | 0.101250 |
| 24 | 45 | Neighborhood label-barycenter spatial-KLA AA | 0.432099 | 0.339757 | 0.030000 |
| 24 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.300810 | 1.198840 | 0.030000 |
| 25 | 46 | Tuned spatial-KLA AA | 2.154868 | 2.012808 | 0.080000 |
| 25 | 46 | Neighborhood label-barycenter spatial-KLA AA | 0.382566 | 0.270360 | 0.030000 |
| 25 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.177443 | 0.995956 | 0.030000 |
| 26 | 47 | Tuned spatial-KLA AA | 2.071717 | 1.841930 | 0.053750 |
| 26 | 47 | Neighborhood label-barycenter spatial-KLA AA | 0.420849 | 0.265068 | 0.045000 |
| 26 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.349538 | 1.168235 | 0.045000 |
| 27 | 48 | Tuned spatial-KLA AA | 2.406899 | 2.417649 | 0.097500 |
| 27 | 48 | Neighborhood label-barycenter spatial-KLA AA | 0.472409 | 0.366351 | 0.045000 |
| 27 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.347739 | 1.610084 | 0.045000 |
| 28 | 49 | Tuned spatial-KLA AA | 2.270851 | 2.066643 | 0.085000 |
| 28 | 49 | Neighborhood label-barycenter spatial-KLA AA | 0.422164 | 0.274985 | 0.035000 |
| 28 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.184500 | 0.995868 | 0.035000 |
| 29 | 50 | Tuned spatial-KLA AA | 2.307950 | 2.056840 | 0.120000 |
| 29 | 50 | Neighborhood label-barycenter spatial-KLA AA | 0.590042 | 0.296345 | 0.090000 |
| 29 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.384082 | 1.168026 | 0.090000 |
| 30 | 51 | Tuned spatial-KLA AA | 2.221093 | 2.006563 | 0.111250 |
| 30 | 51 | Neighborhood label-barycenter spatial-KLA AA | 0.379891 | 0.253396 | 0.040000 |
| 30 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.178032 | 1.045123 | 0.040000 |
| 31 | 52 | Tuned spatial-KLA AA | 2.241449 | 2.446847 | 0.076250 |
| 31 | 52 | Neighborhood label-barycenter spatial-KLA AA | 0.481388 | 0.307543 | 0.055000 |
| 31 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.255422 | 1.232898 | 0.055000 |
| 32 | 53 | Tuned spatial-KLA AA | 2.214440 | 2.066438 | 0.065000 |
| 32 | 53 | Neighborhood label-barycenter spatial-KLA AA | 0.434008 | 0.302279 | 0.040000 |
| 32 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.286218 | 1.178122 | 0.040000 |
| 33 | 54 | Tuned spatial-KLA AA | 2.094617 | 1.890825 | 0.082500 |
| 33 | 54 | Neighborhood label-barycenter spatial-KLA AA | 0.357640 | 0.239148 | 0.035000 |
| 33 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.118218 | 0.995714 | 0.035000 |
| 34 | 55 | Tuned spatial-KLA AA | 2.271982 | 1.984172 | 0.077500 |
| 34 | 55 | Neighborhood label-barycenter spatial-KLA AA | 0.471863 | 0.271381 | 0.055000 |
| 34 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.320444 | 1.122099 | 0.055000 |
| 35 | 56 | Tuned spatial-KLA AA | 2.154506 | 2.165091 | 0.080000 |
| 35 | 56 | Neighborhood label-barycenter spatial-KLA AA | 0.468574 | 0.307082 | 0.045000 |
| 35 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.371731 | 1.189076 | 0.045000 |
| 36 | 57 | Tuned spatial-KLA AA | 2.351453 | 2.061551 | 0.123750 |
| 36 | 57 | Neighborhood label-barycenter spatial-KLA AA | 0.477885 | 0.297452 | 0.055000 |
| 36 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.312513 | 1.193600 | 0.055000 |
| 37 | 58 | Tuned spatial-KLA AA | 2.334127 | 2.179157 | 0.088750 |
| 37 | 58 | Neighborhood label-barycenter spatial-KLA AA | 0.456729 | 0.295016 | 0.045000 |
| 37 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.276431 | 1.107959 | 0.045000 |
| 38 | 59 | Tuned spatial-KLA AA | 2.083651 | 1.813388 | 0.078750 |
| 38 | 59 | Neighborhood label-barycenter spatial-KLA AA | 0.390944 | 0.247745 | 0.035000 |
| 38 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.135088 | 0.931984 | 0.035000 |
| 39 | 60 | Tuned spatial-KLA AA | 2.080789 | 2.225747 | 0.076250 |
| 39 | 60 | Neighborhood label-barycenter spatial-KLA AA | 0.500957 | 0.336214 | 0.055000 |
| 39 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.270756 | 1.172362 | 0.055000 |
| 40 | 61 | Tuned spatial-KLA AA | 2.313733 | 1.943866 | 0.098750 |
| 40 | 61 | Neighborhood label-barycenter spatial-KLA AA | 0.480933 | 0.265337 | 0.045000 |
| 40 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.319165 | 1.072986 | 0.045000 |
| 41 | 62 | Tuned spatial-KLA AA | 2.230561 | 1.966443 | 0.105000 |
| 41 | 62 | Neighborhood label-barycenter spatial-KLA AA | 0.507104 | 0.261164 | 0.070000 |
| 41 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.270086 | 1.018219 | 0.070000 |
| 42 | 63 | Tuned spatial-KLA AA | 2.281954 | 1.996816 | 0.098750 |
| 42 | 63 | Neighborhood label-barycenter spatial-KLA AA | 0.500949 | 0.274402 | 0.065000 |
| 42 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.291423 | 1.095538 | 0.065000 |
| 43 | 64 | Tuned spatial-KLA AA | 2.123472 | 2.102824 | 0.077500 |
| 43 | 64 | Neighborhood label-barycenter spatial-KLA AA | 0.428359 | 0.285886 | 0.035000 |
| 43 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.177903 | 1.009173 | 0.035000 |
| 44 | 65 | Tuned spatial-KLA AA | 2.241317 | 2.057449 | 0.075000 |
| 44 | 65 | Neighborhood label-barycenter spatial-KLA AA | 0.432089 | 0.269658 | 0.040000 |
| 44 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.187078 | 0.987730 | 0.040000 |
| 45 | 66 | Tuned spatial-KLA AA | 2.237130 | 2.258148 | 0.121250 |
| 45 | 66 | Neighborhood label-barycenter spatial-KLA AA | 0.506469 | 0.350219 | 0.060000 |
| 45 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.274080 | 1.341653 | 0.060000 |
| 46 | 67 | Tuned spatial-KLA AA | 2.267666 | 2.534979 | 0.098750 |
| 46 | 67 | Neighborhood label-barycenter spatial-KLA AA | 0.602216 | 0.341363 | 0.080000 |
| 46 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.296975 | 1.049121 | 0.080000 |
| 47 | 68 | Tuned spatial-KLA AA | 2.353300 | 2.369228 | 0.096250 |
| 47 | 68 | Neighborhood label-barycenter spatial-KLA AA | 0.434459 | 0.323312 | 0.030000 |
| 47 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.180170 | 1.049046 | 0.030000 |
| 48 | 69 | Tuned spatial-KLA AA | 2.137312 | 1.994012 | 0.096250 |
| 48 | 69 | Neighborhood label-barycenter spatial-KLA AA | 0.450720 | 0.259711 | 0.050000 |
| 48 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.142884 | 0.924702 | 0.050000 |
| 49 | 70 | Tuned spatial-KLA AA | 2.294279 | 2.628851 | 0.075000 |
| 49 | 70 | Neighborhood label-barycenter spatial-KLA AA | 0.477893 | 0.412857 | 0.035000 |
| 49 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.376723 | 1.488232 | 0.035000 |
| 50 | 71 | Tuned spatial-KLA AA | 2.391190 | 2.276738 | 0.088750 |
| 50 | 71 | Neighborhood label-barycenter spatial-KLA AA | 0.508589 | 0.352364 | 0.045000 |
| 50 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 1.271720 | 1.340593 | 0.045000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 2.224259 | 2.127170 | 0.087475 |
| Neighborhood label-barycenter spatial-KLA AA | 0.454475 | 0.299389 | 0.044100 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 1.262520 | 1.152782 | 0.044100 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 2.224259 +/- 0.112800 | [2.192992, 2.255525] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 0.454475 +/- 0.062230 | [0.437225, 0.471724] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 1.262520 +/- 0.081360 | [1.239968, 1.285072] | 50 |
| Tuned spatial-KLA AA | Loc. disag. | 2.127170 +/- 0.233517 | [2.062443, 2.191898] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 0.299389 +/- 0.042154 | [0.287705, 0.311073] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 1.152782 +/- 0.171733 | [1.105180, 1.200384] | 50 |
| Tuned spatial-KLA AA | Card. disp. | 0.087475 +/- 0.019528 | [0.082062, 0.092888] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.044100 +/- 0.015276 | [0.039866, 0.048334] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.044100 +/- 0.015276 | [0.039866, 0.048334] | 50 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | OSPA | 1.769784 +/- 0.094129 | [1.743693, 1.795875] | 79.57% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | OSPA | 0.961739 +/- 0.098209 | [0.934517, 0.988961] | 43.24% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Loc. disag. | 1.827781 +/- 0.195955 | [1.773465, 1.882097] | 85.93% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Loc. disag. | 0.974388 +/- 0.174832 | [0.925927, 1.022849] | 45.81% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | Card. disp. | 0.043375 +/- 0.015672 | [0.039031, 0.047719] | 49.59% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | Card. disp. | 0.043375 +/- 0.015672 | [0.039031, 0.047719] | 49.59% | 50/50 | 1.776e-15 |

## Per-Trial Local Tracking Metrics
| Trial | Seed | Arm | E-OSPA | RMSE | CardErr |
|------:|-----:|:----|-------:|-----:|--------:|
| 1 | 22 | Tuned spatial-KLA AA | 2.438530 | 3.951127 | 0.173750 |
| 1 | 22 | Neighborhood label-barycenter spatial-KLA AA | 2.066908 | 4.011443 | 0.135000 |
| 1 | 22 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.276603 | 3.956631 | 0.135000 |
| 2 | 23 | Tuned spatial-KLA AA | 2.517841 | 4.279256 | 0.123750 |
| 2 | 23 | Neighborhood label-barycenter spatial-KLA AA | 2.073450 | 3.946079 | 0.095000 |
| 2 | 23 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.316627 | 4.157483 | 0.095000 |
| 3 | 24 | Tuned spatial-KLA AA | 2.379811 | 4.253434 | 0.110000 |
| 3 | 24 | Neighborhood label-barycenter spatial-KLA AA | 1.966324 | 3.894093 | 0.080000 |
| 3 | 24 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.273707 | 4.152599 | 0.080000 |
| 4 | 25 | Tuned spatial-KLA AA | 2.497370 | 4.371260 | 0.100000 |
| 4 | 25 | Neighborhood label-barycenter spatial-KLA AA | 2.052412 | 3.945660 | 0.075000 |
| 4 | 25 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.365297 | 4.261396 | 0.075000 |
| 5 | 26 | Tuned spatial-KLA AA | 2.641708 | 3.871210 | 0.198750 |
| 5 | 26 | Neighborhood label-barycenter spatial-KLA AA | 2.281570 | 3.275600 | 0.170000 |
| 5 | 26 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.483206 | 3.731786 | 0.170000 |
| 6 | 27 | Tuned spatial-KLA AA | 2.584214 | 4.242148 | 0.176250 |
| 6 | 27 | Neighborhood label-barycenter spatial-KLA AA | 2.231574 | 3.985388 | 0.150000 |
| 6 | 27 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.475935 | 4.173273 | 0.150000 |
| 7 | 28 | Tuned spatial-KLA AA | 2.571472 | 4.266203 | 0.151250 |
| 7 | 28 | Neighborhood label-barycenter spatial-KLA AA | 2.083989 | 3.831672 | 0.115000 |
| 7 | 28 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.426183 | 4.165764 | 0.115000 |
| 8 | 29 | Tuned spatial-KLA AA | 2.655641 | 4.436852 | 0.153750 |
| 8 | 29 | Neighborhood label-barycenter spatial-KLA AA | 2.252168 | 3.932791 | 0.125000 |
| 8 | 29 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.497194 | 4.185327 | 0.125000 |
| 9 | 30 | Tuned spatial-KLA AA | 2.497151 | 4.391844 | 0.142500 |
| 9 | 30 | Neighborhood label-barycenter spatial-KLA AA | 1.976496 | 3.788699 | 0.095000 |
| 9 | 30 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.257110 | 4.005065 | 0.095000 |
| 10 | 31 | Tuned spatial-KLA AA | 2.506511 | 4.332826 | 0.146250 |
| 10 | 31 | Neighborhood label-barycenter spatial-KLA AA | 2.090752 | 3.911322 | 0.115000 |
| 10 | 31 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.349938 | 4.178951 | 0.115000 |
| 11 | 32 | Tuned spatial-KLA AA | 2.611292 | 4.449109 | 0.132500 |
| 11 | 32 | Neighborhood label-barycenter spatial-KLA AA | 2.121727 | 4.319630 | 0.085000 |
| 11 | 32 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.423917 | 4.576667 | 0.085000 |
| 12 | 33 | Tuned spatial-KLA AA | 2.506738 | 3.967018 | 0.133750 |
| 12 | 33 | Neighborhood label-barycenter spatial-KLA AA | 2.015803 | 3.589798 | 0.110000 |
| 12 | 33 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.369737 | 3.940682 | 0.110000 |
| 13 | 34 | Tuned spatial-KLA AA | 2.753342 | 4.984107 | 0.167500 |
| 13 | 34 | Neighborhood label-barycenter spatial-KLA AA | 2.377463 | 4.888841 | 0.115000 |
| 13 | 34 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.635835 | 5.031827 | 0.115000 |
| 14 | 35 | Tuned spatial-KLA AA | 2.363601 | 4.404697 | 0.092500 |
| 14 | 35 | Neighborhood label-barycenter spatial-KLA AA | 1.929029 | 3.927770 | 0.070000 |
| 14 | 35 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.214278 | 4.153778 | 0.070000 |
| 15 | 36 | Tuned spatial-KLA AA | 2.462846 | 4.004950 | 0.112500 |
| 15 | 36 | Neighborhood label-barycenter spatial-KLA AA | 2.071202 | 3.551458 | 0.085000 |
| 15 | 36 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.302136 | 3.800287 | 0.085000 |
| 16 | 37 | Tuned spatial-KLA AA | 2.646624 | 4.123144 | 0.168750 |
| 16 | 37 | Neighborhood label-barycenter spatial-KLA AA | 2.132612 | 4.004071 | 0.085000 |
| 16 | 37 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.404615 | 4.164206 | 0.085000 |
| 17 | 38 | Tuned spatial-KLA AA | 2.577484 | 3.724929 | 0.168750 |
| 17 | 38 | Neighborhood label-barycenter spatial-KLA AA | 2.160278 | 3.318615 | 0.135000 |
| 17 | 38 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.415619 | 3.590834 | 0.135000 |
| 18 | 39 | Tuned spatial-KLA AA | 2.498167 | 4.967156 | 0.140000 |
| 18 | 39 | Neighborhood label-barycenter spatial-KLA AA | 2.080570 | 4.614165 | 0.095000 |
| 18 | 39 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.365517 | 4.934294 | 0.095000 |
| 19 | 40 | Tuned spatial-KLA AA | 2.698806 | 4.560018 | 0.207500 |
| 19 | 40 | Neighborhood label-barycenter spatial-KLA AA | 2.294842 | 4.058331 | 0.180000 |
| 19 | 40 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.544940 | 4.087930 | 0.180000 |
| 20 | 41 | Tuned spatial-KLA AA | 2.401407 | 4.106483 | 0.127500 |
| 20 | 41 | Neighborhood label-barycenter spatial-KLA AA | 1.996444 | 3.947830 | 0.090000 |
| 20 | 41 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.222329 | 4.140443 | 0.090000 |
| 21 | 42 | Tuned spatial-KLA AA | 2.496545 | 4.348540 | 0.108750 |
| 21 | 42 | Neighborhood label-barycenter spatial-KLA AA | 2.049414 | 3.967671 | 0.085000 |
| 21 | 42 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.360786 | 4.289664 | 0.085000 |
| 22 | 43 | Tuned spatial-KLA AA | 2.279503 | 4.371876 | 0.102500 |
| 22 | 43 | Neighborhood label-barycenter spatial-KLA AA | 1.904801 | 4.259503 | 0.060000 |
| 22 | 43 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.077534 | 4.226343 | 0.060000 |
| 23 | 44 | Tuned spatial-KLA AA | 2.452923 | 4.225147 | 0.098750 |
| 23 | 44 | Neighborhood label-barycenter spatial-KLA AA | 2.067261 | 4.011278 | 0.065000 |
| 23 | 44 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.276661 | 4.273546 | 0.065000 |
| 24 | 45 | Tuned spatial-KLA AA | 2.729407 | 4.261746 | 0.188750 |
| 24 | 45 | Neighborhood label-barycenter spatial-KLA AA | 2.281820 | 4.067193 | 0.130000 |
| 24 | 45 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.543557 | 4.275222 | 0.130000 |
| 25 | 46 | Tuned spatial-KLA AA | 2.435809 | 3.745575 | 0.152500 |
| 25 | 46 | Neighborhood label-barycenter spatial-KLA AA | 2.032898 | 3.268557 | 0.120000 |
| 25 | 46 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.265341 | 3.617973 | 0.120000 |
| 26 | 47 | Tuned spatial-KLA AA | 2.427040 | 4.031679 | 0.118750 |
| 26 | 47 | Neighborhood label-barycenter spatial-KLA AA | 2.001630 | 3.733587 | 0.105000 |
| 26 | 47 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.298675 | 3.968741 | 0.105000 |
| 27 | 48 | Tuned spatial-KLA AA | 2.609964 | 4.758359 | 0.145000 |
| 27 | 48 | Neighborhood label-barycenter spatial-KLA AA | 2.124141 | 4.288629 | 0.105000 |
| 27 | 48 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.429275 | 4.777378 | 0.105000 |
| 28 | 49 | Tuned spatial-KLA AA | 2.444638 | 4.257526 | 0.135000 |
| 28 | 49 | Neighborhood label-barycenter spatial-KLA AA | 1.944952 | 3.830091 | 0.095000 |
| 28 | 49 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.213994 | 4.079955 | 0.095000 |
| 29 | 50 | Tuned spatial-KLA AA | 2.526551 | 4.226216 | 0.155000 |
| 29 | 50 | Neighborhood label-barycenter spatial-KLA AA | 2.111656 | 3.932767 | 0.130000 |
| 29 | 50 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.336976 | 4.189620 | 0.130000 |
| 30 | 51 | Tuned spatial-KLA AA | 2.499206 | 3.837163 | 0.156250 |
| 30 | 51 | Neighborhood label-barycenter spatial-KLA AA | 2.025466 | 3.756976 | 0.080000 |
| 30 | 51 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.313468 | 4.026425 | 0.080000 |
| 31 | 52 | Tuned spatial-KLA AA | 2.579315 | 4.297918 | 0.166250 |
| 31 | 52 | Neighborhood label-barycenter spatial-KLA AA | 2.186002 | 3.907081 | 0.155000 |
| 31 | 52 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.411396 | 4.130586 | 0.155000 |
| 32 | 53 | Tuned spatial-KLA AA | 2.536621 | 4.616864 | 0.117500 |
| 32 | 53 | Neighborhood label-barycenter spatial-KLA AA | 2.138159 | 4.246284 | 0.100000 |
| 32 | 53 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.348199 | 4.467466 | 0.100000 |
| 33 | 54 | Tuned spatial-KLA AA | 2.499767 | 4.204030 | 0.137500 |
| 33 | 54 | Neighborhood label-barycenter spatial-KLA AA | 2.086530 | 3.899045 | 0.095000 |
| 33 | 54 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.362486 | 4.209780 | 0.095000 |
| 34 | 55 | Tuned spatial-KLA AA | 2.511574 | 4.383223 | 0.132500 |
| 34 | 55 | Neighborhood label-barycenter spatial-KLA AA | 2.055007 | 3.958317 | 0.105000 |
| 34 | 55 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.314421 | 4.128531 | 0.105000 |
| 35 | 56 | Tuned spatial-KLA AA | 2.469101 | 4.616807 | 0.120000 |
| 35 | 56 | Neighborhood label-barycenter spatial-KLA AA | 2.026889 | 4.190709 | 0.085000 |
| 35 | 56 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.316682 | 4.365326 | 0.085000 |
| 36 | 57 | Tuned spatial-KLA AA | 2.635534 | 4.525469 | 0.183750 |
| 36 | 57 | Neighborhood label-barycenter spatial-KLA AA | 2.157412 | 4.163233 | 0.125000 |
| 36 | 57 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.449489 | 4.460098 | 0.125000 |
| 37 | 58 | Tuned spatial-KLA AA | 2.590562 | 4.306559 | 0.141250 |
| 37 | 58 | Neighborhood label-barycenter spatial-KLA AA | 2.185700 | 3.983901 | 0.105000 |
| 37 | 58 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.458861 | 4.178967 | 0.105000 |
| 38 | 59 | Tuned spatial-KLA AA | 2.421603 | 3.650579 | 0.143750 |
| 38 | 59 | Neighborhood label-barycenter spatial-KLA AA | 1.982705 | 3.462015 | 0.105000 |
| 38 | 59 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.242879 | 3.750317 | 0.105000 |
| 39 | 60 | Tuned spatial-KLA AA | 2.564443 | 3.959168 | 0.173750 |
| 39 | 60 | Neighborhood label-barycenter spatial-KLA AA | 2.284559 | 4.148358 | 0.165000 |
| 39 | 60 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.477525 | 4.029524 | 0.165000 |
| 40 | 61 | Tuned spatial-KLA AA | 2.547448 | 4.479075 | 0.136250 |
| 40 | 61 | Neighborhood label-barycenter spatial-KLA AA | 2.080850 | 4.032316 | 0.085000 |
| 40 | 61 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.387745 | 4.461073 | 0.085000 |
| 41 | 62 | Tuned spatial-KLA AA | 2.457820 | 3.835198 | 0.160000 |
| 41 | 62 | Neighborhood label-barycenter spatial-KLA AA | 1.961077 | 3.228850 | 0.130000 |
| 41 | 62 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.260271 | 3.514053 | 0.130000 |
| 42 | 63 | Tuned spatial-KLA AA | 2.500964 | 3.410778 | 0.176250 |
| 42 | 63 | Neighborhood label-barycenter spatial-KLA AA | 2.013991 | 2.898578 | 0.155000 |
| 42 | 63 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.342607 | 3.199409 | 0.155000 |
| 43 | 64 | Tuned spatial-KLA AA | 2.480978 | 4.282931 | 0.150000 |
| 43 | 64 | Neighborhood label-barycenter spatial-KLA AA | 2.085842 | 3.915424 | 0.115000 |
| 43 | 64 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.310603 | 4.244362 | 0.115000 |
| 44 | 65 | Tuned spatial-KLA AA | 2.555603 | 4.570121 | 0.125000 |
| 44 | 65 | Neighborhood label-barycenter spatial-KLA AA | 2.131327 | 4.236176 | 0.090000 |
| 44 | 65 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.335870 | 4.399002 | 0.090000 |
| 45 | 66 | Tuned spatial-KLA AA | 2.459420 | 3.457659 | 0.188750 |
| 45 | 66 | Neighborhood label-barycenter spatial-KLA AA | 2.025191 | 2.989715 | 0.160000 |
| 45 | 66 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.279223 | 3.155798 | 0.160000 |
| 46 | 67 | Tuned spatial-KLA AA | 2.461696 | 4.072714 | 0.141250 |
| 46 | 67 | Neighborhood label-barycenter spatial-KLA AA | 2.210486 | 3.950115 | 0.120000 |
| 46 | 67 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.317610 | 3.938595 | 0.120000 |
| 47 | 68 | Tuned spatial-KLA AA | 2.692906 | 4.303790 | 0.191250 |
| 47 | 68 | Neighborhood label-barycenter spatial-KLA AA | 2.173027 | 3.940745 | 0.130000 |
| 47 | 68 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.499243 | 4.150602 | 0.130000 |
| 48 | 69 | Tuned spatial-KLA AA | 2.359492 | 4.049079 | 0.133750 |
| 48 | 69 | Neighborhood label-barycenter spatial-KLA AA | 1.879486 | 3.788089 | 0.090000 |
| 48 | 69 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.158514 | 4.036481 | 0.090000 |
| 49 | 70 | Tuned spatial-KLA AA | 2.503260 | 4.578119 | 0.137500 |
| 49 | 70 | Neighborhood label-barycenter spatial-KLA AA | 2.038852 | 4.027719 | 0.105000 |
| 49 | 70 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.307204 | 4.185405 | 0.105000 |
| 50 | 71 | Tuned spatial-KLA AA | 2.629417 | 4.401855 | 0.156250 |
| 50 | 71 | Neighborhood label-barycenter spatial-KLA AA | 2.145066 | 4.025816 | 0.125000 |
| 50 | 71 | Neighborhood reference-only label-consensus spatial-KLA AA | 2.414496 | 4.396322 | 0.125000 |


## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.523393 | 4.235071 | 0.146025 |
| Neighborhood label-barycenter spatial-KLA AA | 2.092956 | 3.897040 | 0.110500 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 2.354646 | 4.130316 | 0.110500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.523393 +/- 0.098951 | [2.495965, 2.550821] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 2.092956 +/- 0.108494 | [2.062883, 2.123029] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 2.354646 +/- 0.105233 | [2.325477, 2.383815] | 50 |
| Tuned spatial-KLA AA | RMSE | 4.235071 +/- 0.332021 | [4.143039, 4.327102] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 3.897040 +/- 0.363304 | [3.796337, 3.997743] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 4.130316 +/- 0.352282 | [4.032668, 4.227963] | 50 |
| Tuned spatial-KLA AA | CardErr | 0.146025 +/- 0.027533 | [0.138393, 0.153657] | 50 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.110500 +/- 0.028198 | [0.102684, 0.118316] | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.110500 +/- 0.028198 | [0.102684, 0.118316] | 50 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Neighborhood label-barycenter spatial-KLA AA | E-OSPA | 0.430437 +/- 0.055380 | [0.415087, 0.445788] | 17.06% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | E-OSPA | 0.168747 +/- 0.033859 | [0.159362, 0.178132] | 6.69% | 50/50 | 1.776e-15 |
| Neighborhood label-barycenter spatial-KLA AA | RMSE | 0.338031 +/- 0.166511 | [0.291876, 0.384185] | 7.98% | 48/50 | 2.267e-12 |
| Neighborhood reference-only label-consensus spatial-KLA AA | RMSE | 0.104755 +/- 0.137348 | [0.066684, 0.142826] | 2.47% | 38/50 | 0.0003059 |
| Neighborhood label-barycenter spatial-KLA AA | CardErr | 0.035525 +/- 0.014732 | [0.031441, 0.039609] | 24.33% | 50/50 | 1.776e-15 |
| Neighborhood reference-only label-consensus spatial-KLA AA | CardErr | 0.035525 +/- 0.014732 | [0.031441, 0.039609] | 24.33% | 50/50 | 1.776e-15 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 35.938332 +/- 6.446867 | 1.497431 | 1.000x | 50 |
| Neighborhood label-barycenter spatial-KLA AA | 63.180134 +/- 8.780937 | 2.632506 | 1.767x | 50 |
| Neighborhood reference-only label-consensus spatial-KLA AA | 58.998458 +/- 9.403482 | 2.458269 | 1.655x | 50 |
