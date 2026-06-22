# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 12:06:09

Comparison order: Tuned spatial-KLA AA -> Support-consensus lifecycle spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- trialSeeds: 2
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
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Support-consensus lifecycle spatial-KLA AA
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
- labelPruningThreshold: 0.010000
- labelPruningMinTrajectoryLength: 1
- labelPruningProtectionMode: support-consensus
- labelPruningMaxOutputGap: NaN
- labelSupportMinEffectiveCount: 2.000
- useAaLabelUncertaintyFusion: 0
- useAaLabelSpatialOverlapWeights: 1
- useAaLabelUncertaintyInflation: 1
- useAaLabelExistenceTempering: 0
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Tuned spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| 1 | 2 | Support-consensus lifecycle spatial-KLA AA | 1.682904 | 1.475046 | 0.018750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.682637 | 1.474567 | 0.018750 |
| Support-consensus lifecycle spatial-KLA AA | 1.682904 | 1.475046 | 0.018750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.682637 +/- 0.000000 | [1.682637, 1.682637] | 1 |
| Support-consensus lifecycle spatial-KLA AA | OSPA | 1.682904 +/- 0.000000 | [1.682904, 1.682904] | 1 |
| Tuned spatial-KLA AA | Loc. disag. | 1.474567 +/- 0.000000 | [1.474567, 1.474567] | 1 |
| Support-consensus lifecycle spatial-KLA AA | Loc. disag. | 1.475046 +/- 0.000000 | [1.475046, 1.475046] | 1 |
| Tuned spatial-KLA AA | Card. disp. | 0.018750 +/- 0.000000 | [0.018750, 0.018750] | 1 |
| Support-consensus lifecycle spatial-KLA AA | Card. disp. | 0.018750 +/- 0.000000 | [0.018750, 0.018750] | 1 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Support-consensus lifecycle spatial-KLA AA | OSPA | -0.000267 +/- 0.000000 | [-0.000267, -0.000267] | -0.02% | 0/1 | 1 |
| Support-consensus lifecycle spatial-KLA AA | Loc. disag. | -0.000479 +/- 0.000000 | [-0.000479, -0.000479] | -0.03% | 0/1 | 1 |
| Support-consensus lifecycle spatial-KLA AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.028910 | 4.144950 | 0.066250 |
| Support-consensus lifecycle spatial-KLA AA | 2.029092 | 4.145636 | 0.066250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.028910 +/- 0.000000 | [2.028910, 2.028910] | 1 |
| Support-consensus lifecycle spatial-KLA AA | E-OSPA | 2.029092 +/- 0.000000 | [2.029092, 2.029092] | 1 |
| Tuned spatial-KLA AA | RMSE | 4.144950 +/- 0.000000 | [4.144950, 4.144950] | 1 |
| Support-consensus lifecycle spatial-KLA AA | RMSE | 4.145636 +/- 0.000000 | [4.145636, 4.145636] | 1 |
| Tuned spatial-KLA AA | CardErr | 0.066250 +/- 0.000000 | [0.066250, 0.066250] | 1 |
| Support-consensus lifecycle spatial-KLA AA | CardErr | 0.066250 +/- 0.000000 | [0.066250, 0.066250] | 1 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Support-consensus lifecycle spatial-KLA AA | E-OSPA | -0.000182 +/- 0.000000 | [-0.000182, -0.000182] | -0.01% | 0/1 | 1 |
| Support-consensus lifecycle spatial-KLA AA | RMSE | -0.000685 +/- 0.000000 | [-0.000685, -0.000685] | -0.02% | 0/1 | 1 |
| Support-consensus lifecycle spatial-KLA AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 53.584239 +/- 0.000000 | 2.232677 | 1.000x | 1 |
| Support-consensus lifecycle spatial-KLA AA | 52.550198 +/- 0.000000 | 2.189592 | 0.981x | 1 |
