# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:20:58

Comparison order: fixed AA -> PD target-wise AA -> FI target-wise AA

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
- existenceThreshold: 0.030000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### fixed AA
- enabled: 0
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

### PD target-wise AA
- enabled: 1
- method: pdWeightedGa
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

### FI target-wise AA
- enabled: 1
- method: fiTraceGa
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | fixed AA | 3.925029 | 5.943826 | 0.217500 |
| 1 | 2 | PD target-wise AA | 3.764478 | 6.034736 | 0.178750 |
| 1 | 2 | FI target-wise AA | 3.841703 | 5.918081 | 0.188750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| fixed AA | 3.925029 | 5.943826 | 0.217500 |
| PD target-wise AA | 3.764478 | 6.034736 | 0.178750 |
| FI target-wise AA | 3.841703 | 5.918081 | 0.188750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | OSPA | 3.925029 +/- 0.000000 | [3.925029, 3.925029] | 1 |
| PD target-wise AA | OSPA | 3.764478 +/- 0.000000 | [3.764478, 3.764478] | 1 |
| FI target-wise AA | OSPA | 3.841703 +/- 0.000000 | [3.841703, 3.841703] | 1 |
| fixed AA | Loc. disag. | 5.943826 +/- 0.000000 | [5.943826, 5.943826] | 1 |
| PD target-wise AA | Loc. disag. | 6.034736 +/- 0.000000 | [6.034736, 6.034736] | 1 |
| FI target-wise AA | Loc. disag. | 5.918081 +/- 0.000000 | [5.918081, 5.918081] | 1 |
| fixed AA | Card. disp. | 0.217500 +/- 0.000000 | [0.217500, 0.217500] | 1 |
| PD target-wise AA | Card. disp. | 0.178750 +/- 0.000000 | [0.178750, 0.178750] | 1 |
| FI target-wise AA | Card. disp. | 0.188750 +/- 0.000000 | [0.188750, 0.188750] | 1 |

## Paired Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD target-wise AA | OSPA | 0.160551 +/- 0.000000 | [0.160551, 0.160551] | 4.09% | 1/1 | 1 |
| FI target-wise AA | OSPA | 0.083326 +/- 0.000000 | [0.083326, 0.083326] | 2.12% | 1/1 | 1 |
| PD target-wise AA | Loc. disag. | -0.090910 +/- 0.000000 | [-0.090910, -0.090910] | -1.53% | 0/1 | 1 |
| FI target-wise AA | Loc. disag. | 0.025745 +/- 0.000000 | [0.025745, 0.025745] | 0.43% | 1/1 | 1 |
| PD target-wise AA | Card. disp. | 0.038750 +/- 0.000000 | [0.038750, 0.038750] | 17.82% | 1/1 | 1 |
| FI target-wise AA | Card. disp. | 0.028750 +/- 0.000000 | [0.028750, 0.028750] | 13.22% | 1/1 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed AA | 3.554232 | 4.116995 | 0.415000 |
| PD target-wise AA | 3.898223 | 4.559288 | 0.391250 |
| FI target-wise AA | 3.913401 | 5.304573 | 0.266250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | E-OSPA | 3.554232 +/- 0.000000 | [3.554232, 3.554232] | 1 |
| PD target-wise AA | E-OSPA | 3.898223 +/- 0.000000 | [3.898223, 3.898223] | 1 |
| FI target-wise AA | E-OSPA | 3.913401 +/- 0.000000 | [3.913401, 3.913401] | 1 |
| fixed AA | RMSE | 4.116995 +/- 0.000000 | [4.116995, 4.116995] | 1 |
| PD target-wise AA | RMSE | 4.559288 +/- 0.000000 | [4.559288, 4.559288] | 1 |
| FI target-wise AA | RMSE | 5.304573 +/- 0.000000 | [5.304573, 5.304573] | 1 |
| fixed AA | CardErr | 0.415000 +/- 0.000000 | [0.415000, 0.415000] | 1 |
| PD target-wise AA | CardErr | 0.391250 +/- 0.000000 | [0.391250, 0.391250] | 1 |
| FI target-wise AA | CardErr | 0.266250 +/- 0.000000 | [0.266250, 0.266250] | 1 |

## Paired Local-Metric Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD target-wise AA | E-OSPA | -0.343992 +/- 0.000000 | [-0.343992, -0.343992] | -9.68% | 0/1 | 1 |
| FI target-wise AA | E-OSPA | -0.359169 +/- 0.000000 | [-0.359169, -0.359169] | -10.11% | 0/1 | 1 |
| PD target-wise AA | RMSE | -0.442292 +/- 0.000000 | [-0.442292, -0.442292] | -10.74% | 0/1 | 1 |
| FI target-wise AA | RMSE | -1.187578 +/- 0.000000 | [-1.187578, -1.187578] | -28.85% | 0/1 | 1 |
| PD target-wise AA | CardErr | 0.023750 +/- 0.000000 | [0.023750, 0.023750] | 5.72% | 1/1 | 1 |
| FI target-wise AA | CardErr | 0.148750 +/- 0.000000 | [0.148750, 0.148750] | 35.84% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed AA | 74.078193 +/- 0.000000 | 3.086591 | 1.000x | 1 |
| PD target-wise AA | 87.867168 +/- 0.000000 | 3.661132 | 1.186x | 1 |
| FI target-wise AA | 110.506821 +/- 0.000000 | 4.604451 | 1.492x | 1 |
