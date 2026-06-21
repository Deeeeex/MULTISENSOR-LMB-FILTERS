# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:04:34

Comparison order: fixed AA -> covariance-link AA -> Balanced AA -> Cardinality-critical AA

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

### covariance-link AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
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

### Balanced AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

### Cardinality-critical AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 1
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 1.000
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | fixed AA | 3.925029 | 5.943826 | 0.217500 |
| 1 | 2 | covariance-link AA | 3.445555 | 4.219105 | 0.100000 |
| 1 | 2 | Balanced AA | 3.403461 | 4.198176 | 0.096250 |
| 1 | 2 | Cardinality-critical AA | 3.362122 | 4.124819 | 0.095000 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| fixed AA | 3.925029 | 5.943826 | 0.217500 |
| covariance-link AA | 3.445555 | 4.219105 | 0.100000 |
| Balanced AA | 3.403461 | 4.198176 | 0.096250 |
| Cardinality-critical AA | 3.362122 | 4.124819 | 0.095000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | OSPA | 3.925029 +/- 0.000000 | [3.925029, 3.925029] | 1 |
| covariance-link AA | OSPA | 3.445555 +/- 0.000000 | [3.445555, 3.445555] | 1 |
| Balanced AA | OSPA | 3.403461 +/- 0.000000 | [3.403461, 3.403461] | 1 |
| Cardinality-critical AA | OSPA | 3.362122 +/- 0.000000 | [3.362122, 3.362122] | 1 |
| fixed AA | Loc. disag. | 5.943826 +/- 0.000000 | [5.943826, 5.943826] | 1 |
| covariance-link AA | Loc. disag. | 4.219105 +/- 0.000000 | [4.219105, 4.219105] | 1 |
| Balanced AA | Loc. disag. | 4.198176 +/- 0.000000 | [4.198176, 4.198176] | 1 |
| Cardinality-critical AA | Loc. disag. | 4.124819 +/- 0.000000 | [4.124819, 4.124819] | 1 |
| fixed AA | Card. disp. | 0.217500 +/- 0.000000 | [0.217500, 0.217500] | 1 |
| covariance-link AA | Card. disp. | 0.100000 +/- 0.000000 | [0.100000, 0.100000] | 1 |
| Balanced AA | Card. disp. | 0.096250 +/- 0.000000 | [0.096250, 0.096250] | 1 |
| Cardinality-critical AA | Card. disp. | 0.095000 +/- 0.000000 | [0.095000, 0.095000] | 1 |

## Paired Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | OSPA | 0.479474 +/- 0.000000 | [0.479474, 0.479474] | 12.22% | 1/1 | 1 |
| Balanced AA | OSPA | 0.521568 +/- 0.000000 | [0.521568, 0.521568] | 13.29% | 1/1 | 1 |
| Cardinality-critical AA | OSPA | 0.562907 +/- 0.000000 | [0.562907, 0.562907] | 14.34% | 1/1 | 1 |
| covariance-link AA | Loc. disag. | 1.724721 +/- 0.000000 | [1.724721, 1.724721] | 29.02% | 1/1 | 1 |
| Balanced AA | Loc. disag. | 1.745651 +/- 0.000000 | [1.745651, 1.745651] | 29.37% | 1/1 | 1 |
| Cardinality-critical AA | Loc. disag. | 1.819008 +/- 0.000000 | [1.819008, 1.819008] | 30.60% | 1/1 | 1 |
| covariance-link AA | Card. disp. | 0.117500 +/- 0.000000 | [0.117500, 0.117500] | 54.02% | 1/1 | 1 |
| Balanced AA | Card. disp. | 0.121250 +/- 0.000000 | [0.121250, 0.121250] | 55.75% | 1/1 | 1 |
| Cardinality-critical AA | Card. disp. | 0.122500 +/- 0.000000 | [0.122500, 0.122500] | 56.32% | 1/1 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed AA | 3.554232 | 4.116995 | 0.415000 |
| covariance-link AA | 3.636363 | 4.814045 | 0.165000 |
| Balanced AA | 3.618801 | 4.795842 | 0.161250 |
| Cardinality-critical AA | 3.605280 | 4.883557 | 0.155000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | E-OSPA | 3.554232 +/- 0.000000 | [3.554232, 3.554232] | 1 |
| covariance-link AA | E-OSPA | 3.636363 +/- 0.000000 | [3.636363, 3.636363] | 1 |
| Balanced AA | E-OSPA | 3.618801 +/- 0.000000 | [3.618801, 3.618801] | 1 |
| Cardinality-critical AA | E-OSPA | 3.605280 +/- 0.000000 | [3.605280, 3.605280] | 1 |
| fixed AA | RMSE | 4.116995 +/- 0.000000 | [4.116995, 4.116995] | 1 |
| covariance-link AA | RMSE | 4.814045 +/- 0.000000 | [4.814045, 4.814045] | 1 |
| Balanced AA | RMSE | 4.795842 +/- 0.000000 | [4.795842, 4.795842] | 1 |
| Cardinality-critical AA | RMSE | 4.883557 +/- 0.000000 | [4.883557, 4.883557] | 1 |
| fixed AA | CardErr | 0.415000 +/- 0.000000 | [0.415000, 0.415000] | 1 |
| covariance-link AA | CardErr | 0.165000 +/- 0.000000 | [0.165000, 0.165000] | 1 |
| Balanced AA | CardErr | 0.161250 +/- 0.000000 | [0.161250, 0.161250] | 1 |
| Cardinality-critical AA | CardErr | 0.155000 +/- 0.000000 | [0.155000, 0.155000] | 1 |

## Paired Local-Metric Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | E-OSPA | -0.082131 +/- 0.000000 | [-0.082131, -0.082131] | -2.31% | 0/1 | 1 |
| Balanced AA | E-OSPA | -0.064570 +/- 0.000000 | [-0.064570, -0.064570] | -1.82% | 0/1 | 1 |
| Cardinality-critical AA | E-OSPA | -0.051049 +/- 0.000000 | [-0.051049, -0.051049] | -1.44% | 0/1 | 1 |
| covariance-link AA | RMSE | -0.697050 +/- 0.000000 | [-0.697050, -0.697050] | -16.93% | 0/1 | 1 |
| Balanced AA | RMSE | -0.678846 +/- 0.000000 | [-0.678846, -0.678846] | -16.49% | 0/1 | 1 |
| Cardinality-critical AA | RMSE | -0.766561 +/- 0.000000 | [-0.766561, -0.766561] | -18.62% | 0/1 | 1 |
| covariance-link AA | CardErr | 0.250000 +/- 0.000000 | [0.250000, 0.250000] | 60.24% | 1/1 | 1 |
| Balanced AA | CardErr | 0.253750 +/- 0.000000 | [0.253750, 0.253750] | 61.14% | 1/1 | 1 |
| Cardinality-critical AA | CardErr | 0.260000 +/- 0.000000 | [0.260000, 0.260000] | 62.65% | 1/1 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed AA | 59.890599 +/- 0.000000 | 2.495442 | 1.000x | 1 |
| covariance-link AA | 68.579557 +/- 0.000000 | 2.857482 | 1.145x | 1 |
| Balanced AA | 69.783946 +/- 0.000000 | 2.907664 | 1.165x | 1 |
| Cardinality-critical AA | 99.329868 +/- 0.000000 | 4.138744 | 1.659x | 1 |
