# AA Balanced/Cardinality Validation

Generated at: 2026-06-04 16:53:01

Comparison order: fixed AA -> covariance-link AA -> Balanced AA -> Cardinality-critical AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 10
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11]
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
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 2: [0.2 0.1 0.1 0.1 0.1 0 0.5 0.5]
- Trial 3: [0.1 0.1 0.1 0.2 0 0.5 0.1 0.5]
- Trial 4: [0.1 0 0.5 0.1 0.5 0.1 0.2 0.1]
- Trial 5: [0.1 0.1 0.5 0.1 0.2 0.1 0.5 0]
- Trial 6: [0.1 0 0.1 0.5 0.5 0.1 0.2 0.1]
- Trial 7: [0 0.1 0.1 0.5 0.1 0.2 0.5 0.1]
- Trial 8: [0.1 0.1 0 0.2 0.5 0.5 0.1 0.1]
- Trial 9: [0.5 0.2 0 0.1 0.1 0.1 0.1 0.5]
- Trial 10: [0.5 0.1 0.1 0.1 0 0.2 0.5 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | fixed AA | 4.100201 | 6.424236 | 0.226250 |
| 1 | 2 | covariance-link AA | 3.608416 | 5.107547 | 0.093750 |
| 1 | 2 | Balanced AA | 3.576556 | 5.224524 | 0.092500 |
| 1 | 2 | Cardinality-critical AA | 3.924917 | 18.310111 | 0.307500 |
| 2 | 3 | fixed AA | 4.105940 | 6.378560 | 0.277500 |
| 2 | 3 | covariance-link AA | 3.639377 | 4.900645 | 0.106250 |
| 2 | 3 | Balanced AA | 3.586802 | 4.960326 | 0.107500 |
| 2 | 3 | Cardinality-critical AA | 3.915631 | 30.624078 | 0.276250 |
| 3 | 4 | fixed AA | 4.131552 | 6.124309 | 0.272500 |
| 3 | 4 | covariance-link AA | 3.671729 | 5.008302 | 0.122500 |
| 3 | 4 | Balanced AA | 3.579184 | 4.661051 | 0.122500 |
| 3 | 4 | Cardinality-critical AA | 3.939652 | 20.516929 | 0.295000 |
| 4 | 5 | fixed AA | 4.137002 | 6.539652 | 0.230000 |
| 4 | 5 | covariance-link AA | 3.637359 | 4.883325 | 0.116250 |
| 4 | 5 | Balanced AA | 3.567558 | 4.786687 | 0.115000 |
| 4 | 5 | Cardinality-critical AA | 3.922313 | 28.025732 | 0.263750 |
| 5 | 6 | fixed AA | 4.096832 | 6.326468 | 0.271250 |
| 5 | 6 | covariance-link AA | 3.579524 | 4.673505 | 0.083750 |
| 5 | 6 | Balanced AA | 3.558789 | 4.505194 | 0.087500 |
| 5 | 6 | Cardinality-critical AA | 4.008923 | 28.967791 | 0.305000 |
| 6 | 7 | fixed AA | 4.174253 | 7.931377 | 0.260000 |
| 6 | 7 | covariance-link AA | 3.574234 | 5.077556 | 0.150000 |
| 6 | 7 | Balanced AA | 3.550862 | 5.098205 | 0.142500 |
| 6 | 7 | Cardinality-critical AA | 3.955999 | 27.681646 | 0.303750 |
| 7 | 8 | fixed AA | 4.085619 | 6.916669 | 0.255000 |
| 7 | 8 | covariance-link AA | 3.638195 | 4.995842 | 0.132500 |
| 7 | 8 | Balanced AA | 3.551226 | 4.971273 | 0.131250 |
| 7 | 8 | Cardinality-critical AA | 3.943505 | 17.019851 | 0.276250 |
| 8 | 9 | fixed AA | 4.116315 | 6.377909 | 0.288750 |
| 8 | 9 | covariance-link AA | 3.614503 | 4.825427 | 0.135000 |
| 8 | 9 | Balanced AA | 3.592521 | 4.573170 | 0.131250 |
| 8 | 9 | Cardinality-critical AA | 3.927727 | 21.750652 | 0.308750 |
| 9 | 10 | fixed AA | 4.182848 | 6.772669 | 0.233750 |
| 9 | 10 | covariance-link AA | 3.567109 | 4.851531 | 0.098750 |
| 9 | 10 | Balanced AA | 3.485966 | 4.510582 | 0.092500 |
| 9 | 10 | Cardinality-critical AA | 3.919846 | 21.553073 | 0.275000 |
| 10 | 11 | fixed AA | 4.147098 | 7.394158 | 0.213750 |
| 10 | 11 | covariance-link AA | 3.481027 | 4.472409 | 0.118750 |
| 10 | 11 | Balanced AA | 3.488928 | 4.583987 | 0.115000 |
| 10 | 11 | Cardinality-critical AA | 4.003778 | 41.736717 | 0.308750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| fixed AA | 4.127766 | 6.718601 | 0.252875 |
| covariance-link AA | 3.601147 | 4.879609 | 0.115750 |
| Balanced AA | 3.553839 | 4.787500 | 0.113750 |
| Cardinality-critical AA | 3.946229 | 25.618658 | 0.292000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | OSPA | 4.127766 +/- 0.032946 | [4.104199, 4.151333] | 10 |
| covariance-link AA | OSPA | 3.601147 +/- 0.053891 | [3.562599, 3.639696] | 10 |
| Balanced AA | OSPA | 3.553839 +/- 0.037698 | [3.526874, 3.580804] | 10 |
| Cardinality-critical AA | OSPA | 3.946229 +/- 0.033976 | [3.921926, 3.970533] | 10 |
| fixed AA | Loc. disag. | 6.718601 +/- 0.560425 | [6.317725, 7.119476] | 10 |
| covariance-link AA | Loc. disag. | 4.879609 +/- 0.192551 | [4.741876, 5.017342] | 10 |
| Balanced AA | Loc. disag. | 4.787500 +/- 0.260532 | [4.601139, 4.973861] | 10 |
| Cardinality-critical AA | Loc. disag. | 25.618658 +/- 7.386061 | [20.335355, 30.901961] | 10 |
| fixed AA | Card. disp. | 0.252875 +/- 0.025386 | [0.234716, 0.271034] | 10 |
| covariance-link AA | Card. disp. | 0.115750 +/- 0.020457 | [0.101117, 0.130383] | 10 |
| Balanced AA | Card. disp. | 0.113750 +/- 0.018708 | [0.100368, 0.127132] | 10 |
| Cardinality-critical AA | Card. disp. | 0.292000 +/- 0.017323 | [0.279609, 0.304391] | 10 |

## Paired Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | OSPA | 0.526619 +/- 0.074357 | [0.473431, 0.579807] | 12.76% | 10/10 | 0.001953 |
| Balanced AA | OSPA | 0.573927 +/- 0.063281 | [0.528662, 0.619193] | 13.90% | 10/10 | 0.001953 |
| Cardinality-critical AA | OSPA | 0.181537 +/- 0.048463 | [0.146871, 0.216203] | 4.40% | 10/10 | 0.001953 |
| covariance-link AA | Loc. disag. | 1.838992 +/- 0.604762 | [1.406401, 2.271582] | 27.37% | 10/10 | 0.001953 |
| Balanced AA | Loc. disag. | 1.931101 +/- 0.555063 | [1.534060, 2.328141] | 28.74% | 10/10 | 0.001953 |
| Cardinality-critical AA | Loc. disag. | -18.900057 +/- 7.186148 | [-24.040361, -13.759754] | -281.31% | 0/10 | 0.001953 |
| covariance-link AA | Card. disp. | 0.137125 +/- 0.028753 | [0.116558, 0.157692] | 54.23% | 10/10 | 0.001953 |
| Balanced AA | Card. disp. | 0.139125 +/- 0.026563 | [0.120125, 0.158125] | 55.02% | 10/10 | 0.001953 |
| Cardinality-critical AA | Card. disp. | -0.039125 +/- 0.029006 | [-0.059873, -0.018377] | -15.47% | 1/10 | 0.02148 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed AA | 3.664654 | 4.586015 | 0.460625 |
| covariance-link AA | 3.751530 | 5.065243 | 0.198750 |
| Balanced AA | 3.716691 | 5.067377 | 0.196500 |
| Cardinality-critical AA | 4.023929 | 6.668521 | 0.630750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | E-OSPA | 3.664654 +/- 0.061388 | [3.620743, 3.708565] | 10 |
| covariance-link AA | E-OSPA | 3.751530 +/- 0.053937 | [3.712949, 3.790111] | 10 |
| Balanced AA | E-OSPA | 3.716691 +/- 0.052613 | [3.679057, 3.754326] | 10 |
| Cardinality-critical AA | E-OSPA | 4.023929 +/- 0.059876 | [3.981100, 4.066759] | 10 |
| fixed AA | RMSE | 4.586015 +/- 0.338126 | [4.344151, 4.827879] | 10 |
| covariance-link AA | RMSE | 5.065243 +/- 0.254017 | [4.883542, 5.246943] | 10 |
| Balanced AA | RMSE | 5.067377 +/- 0.291905 | [4.858575, 5.276179] | 10 |
| Cardinality-critical AA | RMSE | 6.668521 +/- 0.560679 | [6.267464, 7.069579] | 10 |
| fixed AA | CardErr | 0.460625 +/- 0.044391 | [0.428872, 0.492378] | 10 |
| covariance-link AA | CardErr | 0.198750 +/- 0.033468 | [0.174810, 0.222690] | 10 |
| Balanced AA | CardErr | 0.196500 +/- 0.031611 | [0.173889, 0.219111] | 10 |
| Cardinality-critical AA | CardErr | 0.630750 +/- 0.074882 | [0.577186, 0.684314] | 10 |

## Paired Local-Metric Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | E-OSPA | -0.086876 +/- 0.072991 | [-0.139087, -0.034665] | -2.37% | 1/10 | 0.02148 |
| Balanced AA | E-OSPA | -0.052037 +/- 0.055011 | [-0.091387, -0.012688] | -1.42% | 2/10 | 0.1094 |
| Cardinality-critical AA | E-OSPA | -0.359275 +/- 0.050358 | [-0.395297, -0.323253] | -9.80% | 0/10 | 0.001953 |
| covariance-link AA | RMSE | -0.479228 +/- 0.281284 | [-0.680433, -0.278024] | -10.45% | 0/10 | 0.001953 |
| Balanced AA | RMSE | -0.481362 +/- 0.292908 | [-0.690881, -0.271842] | -10.50% | 0/10 | 0.001953 |
| Cardinality-critical AA | RMSE | -2.082507 +/- 0.640152 | [-2.540412, -1.624602] | -45.41% | 0/10 | 0.001953 |
| covariance-link AA | CardErr | 0.261875 +/- 0.029073 | [0.241079, 0.282671] | 56.85% | 10/10 | 0.001953 |
| Balanced AA | CardErr | 0.264125 +/- 0.027029 | [0.244791, 0.283459] | 57.34% | 10/10 | 0.001953 |
| Cardinality-critical AA | CardErr | -0.170125 +/- 0.087867 | [-0.232977, -0.107273] | -36.93% | 0/10 | 0.001953 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed AA | 4.576487 +/- 0.736866 | 0.190687 | 1.000x | 10 |
| covariance-link AA | 4.798933 +/- 0.798607 | 0.199956 | 1.050x | 10 |
| Balanced AA | 5.047111 +/- 1.057611 | 0.210296 | 1.106x | 10 |
| Cardinality-critical AA | 10.722454 +/- 2.446291 | 0.446769 | 2.347x | 10 |
