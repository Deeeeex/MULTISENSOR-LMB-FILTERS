# AA Balanced/Cardinality Validation

Generated at: 2026-06-04 17:52:17

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
- fidFiaExistenceStrength: 1.000
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

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
| 1 | 2 | Cardinality-critical AA | 3.552481 | 5.082859 | 0.092500 |
| 2 | 3 | fixed AA | 4.105940 | 6.378560 | 0.277500 |
| 2 | 3 | covariance-link AA | 3.639377 | 4.900645 | 0.106250 |
| 2 | 3 | Balanced AA | 3.586802 | 4.960326 | 0.107500 |
| 2 | 3 | Cardinality-critical AA | 3.581055 | 4.708027 | 0.101250 |
| 3 | 4 | fixed AA | 4.131552 | 6.124309 | 0.272500 |
| 3 | 4 | covariance-link AA | 3.671729 | 5.008302 | 0.122500 |
| 3 | 4 | Balanced AA | 3.579184 | 4.661051 | 0.122500 |
| 3 | 4 | Cardinality-critical AA | 3.614006 | 4.960092 | 0.115000 |
| 4 | 5 | fixed AA | 4.137002 | 6.539652 | 0.230000 |
| 4 | 5 | covariance-link AA | 3.637359 | 4.883325 | 0.116250 |
| 4 | 5 | Balanced AA | 3.567558 | 4.786687 | 0.115000 |
| 4 | 5 | Cardinality-critical AA | 3.528715 | 5.382142 | 0.106250 |
| 5 | 6 | fixed AA | 4.096832 | 6.326468 | 0.271250 |
| 5 | 6 | covariance-link AA | 3.579524 | 4.673505 | 0.083750 |
| 5 | 6 | Balanced AA | 3.558789 | 4.505194 | 0.087500 |
| 5 | 6 | Cardinality-critical AA | 3.568331 | 4.658245 | 0.073750 |
| 6 | 7 | fixed AA | 4.174253 | 7.931377 | 0.260000 |
| 6 | 7 | covariance-link AA | 3.574234 | 5.077556 | 0.150000 |
| 6 | 7 | Balanced AA | 3.550862 | 5.098205 | 0.142500 |
| 6 | 7 | Cardinality-critical AA | 3.528608 | 5.037933 | 0.137500 |
| 7 | 8 | fixed AA | 4.085619 | 6.916669 | 0.255000 |
| 7 | 8 | covariance-link AA | 3.638195 | 4.995842 | 0.132500 |
| 7 | 8 | Balanced AA | 3.551226 | 4.971273 | 0.131250 |
| 7 | 8 | Cardinality-critical AA | 3.526714 | 4.868500 | 0.123750 |
| 8 | 9 | fixed AA | 4.116315 | 6.377909 | 0.288750 |
| 8 | 9 | covariance-link AA | 3.614503 | 4.825427 | 0.135000 |
| 8 | 9 | Balanced AA | 3.592521 | 4.573170 | 0.131250 |
| 8 | 9 | Cardinality-critical AA | 3.511039 | 4.344950 | 0.121250 |
| 9 | 10 | fixed AA | 4.182848 | 6.772669 | 0.233750 |
| 9 | 10 | covariance-link AA | 3.567109 | 4.851531 | 0.098750 |
| 9 | 10 | Balanced AA | 3.485966 | 4.510582 | 0.092500 |
| 9 | 10 | Cardinality-critical AA | 3.528551 | 4.837995 | 0.100000 |
| 10 | 11 | fixed AA | 4.147098 | 7.394158 | 0.213750 |
| 10 | 11 | covariance-link AA | 3.481027 | 4.472409 | 0.118750 |
| 10 | 11 | Balanced AA | 3.488928 | 4.583987 | 0.115000 |
| 10 | 11 | Cardinality-critical AA | 3.541038 | 4.842143 | 0.131250 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| fixed AA | 4.127766 | 6.718601 | 0.252875 |
| covariance-link AA | 3.601147 | 4.879609 | 0.115750 |
| Balanced AA | 3.553839 | 4.787500 | 0.113750 |
| Cardinality-critical AA | 3.548054 | 4.872289 | 0.110250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | OSPA | 4.127766 +/- 0.032946 | [4.104199, 4.151333] | 10 |
| covariance-link AA | OSPA | 3.601147 +/- 0.053891 | [3.562599, 3.639696] | 10 |
| Balanced AA | OSPA | 3.553839 +/- 0.037698 | [3.526874, 3.580804] | 10 |
| Cardinality-critical AA | OSPA | 3.548054 +/- 0.031406 | [3.525589, 3.570518] | 10 |
| fixed AA | Loc. disag. | 6.718601 +/- 0.560425 | [6.317725, 7.119476] | 10 |
| covariance-link AA | Loc. disag. | 4.879609 +/- 0.192551 | [4.741876, 5.017342] | 10 |
| Balanced AA | Loc. disag. | 4.787500 +/- 0.260532 | [4.601139, 4.973861] | 10 |
| Cardinality-critical AA | Loc. disag. | 4.872289 +/- 0.277373 | [4.673882, 5.070695] | 10 |
| fixed AA | Card. disp. | 0.252875 +/- 0.025386 | [0.234716, 0.271034] | 10 |
| covariance-link AA | Card. disp. | 0.115750 +/- 0.020457 | [0.101117, 0.130383] | 10 |
| Balanced AA | Card. disp. | 0.113750 +/- 0.018708 | [0.100368, 0.127132] | 10 |
| Cardinality-critical AA | Card. disp. | 0.110250 +/- 0.019318 | [0.096432, 0.124068] | 10 |

## Paired Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | OSPA | 0.526619 +/- 0.074357 | [0.473431, 0.579807] | 12.76% | 10/10 | 0.001953 |
| Balanced AA | OSPA | 0.573927 +/- 0.063281 | [0.528662, 0.619193] | 13.90% | 10/10 | 0.001953 |
| Cardinality-critical AA | OSPA | 0.579712 +/- 0.050579 | [0.543533, 0.615892] | 14.04% | 10/10 | 0.001953 |
| covariance-link AA | Loc. disag. | 1.838992 +/- 0.604762 | [1.406401, 2.271582] | 27.37% | 10/10 | 0.001953 |
| Balanced AA | Loc. disag. | 1.931101 +/- 0.555063 | [1.534060, 2.328141] | 28.74% | 10/10 | 0.001953 |
| Cardinality-critical AA | Loc. disag. | 1.846312 +/- 0.571078 | [1.437816, 2.254808] | 27.48% | 10/10 | 0.001953 |
| covariance-link AA | Card. disp. | 0.137125 +/- 0.028753 | [0.116558, 0.157692] | 54.23% | 10/10 | 0.001953 |
| Balanced AA | Card. disp. | 0.139125 +/- 0.026563 | [0.120125, 0.158125] | 55.02% | 10/10 | 0.001953 |
| Cardinality-critical AA | Card. disp. | 0.142625 +/- 0.032737 | [0.119208, 0.166042] | 56.40% | 10/10 | 0.001953 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed AA | 3.664654 | 4.586015 | 0.460625 |
| covariance-link AA | 3.751530 | 5.065243 | 0.198750 |
| Balanced AA | 3.716691 | 5.067377 | 0.196500 |
| Cardinality-critical AA | 3.715284 | 5.199835 | 0.187000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | E-OSPA | 3.664654 +/- 0.061388 | [3.620743, 3.708565] | 10 |
| covariance-link AA | E-OSPA | 3.751530 +/- 0.053937 | [3.712949, 3.790111] | 10 |
| Balanced AA | E-OSPA | 3.716691 +/- 0.052613 | [3.679057, 3.754326] | 10 |
| Cardinality-critical AA | E-OSPA | 3.715284 +/- 0.044655 | [3.683342, 3.747225] | 10 |
| fixed AA | RMSE | 4.586015 +/- 0.338126 | [4.344151, 4.827879] | 10 |
| covariance-link AA | RMSE | 5.065243 +/- 0.254017 | [4.883542, 5.246943] | 10 |
| Balanced AA | RMSE | 5.067377 +/- 0.291905 | [4.858575, 5.276179] | 10 |
| Cardinality-critical AA | RMSE | 5.199835 +/- 0.349862 | [4.949576, 5.450094] | 10 |
| fixed AA | CardErr | 0.460625 +/- 0.044391 | [0.428872, 0.492378] | 10 |
| covariance-link AA | CardErr | 0.198750 +/- 0.033468 | [0.174810, 0.222690] | 10 |
| Balanced AA | CardErr | 0.196500 +/- 0.031611 | [0.173889, 0.219111] | 10 |
| Cardinality-critical AA | CardErr | 0.187000 +/- 0.032168 | [0.163990, 0.210010] | 10 |

## Paired Local-Metric Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | E-OSPA | -0.086876 +/- 0.072991 | [-0.139087, -0.034665] | -2.37% | 1/10 | 0.02148 |
| Balanced AA | E-OSPA | -0.052037 +/- 0.055011 | [-0.091387, -0.012688] | -1.42% | 2/10 | 0.1094 |
| Cardinality-critical AA | E-OSPA | -0.050630 +/- 0.053811 | [-0.089121, -0.012138] | -1.38% | 1/10 | 0.02148 |
| covariance-link AA | RMSE | -0.479228 +/- 0.281284 | [-0.680433, -0.278024] | -10.45% | 0/10 | 0.001953 |
| Balanced AA | RMSE | -0.481362 +/- 0.292908 | [-0.690881, -0.271842] | -10.50% | 0/10 | 0.001953 |
| Cardinality-critical AA | RMSE | -0.613820 +/- 0.390997 | [-0.893503, -0.334137] | -13.38% | 0/10 | 0.001953 |
| covariance-link AA | CardErr | 0.261875 +/- 0.029073 | [0.241079, 0.282671] | 56.85% | 10/10 | 0.001953 |
| Balanced AA | CardErr | 0.264125 +/- 0.027029 | [0.244791, 0.283459] | 57.34% | 10/10 | 0.001953 |
| Cardinality-critical AA | CardErr | 0.273625 +/- 0.030118 | [0.252081, 0.295169] | 59.40% | 10/10 | 0.001953 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed AA | 4.646147 +/- 0.994652 | 0.193589 | 1.000x | 10 |
| covariance-link AA | 5.131582 +/- 1.196629 | 0.213816 | 1.111x | 10 |
| Balanced AA | 5.190918 +/- 1.173739 | 0.216288 | 1.125x | 10 |
| Cardinality-critical AA | 7.673924 +/- 1.642755 | 0.319747 | 1.702x | 10 |
