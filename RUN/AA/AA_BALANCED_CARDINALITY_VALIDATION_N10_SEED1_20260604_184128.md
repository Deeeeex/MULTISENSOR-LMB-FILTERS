# AA Balanced/Cardinality Validation

Generated at: 2026-06-04 18:44:38

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
| 1 | 2 | fixed AA | 3.955368 | 5.379265 | 0.210000 |
| 1 | 2 | covariance-link AA | 3.592113 | 4.881101 | 0.086250 |
| 1 | 2 | Balanced AA | 3.541493 | 4.784986 | 0.088750 |
| 1 | 2 | Cardinality-critical AA | 3.560371 | 4.820196 | 0.086250 |
| 2 | 3 | fixed AA | 3.887244 | 4.863213 | 0.237500 |
| 2 | 3 | covariance-link AA | 3.537371 | 4.409575 | 0.090000 |
| 2 | 3 | Balanced AA | 3.469434 | 4.329636 | 0.090000 |
| 2 | 3 | Cardinality-critical AA | 3.479014 | 4.220782 | 0.090000 |
| 3 | 4 | fixed AA | 3.883928 | 4.997512 | 0.255000 |
| 3 | 4 | covariance-link AA | 3.542705 | 4.394782 | 0.111250 |
| 3 | 4 | Balanced AA | 3.452058 | 4.188438 | 0.102500 |
| 3 | 4 | Cardinality-critical AA | 3.442997 | 4.324012 | 0.100000 |
| 4 | 5 | fixed AA | 3.924193 | 5.277472 | 0.208750 |
| 4 | 5 | covariance-link AA | 3.536959 | 4.489757 | 0.107500 |
| 4 | 5 | Balanced AA | 3.425787 | 4.281359 | 0.107500 |
| 4 | 5 | Cardinality-critical AA | 3.445728 | 4.161946 | 0.091250 |
| 5 | 6 | fixed AA | 3.915405 | 4.789348 | 0.223750 |
| 5 | 6 | covariance-link AA | 3.445662 | 4.197563 | 0.067500 |
| 5 | 6 | Balanced AA | 3.424828 | 4.140342 | 0.070000 |
| 5 | 6 | Cardinality-critical AA | 3.435204 | 4.218390 | 0.060000 |
| 6 | 7 | fixed AA | 3.908527 | 5.318853 | 0.247500 |
| 6 | 7 | covariance-link AA | 3.492533 | 4.462398 | 0.138750 |
| 6 | 7 | Balanced AA | 3.426934 | 4.340386 | 0.135000 |
| 6 | 7 | Cardinality-critical AA | 3.453038 | 4.519392 | 0.126250 |
| 7 | 8 | fixed AA | 3.832756 | 5.397270 | 0.223750 |
| 7 | 8 | covariance-link AA | 3.493135 | 4.503820 | 0.101250 |
| 7 | 8 | Balanced AA | 3.427538 | 4.475541 | 0.098750 |
| 7 | 8 | Cardinality-critical AA | 3.438756 | 4.465303 | 0.105000 |
| 8 | 9 | fixed AA | 3.930662 | 5.177282 | 0.260000 |
| 8 | 9 | covariance-link AA | 3.472151 | 4.161277 | 0.122500 |
| 8 | 9 | Balanced AA | 3.444566 | 4.162382 | 0.123750 |
| 8 | 9 | Cardinality-critical AA | 3.423687 | 4.113854 | 0.110000 |
| 9 | 10 | fixed AA | 3.834951 | 4.671009 | 0.190000 |
| 9 | 10 | covariance-link AA | 3.527837 | 4.323244 | 0.091250 |
| 9 | 10 | Balanced AA | 3.467748 | 4.229633 | 0.085000 |
| 9 | 10 | Cardinality-critical AA | 3.467566 | 4.228334 | 0.081250 |
| 10 | 11 | fixed AA | 3.958543 | 5.425828 | 0.201250 |
| 10 | 11 | covariance-link AA | 3.429339 | 4.182908 | 0.112500 |
| 10 | 11 | Balanced AA | 3.411364 | 4.124606 | 0.115000 |
| 10 | 11 | Cardinality-critical AA | 3.422289 | 4.207020 | 0.098750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| fixed AA | 3.903158 | 5.129705 | 0.225750 |
| covariance-link AA | 3.506981 | 4.400642 | 0.102875 |
| Balanced AA | 3.449175 | 4.305731 | 0.101625 |
| Cardinality-critical AA | 3.456865 | 4.327923 | 0.094875 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | OSPA | 3.903158 +/- 0.043973 | [3.871703, 3.934612] | 10 |
| covariance-link AA | OSPA | 3.506981 +/- 0.049646 | [3.471468, 3.542493] | 10 |
| Balanced AA | OSPA | 3.449175 +/- 0.037771 | [3.422157, 3.476193] | 10 |
| Cardinality-critical AA | OSPA | 3.456865 +/- 0.040474 | [3.427914, 3.485816] | 10 |
| fixed AA | Loc. disag. | 5.129705 +/- 0.278123 | [4.930761, 5.328649] | 10 |
| covariance-link AA | Loc. disag. | 4.400642 +/- 0.212208 | [4.248848, 4.552437] | 10 |
| Balanced AA | Loc. disag. | 4.305731 +/- 0.200318 | [4.162442, 4.449019] | 10 |
| Cardinality-critical AA | Loc. disag. | 4.327923 +/- 0.215836 | [4.173534, 4.482312] | 10 |
| fixed AA | Card. disp. | 0.225750 +/- 0.023719 | [0.208784, 0.242716] | 10 |
| covariance-link AA | Card. disp. | 0.102875 +/- 0.020242 | [0.088396, 0.117354] | 10 |
| Balanced AA | Card. disp. | 0.101625 +/- 0.019446 | [0.087715, 0.115535] | 10 |
| Cardinality-critical AA | Card. disp. | 0.094875 +/- 0.017858 | [0.082101, 0.107649] | 10 |

## Paired Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | OSPA | 0.396177 +/- 0.070589 | [0.345684, 0.446670] | 10.15% | 10/10 | 0.001953 |
| Balanced AA | OSPA | 0.453983 +/- 0.054880 | [0.414726, 0.493239] | 11.63% | 10/10 | 0.001953 |
| Cardinality-critical AA | OSPA | 0.446293 +/- 0.054888 | [0.407031, 0.485555] | 11.43% | 10/10 | 0.001953 |
| covariance-link AA | Loc. disag. | 0.729063 +/- 0.279373 | [0.529225, 0.928900] | 14.21% | 10/10 | 0.001953 |
| Balanced AA | Loc. disag. | 0.823974 +/- 0.266996 | [0.632990, 1.014959] | 16.06% | 10/10 | 0.001953 |
| Cardinality-critical AA | Loc. disag. | 0.801782 +/- 0.266824 | [0.610921, 0.992643] | 15.63% | 10/10 | 0.001953 |
| covariance-link AA | Card. disp. | 0.122875 +/- 0.023057 | [0.106382, 0.139368] | 54.43% | 10/10 | 0.001953 |
| Balanced AA | Card. disp. | 0.124125 +/- 0.023199 | [0.107530, 0.140720] | 54.98% | 10/10 | 0.001953 |
| Cardinality-critical AA | Card. disp. | 0.130875 +/- 0.021263 | [0.115666, 0.146084] | 57.97% | 10/10 | 0.001953 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed AA | 3.543016 | 3.902445 | 0.395750 |
| covariance-link AA | 3.667909 | 4.770102 | 0.179125 |
| Balanced AA | 3.640781 | 4.746664 | 0.178125 |
| Cardinality-critical AA | 3.636789 | 4.796507 | 0.165875 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed AA | E-OSPA | 3.543016 +/- 0.048185 | [3.508549, 3.577484] | 10 |
| covariance-link AA | E-OSPA | 3.667909 +/- 0.067840 | [3.619383, 3.716435] | 10 |
| Balanced AA | E-OSPA | 3.640781 +/- 0.061574 | [3.596736, 3.684825] | 10 |
| Cardinality-critical AA | E-OSPA | 3.636789 +/- 0.068419 | [3.587849, 3.685729] | 10 |
| fixed AA | RMSE | 3.902445 +/- 0.203975 | [3.756540, 4.048349] | 10 |
| covariance-link AA | RMSE | 4.770102 +/- 0.271581 | [4.575838, 4.964366] | 10 |
| Balanced AA | RMSE | 4.746664 +/- 0.272367 | [4.551838, 4.941490] | 10 |
| Cardinality-critical AA | RMSE | 4.796507 +/- 0.252090 | [4.616185, 4.976828] | 10 |
| fixed AA | CardErr | 0.395750 +/- 0.046361 | [0.362587, 0.428913] | 10 |
| covariance-link AA | CardErr | 0.179125 +/- 0.032925 | [0.155574, 0.202676] | 10 |
| Balanced AA | CardErr | 0.178125 +/- 0.031788 | [0.155387, 0.200863] | 10 |
| Cardinality-critical AA | CardErr | 0.165875 +/- 0.030023 | [0.144399, 0.187351] | 10 |

## Paired Local-Metric Improvements Relative to fixed AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| covariance-link AA | E-OSPA | -0.124893 +/- 0.057784 | [-0.166226, -0.083559] | -3.53% | 0/10 | 0.001953 |
| Balanced AA | E-OSPA | -0.097764 +/- 0.048073 | [-0.132151, -0.063377] | -2.76% | 0/10 | 0.001953 |
| Cardinality-critical AA | E-OSPA | -0.093773 +/- 0.051128 | [-0.130345, -0.057200] | -2.65% | 0/10 | 0.001953 |
| covariance-link AA | RMSE | -0.867658 +/- 0.238228 | [-1.038064, -0.697251] | -22.23% | 0/10 | 0.001953 |
| Balanced AA | RMSE | -0.844219 +/- 0.231181 | [-1.009585, -0.678853] | -21.63% | 0/10 | 0.001953 |
| Cardinality-critical AA | RMSE | -0.894062 +/- 0.221951 | [-1.052825, -0.735299] | -22.91% | 0/10 | 0.001953 |
| covariance-link AA | CardErr | 0.216625 +/- 0.029292 | [0.195672, 0.237578] | 54.74% | 10/10 | 0.001953 |
| Balanced AA | CardErr | 0.217625 +/- 0.028930 | [0.196931, 0.238319] | 54.99% | 10/10 | 0.001953 |
| Cardinality-critical AA | CardErr | 0.229875 +/- 0.031691 | [0.207206, 0.252544] | 58.09% | 10/10 | 0.001953 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed AA | 3.718394 +/- 0.196972 | 0.154933 | 1.000x | 10 |
| covariance-link AA | 4.092798 +/- 0.127409 | 0.170533 | 1.102x | 10 |
| Balanced AA | 4.115485 +/- 0.114738 | 0.171479 | 1.109x | 10 |
| Cardinality-critical AA | 5.693226 +/- 0.152618 | 0.237218 | 1.534x | 10 |
