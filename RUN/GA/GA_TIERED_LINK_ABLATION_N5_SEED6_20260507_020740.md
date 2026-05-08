# GA Tiered Link Ablation (2026-05-07 02:07:40)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 6 (fixed=1)
- trialSeeds: [7 8 9 10 11]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: structureAwareDecoupledKla

## Arm Configs
### fixed weights
- enabled: 0
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250

### +covariance
- enabled: 1
- useCovariance: 1
- useLinkQuality: 0
- useExistenceConfidence: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250

### +link quality
- enabled: 1
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250

### +existence confidence
- enabled: 1
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250

### +structure-aware decoupled KLA
- enabled: 1
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250

## Per-Trial pDropBySensor
- Trial 1: [0 0.2 0.1 0.5 0.1 0.5 0.1 0.1]
- Trial 2: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 3: [0.5 0.1 0.2 0.1 0.5 0.1 0.1 0]
- Trial 4: [0.5 0.1 0 0.1 0.1 0.5 0.2 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.5 0.1 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 7 | fixed weights | 2.541049 | 2.575987 | 0.747500 |
| 1 | 7 | +covariance | 2.101247 | 2.137851 | 0.448750 |
| 1 | 7 | +link quality | 1.842867 | 1.861863 | 0.210000 |
| 1 | 7 | +existence confidence | 1.825836 | 1.828658 | 0.205000 |
| 1 | 7 | +structure-aware decoupled KLA | 1.823612 | 1.828964 | 0.205000 |
| 2 | 8 | fixed weights | 2.386825 | 2.412256 | 0.563750 |
| 2 | 8 | +covariance | 2.015325 | 2.036865 | 0.400000 |
| 2 | 8 | +link quality | 1.750055 | 1.509791 | 0.178750 |
| 2 | 8 | +existence confidence | 1.779008 | 1.507649 | 0.185000 |
| 2 | 8 | +structure-aware decoupled KLA | 1.762642 | 1.481461 | 0.185000 |
| 3 | 9 | fixed weights | 2.456704 | 2.596131 | 0.663750 |
| 3 | 9 | +covariance | 1.970587 | 2.020098 | 0.382500 |
| 3 | 9 | +link quality | 1.763240 | 1.540145 | 0.172500 |
| 3 | 9 | +existence confidence | 1.768565 | 1.502216 | 0.175000 |
| 3 | 9 | +structure-aware decoupled KLA | 1.756928 | 1.546963 | 0.172500 |
| 4 | 10 | fixed weights | 2.571992 | 3.155163 | 0.736250 |
| 4 | 10 | +covariance | 2.231084 | 2.152015 | 0.563750 |
| 4 | 10 | +link quality | 1.837096 | 2.446852 | 0.218750 |
| 4 | 10 | +existence confidence | 1.845660 | 2.469694 | 0.220000 |
| 4 | 10 | +structure-aware decoupled KLA | 1.823264 | 2.505816 | 0.215000 |
| 5 | 11 | fixed weights | 2.175224 | 2.634355 | 0.416250 |
| 5 | 11 | +covariance | 1.923264 | 2.047288 | 0.347500 |
| 5 | 11 | +link quality | 1.818784 | 1.608326 | 0.205000 |
| 5 | 11 | +existence confidence | 1.822877 | 1.594128 | 0.205000 |
| 5 | 11 | +structure-aware decoupled KLA | 1.831138 | 1.611837 | 0.212500 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.426359 | 2.674778 | 0.625500 |
| +covariance | 2.048302 | 2.078823 | 0.428500 |
| +link quality | 1.802408 | 1.793396 | 0.197000 |
| +existence confidence | 1.808389 | 1.780469 | 0.198000 |
| +structure-aware decoupled KLA | 1.799517 | 1.795008 | 0.198000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.426359 +/- 0.158047 | [2.230149, 2.622569] | 5 |
| +covariance | OSPA | 2.048302 +/- 0.121423 | [1.897559, 2.199044] | 5 |
| +link quality | OSPA | 1.802408 +/- 0.042963 | [1.749071, 1.855746] | 5 |
| +existence confidence | OSPA | 1.808389 +/- 0.032987 | [1.767436, 1.849342] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.799517 +/- 0.036462 | [1.754250, 1.844783] | 5 |
| fixed weights | RMSE | 2.674778 +/- 0.281632 | [2.325142, 3.024415] | 5 |
| +covariance | RMSE | 2.078823 +/- 0.061329 | [2.002686, 2.154961] | 5 |
| +link quality | RMSE | 1.793396 +/- 0.390676 | [1.308385, 2.278407] | 5 |
| +existence confidence | RMSE | 1.780469 +/- 0.407410 | [1.274684, 2.286254] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.795008 +/- 0.418268 | [1.275744, 2.314273] | 5 |
| fixed weights | Cardinality | 0.625500 +/- 0.138032 | [0.454138, 0.796862] | 5 |
| +covariance | Cardinality | 0.428500 +/- 0.083952 | [0.324276, 0.532724] | 5 |
| +link quality | Cardinality | 0.197000 +/- 0.020245 | [0.171867, 0.222133] | 5 |
| +existence confidence | Cardinality | 0.198000 +/- 0.017889 | [0.175792, 0.220208] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.198000 +/- 0.018490 | [0.175045, 0.220955] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.378057 +/- 0.090572 | [0.265615, 0.490499] | 15.58% | 5/5 | 0.0625 |
| +link quality | OSPA | 0.623951 +/- 0.153604 | [0.433256, 0.814645] | 25.72% | 5/5 | 0.0625 |
| +existence confidence | OSPA | 0.617970 +/- 0.155548 | [0.424863, 0.811077] | 25.47% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | OSPA | 0.626842 +/- 0.164576 | [0.422527, 0.831158] | 25.83% | 5/5 | 0.0625 |
| +covariance | RMSE | 0.595955 +/- 0.244866 | [0.291963, 0.899947] | 22.28% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.881383 +/- 0.165667 | [0.675712, 1.087053] | 32.95% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.894309 +/- 0.177805 | [0.673571, 1.115048] | 33.43% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.879770 +/- 0.174923 | [0.662609, 1.096932] | 32.89% | 5/5 | 0.0625 |
| +covariance | Cardinality | 0.197000 +/- 0.094346 | [0.079872, 0.314128] | 31.49% | 5/5 | 0.0625 |
| +link quality | Cardinality | 0.428500 +/- 0.134935 | [0.260983, 0.596017] | 68.51% | 5/5 | 0.0625 |
| +existence confidence | Cardinality | 0.427500 +/- 0.136009 | [0.258650, 0.596350] | 68.35% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.427500 +/- 0.140106 | [0.253563, 0.601437] | 68.35% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.884983 | 0.500000 | 1.622667 | 1.562000 |
| +covariance | 2.707969 | 0.500000 | 1.527981 | 1.292500 |
| +link quality | 2.373367 | 0.500000 | 1.604462 | 0.698500 |
| +existence confidence | 2.376126 | 0.500000 | 1.606453 | 0.696000 |
| +structure-aware decoupled KLA | 2.381163 | 0.500000 | 1.613118 | 0.698500 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.884983 +/- 0.116558 | [2.740281, 3.029685] | 5 |
| +covariance | E-OSPA | 2.707969 +/- 0.130386 | [2.546099, 2.869839] | 5 |
| +link quality | E-OSPA | 2.373367 +/- 0.117621 | [2.227345, 2.519389] | 5 |
| +existence confidence | E-OSPA | 2.376126 +/- 0.111146 | [2.238142, 2.514109] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.381163 +/- 0.111846 | [2.242310, 2.520016] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +covariance | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +link quality | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| +existence confidence | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.622667 +/- 0.075367 | [1.529102, 1.716232] | 5 |
| +covariance | RMSE | 1.527981 +/- 0.075934 | [1.433711, 1.622250] | 5 |
| +link quality | RMSE | 1.604462 +/- 0.080871 | [1.504063, 1.704861] | 5 |
| +existence confidence | RMSE | 1.606453 +/- 0.082244 | [1.504349, 1.708556] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.613118 +/- 0.084517 | [1.508193, 1.718043] | 5 |
| fixed weights | CardErr | 1.562000 +/- 0.222734 | [1.285483, 1.838517] | 5 |
| +covariance | CardErr | 1.292500 +/- 0.201799 | [1.041974, 1.543026] | 5 |
| +link quality | CardErr | 0.698500 +/- 0.100745 | [0.573429, 0.823571] | 5 |
| +existence confidence | CardErr | 0.696000 +/- 0.099916 | [0.571958, 0.820042] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.698500 +/- 0.099209 | [0.575335, 0.821665] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.177014 +/- 0.027734 | [0.142584, 0.211445] | 6.14% | 5/5 | 0.0625 |
| +link quality | E-OSPA | 0.511616 +/- 0.080054 | [0.412232, 0.610999] | 17.73% | 5/5 | 0.0625 |
| +existence confidence | E-OSPA | 0.508857 +/- 0.081549 | [0.407617, 0.610097] | 17.64% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | E-OSPA | 0.503820 +/- 0.082750 | [0.401088, 0.606552] | 17.46% | 5/5 | 0.0625 |
| +covariance | H-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |
| +link quality | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +existence confidence | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000000] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +covariance | RMSE | 0.094686 +/- 0.031937 | [0.055038, 0.134335] | 5.84% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.018204 +/- 0.042057 | [-0.034008, 0.070417] | 1.12% | 3/5 | 1 |
| +existence confidence | RMSE | 0.016214 +/- 0.042872 | [-0.037010, 0.069438] | 1.00% | 3/5 | 1 |
| +structure-aware decoupled KLA | RMSE | 0.009549 +/- 0.044942 | [-0.046246, 0.065343] | 0.59% | 3/5 | 1 |
| +covariance | CardErr | 0.269500 +/- 0.063858 | [0.190223, 0.348777] | 17.25% | 5/5 | 0.0625 |
| +link quality | CardErr | 0.863500 +/- 0.190729 | [0.626716, 1.100284] | 55.28% | 5/5 | 0.0625 |
| +existence confidence | CardErr | 0.866000 +/- 0.194857 | [0.624092, 1.107908] | 55.44% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | CardErr | 0.863500 +/- 0.196684 | [0.619323, 1.107677] | 55.28% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.928414 | 0.500000 | 1.582825 | 1.488000 |
| 1 | +covariance | 2.647539 | 0.500000 | 1.500620 | 1.098000 |
| 1 | +link quality | 2.415947 | 0.500000 | 1.556254 | 0.784000 |
| 1 | +existence confidence | 2.403092 | 0.500000 | 1.563632 | 0.776000 |
| 1 | +structure-aware decoupled KLA | 2.418283 | 0.500000 | 1.571219 | 0.782000 |
| 2 | fixed weights | 2.702078 | 0.500000 | 1.537237 | 1.150000 |
| 2 | +covariance | 2.613432 | 0.500000 | 1.476546 | 1.068000 |
| 2 | +link quality | 2.339637 | 0.500000 | 1.557110 | 0.714000 |
| 2 | +existence confidence | 2.338099 | 0.500000 | 1.556909 | 0.712000 |
| 2 | +structure-aware decoupled KLA | 2.343667 | 0.500000 | 1.562058 | 0.712000 |
| 3 | fixed weights | 2.671070 | 0.500000 | 1.516722 | 1.158000 |
| 3 | +covariance | 2.518254 | 0.500000 | 1.411872 | 1.014000 |
| 3 | +link quality | 2.247355 | 0.500000 | 1.490100 | 0.678000 |
| 3 | +existence confidence | 2.254054 | 0.500000 | 1.490163 | 0.682000 |
| 3 | +structure-aware decoupled KLA | 2.259115 | 0.500000 | 1.502414 | 0.680000 |
| 4 | fixed weights | 2.774375 | 0.500000 | 1.615440 | 1.366000 |
| 4 | +covariance | 2.720259 | 0.500000 | 1.504535 | 1.306000 |
| 4 | +link quality | 2.404470 | 0.500000 | 1.588902 | 0.800000 |
| 4 | +existence confidence | 2.405663 | 0.500000 | 1.590894 | 0.798000 |
| 4 | +structure-aware decoupled KLA | 2.409330 | 0.500000 | 1.596272 | 0.798000 |
| 5 | fixed weights | 3.103272 | 0.500000 | 1.636035 | 1.922000 |
| 5 | +covariance | 2.956904 | 0.500000 | 1.550905 | 1.806000 |
| 5 | +link quality | 2.500284 | 0.500000 | 1.683318 | 0.798000 |
| 5 | +existence confidence | 2.505621 | 0.500000 | 1.682830 | 0.796000 |
| 5 | +structure-aware decoupled KLA | 2.501598 | 0.500000 | 1.690026 | 0.796000 |
| 6 | fixed weights | 3.002259 | 0.500000 | 1.737740 | 1.724000 |
| 6 | +covariance | 2.783720 | 0.500000 | 1.639842 | 1.362000 |
| 6 | +link quality | 2.368069 | 0.500000 | 1.657501 | 0.564000 |
| 6 | +existence confidence | 2.374038 | 0.500000 | 1.658867 | 0.560000 |
| 6 | +structure-aware decoupled KLA | 2.386975 | 0.500000 | 1.670172 | 0.560000 |
| 7 | fixed weights | 2.844626 | 0.500000 | 1.664797 | 1.464000 |
| 7 | +covariance | 2.725836 | 0.500000 | 1.552185 | 1.352000 |
| 7 | +link quality | 2.396990 | 0.500000 | 1.637294 | 0.736000 |
| 7 | +existence confidence | 2.418105 | 0.500000 | 1.638420 | 0.740000 |
| 7 | +structure-aware decoupled KLA | 2.412332 | 0.500000 | 1.645451 | 0.744000 |
| 8 | fixed weights | 3.053768 | 0.500000 | 1.690539 | 2.224000 |
| 8 | +covariance | 2.697804 | 0.500000 | 1.587339 | 1.334000 |
| 8 | +link quality | 2.314186 | 0.499999 | 1.665221 | 0.514000 |
| 8 | +existence confidence | 2.310334 | 0.499999 | 1.669906 | 0.504000 |
| 8 | +structure-aware decoupled KLA | 2.318003 | 0.499999 | 1.667332 | 0.516000 |
