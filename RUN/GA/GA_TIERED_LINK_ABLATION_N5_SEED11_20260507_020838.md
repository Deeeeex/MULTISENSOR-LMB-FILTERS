# GA Tiered Link Ablation (2026-05-07 02:08:38)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
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
- Trial 1: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 2: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 3: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 4: [0.1 0.5 0 0.2 0.5 0.1 0.1 0.1]
- Trial 5: [0 0.1 0.5 0.1 0.1 0.2 0.1 0.5]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 12 | fixed weights | 2.235333 | 2.348212 | 0.482500 |
| 1 | 12 | +covariance | 2.005310 | 2.061706 | 0.396250 |
| 1 | 12 | +link quality | 1.803088 | 1.857358 | 0.223750 |
| 1 | 12 | +existence confidence | 1.804221 | 1.894255 | 0.220000 |
| 1 | 12 | +structure-aware decoupled KLA | 1.797235 | 1.885002 | 0.225000 |
| 2 | 13 | fixed weights | 2.380325 | 2.213801 | 0.656250 |
| 2 | 13 | +covariance | 1.931672 | 1.964170 | 0.347500 |
| 2 | 13 | +link quality | 1.777604 | 1.479052 | 0.277500 |
| 2 | 13 | +existence confidence | 1.776705 | 1.527480 | 0.273750 |
| 2 | 13 | +structure-aware decoupled KLA | 1.786939 | 1.575583 | 0.281250 |
| 3 | 14 | fixed weights | 2.363292 | 2.665071 | 0.552500 |
| 3 | 14 | +covariance | 2.058895 | 2.387448 | 0.357500 |
| 3 | 14 | +link quality | 1.857518 | 1.812120 | 0.225000 |
| 3 | 14 | +existence confidence | 1.854604 | 1.747155 | 0.221250 |
| 3 | 14 | +structure-aware decoupled KLA | 1.850337 | 1.788203 | 0.222500 |
| 4 | 15 | fixed weights | 2.147298 | 2.230920 | 0.382500 |
| 4 | 15 | +covariance | 1.946851 | 1.846261 | 0.310000 |
| 4 | 15 | +link quality | 1.804698 | 1.606526 | 0.245000 |
| 4 | 15 | +existence confidence | 1.800481 | 1.606367 | 0.246250 |
| 4 | 15 | +structure-aware decoupled KLA | 1.798971 | 1.592700 | 0.247500 |
| 5 | 16 | fixed weights | 2.048554 | 2.956503 | 0.333750 |
| 5 | 16 | +covariance | 1.888464 | 2.138081 | 0.310000 |
| 5 | 16 | +link quality | 1.792985 | 2.192421 | 0.217500 |
| 5 | 16 | +existence confidence | 1.779075 | 2.164613 | 0.213750 |
| 5 | 16 | +structure-aware decoupled KLA | 1.765436 | 2.136723 | 0.210000 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.234960 | 2.482901 | 0.481500 |
| +covariance | 1.966238 | 2.079533 | 0.344250 |
| +link quality | 1.807179 | 1.789495 | 0.237750 |
| +existence confidence | 1.803017 | 1.787974 | 0.235000 |
| +structure-aware decoupled KLA | 1.799783 | 1.795642 | 0.237250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.234960 +/- 0.141450 | [2.059355, 2.410565] | 5 |
| +covariance | OSPA | 1.966238 +/- 0.066580 | [1.883582, 2.048895] | 5 |
| +link quality | OSPA | 1.807179 +/- 0.030137 | [1.769764, 1.844593] | 5 |
| +existence confidence | OSPA | 1.803017 +/- 0.031363 | [1.764081, 1.841954] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.799783 +/- 0.031255 | [1.760981, 1.838586] | 5 |
| fixed weights | RMSE | 2.482901 +/- 0.320755 | [2.084696, 2.881107] | 5 |
| +covariance | RMSE | 2.079533 +/- 0.203888 | [1.826413, 2.332653] | 5 |
| +link quality | RMSE | 1.789495 +/- 0.272635 | [1.451029, 2.127962] | 5 |
| +existence confidence | RMSE | 1.787974 +/- 0.252813 | [1.474116, 2.101832] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.795642 +/- 0.231294 | [1.508498, 2.082786] | 5 |
| fixed weights | Cardinality | 0.481500 +/- 0.129625 | [0.320575, 0.642425] | 5 |
| +covariance | Cardinality | 0.344250 +/- 0.036181 | [0.299333, 0.389167] | 5 |
| +link quality | Cardinality | 0.237750 +/- 0.024501 | [0.207333, 0.268167] | 5 |
| +existence confidence | Cardinality | 0.235000 +/- 0.024969 | [0.204002, 0.265998] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.237250 +/- 0.028068 | [0.202405, 0.272095] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.268722 +/- 0.113587 | [0.127708, 0.409736] | 12.02% | 5/5 | 0.0625 |
| +link quality | OSPA | 0.427782 +/- 0.135665 | [0.259358, 0.596205] | 19.14% | 5/5 | 0.0625 |
| +existence confidence | OSPA | 0.431943 +/- 0.131344 | [0.268885, 0.595002] | 19.33% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | OSPA | 0.435177 +/- 0.124251 | [0.280924, 0.589430] | 19.47% | 5/5 | 0.0625 |
| +covariance | RMSE | 0.403368 +/- 0.237552 | [0.108456, 0.698280] | 16.25% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.693406 +/- 0.139597 | [0.520102, 0.866710] | 27.93% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.694927 +/- 0.174799 | [0.477920, 0.911934] | 27.99% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.687259 +/- 0.164712 | [0.482774, 0.891744] | 27.68% | 5/5 | 0.0625 |
| +covariance | Cardinality | 0.137250 +/- 0.114480 | [-0.004874, 0.279374] | 28.50% | 5/5 | 0.0625 |
| +link quality | Cardinality | 0.243750 +/- 0.115119 | [0.100834, 0.386666] | 50.62% | 5/5 | 0.0625 |
| +existence confidence | Cardinality | 0.246500 +/- 0.116288 | [0.102132, 0.390868] | 51.19% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.244250 +/- 0.113004 | [0.103959, 0.384541] | 50.73% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.809376 | 0.500000 | 1.606183 | 1.347500 |
| +covariance | 2.666934 | 0.500000 | 1.533988 | 1.140750 |
| +link quality | 2.373989 | 0.499999 | 1.574639 | 0.702750 |
| +existence confidence | 2.377656 | 0.499999 | 1.575547 | 0.700500 |
| +structure-aware decoupled KLA | 2.380050 | 0.500000 | 1.580965 | 0.700250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.809376 +/- 0.101830 | [2.682958, 2.935794] | 5 |
| +covariance | E-OSPA | 2.666934 +/- 0.075863 | [2.572753, 2.761116] | 5 |
| +link quality | E-OSPA | 2.373989 +/- 0.096350 | [2.254373, 2.493605] | 5 |
| +existence confidence | E-OSPA | 2.377656 +/- 0.098322 | [2.255593, 2.499720] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.380050 +/- 0.098939 | [2.257221, 2.502879] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +covariance | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +link quality | H-OSPA | 0.499999 +/- 0.000001 | [0.499999, 0.500000] | 5 |
| +existence confidence | H-OSPA | 0.499999 +/- 0.000001 | [0.499998, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.606183 +/- 0.079226 | [1.507827, 1.704539] | 5 |
| +covariance | RMSE | 1.533988 +/- 0.074506 | [1.441492, 1.626484] | 5 |
| +link quality | RMSE | 1.574639 +/- 0.064358 | [1.494741, 1.654537] | 5 |
| +existence confidence | RMSE | 1.575547 +/- 0.064850 | [1.495038, 1.656056] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.580965 +/- 0.066187 | [1.498796, 1.663134] | 5 |
| fixed weights | CardErr | 1.347500 +/- 0.164609 | [1.143144, 1.551856] | 5 |
| +covariance | CardErr | 1.140750 +/- 0.095048 | [1.022752, 1.258748] | 5 |
| +link quality | CardErr | 0.702750 +/- 0.071318 | [0.614211, 0.791289] | 5 |
| +existence confidence | CardErr | 0.700500 +/- 0.073013 | [0.609857, 0.791143] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.700250 +/- 0.071874 | [0.611020, 0.789480] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.142441 +/- 0.036007 | [0.097740, 0.187143] | 5.07% | 5/5 | 0.0625 |
| +link quality | E-OSPA | 0.435387 +/- 0.059570 | [0.361433, 0.509340] | 15.50% | 5/5 | 0.0625 |
| +existence confidence | E-OSPA | 0.431719 +/- 0.063323 | [0.353106, 0.510333] | 15.37% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | E-OSPA | 0.429325 +/- 0.064107 | [0.349739, 0.508912] | 15.28% | 5/5 | 0.0625 |
| +covariance | H-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |
| +link quality | H-OSPA | 0.000001 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +existence confidence | H-OSPA | 0.000001 +/- 0.000001 | [-0.000000, 0.000002] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +covariance | RMSE | 0.072195 +/- 0.006987 | [0.063521, 0.080870] | 4.49% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.031544 +/- 0.015633 | [0.012136, 0.050952] | 1.96% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.030636 +/- 0.015819 | [0.010997, 0.050275] | 1.91% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.025218 +/- 0.014664 | [0.007014, 0.043423] | 1.57% | 5/5 | 0.0625 |
| +covariance | CardErr | 0.206750 +/- 0.086515 | [0.099345, 0.314155] | 15.34% | 5/5 | 0.0625 |
| +link quality | CardErr | 0.644750 +/- 0.134157 | [0.478199, 0.811301] | 47.85% | 5/5 | 0.0625 |
| +existence confidence | CardErr | 0.647000 +/- 0.136778 | [0.477195, 0.816805] | 48.01% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | CardErr | 0.647250 +/- 0.132772 | [0.482418, 0.812082] | 48.03% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.672151 | 0.500000 | 1.546066 | 1.090000 |
| 1 | +covariance | 2.490185 | 0.500000 | 1.466527 | 0.878000 |
| 1 | +link quality | 2.294140 | 0.500000 | 1.505400 | 0.674000 |
| 1 | +existence confidence | 2.295892 | 0.500000 | 1.502263 | 0.672000 |
| 1 | +structure-aware decoupled KLA | 2.300713 | 0.500000 | 1.508750 | 0.672000 |
| 2 | fixed weights | 2.709157 | 0.500000 | 1.544048 | 1.046000 |
| 2 | +covariance | 2.655378 | 0.500000 | 1.501484 | 1.034000 |
| 2 | +link quality | 2.411682 | 0.500000 | 1.573744 | 0.762000 |
| 2 | +existence confidence | 2.421794 | 0.500000 | 1.568816 | 0.764000 |
| 2 | +structure-aware decoupled KLA | 2.417000 | 0.500000 | 1.580313 | 0.756000 |
| 3 | fixed weights | 2.590826 | 0.500000 | 1.545617 | 1.008000 |
| 3 | +covariance | 2.506337 | 0.500000 | 1.476739 | 0.922000 |
| 3 | +link quality | 2.352357 | 0.500000 | 1.524249 | 0.754000 |
| 3 | +existence confidence | 2.350469 | 0.500000 | 1.525311 | 0.750000 |
| 3 | +structure-aware decoupled KLA | 2.358253 | 0.500000 | 1.532769 | 0.752000 |
| 4 | fixed weights | 2.925962 | 0.500000 | 1.612146 | 1.402000 |
| 4 | +covariance | 2.716634 | 0.500000 | 1.521194 | 1.226000 |
| 4 | +link quality | 2.434585 | 0.500000 | 1.580502 | 0.830000 |
| 4 | +existence confidence | 2.436172 | 0.500000 | 1.582815 | 0.830000 |
| 4 | +structure-aware decoupled KLA | 2.438183 | 0.500000 | 1.589868 | 0.824000 |
| 5 | fixed weights | 2.759220 | 0.500000 | 1.639050 | 1.332000 |
| 5 | +covariance | 2.714886 | 0.500000 | 1.502666 | 1.384000 |
| 5 | +link quality | 2.429739 | 0.500000 | 1.627057 | 0.800000 |
| 5 | +existence confidence | 2.426234 | 0.500000 | 1.632997 | 0.788000 |
| 5 | +structure-aware decoupled KLA | 2.435773 | 0.500000 | 1.635512 | 0.798000 |
| 6 | fixed weights | 2.844265 | 0.500000 | 1.667184 | 1.290000 |
| 6 | +covariance | 2.707467 | 0.500000 | 1.590660 | 1.136000 |
| 6 | +link quality | 2.319716 | 0.500000 | 1.572922 | 0.534000 |
| 6 | +existence confidence | 2.336857 | 0.500000 | 1.577701 | 0.538000 |
| 6 | +structure-aware decoupled KLA | 2.342740 | 0.500000 | 1.579546 | 0.540000 |
| 7 | fixed weights | 2.764006 | 0.500000 | 1.628361 | 1.256000 |
| 7 | +covariance | 2.734388 | 0.500000 | 1.609631 | 1.288000 |
| 7 | +link quality | 2.386095 | 0.500000 | 1.623909 | 0.682000 |
| 7 | +existence confidence | 2.389683 | 0.500000 | 1.622217 | 0.686000 |
| 7 | +structure-aware decoupled KLA | 2.383606 | 0.500000 | 1.627634 | 0.676000 |
| 8 | fixed weights | 3.209419 | 0.500000 | 1.666992 | 2.356000 |
| 8 | +covariance | 2.810202 | 0.500000 | 1.603001 | 1.258000 |
| 8 | +link quality | 2.363599 | 0.499996 | 1.589328 | 0.586000 |
| 8 | +existence confidence | 2.364149 | 0.499996 | 1.592258 | 0.576000 |
| 8 | +structure-aware decoupled KLA | 2.364135 | 0.499998 | 1.593326 | 0.584000 |
