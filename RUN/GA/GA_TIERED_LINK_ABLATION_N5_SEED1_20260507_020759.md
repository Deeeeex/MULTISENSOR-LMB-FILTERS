# GA Tiered Link Ablation (2026-05-07 02:07:59)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6]
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
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.590531 | 2.268101 | 0.868750 |
| 1 | 2 | +covariance | 2.243220 | 1.774557 | 0.608750 |
| 1 | 2 | +link quality | 1.909508 | 1.621662 | 0.242500 |
| 1 | 2 | +existence confidence | 1.911733 | 1.681976 | 0.238750 |
| 1 | 2 | +structure-aware decoupled KLA | 1.892404 | 1.611468 | 0.241250 |
| 2 | 3 | fixed weights | 2.623249 | 3.319849 | 0.761250 |
| 2 | 3 | +covariance | 2.295843 | 2.943444 | 0.610000 |
| 2 | 3 | +link quality | 1.941786 | 1.634121 | 0.271250 |
| 2 | 3 | +existence confidence | 1.955997 | 1.546617 | 0.275000 |
| 2 | 3 | +structure-aware decoupled KLA | 1.928044 | 1.541570 | 0.267500 |
| 3 | 4 | fixed weights | 2.681097 | 2.507659 | 0.871250 |
| 3 | 4 | +covariance | 2.137026 | 1.754357 | 0.537500 |
| 3 | 4 | +link quality | 1.853284 | 1.495404 | 0.236250 |
| 3 | 4 | +existence confidence | 1.844136 | 1.482744 | 0.233750 |
| 3 | 4 | +structure-aware decoupled KLA | 1.835859 | 1.487550 | 0.237500 |
| 4 | 5 | fixed weights | 2.398507 | 2.920391 | 0.625000 |
| 4 | 5 | +covariance | 2.017490 | 3.202154 | 0.371250 |
| 4 | 5 | +link quality | 1.774282 | 2.228815 | 0.192500 |
| 4 | 5 | +existence confidence | 1.772742 | 2.224507 | 0.188750 |
| 4 | 5 | +structure-aware decoupled KLA | 1.764445 | 2.213187 | 0.191250 |
| 5 | 6 | fixed weights | 2.826942 | 2.497009 | 1.267500 |
| 5 | 6 | +covariance | 2.363987 | 2.380371 | 0.820000 |
| 5 | 6 | +link quality | 1.909992 | 2.024725 | 0.283750 |
| 5 | 6 | +existence confidence | 1.904559 | 1.923063 | 0.285000 |
| 5 | 6 | +structure-aware decoupled KLA | 1.890466 | 1.894265 | 0.283750 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.624065 | 2.702602 | 0.878750 |
| +covariance | 2.211513 | 2.410976 | 0.589500 |
| +link quality | 1.877771 | 1.800945 | 0.245250 |
| +existence confidence | 1.877833 | 1.771781 | 0.244250 |
| +structure-aware decoupled KLA | 1.862244 | 1.749608 | 0.244250 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.624065 +/- 0.155252 | [2.431324, 2.816805] | 5 |
| +covariance | OSPA | 2.211513 +/- 0.136527 | [2.042020, 2.381007] | 5 |
| +link quality | OSPA | 1.877771 +/- 0.066055 | [1.795766, 1.959775] | 5 |
| +existence confidence | OSPA | 1.877833 +/- 0.070980 | [1.789714, 1.965953] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.862244 +/- 0.063829 | [1.783003, 1.941485] | 5 |
| fixed weights | RMSE | 2.702602 +/- 0.417579 | [2.184192, 3.221011] | 5 |
| +covariance | RMSE | 2.410976 +/- 0.660795 | [1.590623, 3.231330] | 5 |
| +link quality | RMSE | 1.800945 +/- 0.310837 | [1.415052, 2.186838] | 5 |
| +existence confidence | RMSE | 1.771781 +/- 0.304201 | [1.394126, 2.149437] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.749608 +/- 0.302848 | [1.373633, 2.125583] | 5 |
| fixed weights | Cardinality | 0.878750 +/- 0.239519 | [0.581395, 1.176105] | 5 |
| +covariance | Cardinality | 0.589500 +/- 0.161445 | [0.389072, 0.789928] | 5 |
| +link quality | Cardinality | 0.245250 +/- 0.035459 | [0.201229, 0.289271] | 5 |
| +existence confidence | Cardinality | 0.244250 +/- 0.038167 | [0.196867, 0.291633] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.244250 +/- 0.035229 | [0.200514, 0.287986] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.412552 +/- 0.089914 | [0.300926, 0.524177] | 15.72% | 5/5 | 0.0625 |
| +link quality | OSPA | 0.746294 +/- 0.121586 | [0.595349, 0.897240] | 28.44% | 5/5 | 0.0625 |
| +existence confidence | OSPA | 0.746231 +/- 0.127042 | [0.588513, 0.903950] | 28.44% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | OSPA | 0.761821 +/- 0.124779 | [0.606912, 0.916730] | 29.03% | 5/5 | 0.0625 |
| +covariance | RMSE | 0.291625 +/- 0.393859 | [-0.197337, 0.780588] | 10.79% | 4/5 | 0.375 |
| +link quality | RMSE | 0.901657 +/- 0.479748 | [0.306067, 1.497246] | 33.36% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.930820 +/- 0.504946 | [0.303948, 1.557693] | 34.44% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.952994 +/- 0.489008 | [0.345907, 1.560081] | 35.26% | 5/5 | 0.0625 |
| +covariance | Cardinality | 0.289250 +/- 0.109753 | [0.152995, 0.425505] | 32.92% | 5/5 | 0.0625 |
| +link quality | Cardinality | 0.633500 +/- 0.214315 | [0.367436, 0.899564] | 72.09% | 5/5 | 0.0625 |
| +existence confidence | Cardinality | 0.634500 +/- 0.213550 | [0.369385, 0.899615] | 72.20% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.634500 +/- 0.213387 | [0.369587, 0.899413] | 72.20% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.945058 | 0.500000 | 1.622083 | 1.762250 |
| +covariance | 2.764376 | 0.500000 | 1.542339 | 1.413000 |
| +link quality | 2.384566 | 0.499999 | 1.598738 | 0.715250 |
| +existence confidence | 2.384174 | 0.499999 | 1.599494 | 0.709250 |
| +structure-aware decoupled KLA | 2.381696 | 0.499999 | 1.602228 | 0.710250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.945058 +/- 0.101250 | [2.819359, 3.070757] | 5 |
| +covariance | E-OSPA | 2.764376 +/- 0.093670 | [2.648088, 2.880665] | 5 |
| +link quality | E-OSPA | 2.384566 +/- 0.073997 | [2.292701, 2.476431] | 5 |
| +existence confidence | E-OSPA | 2.384174 +/- 0.073504 | [2.292921, 2.475427] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.381696 +/- 0.073946 | [2.289894, 2.473497] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +covariance | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +link quality | H-OSPA | 0.499999 +/- 0.000002 | [0.499997, 0.500001] | 5 |
| +existence confidence | H-OSPA | 0.499999 +/- 0.000002 | [0.499996, 0.500001] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.499999 +/- 0.000002 | [0.499997, 0.500001] | 5 |
| fixed weights | RMSE | 1.622083 +/- 0.047195 | [1.563492, 1.680673] | 5 |
| +covariance | RMSE | 1.542339 +/- 0.053324 | [1.476140, 1.608539] | 5 |
| +link quality | RMSE | 1.598738 +/- 0.060427 | [1.523720, 1.673757] | 5 |
| +existence confidence | RMSE | 1.599494 +/- 0.059360 | [1.525800, 1.673187] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.602228 +/- 0.059596 | [1.528241, 1.676214] | 5 |
| fixed weights | CardErr | 1.762250 +/- 0.223078 | [1.485307, 2.039193] | 5 |
| +covariance | CardErr | 1.413000 +/- 0.151205 | [1.225284, 1.600716] | 5 |
| +link quality | CardErr | 0.715250 +/- 0.084101 | [0.610842, 0.819658] | 5 |
| +existence confidence | CardErr | 0.709250 +/- 0.082852 | [0.606392, 0.812108] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.710250 +/- 0.083859 | [0.606142, 0.814358] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.180682 +/- 0.021630 | [0.153828, 0.207535] | 6.14% | 5/5 | 0.0625 |
| +link quality | E-OSPA | 0.560492 +/- 0.037256 | [0.514240, 0.606744] | 19.03% | 5/5 | 0.0625 |
| +existence confidence | E-OSPA | 0.560884 +/- 0.037956 | [0.513763, 0.608004] | 19.04% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | E-OSPA | 0.563362 +/- 0.037356 | [0.516987, 0.609738] | 19.13% | 5/5 | 0.0625 |
| +covariance | H-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |
| +link quality | H-OSPA | 0.000001 +/- 0.000002 | [-0.000001, 0.000003] | 0.00% | 2/5 | 0.5 |
| +existence confidence | H-OSPA | 0.000001 +/- 0.000002 | [-0.000001, 0.000004] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | H-OSPA | 0.000001 +/- 0.000002 | [-0.000001, 0.000003] | 0.00% | 2/5 | 0.5 |
| +covariance | RMSE | 0.079743 +/- 0.026987 | [0.046240, 0.113246] | 4.92% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.023344 +/- 0.039092 | [-0.025188, 0.071876] | 1.44% | 4/5 | 0.375 |
| +existence confidence | RMSE | 0.022589 +/- 0.038308 | [-0.024970, 0.070147] | 1.39% | 4/5 | 0.375 |
| +structure-aware decoupled KLA | RMSE | 0.019855 +/- 0.039355 | [-0.029003, 0.068713] | 1.22% | 4/5 | 0.375 |
| +covariance | CardErr | 0.349250 +/- 0.086452 | [0.241923, 0.456577] | 19.82% | 5/5 | 0.0625 |
| +link quality | CardErr | 1.047000 +/- 0.141644 | [0.871154, 1.222846] | 59.41% | 5/5 | 0.0625 |
| +existence confidence | CardErr | 1.053000 +/- 0.143251 | [0.875159, 1.230841] | 59.75% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | CardErr | 1.052000 +/- 0.144546 | [0.872552, 1.231448] | 59.70% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.761763 | 0.500000 | 1.533529 | 1.304000 |
| 1 | +covariance | 2.548372 | 0.500000 | 1.524321 | 0.996000 |
| 1 | +link quality | 2.349824 | 0.500000 | 1.553848 | 0.696000 |
| 1 | +existence confidence | 2.348211 | 0.500000 | 1.552851 | 0.690000 |
| 1 | +structure-aware decoupled KLA | 2.343192 | 0.500000 | 1.549339 | 0.684000 |
| 2 | fixed weights | 2.545475 | 0.500000 | 1.572886 | 0.938000 |
| 2 | +covariance | 2.550575 | 0.500000 | 1.465693 | 1.030000 |
| 2 | +link quality | 2.315475 | 0.500000 | 1.528814 | 0.694000 |
| 2 | +existence confidence | 2.316161 | 0.500000 | 1.529202 | 0.692000 |
| 2 | +structure-aware decoupled KLA | 2.317623 | 0.500000 | 1.534088 | 0.690000 |
| 3 | fixed weights | 2.838552 | 0.500000 | 1.536159 | 1.406000 |
| 3 | +covariance | 2.679187 | 0.500000 | 1.471706 | 1.166000 |
| 3 | +link quality | 2.368091 | 0.500000 | 1.541894 | 0.730000 |
| 3 | +existence confidence | 2.380103 | 0.500000 | 1.542892 | 0.734000 |
| 3 | +structure-aware decoupled KLA | 2.373107 | 0.500000 | 1.545717 | 0.734000 |
| 4 | fixed weights | 2.794773 | 0.500000 | 1.623058 | 1.542000 |
| 4 | +covariance | 2.655077 | 0.500000 | 1.496799 | 1.388000 |
| 4 | +link quality | 2.386589 | 0.500000 | 1.572003 | 0.798000 |
| 4 | +existence confidence | 2.380907 | 0.500000 | 1.573767 | 0.782000 |
| 4 | +structure-aware decoupled KLA | 2.360557 | 0.500000 | 1.577893 | 0.778000 |
| 5 | fixed weights | 3.259823 | 0.500000 | 1.713498 | 2.320000 |
| 5 | +covariance | 3.046567 | 0.500000 | 1.563051 | 1.972000 |
| 5 | +link quality | 2.512105 | 0.500000 | 1.666556 | 0.884000 |
| 5 | +existence confidence | 2.506318 | 0.500000 | 1.668514 | 0.872000 |
| 5 | +structure-aware decoupled KLA | 2.516018 | 0.500000 | 1.676317 | 0.876000 |
| 6 | fixed weights | 2.899061 | 0.500000 | 1.650463 | 1.534000 |
| 6 | +covariance | 2.751679 | 0.500000 | 1.558976 | 1.318000 |
| 6 | +link quality | 2.268484 | 0.500000 | 1.604234 | 0.518000 |
| 6 | +existence confidence | 2.265959 | 0.500000 | 1.605610 | 0.512000 |
| 6 | +structure-aware decoupled KLA | 2.271183 | 0.500000 | 1.604269 | 0.526000 |
| 7 | fixed weights | 3.269107 | 0.500000 | 1.653931 | 2.368000 |
| 7 | +covariance | 3.012225 | 0.500000 | 1.627615 | 1.788000 |
| 7 | +link quality | 2.548789 | 0.500000 | 1.680140 | 0.834000 |
| 7 | +existence confidence | 2.539649 | 0.500000 | 1.679808 | 0.824000 |
| 7 | +structure-aware decoupled KLA | 2.546423 | 0.500000 | 1.683179 | 0.832000 |
| 8 | fixed weights | 3.191911 | 0.500000 | 1.693135 | 2.686000 |
| 8 | +covariance | 2.871330 | 0.500000 | 1.630554 | 1.646000 |
| 8 | +link quality | 2.327170 | 0.499990 | 1.642419 | 0.568000 |
| 8 | +existence confidence | 2.336085 | 0.499989 | 1.643308 | 0.568000 |
| 8 | +structure-aware decoupled KLA | 2.325462 | 0.499990 | 1.647022 | 0.562000 |
