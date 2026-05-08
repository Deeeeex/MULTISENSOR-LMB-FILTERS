# GA Tiered Link Ablation (2026-05-07 02:08:11)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 16 (fixed=1)
- trialSeeds: [17 18 19 20 21]
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
- Trial 1: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 2: [0 0.1 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 3: [0.2 0 0.5 0.5 0.1 0.1 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.1 0 0.5 0.5 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.1 0.5 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 17 | fixed weights | 2.600917 | 2.376823 | 0.802500 |
| 1 | 17 | +covariance | 2.123460 | 1.822961 | 0.482500 |
| 1 | 17 | +link quality | 1.846428 | 1.442536 | 0.260000 |
| 1 | 17 | +existence confidence | 1.872548 | 1.446483 | 0.265000 |
| 1 | 17 | +structure-aware decoupled KLA | 1.863439 | 1.430953 | 0.268750 |
| 2 | 18 | fixed weights | 2.530244 | 2.232345 | 0.762500 |
| 2 | 18 | +covariance | 2.228898 | 2.359800 | 0.537500 |
| 2 | 18 | +link quality | 1.880047 | 1.767543 | 0.233750 |
| 2 | 18 | +existence confidence | 1.881488 | 1.777117 | 0.232500 |
| 2 | 18 | +structure-aware decoupled KLA | 1.873030 | 1.746324 | 0.237500 |
| 3 | 19 | fixed weights | 2.440243 | 2.295666 | 0.741250 |
| 3 | 19 | +covariance | 1.949824 | 2.437999 | 0.398750 |
| 3 | 19 | +link quality | 1.725516 | 1.706628 | 0.208750 |
| 3 | 19 | +existence confidence | 1.713947 | 1.653472 | 0.212500 |
| 3 | 19 | +structure-aware decoupled KLA | 1.703659 | 1.650134 | 0.208750 |
| 4 | 20 | fixed weights | 2.498598 | 2.662959 | 0.777500 |
| 4 | 20 | +covariance | 2.100110 | 1.792176 | 0.487500 |
| 4 | 20 | +link quality | 1.866973 | 1.688004 | 0.248750 |
| 4 | 20 | +existence confidence | 1.844706 | 1.681704 | 0.246250 |
| 4 | 20 | +structure-aware decoupled KLA | 1.851685 | 1.651547 | 0.253750 |
| 5 | 21 | fixed weights | 2.462637 | 2.808408 | 0.576250 |
| 5 | 21 | +covariance | 2.116350 | 2.094606 | 0.441250 |
| 5 | 21 | +link quality | 1.838601 | 1.759714 | 0.262500 |
| 5 | 21 | +existence confidence | 1.837733 | 1.812616 | 0.267500 |
| 5 | 21 | +structure-aware decoupled KLA | 1.829428 | 1.815996 | 0.266250 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.506528 | 2.475240 | 0.732000 |
| +covariance | 2.103728 | 2.101508 | 0.469500 |
| +link quality | 1.831513 | 1.672885 | 0.242750 |
| +existence confidence | 1.830084 | 1.674279 | 0.244750 |
| +structure-aware decoupled KLA | 1.824248 | 1.658991 | 0.247000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.506528 +/- 0.062958 | [2.428368, 2.584688] | 5 |
| +covariance | OSPA | 2.103728 +/- 0.099894 | [1.979713, 2.227744] | 5 |
| +link quality | OSPA | 1.831513 +/- 0.061484 | [1.755183, 1.907843] | 5 |
| +existence confidence | OSPA | 1.830084 +/- 0.067464 | [1.746330, 1.913839] | 5 |
| +structure-aware decoupled KLA | OSPA | 1.824248 +/- 0.069349 | [1.738154, 1.910342] | 5 |
| fixed weights | RMSE | 2.475240 +/- 0.248581 | [2.166636, 2.783845] | 5 |
| +covariance | RMSE | 2.101508 +/- 0.297180 | [1.732570, 2.470447] | 5 |
| +link quality | RMSE | 1.672885 +/- 0.133161 | [1.507571, 1.838199] | 5 |
| +existence confidence | RMSE | 1.674279 +/- 0.143259 | [1.496428, 1.852129] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.658991 +/- 0.145269 | [1.478644, 1.839338] | 5 |
| fixed weights | Cardinality | 0.732000 +/- 0.089881 | [0.620416, 0.843584] | 5 |
| +covariance | Cardinality | 0.469500 +/- 0.052251 | [0.404632, 0.534368] | 5 |
| +link quality | Cardinality | 0.242750 +/- 0.022139 | [0.215265, 0.270235] | 5 |
| +existence confidence | Cardinality | 0.244750 +/- 0.023022 | [0.216169, 0.273331] | 5 |
| +structure-aware decoupled KLA | Cardinality | 0.247000 +/- 0.024711 | [0.216322, 0.277678] | 5 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.402799 +/- 0.081786 | [0.301265, 0.504334] | 16.07% | 5/5 | 0.0625 |
| +link quality | OSPA | 0.675015 +/- 0.056988 | [0.604266, 0.745764] | 26.93% | 5/5 | 0.0625 |
| +existence confidence | OSPA | 0.676444 +/- 0.047731 | [0.617187, 0.735700] | 26.99% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | OSPA | 0.682280 +/- 0.050702 | [0.619335, 0.745225] | 27.22% | 5/5 | 0.0625 |
| +covariance | RMSE | 0.373732 +/- 0.477668 | [-0.219276, 0.966740] | 15.10% | 3/5 | 1 |
| +link quality | RMSE | 0.802355 +/- 0.258518 | [0.481415, 1.123296] | 32.42% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.800962 +/- 0.240802 | [0.502015, 1.099909] | 32.36% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.816250 +/- 0.236705 | [0.522388, 1.110111] | 32.98% | 5/5 | 0.0625 |
| +covariance | Cardinality | 0.262500 +/- 0.083853 | [0.158400, 0.366600] | 35.86% | 5/5 | 0.0625 |
| +link quality | Cardinality | 0.489250 +/- 0.098269 | [0.367253, 0.611247] | 66.84% | 5/5 | 0.0625 |
| +existence confidence | Cardinality | 0.487250 +/- 0.099841 | [0.363301, 0.611199] | 66.56% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | Cardinality | 0.485000 +/- 0.097928 | [0.363426, 0.606574] | 66.26% | 5/5 | 0.0625 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.943300 | 0.500000 | 1.631986 | 1.603000 |
| +covariance | 2.775119 | 0.500000 | 1.546552 | 1.302000 |
| +link quality | 2.424104 | 0.500000 | 1.593435 | 0.736750 |
| +existence confidence | 2.432519 | 0.500000 | 1.595747 | 0.740250 |
| +structure-aware decoupled KLA | 2.434683 | 0.500000 | 1.599937 | 0.741500 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.943300 +/- 0.105438 | [2.812403, 3.074198] | 5 |
| +covariance | E-OSPA | 2.775119 +/- 0.083096 | [2.671958, 2.878280] | 5 |
| +link quality | E-OSPA | 2.424104 +/- 0.081556 | [2.322855, 2.525353] | 5 |
| +existence confidence | E-OSPA | 2.432519 +/- 0.083539 | [2.328808, 2.536229] | 5 |
| +structure-aware decoupled KLA | E-OSPA | 2.434683 +/- 0.085233 | [2.328870, 2.540497] | 5 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +covariance | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 5 |
| +link quality | H-OSPA | 0.500000 +/- 0.000000 | [0.499999, 0.500000] | 5 |
| +existence confidence | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 5 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 5 |
| fixed weights | RMSE | 1.631986 +/- 0.065246 | [1.550985, 1.712986] | 5 |
| +covariance | RMSE | 1.546552 +/- 0.049976 | [1.484508, 1.608595] | 5 |
| +link quality | RMSE | 1.593435 +/- 0.037639 | [1.546707, 1.640162] | 5 |
| +existence confidence | RMSE | 1.595747 +/- 0.037987 | [1.548587, 1.642907] | 5 |
| +structure-aware decoupled KLA | RMSE | 1.599937 +/- 0.038341 | [1.552338, 1.647537] | 5 |
| fixed weights | CardErr | 1.603000 +/- 0.087948 | [1.493816, 1.712184] | 5 |
| +covariance | CardErr | 1.302000 +/- 0.074218 | [1.209861, 1.394139] | 5 |
| +link quality | CardErr | 0.736750 +/- 0.060030 | [0.662225, 0.811275] | 5 |
| +existence confidence | CardErr | 0.740250 +/- 0.058699 | [0.667377, 0.813123] | 5 |
| +structure-aware decoupled KLA | CardErr | 0.741500 +/- 0.063630 | [0.662506, 0.820494] | 5 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.168181 +/- 0.036000 | [0.123488, 0.212874] | 5.71% | 5/5 | 0.0625 |
| +link quality | E-OSPA | 0.519196 +/- 0.077082 | [0.423502, 0.614891] | 17.64% | 5/5 | 0.0625 |
| +existence confidence | E-OSPA | 0.510782 +/- 0.076801 | [0.415436, 0.606128] | 17.35% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | E-OSPA | 0.508617 +/- 0.077386 | [0.412545, 0.604689] | 17.28% | 5/5 | 0.0625 |
| +covariance | H-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/5 | NaN |
| +link quality | H-OSPA | 0.000000 +/- 0.000000 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +existence confidence | H-OSPA | 0.000000 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 2/5 | 0.5 |
| +covariance | RMSE | 0.085434 +/- 0.019917 | [0.060708, 0.110161] | 5.23% | 5/5 | 0.0625 |
| +link quality | RMSE | 0.038551 +/- 0.032524 | [-0.001826, 0.078929] | 2.36% | 5/5 | 0.0625 |
| +existence confidence | RMSE | 0.036239 +/- 0.031774 | [-0.003208, 0.075686] | 2.22% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | RMSE | 0.032048 +/- 0.034203 | [-0.010413, 0.074510] | 1.96% | 4/5 | 0.375 |
| +covariance | CardErr | 0.301000 +/- 0.045263 | [0.244807, 0.357193] | 18.78% | 5/5 | 0.0625 |
| +link quality | CardErr | 0.866250 +/- 0.110958 | [0.728499, 1.004001] | 54.04% | 5/5 | 0.0625 |
| +existence confidence | CardErr | 0.862750 +/- 0.112838 | [0.722666, 1.002834] | 53.82% | 5/5 | 0.0625 |
| +structure-aware decoupled KLA | CardErr | 0.861500 +/- 0.113782 | [0.720243, 1.002757] | 53.74% | 5/5 | 0.0625 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.853760 | 0.500000 | 1.561902 | 1.382000 |
| 1 | +covariance | 2.642208 | 0.500000 | 1.489517 | 1.080000 |
| 1 | +link quality | 2.413848 | 0.500000 | 1.546992 | 0.768000 |
| 1 | +existence confidence | 2.421340 | 0.500000 | 1.553385 | 0.770000 |
| 1 | +structure-aware decoupled KLA | 2.424764 | 0.500000 | 1.546232 | 0.776000 |
| 2 | fixed weights | 2.743049 | 0.500000 | 1.599421 | 1.182000 |
| 2 | +covariance | 2.702791 | 0.500000 | 1.508559 | 1.150000 |
| 2 | +link quality | 2.383162 | 0.500000 | 1.533344 | 0.754000 |
| 2 | +existence confidence | 2.391936 | 0.500000 | 1.530913 | 0.756000 |
| 2 | +structure-aware decoupled KLA | 2.394380 | 0.500000 | 1.542949 | 0.752000 |
| 3 | fixed weights | 2.869709 | 0.500000 | 1.549348 | 1.356000 |
| 3 | +covariance | 2.682639 | 0.500000 | 1.484661 | 1.130000 |
| 3 | +link quality | 2.446329 | 0.500000 | 1.557445 | 0.826000 |
| 3 | +existence confidence | 2.457232 | 0.500000 | 1.557973 | 0.830000 |
| 3 | +structure-aware decoupled KLA | 2.447432 | 0.500000 | 1.560267 | 0.824000 |
| 4 | fixed weights | 3.013113 | 0.500000 | 1.563764 | 1.908000 |
| 4 | +covariance | 2.794445 | 0.500000 | 1.508787 | 1.462000 |
| 4 | +link quality | 2.476761 | 0.500000 | 1.583705 | 0.888000 |
| 4 | +existence confidence | 2.506446 | 0.500000 | 1.587658 | 0.910000 |
| 4 | +structure-aware decoupled KLA | 2.500704 | 0.500000 | 1.584394 | 0.902000 |
| 5 | fixed weights | 2.927131 | 0.500000 | 1.802351 | 1.510000 |
| 5 | +covariance | 2.886099 | 0.500000 | 1.583695 | 1.540000 |
| 5 | +link quality | 2.508139 | 0.500000 | 1.654299 | 0.786000 |
| 5 | +existence confidence | 2.501776 | 0.500000 | 1.655648 | 0.778000 |
| 5 | +structure-aware decoupled KLA | 2.521199 | 0.500000 | 1.665426 | 0.800000 |
| 6 | fixed weights | 2.907753 | 0.500000 | 1.631867 | 1.484000 |
| 6 | +covariance | 2.774261 | 0.500000 | 1.554392 | 1.278000 |
| 6 | +link quality | 2.331684 | 0.500000 | 1.602518 | 0.548000 |
| 6 | +existence confidence | 2.338367 | 0.500000 | 1.607406 | 0.552000 |
| 6 | +structure-aware decoupled KLA | 2.340783 | 0.500000 | 1.611814 | 0.550000 |
| 7 | fixed weights | 3.047597 | 0.500000 | 1.680301 | 1.720000 |
| 7 | +covariance | 2.864686 | 0.500000 | 1.599366 | 1.404000 |
| 7 | +link quality | 2.481634 | 0.500000 | 1.642608 | 0.756000 |
| 7 | +existence confidence | 2.491924 | 0.500000 | 1.645384 | 0.760000 |
| 7 | +structure-aware decoupled KLA | 2.492258 | 0.500000 | 1.650829 | 0.762000 |
| 8 | fixed weights | 3.184291 | 0.500000 | 1.666934 | 2.282000 |
| 8 | +covariance | 2.853824 | 0.500000 | 1.643434 | 1.372000 |
| 8 | +link quality | 2.351274 | 0.499998 | 1.626566 | 0.568000 |
| 8 | +existence confidence | 2.351128 | 0.499998 | 1.627609 | 0.566000 |
| 8 | +structure-aware decoupled KLA | 2.355948 | 0.499997 | 1.637587 | 0.566000 |
