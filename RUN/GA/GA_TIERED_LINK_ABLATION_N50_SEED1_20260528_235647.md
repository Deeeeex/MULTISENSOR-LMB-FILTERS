# GA Tiered Link Ablation (2026-05-28 23:56:47)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: structureAware

## Arm Configs
### fixed weights
- enabled: 0
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 1.000
- spatialStructureStrength: 0.000
- existenceStructureStrength: 0.000
- structureReliabilityPower: 0.000
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### +structure-aware decoupled KLA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]
- Trial 6: [0 0.2 0.1 0.5 0.1 0.5 0.1 0.1]
- Trial 7: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 8: [0.5 0.1 0.2 0.1 0.5 0.1 0.1 0]
- Trial 9: [0.5 0.1 0 0.1 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0.1 0 0.5 0.1 0.1 0.2]
- Trial 11: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 12: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 13: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 14: [0.1 0.5 0 0.2 0.5 0.1 0.1 0.1]
- Trial 15: [0 0.1 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 16: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 17: [0 0.1 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 18: [0.2 0 0.5 0.5 0.1 0.1 0.1 0.1]
- Trial 19: [0.2 0.1 0.1 0.1 0 0.5 0.5 0.1]
- Trial 20: [0.1 0.5 0.1 0 0.1 0.5 0.1 0.2]
- Trial 21: [0.1 0.5 0.1 0.2 0 0.5 0.1 0.1]
- Trial 22: [0.1 0.2 0.5 0.5 0 0.1 0.1 0.1]
- Trial 23: [0.1 0.1 0.1 0 0.1 0.5 0.5 0.2]
- Trial 24: [0.2 0.1 0.5 0.5 0.1 0.1 0 0.1]
- Trial 25: [0.1 0.5 0.5 0.1 0.2 0.1 0.1 0]
- Trial 26: [0.2 0.1 0.5 0.1 0.1 0 0.1 0.5]
- Trial 27: [0.1 0.5 0 0.1 0.2 0.1 0.1 0.5]
- Trial 28: [0.1 0.5 0.1 0 0.2 0.5 0.1 0.1]
- Trial 29: [0.1 0.1 0.1 0.1 0.2 0.5 0.5 0]
- Trial 30: [0.1 0 0.1 0.1 0.1 0.5 0.2 0.5]
- Trial 31: [0.1 0 0.1 0.5 0.2 0.1 0.5 0.1]
- Trial 32: [0.5 0.2 0 0.1 0.1 0.1 0.1 0.5]
- Trial 33: [0.2 0.1 0.1 0.1 0.5 0.5 0 0.1]
- Trial 34: [0.1 0.1 0 0.1 0.1 0.5 0.2 0.5]
- Trial 35: [0.2 0.1 0.1 0 0.1 0.1 0.5 0.5]
- Trial 36: [0.1 0.2 0.5 0.5 0.1 0 0.1 0.1]
- Trial 37: [0.1 0.1 0.1 0.2 0 0.5 0.1 0.5]
- Trial 38: [0.1 0 0.5 0.1 0.5 0.1 0.1 0.2]
- Trial 39: [0.5 0 0.1 0.1 0.5 0.1 0.2 0.1]
- Trial 40: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 41: [0.1 0.2 0.1 0.1 0.1 0 0.5 0.5]
- Trial 42: [0.5 0 0.2 0.1 0.1 0.1 0.1 0.5]
- Trial 43: [0.2 0.1 0.5 0.5 0 0.1 0.1 0.1]
- Trial 44: [0 0.5 0.5 0.1 0.1 0.2 0.1 0.1]
- Trial 45: [0.5 0.1 0.1 0.2 0.5 0.1 0.1 0]
- Trial 46: [0.5 0.1 0.1 0.1 0.5 0.1 0 0.2]
- Trial 47: [0.5 0.1 0.1 0.2 0.1 0.5 0 0.1]
- Trial 48: [0 0.5 0.1 0.1 0.5 0.1 0.2 0.1]
- Trial 49: [0.1 0 0.5 0.5 0.1 0.2 0.1 0.1]
- Trial 50: [0.1 0.5 0.5 0.1 0 0.1 0.2 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.605320 | 2.258061 | 0.932500 |
| 1 | 2 | +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | +structure-aware decoupled KLA | 1.854447 | 1.371532 | 0.191250 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | +structure-aware decoupled KLA | 1.808801 | 1.431204 | 0.221250 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | +structure-aware decoupled KLA | 1.736420 | 1.923505 | 0.170000 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | +structure-aware decoupled KLA | 1.852532 | 1.777241 | 0.243750 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | +structure-aware decoupled KLA | 1.839920 | 1.764960 | 0.193750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | +structure-aware decoupled KLA | 1.724528 | 1.356549 | 0.161250 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | +structure-aware decoupled KLA | 1.732578 | 1.434821 | 0.177500 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | +structure-aware decoupled KLA | 1.798485 | 2.148753 | 0.180000 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.787456 | 1.493683 | 0.182500 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | +structure-aware decoupled KLA | 1.753632 | 1.572377 | 0.171250 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | +structure-aware decoupled KLA | 1.742848 | 1.486898 | 0.197500 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | +structure-aware decoupled KLA | 1.790615 | 1.451047 | 0.182500 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | +structure-aware decoupled KLA | 1.772452 | 1.466171 | 0.191250 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | +structure-aware decoupled KLA | 1.704826 | 1.517027 | 0.176250 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | +structure-aware decoupled KLA | 1.830609 | 1.438554 | 0.228750 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.863847 | 1.652217 | 0.216250 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | +structure-aware decoupled KLA | 1.673306 | 1.510607 | 0.161250 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | +structure-aware decoupled KLA | 1.812300 | 1.473501 | 0.197500 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | +structure-aware decoupled KLA | 1.759623 | 1.539784 | 0.175000 |
| 21 | 22 | fixed weights | 2.445386 | 2.063249 | 0.706250 |
| 21 | 22 | +structure-aware decoupled KLA | 1.771566 | 1.591554 | 0.178750 |
| 22 | 23 | fixed weights | 2.782054 | 2.774744 | 0.917500 |
| 22 | 23 | +structure-aware decoupled KLA | 1.828140 | 1.410398 | 0.208750 |
| 23 | 24 | fixed weights | 2.811135 | 1.942125 | 1.156250 |
| 23 | 24 | +structure-aware decoupled KLA | 1.793788 | 1.500075 | 0.176250 |
| 24 | 25 | fixed weights | 2.569606 | 2.368173 | 0.711250 |
| 24 | 25 | +structure-aware decoupled KLA | 1.875143 | 1.500632 | 0.221250 |
| 25 | 26 | fixed weights | 2.225426 | 2.278298 | 0.403750 |
| 25 | 26 | +structure-aware decoupled KLA | 1.717305 | 1.374395 | 0.173750 |
| 26 | 27 | fixed weights | 2.159231 | 1.899203 | 0.517500 |
| 26 | 27 | +structure-aware decoupled KLA | 1.646146 | 1.363300 | 0.142500 |
| 27 | 28 | fixed weights | 2.126003 | 1.944541 | 0.406250 |
| 27 | 28 | +structure-aware decoupled KLA | 1.736613 | 1.421164 | 0.208750 |
| 28 | 29 | fixed weights | 2.917753 | 1.994773 | 1.183750 |
| 28 | 29 | +structure-aware decoupled KLA | 1.873211 | 1.616359 | 0.268750 |
| 29 | 30 | fixed weights | 2.600417 | 1.944603 | 1.000000 |
| 29 | 30 | +structure-aware decoupled KLA | 1.870636 | 1.691933 | 0.227500 |
| 30 | 31 | fixed weights | 2.468399 | 2.109218 | 0.737500 |
| 30 | 31 | +structure-aware decoupled KLA | 1.770363 | 1.471423 | 0.161250 |
| 31 | 32 | fixed weights | 2.513279 | 1.885047 | 0.721250 |
| 31 | 32 | +structure-aware decoupled KLA | 1.746697 | 1.452005 | 0.170000 |
| 32 | 33 | fixed weights | 2.019727 | 1.812822 | 0.302500 |
| 32 | 33 | +structure-aware decoupled KLA | 1.679034 | 1.408664 | 0.165000 |
| 33 | 34 | fixed weights | 2.490122 | 2.173185 | 0.962500 |
| 33 | 34 | +structure-aware decoupled KLA | 1.674970 | 1.535775 | 0.183750 |
| 34 | 35 | fixed weights | 2.915586 | 2.674676 | 1.071250 |
| 34 | 35 | +structure-aware decoupled KLA | 1.864837 | 1.559731 | 0.185000 |
| 35 | 36 | fixed weights | 2.381312 | 2.282297 | 0.561250 |
| 35 | 36 | +structure-aware decoupled KLA | 1.692204 | 1.449569 | 0.136250 |
| 36 | 37 | fixed weights | 2.794401 | 3.053361 | 0.971250 |
| 36 | 37 | +structure-aware decoupled KLA | 1.844239 | 1.497974 | 0.215000 |
| 37 | 38 | fixed weights | 2.488304 | 2.240250 | 0.821250 |
| 37 | 38 | +structure-aware decoupled KLA | 1.697548 | 1.538374 | 0.148750 |
| 38 | 39 | fixed weights | 2.292857 | 2.023920 | 0.538750 |
| 38 | 39 | +structure-aware decoupled KLA | 1.826976 | 1.463676 | 0.183750 |
| 39 | 40 | fixed weights | 2.535165 | 2.535969 | 0.646250 |
| 39 | 40 | +structure-aware decoupled KLA | 1.796344 | 1.662470 | 0.206250 |
| 40 | 41 | fixed weights | 2.257174 | 2.481788 | 0.530000 |
| 40 | 41 | +structure-aware decoupled KLA | 1.780970 | 1.560634 | 0.201250 |
| 41 | 42 | fixed weights | 2.094739 | 2.021235 | 0.453750 |
| 41 | 42 | +structure-aware decoupled KLA | 1.593887 | 1.271218 | 0.158750 |
| 42 | 43 | fixed weights | 2.470491 | 2.757701 | 0.583750 |
| 42 | 43 | +structure-aware decoupled KLA | 1.717249 | 1.437860 | 0.095000 |
| 43 | 44 | fixed weights | 2.550641 | 2.146021 | 0.801250 |
| 43 | 44 | +structure-aware decoupled KLA | 1.809049 | 1.398760 | 0.183750 |
| 44 | 45 | fixed weights | 2.216546 | 2.738044 | 0.428750 |
| 44 | 45 | +structure-aware decoupled KLA | 1.710189 | 1.388926 | 0.165000 |
| 45 | 46 | fixed weights | 2.537339 | 2.539290 | 0.686250 |
| 45 | 46 | +structure-aware decoupled KLA | 1.745958 | 1.425635 | 0.173750 |
| 46 | 47 | fixed weights | 2.665038 | 2.010381 | 0.693750 |
| 46 | 47 | +structure-aware decoupled KLA | 1.871566 | 1.802479 | 0.161250 |
| 47 | 48 | fixed weights | 2.238065 | 2.249780 | 0.577500 |
| 47 | 48 | +structure-aware decoupled KLA | 1.751635 | 1.357804 | 0.170000 |
| 48 | 49 | fixed weights | 2.529967 | 3.164643 | 0.665000 |
| 48 | 49 | +structure-aware decoupled KLA | 1.917595 | 1.648404 | 0.238750 |
| 49 | 50 | fixed weights | 2.564502 | 2.102492 | 0.951250 |
| 49 | 50 | +structure-aware decoupled KLA | 1.726094 | 1.431585 | 0.183750 |
| 50 | 51 | fixed weights | 2.714437 | 3.379717 | 0.801250 |
| 50 | 51 | +structure-aware decoupled KLA | 1.913492 | 1.626347 | 0.232500 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.468973 | 2.326034 | 0.716025 |
| +structure-aware decoupled KLA | 1.779218 | 1.522191 | 0.187675 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.468973 +/- 0.219981 | [2.407997, 2.529949] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.779218 +/- 0.072856 | [1.759023, 1.799413] | 50 |
| fixed weights | RMSE | 2.326034 +/- 0.353065 | [2.228170, 2.423899] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.522191 +/- 0.157069 | [1.478654, 1.565728] | 50 |
| fixed weights | Cardinality | 0.716025 +/- 0.223795 | [0.653992, 0.778058] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.187675 +/- 0.031189 | [0.179030, 0.196320] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.689755 +/- 0.179418 | [0.640023, 0.739487] | 27.94% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.803843 +/- 0.347451 | [0.707535, 0.900152] | 34.56% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.528350 +/- 0.210098 | [0.470114, 0.586586] | 73.79% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 62.087930 +/- 11.511902 | 0.620879 | 1.000x | 50 |
| +structure-aware decoupled KLA | 65.468123 +/- 10.979907 | 0.654681 | 1.064x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 3.380193 +/- 8.465052 | 6.44% | 37/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 69.456272 | 0.694563 | 1.000x |
| 1 | 2 | +structure-aware decoupled KLA | 65.667240 | 0.656672 | 0.945x |
| 2 | 3 | fixed weights | 52.019234 | 0.520192 | 1.000x |
| 2 | 3 | +structure-aware decoupled KLA | 58.634266 | 0.586343 | 1.127x |
| 3 | 4 | fixed weights | 53.399096 | 0.533991 | 1.000x |
| 3 | 4 | +structure-aware decoupled KLA | 55.711115 | 0.557111 | 1.043x |
| 4 | 5 | fixed weights | 53.297156 | 0.532972 | 1.000x |
| 4 | 5 | +structure-aware decoupled KLA | 56.182655 | 0.561827 | 1.054x |
| 5 | 6 | fixed weights | 49.805215 | 0.498052 | 1.000x |
| 5 | 6 | +structure-aware decoupled KLA | 53.759906 | 0.537599 | 1.079x |
| 6 | 7 | fixed weights | 49.535760 | 0.495358 | 1.000x |
| 6 | 7 | +structure-aware decoupled KLA | 54.868922 | 0.548689 | 1.108x |
| 7 | 8 | fixed weights | 57.073194 | 0.570732 | 1.000x |
| 7 | 8 | +structure-aware decoupled KLA | 80.979273 | 0.809793 | 1.419x |
| 8 | 9 | fixed weights | 76.255258 | 0.762553 | 1.000x |
| 8 | 9 | +structure-aware decoupled KLA | 61.153262 | 0.611533 | 0.802x |
| 9 | 10 | fixed weights | 49.926178 | 0.499262 | 1.000x |
| 9 | 10 | +structure-aware decoupled KLA | 53.153510 | 0.531535 | 1.065x |
| 10 | 11 | fixed weights | 50.415076 | 0.504151 | 1.000x |
| 10 | 11 | +structure-aware decoupled KLA | 54.387525 | 0.543875 | 1.079x |
| 11 | 12 | fixed weights | 53.163723 | 0.531637 | 1.000x |
| 11 | 12 | +structure-aware decoupled KLA | 59.063666 | 0.590637 | 1.111x |
| 12 | 13 | fixed weights | 57.210865 | 0.572109 | 1.000x |
| 12 | 13 | +structure-aware decoupled KLA | 60.832917 | 0.608329 | 1.063x |
| 13 | 14 | fixed weights | 55.732125 | 0.557321 | 1.000x |
| 13 | 14 | +structure-aware decoupled KLA | 58.560632 | 0.585606 | 1.051x |
| 14 | 15 | fixed weights | 59.198042 | 0.591980 | 1.000x |
| 14 | 15 | +structure-aware decoupled KLA | 60.879781 | 0.608798 | 1.028x |
| 15 | 16 | fixed weights | 55.656671 | 0.556567 | 1.000x |
| 15 | 16 | +structure-aware decoupled KLA | 56.679451 | 0.566795 | 1.018x |
| 16 | 17 | fixed weights | 50.837954 | 0.508380 | 1.000x |
| 16 | 17 | +structure-aware decoupled KLA | 55.689887 | 0.556899 | 1.095x |
| 17 | 18 | fixed weights | 51.018876 | 0.510189 | 1.000x |
| 17 | 18 | +structure-aware decoupled KLA | 58.559924 | 0.585599 | 1.148x |
| 18 | 19 | fixed weights | 51.800351 | 0.518004 | 1.000x |
| 18 | 19 | +structure-aware decoupled KLA | 54.662803 | 0.546628 | 1.055x |
| 19 | 20 | fixed weights | 51.275466 | 0.512755 | 1.000x |
| 19 | 20 | +structure-aware decoupled KLA | 54.935680 | 0.549357 | 1.071x |
| 20 | 21 | fixed weights | 67.538541 | 0.675385 | 1.000x |
| 20 | 21 | +structure-aware decoupled KLA | 66.022939 | 0.660229 | 0.978x |
| 21 | 22 | fixed weights | 64.940075 | 0.649401 | 1.000x |
| 21 | 22 | +structure-aware decoupled KLA | 67.451940 | 0.674519 | 1.039x |
| 22 | 23 | fixed weights | 62.108734 | 0.621087 | 1.000x |
| 22 | 23 | +structure-aware decoupled KLA | 85.607416 | 0.856074 | 1.378x |
| 23 | 24 | fixed weights | 65.660120 | 0.656601 | 1.000x |
| 23 | 24 | +structure-aware decoupled KLA | 64.030391 | 0.640304 | 0.975x |
| 24 | 25 | fixed weights | 54.723849 | 0.547238 | 1.000x |
| 24 | 25 | +structure-aware decoupled KLA | 52.420131 | 0.524201 | 0.958x |
| 25 | 26 | fixed weights | 50.278245 | 0.502782 | 1.000x |
| 25 | 26 | +structure-aware decoupled KLA | 52.098357 | 0.520984 | 1.036x |
| 26 | 27 | fixed weights | 48.344859 | 0.483449 | 1.000x |
| 26 | 27 | +structure-aware decoupled KLA | 55.870710 | 0.558707 | 1.156x |
| 27 | 28 | fixed weights | 48.947528 | 0.489475 | 1.000x |
| 27 | 28 | +structure-aware decoupled KLA | 53.672382 | 0.536724 | 1.097x |
| 28 | 29 | fixed weights | 52.821171 | 0.528212 | 1.000x |
| 28 | 29 | +structure-aware decoupled KLA | 62.979830 | 0.629798 | 1.192x |
| 29 | 30 | fixed weights | 65.208484 | 0.652085 | 1.000x |
| 29 | 30 | +structure-aware decoupled KLA | 89.076250 | 0.890763 | 1.366x |
| 30 | 31 | fixed weights | 70.015279 | 0.700153 | 1.000x |
| 30 | 31 | +structure-aware decoupled KLA | 72.615830 | 0.726158 | 1.037x |
| 31 | 32 | fixed weights | 73.707910 | 0.737079 | 1.000x |
| 31 | 32 | +structure-aware decoupled KLA | 80.024881 | 0.800249 | 1.086x |
| 32 | 33 | fixed weights | 76.469043 | 0.764690 | 1.000x |
| 32 | 33 | +structure-aware decoupled KLA | 84.531870 | 0.845319 | 1.105x |
| 33 | 34 | fixed weights | 93.677991 | 0.936780 | 1.000x |
| 33 | 34 | +structure-aware decoupled KLA | 81.115212 | 0.811152 | 0.866x |
| 34 | 35 | fixed weights | 78.047873 | 0.780479 | 1.000x |
| 34 | 35 | +structure-aware decoupled KLA | 75.214310 | 0.752143 | 0.964x |
| 35 | 36 | fixed weights | 73.030813 | 0.730308 | 1.000x |
| 35 | 36 | +structure-aware decoupled KLA | 77.202410 | 0.772024 | 1.057x |
| 36 | 37 | fixed weights | 61.936535 | 0.619365 | 1.000x |
| 36 | 37 | +structure-aware decoupled KLA | 76.107691 | 0.761077 | 1.229x |
| 37 | 38 | fixed weights | 65.573542 | 0.655735 | 1.000x |
| 37 | 38 | +structure-aware decoupled KLA | 73.284769 | 0.732848 | 1.118x |
| 38 | 39 | fixed weights | 64.116743 | 0.641167 | 1.000x |
| 38 | 39 | +structure-aware decoupled KLA | 71.748150 | 0.717482 | 1.119x |
| 39 | 40 | fixed weights | 73.394702 | 0.733947 | 1.000x |
| 39 | 40 | +structure-aware decoupled KLA | 63.979722 | 0.639797 | 0.872x |
| 40 | 41 | fixed weights | 68.609365 | 0.686094 | 1.000x |
| 40 | 41 | +structure-aware decoupled KLA | 86.444538 | 0.864445 | 1.260x |
| 41 | 42 | fixed weights | 79.839127 | 0.798391 | 1.000x |
| 41 | 42 | +structure-aware decoupled KLA | 86.800259 | 0.868003 | 1.087x |
| 42 | 43 | fixed weights | 83.004868 | 0.830049 | 1.000x |
| 42 | 43 | +structure-aware decoupled KLA | 79.991979 | 0.799920 | 0.964x |
| 43 | 44 | fixed weights | 78.742865 | 0.787429 | 1.000x |
| 43 | 44 | +structure-aware decoupled KLA | 75.114700 | 0.751147 | 0.954x |
| 44 | 45 | fixed weights | 90.768593 | 0.907686 | 1.000x |
| 44 | 45 | +structure-aware decoupled KLA | 68.747469 | 0.687475 | 0.757x |
| 45 | 46 | fixed weights | 61.029470 | 0.610295 | 1.000x |
| 45 | 46 | +structure-aware decoupled KLA | 57.950693 | 0.579507 | 0.950x |
| 46 | 47 | fixed weights | 55.935560 | 0.559356 | 1.000x |
| 46 | 47 | +structure-aware decoupled KLA | 59.107860 | 0.591079 | 1.057x |
| 47 | 48 | fixed weights | 63.259633 | 0.632596 | 1.000x |
| 47 | 48 | +structure-aware decoupled KLA | 62.079152 | 0.620792 | 0.981x |
| 48 | 49 | fixed weights | 54.674883 | 0.546749 | 1.000x |
| 48 | 49 | +structure-aware decoupled KLA | 61.145259 | 0.611453 | 1.118x |
| 49 | 50 | fixed weights | 55.019708 | 0.550197 | 1.000x |
| 49 | 50 | +structure-aware decoupled KLA | 57.467361 | 0.574674 | 1.044x |
| 50 | 51 | fixed weights | 55.893872 | 0.558939 | 1.000x |
| 50 | 51 | +structure-aware decoupled KLA | 59.139268 | 0.591393 | 1.058x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.862938 | 1.649569 | 1.455125 |
| +structure-aware decoupled KLA | 2.334915 | 1.605910 | 0.578775 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.862938 +/- 0.123917 | [2.828590, 2.897286] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 2.334915 +/- 0.071651 | [2.315054, 2.354776] | 50 |
| fixed weights | RMSE | 1.649569 +/- 0.078096 | [1.627922, 1.671216] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.605910 +/- 0.049506 | [1.592188, 1.619633] | 50 |
| fixed weights | CardErr | 1.455125 +/- 0.241978 | [1.388052, 1.522198] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.578775 +/- 0.066558 | [0.560326, 0.597224] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.528023 +/- 0.089961 | [0.503087, 0.552959] | 18.44% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.043659 +/- 0.059480 | [0.027172, 0.060146] | 2.65% | 43/50 | 2.099e-07 |
| +structure-aware decoupled KLA | CardErr | 0.876350 +/- 0.211610 | [0.817695, 0.935005] | 60.23% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.719987 | 1.619776 | 1.083600 |
| 1 | +structure-aware decoupled KLA | 2.289633 | 1.559677 | 0.542000 |
| 2 | fixed weights | 2.670351 | 1.594302 | 0.999200 |
| 2 | +structure-aware decoupled KLA | 2.297153 | 1.572246 | 0.546000 |
| 3 | fixed weights | 2.740374 | 1.607566 | 1.150400 |
| 3 | +structure-aware decoupled KLA | 2.304960 | 1.573270 | 0.573600 |
| 4 | fixed weights | 2.883255 | 1.654573 | 1.494400 |
| 4 | +structure-aware decoupled KLA | 2.413320 | 1.639044 | 0.681200 |
| 5 | fixed weights | 2.918523 | 1.691261 | 1.520000 |
| 5 | +structure-aware decoupled KLA | 2.375721 | 1.635650 | 0.619400 |
| 6 | fixed weights | 2.913753 | 1.708755 | 1.535200 |
| 6 | +structure-aware decoupled KLA | 2.327530 | 1.618789 | 0.542600 |
| 7 | fixed weights | 2.925998 | 1.663623 | 1.565800 |
| 7 | +structure-aware decoupled KLA | 2.370662 | 1.633665 | 0.605400 |
| 8 | fixed weights | 3.131265 | 1.656698 | 2.292400 |
| 8 | +structure-aware decoupled KLA | 2.300340 | 1.614939 | 0.520000 |
