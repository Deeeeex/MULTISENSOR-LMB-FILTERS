# GA Tiered Link Ablation (2026-05-12 15:57:14)

Comparison order: fixed weights -> Cao-Zhao FID-FIA baseline -> +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 20
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: fidFiaExistenceRefinement

## Arm Configs
### fixed weights
- enabled: 0
- method: factorized
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
- useFidFiaExistence: 0

### Cao-Zhao FID-FIA baseline
- enabled: 1
- method: fidFia
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
- useFidFiaExistence: 0
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

### +structure-aware decoupled KLA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
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

### +FID-FIA existence refinement
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.050
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 1
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

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

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.605320 | 2.258061 | 0.932500 |
| 1 | 2 | Cao-Zhao FID-FIA baseline | 1.919750 | 1.703510 | 0.160000 |
| 1 | 2 | +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| 1 | 2 | +FID-FIA existence refinement | 1.714819 | 1.478188 | 0.066250 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 1.760426 | 1.579121 | 0.085000 |
| 2 | 3 | +structure-aware decoupled KLA | 1.854447 | 1.371532 | 0.191250 |
| 2 | 3 | +FID-FIA existence refinement | 1.663486 | 1.698166 | 0.052500 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 1.765981 | 1.628179 | 0.092500 |
| 3 | 4 | +structure-aware decoupled KLA | 1.808801 | 1.431204 | 0.221250 |
| 3 | 4 | +FID-FIA existence refinement | 1.636377 | 1.664398 | 0.055000 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 1.842140 | 1.761535 | 0.131250 |
| 4 | 5 | +structure-aware decoupled KLA | 1.736420 | 1.923505 | 0.170000 |
| 4 | 5 | +FID-FIA existence refinement | 1.644623 | 1.459300 | 0.048750 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 1.860653 | 1.788350 | 0.146250 |
| 5 | 6 | +structure-aware decoupled KLA | 1.852532 | 1.777241 | 0.243750 |
| 5 | 6 | +FID-FIA existence refinement | 1.679733 | 1.496193 | 0.053750 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 1.869464 | 1.794725 | 0.157500 |
| 6 | 7 | +structure-aware decoupled KLA | 1.839920 | 1.764960 | 0.193750 |
| 6 | 7 | +FID-FIA existence refinement | 1.783806 | 1.798401 | 0.068750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 1.777385 | 1.501640 | 0.125000 |
| 7 | 8 | +structure-aware decoupled KLA | 1.724528 | 1.356549 | 0.161250 |
| 7 | 8 | +FID-FIA existence refinement | 1.602056 | 1.442085 | 0.050000 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 1.765026 | 1.553181 | 0.090000 |
| 8 | 9 | +structure-aware decoupled KLA | 1.732578 | 1.434821 | 0.177500 |
| 8 | 9 | +FID-FIA existence refinement | 1.616239 | 1.433022 | 0.043750 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 1.832008 | 1.807331 | 0.133750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.798485 | 2.148753 | 0.180000 |
| 9 | 10 | +FID-FIA existence refinement | 1.710576 | 1.520268 | 0.077500 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 1.791028 | 1.578027 | 0.115000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.787456 | 1.493683 | 0.182500 |
| 10 | 11 | +FID-FIA existence refinement | 1.672873 | 1.598233 | 0.070000 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 1.872682 | 1.777547 | 0.136250 |
| 11 | 12 | +structure-aware decoupled KLA | 1.753632 | 1.572377 | 0.171250 |
| 11 | 12 | +FID-FIA existence refinement | 1.708450 | 1.733906 | 0.086250 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 1.912568 | 1.624291 | 0.148750 |
| 12 | 13 | +structure-aware decoupled KLA | 1.742848 | 1.486898 | 0.197500 |
| 12 | 13 | +FID-FIA existence refinement | 1.679613 | 1.530487 | 0.068750 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 1.875715 | 1.586621 | 0.125000 |
| 13 | 14 | +structure-aware decoupled KLA | 1.790615 | 1.451047 | 0.182500 |
| 13 | 14 | +FID-FIA existence refinement | 1.704557 | 1.463948 | 0.072500 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 1.834527 | 1.599571 | 0.117500 |
| 14 | 15 | +structure-aware decoupled KLA | 1.772452 | 1.466171 | 0.191250 |
| 14 | 15 | +FID-FIA existence refinement | 1.685614 | 1.464790 | 0.095000 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 1.666159 | 1.429455 | 0.115000 |
| 15 | 16 | +structure-aware decoupled KLA | 1.704826 | 1.517027 | 0.176250 |
| 15 | 16 | +FID-FIA existence refinement | 1.543640 | 1.367411 | 0.048750 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 1.789898 | 1.591812 | 0.147500 |
| 16 | 17 | +structure-aware decoupled KLA | 1.830609 | 1.438554 | 0.228750 |
| 16 | 17 | +FID-FIA existence refinement | 1.628893 | 1.409847 | 0.072500 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 1.912720 | 1.789253 | 0.188750 |
| 17 | 18 | +structure-aware decoupled KLA | 1.863847 | 1.652217 | 0.216250 |
| 17 | 18 | +FID-FIA existence refinement | 1.733908 | 1.473600 | 0.075000 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 1.744260 | 1.643484 | 0.107500 |
| 18 | 19 | +structure-aware decoupled KLA | 1.673306 | 1.510607 | 0.161250 |
| 18 | 19 | +FID-FIA existence refinement | 1.623403 | 1.457256 | 0.036250 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 1.816670 | 1.618665 | 0.110000 |
| 19 | 20 | +structure-aware decoupled KLA | 1.812300 | 1.473501 | 0.197500 |
| 19 | 20 | +FID-FIA existence refinement | 1.685378 | 1.551796 | 0.043750 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 1.795517 | 1.591942 | 0.091250 |
| 20 | 21 | +structure-aware decoupled KLA | 1.759623 | 1.539784 | 0.175000 |
| 20 | 21 | +FID-FIA existence refinement | 1.661180 | 1.522340 | 0.036250 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.453677 | 2.335508 | 0.714625 |
| Cao-Zhao FID-FIA baseline | 1.820229 | 1.647412 | 0.126188 |
| +structure-aware decoupled KLA | 1.785873 | 1.562521 | 0.192938 |
| +FID-FIA existence refinement | 1.668961 | 1.528182 | 0.061062 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.453677 +/- 0.187964 | [2.365708, 2.541646] | 20 |
| Cao-Zhao FID-FIA baseline | OSPA | 1.820229 +/- 0.065295 | [1.789670, 1.850787] | 20 |
| +structure-aware decoupled KLA | OSPA | 1.785873 +/- 0.057187 | [1.759109, 1.812637] | 20 |
| +FID-FIA existence refinement | OSPA | 1.668961 +/- 0.052938 | [1.644186, 1.693737] | 20 |
| fixed weights | RMSE | 2.335508 +/- 0.257319 | [2.215081, 2.455936] | 20 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.647412 +/- 0.107941 | [1.596895, 1.697929] | 20 |
| +structure-aware decoupled KLA | RMSE | 1.562521 +/- 0.199660 | [1.469078, 1.655963] | 20 |
| +FID-FIA existence refinement | RMSE | 1.528182 +/- 0.114501 | [1.474594, 1.581769] | 20 |
| fixed weights | Cardinality | 0.714625 +/- 0.217152 | [0.612996, 0.816254] | 20 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.126188 +/- 0.027037 | [0.113534, 0.138841] | 20 |
| +structure-aware decoupled KLA | Cardinality | 0.192938 +/- 0.024816 | [0.181323, 0.204552] | 20 |
| +FID-FIA existence refinement | Cardinality | 0.061062 +/- 0.016475 | [0.053352, 0.068773] | 20 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | OSPA | 0.633448 +/- 0.180741 | [0.548860, 0.718037] | 25.82% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | OSPA | 0.667804 +/- 0.158866 | [0.593453, 0.742155] | 27.22% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | OSPA | 0.784716 +/- 0.178159 | [0.701336, 0.868096] | 31.98% | 20/20 | 1.907e-06 |
| Cao-Zhao FID-FIA baseline | RMSE | 0.688096 +/- 0.244776 | [0.573539, 0.802654] | 29.46% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | RMSE | 0.772988 +/- 0.244197 | [0.658701, 0.887274] | 33.10% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | RMSE | 0.807327 +/- 0.241677 | [0.694220, 0.920434] | 34.57% | 20/20 | 1.907e-06 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.588438 +/- 0.212492 | [0.488989, 0.687886] | 82.34% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | Cardinality | 0.521687 +/- 0.200965 | [0.427634, 0.615741] | 73.00% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | Cardinality | 0.653563 +/- 0.221598 | [0.549853, 0.757272] | 91.46% | 20/20 | 1.907e-06 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|:----|-------:|-------:|-----:|--------:|
| fixed weights | 2.853096 | 0.500000 | 1.637556 | 1.454500 |
| Cao-Zhao FID-FIA baseline | 2.183127 | 0.500000 | 1.715746 | 0.392313 |
| +structure-aware decoupled KLA | 2.328672 | 0.500000 | 1.598561 | 0.578688 |
| +FID-FIA existence refinement | 2.009084 | 0.500000 | 1.704538 | 0.221563 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.853096 +/- 0.117347 | [2.798176, 2.908015] | 20 |
| Cao-Zhao FID-FIA baseline | E-OSPA | 2.183127 +/- 0.057856 | [2.156050, 2.210204] | 20 |
| +structure-aware decoupled KLA | E-OSPA | 2.328672 +/- 0.081925 | [2.290330, 2.367013] | 20 |
| +FID-FIA existence refinement | E-OSPA | 2.009084 +/- 0.053408 | [1.984089, 2.034080] | 20 |
| fixed weights | H-OSPA | 0.500000 +/- 0.000000 | [0.500000, 0.500000] | 20 |
| Cao-Zhao FID-FIA baseline | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 20 |
| +structure-aware decoupled KLA | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 20 |
| +FID-FIA existence refinement | H-OSPA | 0.500000 +/- 0.000001 | [0.499999, 0.500000] | 20 |
| fixed weights | RMSE | 1.637556 +/- 0.062230 | [1.608432, 1.666680] | 20 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.715746 +/- 0.059990 | [1.687670, 1.743822] | 20 |
| +structure-aware decoupled KLA | RMSE | 1.598561 +/- 0.062934 | [1.569107, 1.628015] | 20 |
| +FID-FIA existence refinement | RMSE | 1.704538 +/- 0.148731 | [1.634931, 1.774146] | 20 |
| fixed weights | CardErr | 1.454500 +/- 0.237677 | [1.343265, 1.565735] | 20 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.392313 +/- 0.049096 | [0.369335, 0.415290] | 20 |
| +structure-aware decoupled KLA | CardErr | 0.578688 +/- 0.059940 | [0.550635, 0.606740] | 20 |
| +FID-FIA existence refinement | CardErr | 0.221563 +/- 0.022492 | [0.211036, 0.232089] | 20 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | E-OSPA | 0.669969 +/- 0.100538 | [0.622916, 0.717021] | 23.48% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | E-OSPA | 0.524424 +/- 0.079759 | [0.487096, 0.561752] | 18.38% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | E-OSPA | 0.844012 +/- 0.101125 | [0.796684, 0.891339] | 29.58% | 20/20 | 1.907e-06 |
| Cao-Zhao FID-FIA baseline | H-OSPA | 0.000000 +/- 0.000001 | [-0.000000, 0.000001] | 0.00% | 6/20 | 0.03125 |
| +structure-aware decoupled KLA | H-OSPA | 0.000000 +/- 0.000001 | [0.000000, 0.000001] | 0.00% | 8/20 | 0.007813 |
| +FID-FIA existence refinement | H-OSPA | 0.000000 +/- 0.000001 | [0.000000, 0.000001] | 0.00% | 8/20 | 0.007813 |
| Cao-Zhao FID-FIA baseline | RMSE | -0.078190 +/- 0.053310 | [-0.103140, -0.053240] | -4.77% | 1/20 | 4.005e-05 |
| +structure-aware decoupled KLA | RMSE | 0.038995 +/- 0.029748 | [0.025073, 0.052917] | 2.38% | 19/20 | 4.005e-05 |
| +FID-FIA existence refinement | RMSE | -0.066982 +/- 0.153841 | [-0.138981, 0.005017] | -4.09% | 8/20 | 0.5034 |
| Cao-Zhao FID-FIA baseline | CardErr | 1.062187 +/- 0.219295 | [0.959556, 1.164819] | 73.03% | 20/20 | 1.907e-06 |
| +structure-aware decoupled KLA | CardErr | 0.875812 +/- 0.209378 | [0.777822, 0.973803] | 60.21% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | CardErr | 1.232938 +/- 0.241880 | [1.119735, 1.346140] | 84.77% | 20/20 | 1.907e-06 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |
|------:|:----|-------:|-------:|-----:|--------:|
| 1 | fixed weights | 2.718334 | 0.500000 | 1.588423 | 1.093500 |
| 1 | Cao-Zhao FID-FIA baseline | 2.123940 | 0.500000 | 1.674516 | 0.346000 |
| 1 | +structure-aware decoupled KLA | 2.264587 | 0.500000 | 1.535255 | 0.529500 |
| 1 | +FID-FIA existence refinement | 1.962261 | 0.500000 | 1.629232 | 0.224500 |
| 2 | fixed weights | 2.588402 | 0.500000 | 1.566481 | 0.905000 |
| 2 | Cao-Zhao FID-FIA baseline | 2.144391 | 0.500000 | 1.671760 | 0.364000 |
| 2 | +structure-aware decoupled KLA | 2.267167 | 0.500000 | 1.555182 | 0.538500 |
| 2 | +FID-FIA existence refinement | 1.970931 | 0.500000 | 1.694455 | 0.221500 |
| 3 | fixed weights | 2.689999 | 0.500000 | 1.576809 | 1.076000 |
| 3 | Cao-Zhao FID-FIA baseline | 2.159997 | 0.500000 | 1.731855 | 0.396000 |
| 3 | +structure-aware decoupled KLA | 2.260636 | 0.500000 | 1.535872 | 0.551500 |
| 3 | +FID-FIA existence refinement | 1.950249 | 0.500000 | 1.610944 | 0.217500 |
| 4 | fixed weights | 2.844041 | 0.500000 | 1.635622 | 1.437500 |
| 4 | Cao-Zhao FID-FIA baseline | 2.234087 | 0.500000 | 1.703556 | 0.506000 |
| 4 | +structure-aware decoupled KLA | 2.354684 | 0.500000 | 1.602667 | 0.649000 |
| 4 | +FID-FIA existence refinement | 2.012643 | 0.500000 | 1.683487 | 0.232000 |
| 5 | fixed weights | 2.966105 | 0.500000 | 1.715895 | 1.623500 |
| 5 | Cao-Zhao FID-FIA baseline | 2.235192 | 0.500000 | 1.745856 | 0.431500 |
| 5 | +structure-aware decoupled KLA | 2.414888 | 0.500000 | 1.658761 | 0.651500 |
| 5 | +FID-FIA existence refinement | 2.069941 | 0.500000 | 1.817451 | 0.236500 |
| 6 | fixed weights | 2.906002 | 0.500000 | 1.672976 | 1.490000 |
| 6 | Cao-Zhao FID-FIA baseline | 2.167026 | 0.500000 | 1.720839 | 0.342500 |
| 6 | +structure-aware decoupled KLA | 2.328876 | 0.500000 | 1.618695 | 0.526500 |
| 6 | +FID-FIA existence refinement | 2.020828 | 0.500000 | 1.700994 | 0.208000 |
| 7 | fixed weights | 2.953074 | 0.500000 | 1.658576 | 1.628000 |
| 7 | Cao-Zhao FID-FIA baseline | 2.219171 | 0.500000 | 1.743479 | 0.402500 |
| 7 | +structure-aware decoupled KLA | 2.399046 | 0.500000 | 1.644226 | 0.631500 |
| 7 | +FID-FIA existence refinement | 2.046182 | 0.500000 | 1.728796 | 0.222500 |
| 8 | fixed weights | 3.158809 | 0.500000 | 1.685666 | 2.382500 |
| 8 | Cao-Zhao FID-FIA baseline | 2.181212 | 0.499998 | 1.734106 | 0.350000 |
| 8 | +structure-aware decoupled KLA | 2.339491 | 0.499996 | 1.637831 | 0.551500 |
| 8 | +FID-FIA existence refinement | 2.039638 | 0.499996 | 1.770948 | 0.210000 |
