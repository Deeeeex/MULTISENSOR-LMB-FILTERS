# GA Tiered Link Ablation (2026-05-15 10:51:37)

Comparison order: fixed weights -> Cao-Zhao FID-FIA baseline -> +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 3
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4]
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

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.633016 | 2.466452 | 0.875000 |
| Cao-Zhao FID-FIA baseline | 1.815386 | 1.636936 | 0.112500 |
| +structure-aware decoupled KLA | 1.847161 | 1.414241 | 0.217500 |
| +FID-FIA existence refinement | 1.671561 | 1.613584 | 0.057917 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.633016 +/- 0.049105 | [2.511022, 2.755010] | 3 |
| Cao-Zhao FID-FIA baseline | OSPA | 1.815386 +/- 0.090425 | [1.590740, 2.040031] | 3 |
| +structure-aware decoupled KLA | OSPA | 1.847161 +/- 0.035285 | [1.759500, 1.934821] | 3 |
| +FID-FIA existence refinement | OSPA | 1.671561 +/- 0.039840 | [1.572586, 1.770536] | 3 |
| fixed weights | RMSE | 2.466452 +/- 0.267891 | [1.800919, 3.131985] | 3 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.636936 +/- 0.062655 | [1.481279, 1.792594] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.414241 +/- 0.037247 | [1.321707, 1.506774] | 3 |
| +FID-FIA existence refinement | RMSE | 1.613584 +/- 0.118466 | [1.319275, 1.907893] | 3 |
| fixed weights | Cardinality | 0.875000 +/- 0.086927 | [0.659045, 1.090955] | 3 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.112500 +/- 0.041307 | [0.009880, 0.215120] | 3 |
| +structure-aware decoupled KLA | Cardinality | 0.217500 +/- 0.024590 | [0.156409, 0.278591] | 3 |
| +FID-FIA existence refinement | Cardinality | 0.057917 +/- 0.007324 | [0.039721, 0.076113] | 3 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | OSPA | 0.817630 +/- 0.121184 | [0.516568, 1.118693] | 31.05% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | OSPA | 0.785855 +/- 0.083085 | [0.579444, 0.992267] | 29.85% | 3/3 | 0.25 |
| +FID-FIA existence refinement | OSPA | 0.961455 +/- 0.083410 | [0.754237, 1.168674] | 36.52% | 3/3 | 0.25 |
| Cao-Zhao FID-FIA baseline | RMSE | 0.829515 +/- 0.325905 | [0.019857, 1.639173] | 33.63% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 1.052211 +/- 0.304983 | [0.294530, 1.809892] | 42.66% | 3/3 | 0.25 |
| +FID-FIA existence refinement | RMSE | 0.852868 +/- 0.191811 | [0.376343, 1.329392] | 34.58% | 3/3 | 0.25 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.762500 +/- 0.068053 | [0.593433, 0.931567] | 87.14% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | Cardinality | 0.657500 +/- 0.063897 | [0.498759, 0.816241] | 75.14% | 3/3 | 0.25 |
| +FID-FIA existence refinement | Cardinality | 0.817083 +/- 0.081933 | [0.613534, 1.020633] | 93.38% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 47.334410 +/- 0.731642 | 0.473344 | 1.000x | 3 |
| Cao-Zhao FID-FIA baseline | 137.830252 +/- 3.479381 | 1.378303 | 2.913x | 3 |
| +structure-aware decoupled KLA | 55.602754 +/- 5.057483 | 0.556028 | 1.174x | 3 |
| +FID-FIA existence refinement | 143.107455 +/- 2.949774 | 1.431075 | 3.023x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| Cao-Zhao FID-FIA baseline | 90.495841 +/- 4.048142 | 191.29% | 3/3 |
| +structure-aware decoupled KLA | 8.268344 +/- 4.431121 | 17.39% | 3/3 |
| +FID-FIA existence refinement | 95.773044 +/- 2.495640 | 202.34% | 3/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 47.177374 | 0.471774 | 1.000x |
| 1 | 2 | Cao-Zhao FID-FIA baseline | 135.706280 | 1.357063 | 2.877x |
| 1 | 2 | +structure-aware decoupled KLA | 51.878908 | 0.518789 | 1.100x |
| 1 | 2 | +FID-FIA existence refinement | 140.266158 | 1.402662 | 2.973x |
| 2 | 3 | fixed weights | 46.694037 | 0.466940 | 1.000x |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 141.845651 | 1.418457 | 3.038x |
| 2 | 3 | +structure-aware decoupled KLA | 53.568800 | 0.535688 | 1.147x |
| 2 | 3 | +FID-FIA existence refinement | 142.901314 | 1.429013 | 3.060x |
| 3 | 4 | fixed weights | 48.131820 | 0.481318 | 1.000x |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 135.938824 | 1.359388 | 2.824x |
| 3 | 4 | +structure-aware decoupled KLA | 61.360554 | 0.613606 | 1.275x |
| 3 | 4 | +FID-FIA existence refinement | 146.154892 | 1.461549 | 3.037x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.898247 | 1.629533 | 1.598333 |
| Cao-Zhao FID-FIA baseline | 2.166131 | 1.702721 | 0.375833 |
| +structure-aware decoupled KLA | 2.325830 | 1.585090 | 0.578333 |
| +FID-FIA existence refinement | 2.005475 | 1.789950 | 0.214583 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.898247 +/- 0.082407 | [2.693521, 3.102973] | 3 |
| Cao-Zhao FID-FIA baseline | E-OSPA | 2.166131 +/- 0.124331 | [1.857251, 2.475011] | 3 |
| +structure-aware decoupled KLA | E-OSPA | 2.325830 +/- 0.078195 | [2.131567, 2.520093] | 3 |
| +FID-FIA existence refinement | E-OSPA | 2.005475 +/- 0.095789 | [1.767504, 2.243447] | 3 |
| fixed weights | RMSE | 1.629533 +/- 0.065863 | [1.465908, 1.793158] | 3 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.702721 +/- 0.072643 | [1.522250, 1.883192] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.585090 +/- 0.074857 | [1.399121, 1.771060] | 3 |
| +FID-FIA existence refinement | RMSE | 1.789950 +/- 0.193041 | [1.310370, 2.269530] | 3 |
| fixed weights | CardErr | 1.598333 +/- 0.141097 | [1.247801, 1.948866] | 3 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.375833 +/- 0.101530 | [0.123599, 0.628068] | 3 |
| +structure-aware decoupled KLA | CardErr | 0.578333 +/- 0.073169 | [0.396558, 0.760109] | 3 |
| +FID-FIA existence refinement | CardErr | 0.214583 +/- 0.011880 | [0.185068, 0.244098] | 3 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | E-OSPA | 0.732116 +/- 0.045542 | [0.618975, 0.845258] | 25.26% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | E-OSPA | 0.572417 +/- 0.015661 | [0.533511, 0.611324] | 19.75% | 3/3 | 0.25 |
| +FID-FIA existence refinement | E-OSPA | 0.892772 +/- 0.045732 | [0.779159, 1.006385] | 30.80% | 3/3 | 0.25 |
| Cao-Zhao FID-FIA baseline | RMSE | -0.073188 +/- 0.018613 | [-0.119430, -0.026946] | -4.49% | 0/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.044442 +/- 0.028338 | [-0.025960, 0.114845] | 2.73% | 3/3 | 0.25 |
| +FID-FIA existence refinement | RMSE | -0.160417 +/- 0.179198 | [-0.605606, 0.284772] | -9.84% | 0/3 | 0.25 |
| Cao-Zhao FID-FIA baseline | CardErr | 1.222500 +/- 0.039686 | [1.123906, 1.321094] | 76.49% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | CardErr | 1.020000 +/- 0.069293 | [0.847852, 1.192148] | 63.82% | 3/3 | 0.25 |
| +FID-FIA existence refinement | CardErr | 1.383750 +/- 0.130390 | [1.059817, 1.707683] | 86.57% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.808112 | 1.593222 | 1.263333 |
| 1 | Cao-Zhao FID-FIA baseline | 2.090008 | 1.649203 | 0.313333 |
| 1 | +structure-aware decoupled KLA | 2.309749 | 1.534062 | 0.533333 |
| 1 | +FID-FIA existence refinement | 1.982919 | 1.932290 | 0.226667 |
| 2 | fixed weights | 2.564830 | 1.596495 | 0.856667 |
| 2 | Cao-Zhao FID-FIA baseline | 2.101920 | 1.643981 | 0.356667 |
| 2 | +structure-aware decoupled KLA | 2.311787 | 1.553208 | 0.573333 |
| 2 | +FID-FIA existence refinement | 1.966255 | 2.272710 | 0.223333 |
| 3 | fixed weights | 2.788741 | 1.520167 | 1.313333 |
| 3 | Cao-Zhao FID-FIA baseline | 2.107256 | 1.633693 | 0.366667 |
| 3 | +structure-aware decoupled KLA | 2.292917 | 1.524602 | 0.553333 |
| 3 | +FID-FIA existence refinement | 1.951988 | 1.562678 | 0.216667 |
| 4 | fixed weights | 2.744554 | 1.647656 | 1.406667 |
| 4 | Cao-Zhao FID-FIA baseline | 2.184069 | 1.705138 | 0.420000 |
| 4 | +structure-aware decoupled KLA | 2.283974 | 1.571431 | 0.586667 |
| 4 | +FID-FIA existence refinement | 1.985588 | 1.614272 | 0.213333 |
| 5 | fixed weights | 3.253490 | 1.736795 | 2.083333 |
| 5 | Cao-Zhao FID-FIA baseline | 2.261033 | 1.765658 | 0.460000 |
| 5 | +structure-aware decoupled KLA | 2.447372 | 1.652546 | 0.703333 |
| 5 | +FID-FIA existence refinement | 2.088643 | 1.987850 | 0.233333 |
| 6 | fixed weights | 2.936944 | 1.678346 | 1.620000 |
| 6 | Cao-Zhao FID-FIA baseline | 2.195035 | 1.778477 | 0.316667 |
| 6 | +structure-aware decoupled KLA | 2.269246 | 1.609449 | 0.500000 |
| 6 | +FID-FIA existence refinement | 2.005828 | 1.646314 | 0.190000 |
| 7 | fixed weights | 3.097092 | 1.610141 | 2.013333 |
| 7 | Cao-Zhao FID-FIA baseline | 2.231431 | 1.724479 | 0.443333 |
| 7 | +structure-aware decoupled KLA | 2.442940 | 1.651141 | 0.666667 |
| 7 | +FID-FIA existence refinement | 2.059805 | 1.671644 | 0.216667 |
| 8 | fixed weights | 2.992216 | 1.653440 | 2.230000 |
| 8 | Cao-Zhao FID-FIA baseline | 2.158297 | 1.721137 | 0.330000 |
| 8 | +structure-aware decoupled KLA | 2.248655 | 1.584285 | 0.510000 |
| 8 | +FID-FIA existence refinement | 2.002776 | 1.631841 | 0.196667 |
