# GA Tiered Link Ablation (2026-05-13 01:42:11)

Comparison order: Cao-Zhao FID-FIA baseline -> +FID-FIA existence refinement

## Run Config
- Trials: 20
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.000
- pDropLevels: []
- pDropLevelCounts: []

- finalArmMode: fidFiaExistenceRefinement

## Arm Configs
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
- Trial 1: [0 0 0 0 0 0 0 0]
- Trial 2: [0 0 0 0 0 0 0 0]
- Trial 3: [0 0 0 0 0 0 0 0]
- Trial 4: [0 0 0 0 0 0 0 0]
- Trial 5: [0 0 0 0 0 0 0 0]
- Trial 6: [0 0 0 0 0 0 0 0]
- Trial 7: [0 0 0 0 0 0 0 0]
- Trial 8: [0 0 0 0 0 0 0 0]
- Trial 9: [0 0 0 0 0 0 0 0]
- Trial 10: [0 0 0 0 0 0 0 0]
- Trial 11: [0 0 0 0 0 0 0 0]
- Trial 12: [0 0 0 0 0 0 0 0]
- Trial 13: [0 0 0 0 0 0 0 0]
- Trial 14: [0 0 0 0 0 0 0 0]
- Trial 15: [0 0 0 0 0 0 0 0]
- Trial 16: [0 0 0 0 0 0 0 0]
- Trial 17: [0 0 0 0 0 0 0 0]
- Trial 18: [0 0 0 0 0 0 0 0]
- Trial 19: [0 0 0 0 0 0 0 0]
- Trial 20: [0 0 0 0 0 0 0 0]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | Cao-Zhao FID-FIA baseline | 1.576684 | 1.359953 | 0.061250 |
| 1 | 2 | +FID-FIA existence refinement | 1.454868 | 1.430767 | 0.055000 |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 1.491200 | 1.328864 | 0.035000 |
| 2 | 3 | +FID-FIA existence refinement | 1.401287 | 1.438796 | 0.040000 |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 1.518703 | 1.340718 | 0.048750 |
| 3 | 4 | +FID-FIA existence refinement | 1.440262 | 1.249647 | 0.050000 |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 1.552154 | 1.357068 | 0.050000 |
| 4 | 5 | +FID-FIA existence refinement | 1.421485 | 1.243866 | 0.040000 |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 1.508649 | 1.322932 | 0.045000 |
| 5 | 6 | +FID-FIA existence refinement | 1.403211 | 1.231325 | 0.030000 |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 1.515673 | 1.325429 | 0.058750 |
| 6 | 7 | +FID-FIA existence refinement | 1.424927 | 1.376171 | 0.048750 |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 1.503562 | 1.273071 | 0.062500 |
| 7 | 8 | +FID-FIA existence refinement | 1.382398 | 1.218607 | 0.043750 |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 1.521106 | 1.301308 | 0.068750 |
| 8 | 9 | +FID-FIA existence refinement | 1.411570 | 1.189067 | 0.061250 |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 1.529979 | 1.339836 | 0.037500 |
| 9 | 10 | +FID-FIA existence refinement | 1.403631 | 1.244247 | 0.027500 |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 1.492009 | 1.285953 | 0.058750 |
| 10 | 11 | +FID-FIA existence refinement | 1.444671 | 1.303870 | 0.075000 |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 1.556432 | 1.371657 | 0.045000 |
| 11 | 12 | +FID-FIA existence refinement | 1.530914 | 1.744707 | 0.065000 |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 1.574093 | 1.374612 | 0.068750 |
| 12 | 13 | +FID-FIA existence refinement | 1.430405 | 1.229836 | 0.052500 |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 1.658929 | 1.400320 | 0.067500 |
| 13 | 14 | +FID-FIA existence refinement | 1.504244 | 1.268134 | 0.066250 |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 1.550249 | 1.328376 | 0.052500 |
| 14 | 15 | +FID-FIA existence refinement | 1.464468 | 1.224987 | 0.053750 |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 1.480138 | 1.286621 | 0.055000 |
| 15 | 16 | +FID-FIA existence refinement | 1.441616 | 1.243822 | 0.048750 |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 1.529579 | 1.290892 | 0.061250 |
| 16 | 17 | +FID-FIA existence refinement | 1.399719 | 1.222217 | 0.058750 |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 1.564532 | 1.364642 | 0.055000 |
| 17 | 18 | +FID-FIA existence refinement | 1.510583 | 1.686997 | 0.063750 |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 1.463397 | 1.268650 | 0.065000 |
| 18 | 19 | +FID-FIA existence refinement | 1.373100 | 1.184614 | 0.042500 |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 1.523979 | 1.343827 | 0.040000 |
| 19 | 20 | +FID-FIA existence refinement | 1.390336 | 1.198149 | 0.043750 |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 1.532123 | 1.374982 | 0.046250 |
| 20 | 21 | +FID-FIA existence refinement | 1.423206 | 1.258635 | 0.031250 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | 1.532158 | 1.331986 | 0.054125 |
| +FID-FIA existence refinement | 1.432845 | 1.309423 | 0.049875 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cao-Zhao FID-FIA baseline | OSPA | 1.532158 +/- 0.042873 | [1.512093, 1.552223] | 20 |
| +FID-FIA existence refinement | OSPA | 1.432845 +/- 0.042923 | [1.412757, 1.452933] | 20 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.331986 +/- 0.037768 | [1.314310, 1.349661] | 20 |
| +FID-FIA existence refinement | RMSE | 1.309423 +/- 0.156701 | [1.236086, 1.382761] | 20 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.054125 +/- 0.010412 | [0.049252, 0.058998] | 20 |
| +FID-FIA existence refinement | Cardinality | 0.049875 +/- 0.012869 | [0.043852, 0.055898] | 20 |

## Paired Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | OSPA | 0.099313 +/- 0.036158 | [0.082391, 0.116236] | 6.48% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | RMSE | 0.022563 +/- 0.145925 | [-0.045732, 0.090857] | 1.69% | 14/20 | 0.1153 |
| +FID-FIA existence refinement | Cardinality | 0.004250 +/- 0.011322 | [-0.001049, 0.009549] | 7.85% | 13/20 | 0.2632 |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Cao-Zhao FID-FIA baseline | 1.871222 | 1.498863 | 0.217750 |
| +FID-FIA existence refinement | 1.755551 | 1.492790 | 0.191875 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Cao-Zhao FID-FIA baseline | E-OSPA | 1.871222 +/- 0.039415 | [1.852775, 1.889668] | 20 |
| +FID-FIA existence refinement | E-OSPA | 1.755551 +/- 0.041398 | [1.736176, 1.774926] | 20 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.498863 +/- 0.061665 | [1.470003, 1.527723] | 20 |
| +FID-FIA existence refinement | RMSE | 1.492790 +/- 0.165530 | [1.415320, 1.570259] | 20 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.217750 +/- 0.019278 | [0.208728, 0.226772] | 20 |
| +FID-FIA existence refinement | CardErr | 0.191875 +/- 0.017865 | [0.183514, 0.200236] | 20 |

## Paired Local-Metric Improvements Relative to Fixed Weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | E-OSPA | 0.115671 +/- 0.036335 | [0.098665, 0.132676] | 6.18% | 20/20 | 1.907e-06 |
| +FID-FIA existence refinement | RMSE | 0.006073 +/- 0.153080 | [-0.065570, 0.077716] | 0.41% | 14/20 | 0.1153 |
| +FID-FIA existence refinement | CardErr | 0.025875 +/- 0.014702 | [0.018995, 0.032755] | 11.88% | 19/20 | 4.005e-05 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | Cao-Zhao FID-FIA baseline | 1.903236 | 1.510130 | 0.238500 |
| 1 | +FID-FIA existence refinement | 1.794060 | 1.427642 | 0.213000 |
| 2 | Cao-Zhao FID-FIA baseline | 1.899167 | 1.491917 | 0.244000 |
| 2 | +FID-FIA existence refinement | 1.773163 | 1.432334 | 0.209500 |
| 3 | Cao-Zhao FID-FIA baseline | 1.899421 | 1.493854 | 0.243000 |
| 3 | +FID-FIA existence refinement | 1.769452 | 1.438941 | 0.203500 |
| 4 | Cao-Zhao FID-FIA baseline | 1.887837 | 1.495539 | 0.225500 |
| 4 | +FID-FIA existence refinement | 1.762310 | 1.563474 | 0.201000 |
| 5 | Cao-Zhao FID-FIA baseline | 1.858165 | 1.529795 | 0.209500 |
| 5 | +FID-FIA existence refinement | 1.748533 | 1.588655 | 0.189500 |
| 6 | Cao-Zhao FID-FIA baseline | 1.826092 | 1.462675 | 0.186500 |
| 6 | +FID-FIA existence refinement | 1.730692 | 1.444150 | 0.167500 |
| 7 | Cao-Zhao FID-FIA baseline | 1.854512 | 1.478865 | 0.207000 |
| 7 | +FID-FIA existence refinement | 1.745091 | 1.520045 | 0.184500 |
| 8 | Cao-Zhao FID-FIA baseline | 1.841343 | 1.528131 | 0.188000 |
| 8 | +FID-FIA existence refinement | 1.721107 | 1.527079 | 0.166500 |
