# GA Tiered Link Ablation (2026-06-08 00:52:20)

Comparison order: +FID-FIA existence refinement

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

- finalArmMode: fidFiaExistenceRefinement

## Arm Configs
### +FID-FIA existence refinement
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
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
| 1 | 2 | +FID-FIA existence refinement | 1.757154 | 1.552177 | 0.065000 |
| 2 | 3 | +FID-FIA existence refinement | 1.715041 | 1.750601 | 0.056250 |
| 3 | 4 | +FID-FIA existence refinement | 1.714353 | 2.202372 | 0.063750 |
| 4 | 5 | +FID-FIA existence refinement | 1.679356 | 1.489914 | 0.046250 |
| 5 | 6 | +FID-FIA existence refinement | 1.763969 | 1.547200 | 0.063750 |
| 6 | 7 | +FID-FIA existence refinement | 1.785122 | 1.781562 | 0.075000 |
| 7 | 8 | +FID-FIA existence refinement | 1.643277 | 1.455257 | 0.050000 |
| 8 | 9 | +FID-FIA existence refinement | 1.647766 | 1.448913 | 0.042500 |
| 9 | 10 | +FID-FIA existence refinement | 1.761036 | 1.617227 | 0.068750 |
| 10 | 11 | +FID-FIA existence refinement | 1.689414 | 1.611975 | 0.058750 |
| 11 | 12 | +FID-FIA existence refinement | 1.769573 | 1.992613 | 0.076250 |
| 12 | 13 | +FID-FIA existence refinement | 1.724042 | 1.484459 | 0.075000 |
| 13 | 14 | +FID-FIA existence refinement | 1.734384 | 1.493606 | 0.067500 |
| 14 | 15 | +FID-FIA existence refinement | 1.728289 | 1.517674 | 0.083750 |
| 15 | 16 | +FID-FIA existence refinement | 1.600083 | 1.380926 | 0.055000 |
| 16 | 17 | +FID-FIA existence refinement | 1.676793 | 1.571754 | 0.073750 |
| 17 | 18 | +FID-FIA existence refinement | 1.752875 | 1.501910 | 0.065000 |
| 18 | 19 | +FID-FIA existence refinement | 1.653194 | 1.476268 | 0.038750 |
| 19 | 20 | +FID-FIA existence refinement | 1.718724 | 1.539681 | 0.041250 |
| 20 | 21 | +FID-FIA existence refinement | 1.702382 | 1.546150 | 0.045000 |
| 21 | 22 | +FID-FIA existence refinement | 1.715295 | 1.504755 | 0.078750 |
| 22 | 23 | +FID-FIA existence refinement | 1.750890 | 1.533471 | 0.056250 |
| 23 | 24 | +FID-FIA existence refinement | 1.735834 | 1.547131 | 0.042500 |
| 24 | 25 | +FID-FIA existence refinement | 1.769534 | 1.658290 | 0.082500 |
| 25 | 26 | +FID-FIA existence refinement | 1.696862 | 1.504359 | 0.083750 |
| 26 | 27 | +FID-FIA existence refinement | 1.690825 | 1.537240 | 0.053750 |
| 27 | 28 | +FID-FIA existence refinement | 1.705633 | 1.528694 | 0.051250 |
| 28 | 29 | +FID-FIA existence refinement | 1.705834 | 1.530931 | 0.087500 |
| 29 | 30 | +FID-FIA existence refinement | 1.820506 | 1.530765 | 0.080000 |
| 30 | 31 | +FID-FIA existence refinement | 1.709065 | 1.470384 | 0.056250 |
| 31 | 32 | +FID-FIA existence refinement | 1.664397 | 1.485961 | 0.065000 |
| 32 | 33 | +FID-FIA existence refinement | 1.647237 | 1.402984 | 0.065000 |
| 33 | 34 | +FID-FIA existence refinement | 1.776670 | 1.820207 | 0.065000 |
| 34 | 35 | +FID-FIA existence refinement | 1.803691 | 1.586032 | 0.087500 |
| 35 | 36 | +FID-FIA existence refinement | 1.682858 | 2.100397 | 0.042500 |
| 36 | 37 | +FID-FIA existence refinement | 1.698246 | 1.502763 | 0.057500 |
| 37 | 38 | +FID-FIA existence refinement | 1.702149 | 1.901133 | 0.087500 |
| 38 | 39 | +FID-FIA existence refinement | 1.724577 | 1.667896 | 0.051250 |
| 39 | 40 | +FID-FIA existence refinement | 1.672888 | 1.561825 | 0.096250 |
| 40 | 41 | +FID-FIA existence refinement | 1.743135 | 1.782864 | 0.056250 |
| 41 | 42 | +FID-FIA existence refinement | 1.600594 | 1.603406 | 0.047500 |
| 42 | 43 | +FID-FIA existence refinement | 1.728270 | 1.676496 | 0.068750 |
| 43 | 44 | +FID-FIA existence refinement | 1.751952 | 1.533490 | 0.056250 |
| 44 | 45 | +FID-FIA existence refinement | 1.653535 | 1.434471 | 0.045000 |
| 45 | 46 | +FID-FIA existence refinement | 1.721479 | 1.522666 | 0.062500 |
| 46 | 47 | +FID-FIA existence refinement | 1.651413 | 1.514603 | 0.033750 |
| 47 | 48 | +FID-FIA existence refinement | 1.662566 | 1.502862 | 0.033750 |
| 48 | 49 | +FID-FIA existence refinement | 1.749693 | 1.518074 | 0.066250 |
| 49 | 50 | +FID-FIA existence refinement | 1.783401 | 1.550735 | 0.071250 |
| 50 | 51 | +FID-FIA existence refinement | 1.725399 | 1.547485 | 0.042500 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| +FID-FIA existence refinement | 1.713225 | 1.590492 | 0.061700 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +FID-FIA existence refinement | OSPA | 1.713225 +/- 0.048967 | [1.699652, 1.726798] | 50 |
| +FID-FIA existence refinement | RMSE | 1.590492 +/- 0.167107 | [1.544173, 1.636812] | 50 |
| +FID-FIA existence refinement | Cardinality | 0.061700 +/- 0.015557 | [0.057388, 0.066012] | 50 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to +FID-FIA existence refinement | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| +FID-FIA existence refinement | 144.056856 +/- 5.002918 | 1.440569 | 1.000x | 50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to +FID-FIA existence refinement |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | +FID-FIA existence refinement | 139.102550 | 1.391026 | 1.000x |
| 2 | 3 | +FID-FIA existence refinement | 140.935981 | 1.409360 | 1.000x |
| 3 | 4 | +FID-FIA existence refinement | 146.876739 | 1.468767 | 1.000x |
| 4 | 5 | +FID-FIA existence refinement | 144.614688 | 1.446147 | 1.000x |
| 5 | 6 | +FID-FIA existence refinement | 135.707606 | 1.357076 | 1.000x |
| 6 | 7 | +FID-FIA existence refinement | 148.262103 | 1.482621 | 1.000x |
| 7 | 8 | +FID-FIA existence refinement | 143.766740 | 1.437667 | 1.000x |
| 8 | 9 | +FID-FIA existence refinement | 136.176941 | 1.361769 | 1.000x |
| 9 | 10 | +FID-FIA existence refinement | 148.690098 | 1.486901 | 1.000x |
| 10 | 11 | +FID-FIA existence refinement | 137.803299 | 1.378033 | 1.000x |
| 11 | 12 | +FID-FIA existence refinement | 158.132236 | 1.581322 | 1.000x |
| 12 | 13 | +FID-FIA existence refinement | 142.715427 | 1.427154 | 1.000x |
| 13 | 14 | +FID-FIA existence refinement | 155.972264 | 1.559723 | 1.000x |
| 14 | 15 | +FID-FIA existence refinement | 149.181232 | 1.491812 | 1.000x |
| 15 | 16 | +FID-FIA existence refinement | 144.000687 | 1.440007 | 1.000x |
| 16 | 17 | +FID-FIA existence refinement | 138.210872 | 1.382109 | 1.000x |
| 17 | 18 | +FID-FIA existence refinement | 148.383535 | 1.483835 | 1.000x |
| 18 | 19 | +FID-FIA existence refinement | 141.226002 | 1.412260 | 1.000x |
| 19 | 20 | +FID-FIA existence refinement | 135.765378 | 1.357654 | 1.000x |
| 20 | 21 | +FID-FIA existence refinement | 142.711261 | 1.427113 | 1.000x |
| 21 | 22 | +FID-FIA existence refinement | 144.227159 | 1.442272 | 1.000x |
| 22 | 23 | +FID-FIA existence refinement | 142.982392 | 1.429824 | 1.000x |
| 23 | 24 | +FID-FIA existence refinement | 144.759041 | 1.447590 | 1.000x |
| 24 | 25 | +FID-FIA existence refinement | 145.128748 | 1.451287 | 1.000x |
| 25 | 26 | +FID-FIA existence refinement | 149.934398 | 1.499344 | 1.000x |
| 26 | 27 | +FID-FIA existence refinement | 135.190286 | 1.351903 | 1.000x |
| 27 | 28 | +FID-FIA existence refinement | 141.951164 | 1.419512 | 1.000x |
| 28 | 29 | +FID-FIA existence refinement | 139.025853 | 1.390259 | 1.000x |
| 29 | 30 | +FID-FIA existence refinement | 147.637669 | 1.476377 | 1.000x |
| 30 | 31 | +FID-FIA existence refinement | 147.547553 | 1.475476 | 1.000x |
| 31 | 32 | +FID-FIA existence refinement | 143.150368 | 1.431504 | 1.000x |
| 32 | 33 | +FID-FIA existence refinement | 146.239805 | 1.462398 | 1.000x |
| 33 | 34 | +FID-FIA existence refinement | 140.924262 | 1.409243 | 1.000x |
| 34 | 35 | +FID-FIA existence refinement | 141.192627 | 1.411926 | 1.000x |
| 35 | 36 | +FID-FIA existence refinement | 141.894464 | 1.418945 | 1.000x |
| 36 | 37 | +FID-FIA existence refinement | 139.976982 | 1.399770 | 1.000x |
| 37 | 38 | +FID-FIA existence refinement | 151.740132 | 1.517401 | 1.000x |
| 38 | 39 | +FID-FIA existence refinement | 143.142921 | 1.431429 | 1.000x |
| 39 | 40 | +FID-FIA existence refinement | 145.254810 | 1.452548 | 1.000x |
| 40 | 41 | +FID-FIA existence refinement | 141.336505 | 1.413365 | 1.000x |
| 41 | 42 | +FID-FIA existence refinement | 145.770062 | 1.457701 | 1.000x |
| 42 | 43 | +FID-FIA existence refinement | 141.744073 | 1.417441 | 1.000x |
| 43 | 44 | +FID-FIA existence refinement | 139.615973 | 1.396160 | 1.000x |
| 44 | 45 | +FID-FIA existence refinement | 144.241169 | 1.442412 | 1.000x |
| 45 | 46 | +FID-FIA existence refinement | 152.991402 | 1.529914 | 1.000x |
| 46 | 47 | +FID-FIA existence refinement | 146.984573 | 1.469846 | 1.000x |
| 47 | 48 | +FID-FIA existence refinement | 138.908922 | 1.389089 | 1.000x |
| 48 | 49 | +FID-FIA existence refinement | 150.637986 | 1.506380 | 1.000x |
| 49 | 50 | +FID-FIA existence refinement | 144.643839 | 1.446438 | 1.000x |
| 50 | 51 | +FID-FIA existence refinement | 145.832039 | 1.458320 | 1.000x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| +FID-FIA existence refinement | 2.029918 | 1.744456 | 0.208750 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +FID-FIA existence refinement | E-OSPA | 2.029918 +/- 0.042433 | [2.018157, 2.041680] | 50 |
| +FID-FIA existence refinement | RMSE | 1.744456 +/- 0.160642 | [1.699928, 1.788984] | 50 |
| +FID-FIA existence refinement | CardErr | 0.208750 +/- 0.025855 | [0.201583, 0.215917] | 50 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | +FID-FIA existence refinement | 1.990529 | 1.719836 | 0.213200 |
| 2 | +FID-FIA existence refinement | 1.996600 | 1.767235 | 0.210400 |
| 3 | +FID-FIA existence refinement | 1.993062 | 1.735874 | 0.213400 |
| 4 | +FID-FIA existence refinement | 2.072584 | 1.776259 | 0.234600 |
| 5 | +FID-FIA existence refinement | 2.057820 | 1.775836 | 0.210200 |
| 6 | +FID-FIA existence refinement | 2.042391 | 1.717096 | 0.192200 |
| 7 | +FID-FIA existence refinement | 2.055586 | 1.732783 | 0.206400 |
| 8 | +FID-FIA existence refinement | 2.030777 | 1.730731 | 0.189600 |
