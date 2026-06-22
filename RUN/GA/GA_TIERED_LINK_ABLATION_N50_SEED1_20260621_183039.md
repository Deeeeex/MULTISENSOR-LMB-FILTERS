# GA Tiered Link Ablation (2026-06-21 18:30:39)

Comparison order: +structure-aware decoupled KLA -> +FID-FIA existence refinement

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
### +structure-aware decoupled KLA
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
- useFidFiaExistence: 0

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
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.5 0.2]
- Trial 3: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0 0.1 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 5: [0.1 0.1 0.1 0.5 0.2 0 0.1 0.5]
- Trial 6: [0.1 0.2 0.1 0.5 0.1 0.5 0.1 0]
- Trial 7: [0.1 0 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 8: [0 0.1 0.5 0.1 0.1 0.2 0.5 0.1]
- Trial 9: [0.5 0.1 0.1 0 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0 0.5 0.1 0.2 0.1 0.1]
- Trial 11: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 12: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 13: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 14: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 15: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]
- Trial 16: [0.1 0.2 0.1 0.5 0 0.1 0.5 0.1]
- Trial 17: [0.1 0.5 0.1 0.2 0.1 0.5 0.1 0]
- Trial 18: [0.1 0.5 0.1 0.5 0.2 0.1 0.1 0]
- Trial 19: [0.1 0.1 0.1 0.1 0.5 0.5 0 0.2]
- Trial 20: [0.5 0.5 0 0.1 0.1 0.1 0.1 0.2]
- Trial 21: [0.5 0.2 0.5 0.1 0.1 0 0.1 0.1]
- Trial 22: [0.1 0.1 0.1 0.5 0 0.2 0.1 0.5]
- Trial 23: [0.5 0.2 0.1 0.1 0.1 0.1 0 0.5]
- Trial 24: [0.1 0.5 0.2 0 0.5 0.1 0.1 0.1]
- Trial 25: [0.1 0.2 0.1 0.5 0.5 0 0.1 0.1]
- Trial 26: [0.1 0.1 0.5 0.1 0 0.5 0.1 0.2]
- Trial 27: [0.1 0.5 0.1 0.1 0 0.5 0.1 0.2]
- Trial 28: [0 0.5 0.1 0.2 0.1 0.5 0.1 0.1]
- Trial 29: [0.1 0.1 0 0.1 0.5 0.1 0.2 0.5]
- Trial 30: [0.1 0.1 0.5 0 0.1 0.5 0.2 0.1]
- Trial 31: [0.1 0.5 0.5 0.1 0 0.2 0.1 0.1]
- Trial 32: [0.5 0.5 0.1 0.1 0.2 0.1 0 0.1]
- Trial 33: [0.1 0.1 0.1 0 0.5 0.1 0.2 0.5]
- Trial 34: [0.2 0.1 0.1 0 0.5 0.5 0.1 0.1]
- Trial 35: [0.2 0.1 0.5 0.1 0 0.5 0.1 0.1]
- Trial 36: [0.5 0 0.1 0.5 0.2 0.1 0.1 0.1]
- Trial 37: [0.2 0.5 0.1 0.1 0.1 0.5 0 0.1]
- Trial 38: [0.1 0.5 0 0.1 0.5 0.1 0.1 0.2]
- Trial 39: [0.2 0.5 0 0.1 0.1 0.1 0.1 0.5]
- Trial 40: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 41: [0.1 0 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 42: [0.1 0.1 0.5 0.5 0.1 0.2 0 0.1]
- Trial 43: [0.2 0.1 0.5 0 0.1 0.1 0.1 0.5]
- Trial 44: [0.5 0.2 0 0.1 0.5 0.1 0.1 0.1]
- Trial 45: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 46: [0.2 0.1 0.5 0.1 0.1 0.1 0 0.5]
- Trial 47: [0.2 0.1 0 0.1 0.1 0.5 0.5 0.1]
- Trial 48: [0.1 0.1 0.1 0.1 0 0.5 0.2 0.5]
- Trial 49: [0.1 0.5 0.1 0.2 0 0.1 0.5 0.1]
- Trial 50: [0.5 0.5 0.1 0.2 0.1 0.1 0.1 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | +structure-aware decoupled KLA | 1.786737 | 1.505206 | 0.091250 |
| 1 | 2 | +FID-FIA existence refinement | 1.864203 | 1.916324 | 0.078750 |
| 2 | 3 | +structure-aware decoupled KLA | 1.737537 | 1.665656 | 0.068750 |
| 2 | 3 | +FID-FIA existence refinement | 1.763895 | 2.076323 | 0.062500 |
| 3 | 4 | +structure-aware decoupled KLA | 1.709809 | 1.403508 | 0.067500 |
| 3 | 4 | +FID-FIA existence refinement | 1.849800 | 2.159309 | 0.082500 |
| 4 | 5 | +structure-aware decoupled KLA | 1.721621 | 1.382597 | 0.082500 |
| 4 | 5 | +FID-FIA existence refinement | 1.731440 | 1.720011 | 0.077500 |
| 5 | 6 | +structure-aware decoupled KLA | 1.635518 | 1.402009 | 0.063750 |
| 5 | 6 | +FID-FIA existence refinement | 1.719365 | 1.673258 | 0.053750 |
| 6 | 7 | +structure-aware decoupled KLA | 1.629848 | 1.346446 | 0.083750 |
| 6 | 7 | +FID-FIA existence refinement | 1.709856 | 1.632654 | 0.060000 |
| 7 | 8 | +structure-aware decoupled KLA | 1.874411 | 1.628423 | 0.091250 |
| 7 | 8 | +FID-FIA existence refinement | 1.832476 | 1.847821 | 0.077500 |
| 8 | 9 | +structure-aware decoupled KLA | 1.679424 | 1.366166 | 0.091250 |
| 8 | 9 | +FID-FIA existence refinement | 1.777212 | 1.551432 | 0.088750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.723533 | 1.539905 | 0.095000 |
| 9 | 10 | +FID-FIA existence refinement | 1.780593 | 1.639807 | 0.065000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.823001 | 1.440625 | 0.095000 |
| 10 | 11 | +FID-FIA existence refinement | 1.991834 | 2.037224 | 0.108750 |
| 11 | 12 | +structure-aware decoupled KLA | 1.782355 | 1.319966 | 0.103750 |
| 11 | 12 | +FID-FIA existence refinement | 1.982402 | 1.724073 | 0.122500 |
| 12 | 13 | +structure-aware decoupled KLA | 1.861401 | 1.481666 | 0.097500 |
| 12 | 13 | +FID-FIA existence refinement | 1.941604 | 2.206375 | 0.088750 |
| 13 | 14 | +structure-aware decoupled KLA | 1.728110 | 1.444548 | 0.057500 |
| 13 | 14 | +FID-FIA existence refinement | 1.807279 | 1.895908 | 0.062500 |
| 14 | 15 | +structure-aware decoupled KLA | 1.666402 | 1.431406 | 0.033750 |
| 14 | 15 | +FID-FIA existence refinement | 1.860982 | 2.318820 | 0.067500 |
| 15 | 16 | +structure-aware decoupled KLA | 1.789684 | 1.504912 | 0.106250 |
| 15 | 16 | +FID-FIA existence refinement | 1.860412 | 1.625690 | 0.107500 |
| 16 | 17 | +structure-aware decoupled KLA | 1.719580 | 1.463782 | 0.096250 |
| 16 | 17 | +FID-FIA existence refinement | 1.891600 | 1.942158 | 0.095000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.744339 | 1.494156 | 0.071250 |
| 17 | 18 | +FID-FIA existence refinement | 1.870550 | 1.987122 | 0.072500 |
| 18 | 19 | +structure-aware decoupled KLA | 1.727070 | 1.428183 | 0.061250 |
| 18 | 19 | +FID-FIA existence refinement | 1.755856 | 1.720455 | 0.052500 |
| 19 | 20 | +structure-aware decoupled KLA | 1.724584 | 1.438174 | 0.077500 |
| 19 | 20 | +FID-FIA existence refinement | 1.812650 | 2.166721 | 0.075000 |
| 20 | 21 | +structure-aware decoupled KLA | 1.854374 | 1.553011 | 0.100000 |
| 20 | 21 | +FID-FIA existence refinement | 1.873588 | 2.299428 | 0.078750 |
| 21 | 22 | +structure-aware decoupled KLA | 1.731054 | 1.473185 | 0.070000 |
| 21 | 22 | +FID-FIA existence refinement | 1.856755 | 2.041393 | 0.098750 |
| 22 | 23 | +structure-aware decoupled KLA | 1.754388 | 1.467466 | 0.065000 |
| 22 | 23 | +FID-FIA existence refinement | 1.810130 | 1.714854 | 0.066250 |
| 23 | 24 | +structure-aware decoupled KLA | 1.813079 | 1.557879 | 0.060000 |
| 23 | 24 | +FID-FIA existence refinement | 1.875918 | 1.927316 | 0.068750 |
| 24 | 25 | +structure-aware decoupled KLA | 1.823886 | 1.463506 | 0.085000 |
| 24 | 25 | +FID-FIA existence refinement | 1.903106 | 2.612868 | 0.086250 |
| 25 | 26 | +structure-aware decoupled KLA | 1.878592 | 1.627637 | 0.106250 |
| 25 | 26 | +FID-FIA existence refinement | 1.983604 | 2.344187 | 0.102500 |
| 26 | 27 | +structure-aware decoupled KLA | 1.856677 | 1.433120 | 0.101250 |
| 26 | 27 | +FID-FIA existence refinement | 1.820720 | 1.763149 | 0.087500 |
| 27 | 28 | +structure-aware decoupled KLA | 1.948844 | 1.523913 | 0.106250 |
| 27 | 28 | +FID-FIA existence refinement | 1.932416 | 1.874281 | 0.067500 |
| 28 | 29 | +structure-aware decoupled KLA | 1.803150 | 1.353581 | 0.128750 |
| 28 | 29 | +FID-FIA existence refinement | 1.805981 | 2.042940 | 0.085000 |
| 29 | 30 | +structure-aware decoupled KLA | 1.763947 | 1.522050 | 0.083750 |
| 29 | 30 | +FID-FIA existence refinement | 1.987161 | 2.908199 | 0.120000 |
| 30 | 31 | +structure-aware decoupled KLA | 1.726072 | 1.402133 | 0.073750 |
| 30 | 31 | +FID-FIA existence refinement | 1.873040 | 1.839965 | 0.092500 |
| 31 | 32 | +structure-aware decoupled KLA | 1.767941 | 1.527219 | 0.087500 |
| 31 | 32 | +FID-FIA existence refinement | 1.847892 | 1.791024 | 0.071250 |
| 32 | 33 | +structure-aware decoupled KLA | 1.620887 | 1.415614 | 0.053750 |
| 32 | 33 | +FID-FIA existence refinement | 1.787802 | 1.856625 | 0.066250 |
| 33 | 34 | +structure-aware decoupled KLA | 1.765549 | 1.527154 | 0.075000 |
| 33 | 34 | +FID-FIA existence refinement | 1.920479 | 2.100070 | 0.101250 |
| 34 | 35 | +structure-aware decoupled KLA | 1.653995 | 1.371998 | 0.063750 |
| 34 | 35 | +FID-FIA existence refinement | 1.852343 | 2.177672 | 0.083750 |
| 35 | 36 | +structure-aware decoupled KLA | 1.685770 | 1.471884 | 0.055000 |
| 35 | 36 | +FID-FIA existence refinement | 1.739858 | 1.880898 | 0.050000 |
| 36 | 37 | +structure-aware decoupled KLA | 1.764532 | 1.503565 | 0.045000 |
| 36 | 37 | +FID-FIA existence refinement | 1.895357 | 1.739594 | 0.061250 |
| 37 | 38 | +structure-aware decoupled KLA | 1.754326 | 1.353490 | 0.102500 |
| 37 | 38 | +FID-FIA existence refinement | 1.822429 | 1.979147 | 0.108750 |
| 38 | 39 | +structure-aware decoupled KLA | 1.742773 | 1.512837 | 0.087500 |
| 38 | 39 | +FID-FIA existence refinement | 1.786720 | 2.159334 | 0.070000 |
| 39 | 40 | +structure-aware decoupled KLA | 1.873787 | 1.369308 | 0.126250 |
| 39 | 40 | +FID-FIA existence refinement | 1.756536 | 1.722978 | 0.067500 |
| 40 | 41 | +structure-aware decoupled KLA | 1.730708 | 1.442374 | 0.075000 |
| 40 | 41 | +FID-FIA existence refinement | 1.823245 | 2.010510 | 0.063750 |
| 41 | 42 | +structure-aware decoupled KLA | 1.693930 | 1.694101 | 0.078750 |
| 41 | 42 | +FID-FIA existence refinement | 1.742564 | 3.262069 | 0.047500 |
| 42 | 43 | +structure-aware decoupled KLA | 1.794738 | 1.392593 | 0.102500 |
| 42 | 43 | +FID-FIA existence refinement | 1.803182 | 1.751493 | 0.098750 |
| 43 | 44 | +structure-aware decoupled KLA | 1.597010 | 1.326570 | 0.061250 |
| 43 | 44 | +FID-FIA existence refinement | 1.796533 | 1.643096 | 0.101250 |
| 44 | 45 | +structure-aware decoupled KLA | 1.841338 | 1.409389 | 0.103750 |
| 44 | 45 | +FID-FIA existence refinement | 1.936537 | 1.569381 | 0.096250 |
| 45 | 46 | +structure-aware decoupled KLA | 1.725413 | 1.389613 | 0.090000 |
| 45 | 46 | +FID-FIA existence refinement | 1.863483 | 1.625332 | 0.116250 |
| 46 | 47 | +structure-aware decoupled KLA | 1.796416 | 1.482972 | 0.092500 |
| 46 | 47 | +FID-FIA existence refinement | 1.794996 | 1.776425 | 0.067500 |
| 47 | 48 | +structure-aware decoupled KLA | 1.872743 | 1.547357 | 0.093750 |
| 47 | 48 | +FID-FIA existence refinement | 1.874490 | 1.581409 | 0.070000 |
| 48 | 49 | +structure-aware decoupled KLA | 1.729225 | 1.379463 | 0.086250 |
| 48 | 49 | +FID-FIA existence refinement | 1.947176 | 1.753232 | 0.093750 |
| 49 | 50 | +structure-aware decoupled KLA | 1.863970 | 1.491170 | 0.128750 |
| 49 | 50 | +FID-FIA existence refinement | 1.944541 | 2.113026 | 0.086250 |
| 50 | 51 | +structure-aware decoupled KLA | 1.762775 | 1.472266 | 0.082500 |
| 50 | 51 | +FID-FIA existence refinement | 1.920789 | 2.215508 | 0.090000 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| +structure-aware decoupled KLA | 1.761137 | 1.462915 | 0.084125 |
| +FID-FIA existence refinement | 1.847668 | 1.958378 | 0.081275 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | OSPA | 1.761137 +/- 0.075996 | [1.740072, 1.782202] | 50 |
| +FID-FIA existence refinement | OSPA | 1.847668 +/- 0.074235 | [1.827091, 1.868244] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.462915 +/- 0.084865 | [1.439392, 1.486438] | 50 |
| +FID-FIA existence refinement | RMSE | 1.958378 +/- 0.333434 | [1.865955, 2.050801] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.084125 +/- 0.020583 | [0.078420, 0.089830] | 50 |
| +FID-FIA existence refinement | Cardinality | 0.081275 +/- 0.018728 | [0.076084, 0.086466] | 50 |

## Paired Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | OSPA | -0.086531 +/- 0.073900 | [-0.107015, -0.066047] | -4.91% | 5/50 | 4.21e-09 |
| +FID-FIA existence refinement | RMSE | -0.495463 +/- 0.303835 | [-0.579681, -0.411244] | -33.87% | 0/50 | 1.776e-15 |
| +FID-FIA existence refinement | Cardinality | 0.002850 +/- 0.021417 | [-0.003086, 0.008786] | 3.39% | 28/50 | 0.4799 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| +structure-aware decoupled KLA | 56.439069 +/- 7.199003 | 2.351628 | 1.000x | 50 |
| +FID-FIA existence refinement | 125.588774 +/- 15.344780 | 5.232866 | 2.232x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +FID-FIA existence refinement | 69.149704 +/- 10.969146 | 123.18% | 50/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | +structure-aware decoupled KLA | 77.634478 | 3.234770 | 1.000x |
| 1 | 2 | +FID-FIA existence refinement | 164.569289 | 6.857054 | 2.120x |
| 2 | 3 | +structure-aware decoupled KLA | 56.875340 | 2.369806 | 1.000x |
| 2 | 3 | +FID-FIA existence refinement | 123.691255 | 5.153802 | 2.175x |
| 3 | 4 | +structure-aware decoupled KLA | 55.745028 | 2.322710 | 1.000x |
| 3 | 4 | +FID-FIA existence refinement | 121.699040 | 5.070793 | 2.183x |
| 4 | 5 | +structure-aware decoupled KLA | 55.971162 | 2.332132 | 1.000x |
| 4 | 5 | +FID-FIA existence refinement | 133.913100 | 5.579713 | 2.393x |
| 5 | 6 | +structure-aware decoupled KLA | 61.992738 | 2.583031 | 1.000x |
| 5 | 6 | +FID-FIA existence refinement | 106.486854 | 4.436952 | 1.718x |
| 6 | 7 | +structure-aware decoupled KLA | 53.919464 | 2.246644 | 1.000x |
| 6 | 7 | +FID-FIA existence refinement | 125.393400 | 5.224725 | 2.326x |
| 7 | 8 | +structure-aware decoupled KLA | 52.230660 | 2.176277 | 1.000x |
| 7 | 8 | +FID-FIA existence refinement | 121.243296 | 5.051804 | 2.321x |
| 8 | 9 | +structure-aware decoupled KLA | 55.276168 | 2.303174 | 1.000x |
| 8 | 9 | +FID-FIA existence refinement | 117.085064 | 4.878544 | 2.118x |
| 9 | 10 | +structure-aware decoupled KLA | 54.549543 | 2.272898 | 1.000x |
| 9 | 10 | +FID-FIA existence refinement | 116.972402 | 4.873850 | 2.144x |
| 10 | 11 | +structure-aware decoupled KLA | 52.074393 | 2.169766 | 1.000x |
| 10 | 11 | +FID-FIA existence refinement | 111.387950 | 4.641165 | 2.139x |
| 11 | 12 | +structure-aware decoupled KLA | 53.345845 | 2.222744 | 1.000x |
| 11 | 12 | +FID-FIA existence refinement | 136.448673 | 5.685361 | 2.558x |
| 12 | 13 | +structure-aware decoupled KLA | 51.654791 | 2.152283 | 1.000x |
| 12 | 13 | +FID-FIA existence refinement | 114.620465 | 4.775853 | 2.219x |
| 13 | 14 | +structure-aware decoupled KLA | 58.167618 | 2.423651 | 1.000x |
| 13 | 14 | +FID-FIA existence refinement | 140.504544 | 5.854356 | 2.416x |
| 14 | 15 | +structure-aware decoupled KLA | 59.926917 | 2.496955 | 1.000x |
| 14 | 15 | +FID-FIA existence refinement | 129.747278 | 5.406137 | 2.165x |
| 15 | 16 | +structure-aware decoupled KLA | 59.961166 | 2.498382 | 1.000x |
| 15 | 16 | +FID-FIA existence refinement | 168.065258 | 7.002719 | 2.803x |
| 16 | 17 | +structure-aware decoupled KLA | 88.469833 | 3.686243 | 1.000x |
| 16 | 17 | +FID-FIA existence refinement | 153.682150 | 6.403423 | 1.737x |
| 17 | 18 | +structure-aware decoupled KLA | 55.946180 | 2.331091 | 1.000x |
| 17 | 18 | +FID-FIA existence refinement | 117.450956 | 4.893790 | 2.099x |
| 18 | 19 | +structure-aware decoupled KLA | 56.790983 | 2.366291 | 1.000x |
| 18 | 19 | +FID-FIA existence refinement | 119.191189 | 4.966300 | 2.099x |
| 19 | 20 | +structure-aware decoupled KLA | 53.213121 | 2.217213 | 1.000x |
| 19 | 20 | +FID-FIA existence refinement | 116.359681 | 4.848320 | 2.187x |
| 20 | 21 | +structure-aware decoupled KLA | 55.102195 | 2.295925 | 1.000x |
| 20 | 21 | +FID-FIA existence refinement | 122.216973 | 5.092374 | 2.218x |
| 21 | 22 | +structure-aware decoupled KLA | 53.009934 | 2.208747 | 1.000x |
| 21 | 22 | +FID-FIA existence refinement | 120.473957 | 5.019748 | 2.273x |
| 22 | 23 | +structure-aware decoupled KLA | 54.010520 | 2.250438 | 1.000x |
| 22 | 23 | +FID-FIA existence refinement | 117.029879 | 4.876245 | 2.167x |
| 23 | 24 | +structure-aware decoupled KLA | 52.548536 | 2.189522 | 1.000x |
| 23 | 24 | +FID-FIA existence refinement | 116.057801 | 4.835742 | 2.209x |
| 24 | 25 | +structure-aware decoupled KLA | 54.422660 | 2.267611 | 1.000x |
| 24 | 25 | +FID-FIA existence refinement | 124.373793 | 5.182241 | 2.285x |
| 25 | 26 | +structure-aware decoupled KLA | 54.626851 | 2.276119 | 1.000x |
| 25 | 26 | +FID-FIA existence refinement | 121.611998 | 5.067167 | 2.226x |
| 26 | 27 | +structure-aware decoupled KLA | 53.950108 | 2.247921 | 1.000x |
| 26 | 27 | +FID-FIA existence refinement | 112.790741 | 4.699614 | 2.091x |
| 27 | 28 | +structure-aware decoupled KLA | 53.104238 | 2.212677 | 1.000x |
| 27 | 28 | +FID-FIA existence refinement | 120.621606 | 5.025900 | 2.271x |
| 28 | 29 | +structure-aware decoupled KLA | 54.745215 | 2.281051 | 1.000x |
| 28 | 29 | +FID-FIA existence refinement | 119.505341 | 4.979389 | 2.183x |
| 29 | 30 | +structure-aware decoupled KLA | 54.129403 | 2.255392 | 1.000x |
| 29 | 30 | +FID-FIA existence refinement | 130.022633 | 5.417610 | 2.402x |
| 30 | 31 | +structure-aware decoupled KLA | 54.624436 | 2.276018 | 1.000x |
| 30 | 31 | +FID-FIA existence refinement | 123.393443 | 5.141393 | 2.259x |
| 31 | 32 | +structure-aware decoupled KLA | 54.742828 | 2.280951 | 1.000x |
| 31 | 32 | +FID-FIA existence refinement | 124.898179 | 5.204091 | 2.282x |
| 32 | 33 | +structure-aware decoupled KLA | 77.594621 | 3.233109 | 1.000x |
| 32 | 33 | +FID-FIA existence refinement | 170.375379 | 7.098974 | 2.196x |
| 33 | 34 | +structure-aware decoupled KLA | 54.472777 | 2.269699 | 1.000x |
| 33 | 34 | +FID-FIA existence refinement | 132.117180 | 5.504882 | 2.425x |
| 34 | 35 | +structure-aware decoupled KLA | 53.266580 | 2.219441 | 1.000x |
| 34 | 35 | +FID-FIA existence refinement | 117.017081 | 4.875712 | 2.197x |
| 35 | 36 | +structure-aware decoupled KLA | 53.356189 | 2.223175 | 1.000x |
| 35 | 36 | +FID-FIA existence refinement | 114.884844 | 4.786869 | 2.153x |
| 36 | 37 | +structure-aware decoupled KLA | 52.969688 | 2.207070 | 1.000x |
| 36 | 37 | +FID-FIA existence refinement | 123.889518 | 5.162063 | 2.339x |
| 37 | 38 | +structure-aware decoupled KLA | 53.884649 | 2.245194 | 1.000x |
| 37 | 38 | +FID-FIA existence refinement | 129.149046 | 5.381210 | 2.397x |
| 38 | 39 | +structure-aware decoupled KLA | 53.038074 | 2.209920 | 1.000x |
| 38 | 39 | +FID-FIA existence refinement | 111.324659 | 4.638527 | 2.099x |
| 39 | 40 | +structure-aware decoupled KLA | 53.705670 | 2.237736 | 1.000x |
| 39 | 40 | +FID-FIA existence refinement | 111.417133 | 4.642381 | 2.075x |
| 40 | 41 | +structure-aware decoupled KLA | 52.472184 | 2.186341 | 1.000x |
| 40 | 41 | +FID-FIA existence refinement | 121.653355 | 5.068890 | 2.318x |
| 41 | 42 | +structure-aware decoupled KLA | 53.128454 | 2.213686 | 1.000x |
| 41 | 42 | +FID-FIA existence refinement | 114.225286 | 4.759387 | 2.150x |
| 42 | 43 | +structure-aware decoupled KLA | 54.338432 | 2.264101 | 1.000x |
| 42 | 43 | +FID-FIA existence refinement | 128.071170 | 5.336299 | 2.357x |
| 43 | 44 | +structure-aware decoupled KLA | 53.283262 | 2.220136 | 1.000x |
| 43 | 44 | +FID-FIA existence refinement | 117.056626 | 4.877359 | 2.197x |
| 44 | 45 | +structure-aware decoupled KLA | 52.467502 | 2.186146 | 1.000x |
| 44 | 45 | +FID-FIA existence refinement | 119.492843 | 4.978868 | 2.277x |
| 45 | 46 | +structure-aware decoupled KLA | 71.726103 | 2.988588 | 1.000x |
| 45 | 46 | +FID-FIA existence refinement | 173.827013 | 7.242792 | 2.423x |
| 46 | 47 | +structure-aware decoupled KLA | 56.299978 | 2.345832 | 1.000x |
| 46 | 47 | +FID-FIA existence refinement | 118.950670 | 4.956278 | 2.113x |
| 47 | 48 | +structure-aware decoupled KLA | 52.400200 | 2.183342 | 1.000x |
| 47 | 48 | +FID-FIA existence refinement | 113.371809 | 4.723825 | 2.164x |
| 48 | 49 | +structure-aware decoupled KLA | 53.726572 | 2.238607 | 1.000x |
| 48 | 49 | +FID-FIA existence refinement | 127.046047 | 5.293585 | 2.365x |
| 49 | 50 | +structure-aware decoupled KLA | 53.933194 | 2.247216 | 1.000x |
| 49 | 50 | +FID-FIA existence refinement | 121.760740 | 5.073364 | 2.258x |
| 50 | 51 | +structure-aware decoupled KLA | 53.126984 | 2.213624 | 1.000x |
| 50 | 51 | +FID-FIA existence refinement | 122.300102 | 5.095838 | 2.302x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| +structure-aware decoupled KLA | 2.213284 | 4.228361 | 0.203975 |
| +FID-FIA existence refinement | 2.179948 | 4.695098 | 0.151025 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | E-OSPA | 2.213284 +/- 0.079558 | [2.191232, 2.235336] | 50 |
| +FID-FIA existence refinement | E-OSPA | 2.179948 +/- 0.073651 | [2.159533, 2.200363] | 50 |
| +structure-aware decoupled KLA | RMSE | 4.228361 +/- 0.117318 | [4.195842, 4.260880] | 50 |
| +FID-FIA existence refinement | RMSE | 4.695098 +/- 0.408083 | [4.581983, 4.808213] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.203975 +/- 0.038024 | [0.193435, 0.214515] | 50 |
| +FID-FIA existence refinement | CardErr | 0.151025 +/- 0.024511 | [0.144231, 0.157819] | 50 |

## Paired Local-Metric Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | E-OSPA | 0.033336 +/- 0.058164 | [0.017213, 0.049458] | 1.51% | 35/50 | 0.0066 |
| +FID-FIA existence refinement | RMSE | -0.466737 +/- 0.397146 | [-0.576821, -0.356654] | -11.04% | 3/50 | 3.708e-11 |
| +FID-FIA existence refinement | CardErr | 0.052950 +/- 0.025888 | [0.045774, 0.060126] | 25.96% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | +structure-aware decoupled KLA | 2.302742 | 4.330722 | 0.227400 |
| 1 | +FID-FIA existence refinement | 2.268755 | 4.832174 | 0.165400 |
| 2 | +structure-aware decoupled KLA | 2.320684 | 4.397361 | 0.225800 |
| 2 | +FID-FIA existence refinement | 2.260343 | 4.717685 | 0.163000 |
| 3 | +structure-aware decoupled KLA | 2.285117 | 4.324681 | 0.220600 |
| 3 | +FID-FIA existence refinement | 2.264791 | 4.877628 | 0.169000 |
| 4 | +structure-aware decoupled KLA | 2.303555 | 4.339252 | 0.221400 |
| 4 | +FID-FIA existence refinement | 2.265044 | 4.762789 | 0.170200 |
| 5 | +structure-aware decoupled KLA | 2.144313 | 4.130295 | 0.188000 |
| 5 | +FID-FIA existence refinement | 2.125535 | 4.717368 | 0.141600 |
| 6 | +structure-aware decoupled KLA | 2.113302 | 4.054317 | 0.180000 |
| 6 | +FID-FIA existence refinement | 2.091285 | 4.471837 | 0.130800 |
| 7 | +structure-aware decoupled KLA | 2.133297 | 4.135972 | 0.188800 |
| 7 | +FID-FIA existence refinement | 2.114822 | 4.654032 | 0.143200 |
| 8 | +structure-aware decoupled KLA | 2.103261 | 4.114287 | 0.179800 |
| 8 | +FID-FIA existence refinement | 2.049012 | 4.527273 | 0.125000 |
