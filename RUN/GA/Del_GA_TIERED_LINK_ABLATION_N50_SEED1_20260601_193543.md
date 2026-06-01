# GA Tiered Link Ablation (2026-06-01 19:35:43)

Comparison order: fixed weights -> PD-weighted GA

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.000
- pDropLevels: []
- pDropLevelCounts: []

- finalArmMode: fiWeightedGa

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

### PD-weighted GA
- enabled: 1
- method: pdWeightedGa
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useFreshness: 0
- useCtFiDecay: 0
- pdWeightPower: 1.000
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
- Trial 21: [0 0 0 0 0 0 0 0]
- Trial 22: [0 0 0 0 0 0 0 0]
- Trial 23: [0 0 0 0 0 0 0 0]
- Trial 24: [0 0 0 0 0 0 0 0]
- Trial 25: [0 0 0 0 0 0 0 0]
- Trial 26: [0 0 0 0 0 0 0 0]
- Trial 27: [0 0 0 0 0 0 0 0]
- Trial 28: [0 0 0 0 0 0 0 0]
- Trial 29: [0 0 0 0 0 0 0 0]
- Trial 30: [0 0 0 0 0 0 0 0]
- Trial 31: [0 0 0 0 0 0 0 0]
- Trial 32: [0 0 0 0 0 0 0 0]
- Trial 33: [0 0 0 0 0 0 0 0]
- Trial 34: [0 0 0 0 0 0 0 0]
- Trial 35: [0 0 0 0 0 0 0 0]
- Trial 36: [0 0 0 0 0 0 0 0]
- Trial 37: [0 0 0 0 0 0 0 0]
- Trial 38: [0 0 0 0 0 0 0 0]
- Trial 39: [0 0 0 0 0 0 0 0]
- Trial 40: [0 0 0 0 0 0 0 0]
- Trial 41: [0 0 0 0 0 0 0 0]
- Trial 42: [0 0 0 0 0 0 0 0]
- Trial 43: [0 0 0 0 0 0 0 0]
- Trial 44: [0 0 0 0 0 0 0 0]
- Trial 45: [0 0 0 0 0 0 0 0]
- Trial 46: [0 0 0 0 0 0 0 0]
- Trial 47: [0 0 0 0 0 0 0 0]
- Trial 48: [0 0 0 0 0 0 0 0]
- Trial 49: [0 0 0 0 0 0 0 0]
- Trial 50: [0 0 0 0 0 0 0 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 1.650878 | 1.358804 | 0.093750 |
| 1 | 2 | PD-weighted GA | 1.445152 | 1.212161 | 0.061250 |
| 2 | 3 | fixed weights | 1.606838 | 1.378415 | 0.061250 |
| 2 | 3 | PD-weighted GA | 1.403924 | 1.233425 | 0.025000 |
| 3 | 4 | fixed weights | 1.582975 | 1.375559 | 0.066250 |
| 3 | 4 | PD-weighted GA | 1.398030 | 1.219838 | 0.041250 |
| 4 | 5 | fixed weights | 1.661952 | 1.390417 | 0.095000 |
| 4 | 5 | PD-weighted GA | 1.467328 | 1.231699 | 0.065000 |
| 5 | 6 | fixed weights | 1.606861 | 1.371722 | 0.082500 |
| 5 | 6 | PD-weighted GA | 1.417905 | 1.220301 | 0.052500 |
| 6 | 7 | fixed weights | 1.619482 | 1.402597 | 0.082500 |
| 6 | 7 | PD-weighted GA | 1.432995 | 1.238120 | 0.051250 |
| 7 | 8 | fixed weights | 1.609838 | 1.320463 | 0.108750 |
| 7 | 8 | PD-weighted GA | 1.399212 | 1.171448 | 0.056250 |
| 8 | 9 | fixed weights | 1.618389 | 1.339729 | 0.112500 |
| 8 | 9 | PD-weighted GA | 1.419289 | 1.163371 | 0.075000 |
| 9 | 10 | fixed weights | 1.615154 | 1.441848 | 0.048750 |
| 9 | 10 | PD-weighted GA | 1.409543 | 1.254511 | 0.028750 |
| 10 | 11 | fixed weights | 1.634175 | 1.365073 | 0.093750 |
| 10 | 11 | PD-weighted GA | 1.420926 | 1.208938 | 0.051250 |
| 11 | 12 | fixed weights | 1.634779 | 1.436796 | 0.060000 |
| 11 | 12 | PD-weighted GA | 1.430179 | 1.264973 | 0.028750 |
| 12 | 13 | fixed weights | 1.646082 | 1.361098 | 0.095000 |
| 12 | 13 | PD-weighted GA | 1.428221 | 1.209619 | 0.055000 |
| 13 | 14 | fixed weights | 1.698178 | 1.442592 | 0.088750 |
| 13 | 14 | PD-weighted GA | 1.476713 | 1.276853 | 0.051250 |
| 14 | 15 | fixed weights | 1.638514 | 1.375393 | 0.075000 |
| 14 | 15 | PD-weighted GA | 1.456162 | 1.228484 | 0.053750 |
| 15 | 16 | fixed weights | 1.690784 | 1.388506 | 0.108750 |
| 15 | 16 | PD-weighted GA | 1.489073 | 1.237728 | 0.070000 |
| 16 | 17 | fixed weights | 1.669561 | 1.381232 | 0.098750 |
| 16 | 17 | PD-weighted GA | 1.403944 | 1.182372 | 0.050000 |
| 17 | 18 | fixed weights | 1.651100 | 1.411833 | 0.080000 |
| 17 | 18 | PD-weighted GA | 1.462813 | 1.263806 | 0.050000 |
| 18 | 19 | fixed weights | 1.593401 | 1.285643 | 0.113750 |
| 18 | 19 | PD-weighted GA | 1.418961 | 1.149084 | 0.073750 |
| 19 | 20 | fixed weights | 1.604889 | 1.333492 | 0.075000 |
| 19 | 20 | PD-weighted GA | 1.410141 | 1.180229 | 0.048750 |
| 20 | 21 | fixed weights | 1.720672 | 1.434714 | 0.095000 |
| 20 | 21 | PD-weighted GA | 1.469069 | 1.241961 | 0.052500 |
| 21 | 22 | fixed weights | 1.595483 | 1.378553 | 0.073750 |
| 21 | 22 | PD-weighted GA | 1.414167 | 1.227946 | 0.052500 |
| 22 | 23 | fixed weights | 1.612717 | 1.411055 | 0.063750 |
| 22 | 23 | PD-weighted GA | 1.426997 | 1.227765 | 0.046250 |
| 23 | 24 | fixed weights | 1.633547 | 1.404581 | 0.090000 |
| 23 | 24 | PD-weighted GA | 1.466560 | 1.272994 | 0.060000 |
| 24 | 25 | fixed weights | 1.590987 | 1.381488 | 0.065000 |
| 24 | 25 | PD-weighted GA | 1.396393 | 1.220150 | 0.033750 |
| 25 | 26 | fixed weights | 1.574382 | 1.328371 | 0.086250 |
| 25 | 26 | PD-weighted GA | 1.376169 | 1.182817 | 0.045000 |
| 26 | 27 | fixed weights | 1.583305 | 1.308135 | 0.072500 |
| 26 | 27 | PD-weighted GA | 1.428293 | 1.162880 | 0.067500 |
| 27 | 28 | fixed weights | 1.741169 | 1.374388 | 0.121250 |
| 27 | 28 | PD-weighted GA | 1.510058 | 1.185388 | 0.080000 |
| 28 | 29 | fixed weights | 1.700899 | 1.329079 | 0.156250 |
| 28 | 29 | PD-weighted GA | 1.490423 | 1.177226 | 0.108750 |
| 29 | 30 | fixed weights | 1.645895 | 1.392230 | 0.080000 |
| 29 | 30 | PD-weighted GA | 1.444514 | 1.251577 | 0.053750 |
| 30 | 31 | fixed weights | 1.607163 | 1.362216 | 0.076250 |
| 30 | 31 | PD-weighted GA | 1.416793 | 1.218309 | 0.052500 |
| 31 | 32 | fixed weights | 1.607239 | 1.353930 | 0.085000 |
| 31 | 32 | PD-weighted GA | 1.404597 | 1.199261 | 0.047500 |
| 32 | 33 | fixed weights | 1.664329 | 1.367188 | 0.148750 |
| 32 | 33 | PD-weighted GA | 1.420271 | 1.215779 | 0.053750 |
| 33 | 34 | fixed weights | 1.594309 | 1.418825 | 0.061250 |
| 33 | 34 | PD-weighted GA | 1.424628 | 1.225201 | 0.047500 |
| 34 | 35 | fixed weights | 1.710505 | 1.505018 | 0.102500 |
| 34 | 35 | PD-weighted GA | 1.484798 | 1.262316 | 0.068750 |
| 35 | 36 | fixed weights | 1.632125 | 1.390001 | 0.098750 |
| 35 | 36 | PD-weighted GA | 1.415735 | 1.237453 | 0.050000 |
| 36 | 37 | fixed weights | 1.619751 | 1.368349 | 0.092500 |
| 36 | 37 | PD-weighted GA | 1.401466 | 1.210967 | 0.051250 |
| 37 | 38 | fixed weights | 1.576180 | 1.351863 | 0.080000 |
| 37 | 38 | PD-weighted GA | 1.402686 | 1.207278 | 0.055000 |
| 38 | 39 | fixed weights | 1.602643 | 1.362600 | 0.081250 |
| 38 | 39 | PD-weighted GA | 1.453566 | 1.219028 | 0.073750 |
| 39 | 40 | fixed weights | 1.617306 | 1.391714 | 0.071250 |
| 39 | 40 | PD-weighted GA | 1.420238 | 1.215523 | 0.060000 |
| 40 | 41 | fixed weights | 1.600522 | 1.360449 | 0.085000 |
| 40 | 41 | PD-weighted GA | 1.407346 | 1.197404 | 0.057500 |
| 41 | 42 | fixed weights | 1.578523 | 1.357841 | 0.086250 |
| 41 | 42 | PD-weighted GA | 1.410383 | 1.184860 | 0.062500 |
| 42 | 43 | fixed weights | 1.679407 | 1.408957 | 0.086250 |
| 42 | 43 | PD-weighted GA | 1.480046 | 1.244018 | 0.058750 |
| 43 | 44 | fixed weights | 1.613978 | 1.390679 | 0.086250 |
| 43 | 44 | PD-weighted GA | 1.409157 | 1.217004 | 0.050000 |
| 44 | 45 | fixed weights | 1.609875 | 1.414953 | 0.090000 |
| 44 | 45 | PD-weighted GA | 1.426270 | 1.188528 | 0.067500 |
| 45 | 46 | fixed weights | 1.672819 | 1.356082 | 0.115000 |
| 45 | 46 | PD-weighted GA | 1.437125 | 1.211894 | 0.057500 |
| 46 | 47 | fixed weights | 1.667269 | 1.364952 | 0.117500 |
| 46 | 47 | PD-weighted GA | 1.474060 | 1.230280 | 0.077500 |
| 47 | 48 | fixed weights | 1.633059 | 1.427702 | 0.081250 |
| 47 | 48 | PD-weighted GA | 1.426343 | 1.269855 | 0.038750 |
| 48 | 49 | fixed weights | 1.660203 | 1.390574 | 0.111250 |
| 48 | 49 | PD-weighted GA | 1.459983 | 1.216704 | 0.062500 |
| 49 | 50 | fixed weights | 1.673478 | 1.344760 | 0.106250 |
| 49 | 50 | PD-weighted GA | 1.485704 | 1.202666 | 0.072500 |
| 50 | 51 | fixed weights | 1.660892 | 1.479252 | 0.100000 |
| 50 | 51 | PD-weighted GA | 1.437629 | 1.243739 | 0.058750 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.634289 | 1.381434 | 0.090125 |
| PD-weighted GA | 1.434240 | 1.218315 | 0.056250 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.634289 +/- 0.040164 | [1.623156, 1.645422] | 50 |
| PD-weighted GA | OSPA | 1.434240 +/- 0.030798 | [1.425703, 1.442776] | 50 |
| fixed weights | RMSE | 1.381434 +/- 0.040960 | [1.370081, 1.392788] | 50 |
| PD-weighted GA | RMSE | 1.218315 +/- 0.030275 | [1.209923, 1.226706] | 50 |
| fixed weights | Cardinality | 0.090125 +/- 0.020982 | [0.084309, 0.095941] | 50 |
| PD-weighted GA | Cardinality | 0.056250 +/- 0.014370 | [0.052267, 0.060233] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | OSPA | 0.200050 +/- 0.022831 | [0.193721, 0.206378] | 12.24% | 50/50 | 1.776e-15 |
| PD-weighted GA | RMSE | 0.163119 +/- 0.024423 | [0.156350, 0.169889] | 11.81% | 50/50 | 1.776e-15 |
| PD-weighted GA | Cardinality | 0.033875 +/- 0.014369 | [0.029892, 0.037858] | 37.59% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 65.852152 +/- 8.830047 | 0.658522 | 1.000x | 50 |
| PD-weighted GA | 77.746736 +/- 9.916472 | 0.777467 | 1.185x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| PD-weighted GA | 11.894584 +/- 6.512537 | 18.51% | 48/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 55.582538 | 0.555825 | 1.000x |
| 1 | 2 | PD-weighted GA | 66.670146 | 0.666701 | 1.199x |
| 2 | 3 | fixed weights | 55.289668 | 0.552897 | 1.000x |
| 2 | 3 | PD-weighted GA | 66.318884 | 0.663189 | 1.199x |
| 3 | 4 | fixed weights | 57.129928 | 0.571299 | 1.000x |
| 3 | 4 | PD-weighted GA | 69.341815 | 0.693418 | 1.214x |
| 4 | 5 | fixed weights | 55.056596 | 0.550566 | 1.000x |
| 4 | 5 | PD-weighted GA | 69.089758 | 0.690898 | 1.255x |
| 5 | 6 | fixed weights | 64.032717 | 0.640327 | 1.000x |
| 5 | 6 | PD-weighted GA | 68.880693 | 0.688807 | 1.076x |
| 6 | 7 | fixed weights | 67.520180 | 0.675202 | 1.000x |
| 6 | 7 | PD-weighted GA | 82.450722 | 0.824507 | 1.221x |
| 7 | 8 | fixed weights | 65.052129 | 0.650521 | 1.000x |
| 7 | 8 | PD-weighted GA | 76.557429 | 0.765574 | 1.177x |
| 8 | 9 | fixed weights | 73.749732 | 0.737497 | 1.000x |
| 8 | 9 | PD-weighted GA | 77.805608 | 0.778056 | 1.055x |
| 9 | 10 | fixed weights | 63.090715 | 0.630907 | 1.000x |
| 9 | 10 | PD-weighted GA | 72.175056 | 0.721751 | 1.144x |
| 10 | 11 | fixed weights | 57.743155 | 0.577432 | 1.000x |
| 10 | 11 | PD-weighted GA | 68.547021 | 0.685470 | 1.187x |
| 11 | 12 | fixed weights | 58.146263 | 0.581463 | 1.000x |
| 11 | 12 | PD-weighted GA | 69.849071 | 0.698491 | 1.201x |
| 12 | 13 | fixed weights | 58.340718 | 0.583407 | 1.000x |
| 12 | 13 | PD-weighted GA | 71.760090 | 0.717601 | 1.230x |
| 13 | 14 | fixed weights | 71.614358 | 0.716144 | 1.000x |
| 13 | 14 | PD-weighted GA | 71.040096 | 0.710401 | 0.992x |
| 14 | 15 | fixed weights | 59.313415 | 0.593134 | 1.000x |
| 14 | 15 | PD-weighted GA | 70.740325 | 0.707403 | 1.193x |
| 15 | 16 | fixed weights | 60.290263 | 0.602903 | 1.000x |
| 15 | 16 | PD-weighted GA | 81.994707 | 0.819947 | 1.360x |
| 16 | 17 | fixed weights | 89.146366 | 0.891464 | 1.000x |
| 16 | 17 | PD-weighted GA | 102.242262 | 1.022423 | 1.147x |
| 17 | 18 | fixed weights | 60.352306 | 0.603523 | 1.000x |
| 17 | 18 | PD-weighted GA | 73.594714 | 0.735947 | 1.219x |
| 18 | 19 | fixed weights | 69.167910 | 0.691679 | 1.000x |
| 18 | 19 | PD-weighted GA | 69.345462 | 0.693455 | 1.003x |
| 19 | 20 | fixed weights | 59.388197 | 0.593882 | 1.000x |
| 19 | 20 | PD-weighted GA | 71.977309 | 0.719773 | 1.212x |
| 20 | 21 | fixed weights | 58.961624 | 0.589616 | 1.000x |
| 20 | 21 | PD-weighted GA | 71.326988 | 0.713270 | 1.210x |
| 21 | 22 | fixed weights | 61.943590 | 0.619436 | 1.000x |
| 21 | 22 | PD-weighted GA | 73.064223 | 0.730642 | 1.180x |
| 22 | 23 | fixed weights | 61.201062 | 0.612011 | 1.000x |
| 22 | 23 | PD-weighted GA | 71.578874 | 0.715789 | 1.170x |
| 23 | 24 | fixed weights | 61.081987 | 0.610820 | 1.000x |
| 23 | 24 | PD-weighted GA | 72.419374 | 0.724194 | 1.186x |
| 24 | 25 | fixed weights | 60.372505 | 0.603725 | 1.000x |
| 24 | 25 | PD-weighted GA | 72.683255 | 0.726833 | 1.204x |
| 25 | 26 | fixed weights | 61.152914 | 0.611529 | 1.000x |
| 25 | 26 | PD-weighted GA | 72.721444 | 0.727214 | 1.189x |
| 26 | 27 | fixed weights | 61.082738 | 0.610827 | 1.000x |
| 26 | 27 | PD-weighted GA | 72.069796 | 0.720698 | 1.180x |
| 27 | 28 | fixed weights | 60.745256 | 0.607453 | 1.000x |
| 27 | 28 | PD-weighted GA | 72.618795 | 0.726188 | 1.195x |
| 28 | 29 | fixed weights | 61.218494 | 0.612185 | 1.000x |
| 28 | 29 | PD-weighted GA | 74.124608 | 0.741246 | 1.211x |
| 29 | 30 | fixed weights | 61.053673 | 0.610537 | 1.000x |
| 29 | 30 | PD-weighted GA | 73.423410 | 0.734234 | 1.203x |
| 30 | 31 | fixed weights | 60.723358 | 0.607234 | 1.000x |
| 30 | 31 | PD-weighted GA | 71.990924 | 0.719909 | 1.186x |
| 31 | 32 | fixed weights | 67.209351 | 0.672094 | 1.000x |
| 31 | 32 | PD-weighted GA | 83.429032 | 0.834290 | 1.241x |
| 32 | 33 | fixed weights | 69.164411 | 0.691644 | 1.000x |
| 32 | 33 | PD-weighted GA | 81.673457 | 0.816735 | 1.181x |
| 33 | 34 | fixed weights | 69.740181 | 0.697402 | 1.000x |
| 33 | 34 | PD-weighted GA | 81.315020 | 0.813150 | 1.166x |
| 34 | 35 | fixed weights | 69.969060 | 0.699691 | 1.000x |
| 34 | 35 | PD-weighted GA | 82.343664 | 0.823437 | 1.177x |
| 35 | 36 | fixed weights | 68.535236 | 0.685352 | 1.000x |
| 35 | 36 | PD-weighted GA | 80.984368 | 0.809844 | 1.182x |
| 36 | 37 | fixed weights | 69.362689 | 0.693627 | 1.000x |
| 36 | 37 | PD-weighted GA | 81.881691 | 0.818817 | 1.180x |
| 37 | 38 | fixed weights | 70.166855 | 0.701669 | 1.000x |
| 37 | 38 | PD-weighted GA | 81.408176 | 0.814082 | 1.160x |
| 38 | 39 | fixed weights | 70.710386 | 0.707104 | 1.000x |
| 38 | 39 | PD-weighted GA | 81.630399 | 0.816304 | 1.154x |
| 39 | 40 | fixed weights | 66.882828 | 0.668828 | 1.000x |
| 39 | 40 | PD-weighted GA | 83.145771 | 0.831458 | 1.243x |
| 40 | 41 | fixed weights | 69.248232 | 0.692482 | 1.000x |
| 40 | 41 | PD-weighted GA | 86.558967 | 0.865590 | 1.250x |
| 41 | 42 | fixed weights | 68.335046 | 0.683350 | 1.000x |
| 41 | 42 | PD-weighted GA | 107.618638 | 1.076186 | 1.575x |
| 42 | 43 | fixed weights | 99.530429 | 0.995304 | 1.000x |
| 42 | 43 | PD-weighted GA | 88.675206 | 0.886752 | 0.891x |
| 43 | 44 | fixed weights | 65.325468 | 0.653255 | 1.000x |
| 43 | 44 | PD-weighted GA | 80.537069 | 0.805371 | 1.233x |
| 44 | 45 | fixed weights | 94.948802 | 0.949488 | 1.000x |
| 44 | 45 | PD-weighted GA | 118.932932 | 1.189329 | 1.253x |
| 45 | 46 | fixed weights | 69.227814 | 0.692278 | 1.000x |
| 45 | 46 | PD-weighted GA | 77.269269 | 0.772693 | 1.116x |
| 46 | 47 | fixed weights | 70.362255 | 0.703623 | 1.000x |
| 46 | 47 | PD-weighted GA | 77.101968 | 0.771020 | 1.096x |
| 47 | 48 | fixed weights | 65.552079 | 0.655521 | 1.000x |
| 47 | 48 | PD-weighted GA | 77.342200 | 0.773422 | 1.180x |
| 48 | 49 | fixed weights | 67.505581 | 0.675056 | 1.000x |
| 48 | 49 | PD-weighted GA | 79.009255 | 0.790093 | 1.170x |
| 49 | 50 | fixed weights | 65.712931 | 0.657129 | 1.000x |
| 49 | 50 | PD-weighted GA | 79.464563 | 0.794646 | 1.209x |
| 50 | 51 | fixed weights | 65.573599 | 0.655736 | 1.000x |
| 50 | 51 | PD-weighted GA | 78.612300 | 0.786123 | 1.199x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 1.908084 | 1.441941 | 0.281625 |
| PD-weighted GA | 1.825248 | 1.376965 | 0.243300 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 1.908084 +/- 0.054512 | [1.892974, 1.923194] | 50 |
| PD-weighted GA | E-OSPA | 1.825248 +/- 0.053653 | [1.810376, 1.840119] | 50 |
| fixed weights | RMSE | 1.441941 +/- 0.034792 | [1.432297, 1.451585] | 50 |
| PD-weighted GA | RMSE | 1.376965 +/- 0.035204 | [1.367207, 1.386723] | 50 |
| fixed weights | CardErr | 0.281625 +/- 0.032507 | [0.272614, 0.290636] | 50 |
| PD-weighted GA | CardErr | 0.243300 +/- 0.027076 | [0.235795, 0.250805] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | E-OSPA | 0.082836 +/- 0.012106 | [0.079481, 0.086192] | 4.34% | 50/50 | 1.776e-15 |
| PD-weighted GA | RMSE | 0.064976 +/- 0.005446 | [0.063466, 0.066485] | 4.51% | 50/50 | 1.776e-15 |
| PD-weighted GA | CardErr | 0.038325 +/- 0.014886 | [0.034199, 0.042451] | 13.61% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 1.958102 | 1.448123 | 0.345600 |
| 1 | PD-weighted GA | 1.848195 | 1.395594 | 0.244400 |
| 2 | fixed weights | 1.930062 | 1.451044 | 0.294600 |
| 2 | PD-weighted GA | 1.851055 | 1.390137 | 0.251400 |
| 3 | fixed weights | 1.951673 | 1.443968 | 0.343800 |
| 3 | PD-weighted GA | 1.839826 | 1.386813 | 0.243200 |
| 4 | fixed weights | 1.915241 | 1.452566 | 0.277600 |
| 4 | PD-weighted GA | 1.833988 | 1.387491 | 0.238800 |
| 5 | fixed weights | 1.891303 | 1.442694 | 0.256200 |
| 5 | PD-weighted GA | 1.811739 | 1.367902 | 0.240600 |
| 6 | fixed weights | 1.877376 | 1.437574 | 0.241600 |
| 6 | PD-weighted GA | 1.802841 | 1.363825 | 0.239600 |
| 7 | fixed weights | 1.884340 | 1.435728 | 0.255200 |
| 7 | PD-weighted GA | 1.812444 | 1.366899 | 0.244600 |
| 8 | fixed weights | 1.856573 | 1.423829 | 0.238400 |
| 8 | PD-weighted GA | 1.801892 | 1.357059 | 0.243800 |
