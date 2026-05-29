# GA Tiered Link Ablation (2026-05-29 01:59:09)

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
| 1 | 2 | fixed weights | 2.697089 | 2.361155 | 0.901250 |
| 1 | 2 | +structure-aware decoupled KLA | 1.802657 | 1.724150 | 0.183750 |
| 2 | 3 | fixed weights | 2.682917 | 2.987584 | 0.676250 |
| 2 | 3 | +structure-aware decoupled KLA | 1.839445 | 1.473470 | 0.178750 |
| 3 | 4 | fixed weights | 2.881135 | 3.089446 | 1.373750 |
| 3 | 4 | +structure-aware decoupled KLA | 1.744157 | 1.521012 | 0.177500 |
| 4 | 5 | fixed weights | 2.404644 | 2.270953 | 0.748750 |
| 4 | 5 | +structure-aware decoupled KLA | 1.741661 | 1.787408 | 0.171250 |
| 5 | 6 | fixed weights | 2.825544 | 2.971556 | 1.146250 |
| 5 | 6 | +structure-aware decoupled KLA | 1.948914 | 1.792190 | 0.273750 |
| 6 | 7 | fixed weights | 2.740126 | 3.106129 | 0.800000 |
| 6 | 7 | +structure-aware decoupled KLA | 1.898312 | 2.083014 | 0.211250 |
| 7 | 8 | fixed weights | 2.551920 | 2.625517 | 0.745000 |
| 7 | 8 | +structure-aware decoupled KLA | 1.780790 | 1.375747 | 0.177500 |
| 8 | 9 | fixed weights | 2.499336 | 2.903023 | 0.665000 |
| 8 | 9 | +structure-aware decoupled KLA | 1.803270 | 1.770846 | 0.158750 |
| 9 | 10 | fixed weights | 2.444556 | 3.485928 | 0.537500 |
| 9 | 10 | +structure-aware decoupled KLA | 1.788458 | 1.627189 | 0.142500 |
| 10 | 11 | fixed weights | 2.443256 | 2.177813 | 0.692500 |
| 10 | 11 | +structure-aware decoupled KLA | 1.719612 | 1.379906 | 0.151250 |
| 11 | 12 | fixed weights | 2.462705 | 2.769625 | 0.538750 |
| 11 | 12 | +structure-aware decoupled KLA | 1.695273 | 1.402323 | 0.177500 |
| 12 | 13 | fixed weights | 2.761884 | 2.323788 | 1.000000 |
| 12 | 13 | +structure-aware decoupled KLA | 1.817693 | 1.458688 | 0.210000 |
| 13 | 14 | fixed weights | 2.693921 | 3.347580 | 1.003750 |
| 13 | 14 | +structure-aware decoupled KLA | 1.883752 | 1.687151 | 0.182500 |
| 14 | 15 | fixed weights | 2.360208 | 2.035834 | 0.510000 |
| 14 | 15 | +structure-aware decoupled KLA | 1.844936 | 1.592009 | 0.173750 |
| 15 | 16 | fixed weights | 2.648751 | 2.634546 | 1.142500 |
| 15 | 16 | +structure-aware decoupled KLA | 1.812853 | 1.625822 | 0.207500 |
| 16 | 17 | fixed weights | 2.864129 | 2.589842 | 1.075000 |
| 16 | 17 | +structure-aware decoupled KLA | 1.827445 | 1.547472 | 0.280000 |
| 17 | 18 | fixed weights | 3.108248 | 2.480070 | 1.340000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.898279 | 1.764562 | 0.217500 |
| 18 | 19 | fixed weights | 2.555419 | 1.950384 | 0.905000 |
| 18 | 19 | +structure-aware decoupled KLA | 1.744814 | 1.523687 | 0.197500 |
| 19 | 20 | fixed weights | 2.624366 | 2.025804 | 1.207500 |
| 19 | 20 | +structure-aware decoupled KLA | 1.742347 | 1.498961 | 0.198750 |
| 20 | 21 | fixed weights | 2.768340 | 2.934217 | 1.001250 |
| 20 | 21 | +structure-aware decoupled KLA | 1.791306 | 1.370516 | 0.181250 |
| 21 | 22 | fixed weights | 3.200506 | 3.869351 | 1.265000 |
| 21 | 22 | +structure-aware decoupled KLA | 1.914776 | 1.975625 | 0.242500 |
| 22 | 23 | fixed weights | 2.499708 | 2.389196 | 0.800000 |
| 22 | 23 | +structure-aware decoupled KLA | 1.743763 | 1.497485 | 0.181250 |
| 23 | 24 | fixed weights | 2.780341 | 2.437650 | 1.300000 |
| 23 | 24 | +structure-aware decoupled KLA | 1.810097 | 1.470928 | 0.197500 |
| 24 | 25 | fixed weights | 2.478582 | 2.501270 | 0.745000 |
| 24 | 25 | +structure-aware decoupled KLA | 1.812740 | 1.515910 | 0.222500 |
| 25 | 26 | fixed weights | 2.684841 | 3.377187 | 0.931250 |
| 25 | 26 | +structure-aware decoupled KLA | 1.771884 | 1.410888 | 0.227500 |
| 26 | 27 | fixed weights | 2.356044 | 2.550241 | 0.612500 |
| 26 | 27 | +structure-aware decoupled KLA | 1.727849 | 1.435209 | 0.163750 |
| 27 | 28 | fixed weights | 2.650818 | 2.757309 | 0.978750 |
| 27 | 28 | +structure-aware decoupled KLA | 1.878487 | 1.523055 | 0.256250 |
| 28 | 29 | fixed weights | 2.819009 | 2.438367 | 1.111250 |
| 28 | 29 | +structure-aware decoupled KLA | 1.753607 | 1.520760 | 0.158750 |
| 29 | 30 | fixed weights | 3.142839 | 2.738929 | 1.862500 |
| 29 | 30 | +structure-aware decoupled KLA | 1.845443 | 1.672481 | 0.246250 |
| 30 | 31 | fixed weights | 3.001240 | 2.359138 | 1.752500 |
| 30 | 31 | +structure-aware decoupled KLA | 1.837375 | 1.531012 | 0.205000 |
| 31 | 32 | fixed weights | 2.776722 | 2.722199 | 0.917500 |
| 31 | 32 | +structure-aware decoupled KLA | 1.778825 | 1.526053 | 0.173750 |
| 32 | 33 | fixed weights | 2.242943 | 2.420053 | 0.476250 |
| 32 | 33 | +structure-aware decoupled KLA | 1.749809 | 1.376511 | 0.173750 |
| 33 | 34 | fixed weights | 2.711200 | 3.177628 | 1.140000 |
| 33 | 34 | +structure-aware decoupled KLA | 1.728188 | 1.487920 | 0.181250 |
| 34 | 35 | fixed weights | 3.005500 | 2.707829 | 1.385000 |
| 34 | 35 | +structure-aware decoupled KLA | 1.918878 | 1.738010 | 0.207500 |
| 35 | 36 | fixed weights | 2.622531 | 2.239509 | 1.121250 |
| 35 | 36 | +structure-aware decoupled KLA | 1.803747 | 1.475750 | 0.220000 |
| 36 | 37 | fixed weights | 2.902576 | 2.811414 | 1.215000 |
| 36 | 37 | +structure-aware decoupled KLA | 1.854738 | 1.542916 | 0.225000 |
| 37 | 38 | fixed weights | 2.748929 | 2.514925 | 1.002500 |
| 37 | 38 | +structure-aware decoupled KLA | 1.742034 | 1.619281 | 0.158750 |
| 38 | 39 | fixed weights | 2.366937 | 3.692156 | 0.537500 |
| 38 | 39 | +structure-aware decoupled KLA | 1.808801 | 1.731693 | 0.151250 |
| 39 | 40 | fixed weights | 2.475212 | 2.475429 | 0.587500 |
| 39 | 40 | +structure-aware decoupled KLA | 1.770414 | 1.543190 | 0.186250 |
| 40 | 41 | fixed weights | 2.524640 | 2.607918 | 0.681250 |
| 40 | 41 | +structure-aware decoupled KLA | 1.780575 | 1.499820 | 0.196250 |
| 41 | 42 | fixed weights | 2.309550 | 2.631592 | 0.510000 |
| 41 | 42 | +structure-aware decoupled KLA | 1.639814 | 1.412486 | 0.122500 |
| 42 | 43 | fixed weights | 2.550971 | 2.762190 | 0.603750 |
| 42 | 43 | +structure-aware decoupled KLA | 1.736881 | 1.502225 | 0.161250 |
| 43 | 44 | fixed weights | 2.450680 | 2.075874 | 0.762500 |
| 43 | 44 | +structure-aware decoupled KLA | 1.803029 | 1.513847 | 0.190000 |
| 44 | 45 | fixed weights | 2.716512 | 3.393022 | 1.031250 |
| 44 | 45 | +structure-aware decoupled KLA | 1.818592 | 1.646521 | 0.190000 |
| 45 | 46 | fixed weights | 2.748964 | 2.194313 | 1.286250 |
| 45 | 46 | +structure-aware decoupled KLA | 1.683714 | 1.391127 | 0.173750 |
| 46 | 47 | fixed weights | 2.781531 | 3.001996 | 0.825000 |
| 46 | 47 | +structure-aware decoupled KLA | 1.865267 | 1.583016 | 0.190000 |
| 47 | 48 | fixed weights | 2.567562 | 2.999751 | 0.587500 |
| 47 | 48 | +structure-aware decoupled KLA | 1.787562 | 1.456953 | 0.177500 |
| 48 | 49 | fixed weights | 2.513590 | 2.527542 | 1.123750 |
| 48 | 49 | +structure-aware decoupled KLA | 1.873154 | 1.659861 | 0.226250 |
| 49 | 50 | fixed weights | 2.763545 | 2.489193 | 1.013750 |
| 49 | 50 | +structure-aware decoupled KLA | 1.775829 | 1.487938 | 0.157500 |
| 50 | 51 | fixed weights | 3.132377 | 4.291266 | 1.063750 |
| 50 | 51 | +structure-aware decoupled KLA | 1.844628 | 1.725925 | 0.208750 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.670898 | 2.724505 | 0.944800 |
| +structure-aware decoupled KLA | 1.799730 | 1.569610 | 0.192500 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.670898 +/- 0.225844 | [2.608297, 2.733499] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.799730 +/- 0.064590 | [1.781826, 1.817633] | 50 |
| fixed weights | RMSE | 2.724505 +/- 0.486455 | [2.589666, 2.859343] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.569610 +/- 0.152480 | [1.527345, 1.611876] | 50 |
| fixed weights | Cardinality | 0.944800 +/- 0.314742 | [0.857558, 1.032042] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.192500 +/- 0.032530 | [0.183483, 0.201517] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.871168 +/- 0.198848 | [0.816050, 0.926286] | 32.62% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 1.154894 +/- 0.447651 | [1.030812, 1.278977] | 42.39% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.752300 +/- 0.298743 | [0.669493, 0.835107] | 79.63% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 53.093984 +/- 6.238569 | 0.530940 | 1.000x | 50 |
| +structure-aware decoupled KLA | 56.145436 +/- 6.958910 | 0.561454 | 1.061x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 3.051451 +/- 6.149072 | 6.12% | 44/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 52.597750 | 0.525977 | 1.000x |
| 1 | 2 | +structure-aware decoupled KLA | 56.848218 | 0.568482 | 1.081x |
| 2 | 3 | fixed weights | 54.439151 | 0.544392 | 1.000x |
| 2 | 3 | +structure-aware decoupled KLA | 57.057278 | 0.570573 | 1.048x |
| 3 | 4 | fixed weights | 56.063560 | 0.560636 | 1.000x |
| 3 | 4 | +structure-aware decoupled KLA | 56.936497 | 0.569365 | 1.016x |
| 4 | 5 | fixed weights | 55.897934 | 0.558979 | 1.000x |
| 4 | 5 | +structure-aware decoupled KLA | 60.689387 | 0.606894 | 1.086x |
| 5 | 6 | fixed weights | 82.809765 | 0.828098 | 1.000x |
| 5 | 6 | +structure-aware decoupled KLA | 59.308666 | 0.593087 | 0.716x |
| 6 | 7 | fixed weights | 51.882540 | 0.518825 | 1.000x |
| 6 | 7 | +structure-aware decoupled KLA | 56.536524 | 0.565365 | 1.090x |
| 7 | 8 | fixed weights | 55.185108 | 0.551851 | 1.000x |
| 7 | 8 | +structure-aware decoupled KLA | 61.927757 | 0.619278 | 1.122x |
| 8 | 9 | fixed weights | 58.593737 | 0.585937 | 1.000x |
| 8 | 9 | +structure-aware decoupled KLA | 60.128166 | 0.601282 | 1.026x |
| 9 | 10 | fixed weights | 61.075812 | 0.610758 | 1.000x |
| 9 | 10 | +structure-aware decoupled KLA | 63.599719 | 0.635997 | 1.041x |
| 10 | 11 | fixed weights | 58.270961 | 0.582710 | 1.000x |
| 10 | 11 | +structure-aware decoupled KLA | 61.472274 | 0.614723 | 1.055x |
| 11 | 12 | fixed weights | 57.909694 | 0.579097 | 1.000x |
| 11 | 12 | +structure-aware decoupled KLA | 64.929546 | 0.649295 | 1.121x |
| 12 | 13 | fixed weights | 59.415654 | 0.594157 | 1.000x |
| 12 | 13 | +structure-aware decoupled KLA | 60.278524 | 0.602785 | 1.015x |
| 13 | 14 | fixed weights | 54.640057 | 0.546401 | 1.000x |
| 13 | 14 | +structure-aware decoupled KLA | 59.214709 | 0.592147 | 1.084x |
| 14 | 15 | fixed weights | 53.239368 | 0.532394 | 1.000x |
| 14 | 15 | +structure-aware decoupled KLA | 57.269122 | 0.572691 | 1.076x |
| 15 | 16 | fixed weights | 57.234053 | 0.572341 | 1.000x |
| 15 | 16 | +structure-aware decoupled KLA | 81.686196 | 0.816862 | 1.427x |
| 16 | 17 | fixed weights | 55.784617 | 0.557846 | 1.000x |
| 16 | 17 | +structure-aware decoupled KLA | 55.833630 | 0.558336 | 1.001x |
| 17 | 18 | fixed weights | 52.290384 | 0.522904 | 1.000x |
| 17 | 18 | +structure-aware decoupled KLA | 55.132336 | 0.551323 | 1.054x |
| 18 | 19 | fixed weights | 47.862355 | 0.478624 | 1.000x |
| 18 | 19 | +structure-aware decoupled KLA | 52.563766 | 0.525638 | 1.098x |
| 19 | 20 | fixed weights | 54.344663 | 0.543447 | 1.000x |
| 19 | 20 | +structure-aware decoupled KLA | 54.899327 | 0.548993 | 1.010x |
| 20 | 21 | fixed weights | 50.328692 | 0.503287 | 1.000x |
| 20 | 21 | +structure-aware decoupled KLA | 54.530970 | 0.545310 | 1.083x |
| 21 | 22 | fixed weights | 61.928096 | 0.619281 | 1.000x |
| 21 | 22 | +structure-aware decoupled KLA | 66.564454 | 0.665645 | 1.075x |
| 22 | 23 | fixed weights | 56.228199 | 0.562282 | 1.000x |
| 22 | 23 | +structure-aware decoupled KLA | 55.300046 | 0.553000 | 0.983x |
| 23 | 24 | fixed weights | 49.080079 | 0.490801 | 1.000x |
| 23 | 24 | +structure-aware decoupled KLA | 52.698558 | 0.526986 | 1.074x |
| 24 | 25 | fixed weights | 49.618224 | 0.496182 | 1.000x |
| 24 | 25 | +structure-aware decoupled KLA | 53.623501 | 0.536235 | 1.081x |
| 25 | 26 | fixed weights | 52.010110 | 0.520101 | 1.000x |
| 25 | 26 | +structure-aware decoupled KLA | 54.109141 | 0.541091 | 1.040x |
| 26 | 27 | fixed weights | 52.115697 | 0.521157 | 1.000x |
| 26 | 27 | +structure-aware decoupled KLA | 57.556769 | 0.575568 | 1.104x |
| 27 | 28 | fixed weights | 55.897449 | 0.558974 | 1.000x |
| 27 | 28 | +structure-aware decoupled KLA | 50.494831 | 0.504948 | 0.903x |
| 28 | 29 | fixed weights | 47.929257 | 0.479293 | 1.000x |
| 28 | 29 | +structure-aware decoupled KLA | 52.849988 | 0.528500 | 1.103x |
| 29 | 30 | fixed weights | 47.996030 | 0.479960 | 1.000x |
| 29 | 30 | +structure-aware decoupled KLA | 55.073101 | 0.550731 | 1.147x |
| 30 | 31 | fixed weights | 56.893447 | 0.568934 | 1.000x |
| 30 | 31 | +structure-aware decoupled KLA | 51.441465 | 0.514415 | 0.904x |
| 31 | 32 | fixed weights | 52.550143 | 0.525501 | 1.000x |
| 31 | 32 | +structure-aware decoupled KLA | 51.318646 | 0.513186 | 0.977x |
| 32 | 33 | fixed weights | 51.963117 | 0.519631 | 1.000x |
| 32 | 33 | +structure-aware decoupled KLA | 53.957565 | 0.539576 | 1.038x |
| 33 | 34 | fixed weights | 50.081607 | 0.500816 | 1.000x |
| 33 | 34 | +structure-aware decoupled KLA | 53.105861 | 0.531059 | 1.060x |
| 34 | 35 | fixed weights | 51.496419 | 0.514964 | 1.000x |
| 34 | 35 | +structure-aware decoupled KLA | 53.068870 | 0.530689 | 1.031x |
| 35 | 36 | fixed weights | 52.063563 | 0.520636 | 1.000x |
| 35 | 36 | +structure-aware decoupled KLA | 70.944289 | 0.709443 | 1.363x |
| 36 | 37 | fixed weights | 54.064940 | 0.540649 | 1.000x |
| 36 | 37 | +structure-aware decoupled KLA | 52.931351 | 0.529314 | 0.979x |
| 37 | 38 | fixed weights | 48.624046 | 0.486240 | 1.000x |
| 37 | 38 | +structure-aware decoupled KLA | 50.363685 | 0.503637 | 1.036x |
| 38 | 39 | fixed weights | 48.343722 | 0.483437 | 1.000x |
| 38 | 39 | +structure-aware decoupled KLA | 49.711842 | 0.497118 | 1.028x |
| 39 | 40 | fixed weights | 47.669263 | 0.476693 | 1.000x |
| 39 | 40 | +structure-aware decoupled KLA | 50.575523 | 0.505755 | 1.061x |
| 40 | 41 | fixed weights | 47.298031 | 0.472980 | 1.000x |
| 40 | 41 | +structure-aware decoupled KLA | 49.762934 | 0.497629 | 1.052x |
| 41 | 42 | fixed weights | 47.256722 | 0.472567 | 1.000x |
| 41 | 42 | +structure-aware decoupled KLA | 49.727414 | 0.497274 | 1.052x |
| 42 | 43 | fixed weights | 47.332546 | 0.473325 | 1.000x |
| 42 | 43 | +structure-aware decoupled KLA | 52.817504 | 0.528175 | 1.116x |
| 43 | 44 | fixed weights | 63.860748 | 0.638607 | 1.000x |
| 43 | 44 | +structure-aware decoupled KLA | 77.921257 | 0.779213 | 1.220x |
| 44 | 45 | fixed weights | 48.130840 | 0.481308 | 1.000x |
| 44 | 45 | +structure-aware decoupled KLA | 49.297092 | 0.492971 | 1.024x |
| 45 | 46 | fixed weights | 44.264965 | 0.442650 | 1.000x |
| 45 | 46 | +structure-aware decoupled KLA | 47.984772 | 0.479848 | 1.084x |
| 46 | 47 | fixed weights | 48.114380 | 0.481144 | 1.000x |
| 46 | 47 | +structure-aware decoupled KLA | 50.320057 | 0.503201 | 1.046x |
| 47 | 48 | fixed weights | 46.961653 | 0.469617 | 1.000x |
| 47 | 48 | +structure-aware decoupled KLA | 50.099910 | 0.500999 | 1.067x |
| 48 | 49 | fixed weights | 46.768315 | 0.467683 | 1.000x |
| 48 | 49 | +structure-aware decoupled KLA | 50.229446 | 0.502294 | 1.074x |
| 49 | 50 | fixed weights | 46.125385 | 0.461254 | 1.000x |
| 49 | 50 | +structure-aware decoupled KLA | 48.415580 | 0.484156 | 1.050x |
| 50 | 51 | fixed weights | 52.166374 | 0.521664 | 1.000x |
| 50 | 51 | +structure-aware decoupled KLA | 54.163729 | 0.541637 | 1.038x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.996884 | 1.700369 | 1.766050 |
| +structure-aware decoupled KLA | 2.361302 | 1.633429 | 0.596600 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.996884 +/- 0.147938 | [2.955878, 3.037891] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 2.361302 +/- 0.083660 | [2.338113, 2.384491] | 50 |
| fixed weights | RMSE | 1.700369 +/- 0.105039 | [1.671254, 1.729484] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.633429 +/- 0.041552 | [1.621911, 1.644947] | 50 |
| fixed weights | CardErr | 1.766050 +/- 0.326509 | [1.675546, 1.856554] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.596600 +/- 0.074485 | [0.575954, 0.617246] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.635582 +/- 0.107222 | [0.605862, 0.665303] | 21.21% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.066940 +/- 0.096162 | [0.040285, 0.093595] | 3.94% | 40/50 | 2.386e-05 |
| +structure-aware decoupled KLA | CardErr | 1.169450 +/- 0.284505 | [1.090589, 1.248311] | 66.22% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.809994 | 1.655953 | 1.290400 |
| 1 | +structure-aware decoupled KLA | 2.303568 | 1.590291 | 0.550200 |
| 2 | fixed weights | 2.831251 | 1.670273 | 1.288000 |
| 2 | +structure-aware decoupled KLA | 2.330935 | 1.591285 | 0.572400 |
| 3 | fixed weights | 2.807232 | 1.606157 | 1.289200 |
| 3 | +structure-aware decoupled KLA | 2.334174 | 1.590450 | 0.600200 |
| 4 | fixed weights | 2.989661 | 1.709509 | 1.783400 |
| 4 | +structure-aware decoupled KLA | 2.430668 | 1.655514 | 0.707400 |
| 5 | fixed weights | 3.053319 | 1.744819 | 1.871000 |
| 5 | +structure-aware decoupled KLA | 2.397358 | 1.667167 | 0.626800 |
| 6 | fixed weights | 3.095864 | 1.755538 | 1.941000 |
| 6 | +structure-aware decoupled KLA | 2.352419 | 1.660149 | 0.546000 |
| 7 | fixed weights | 3.068264 | 1.715709 | 1.902200 |
| 7 | +structure-aware decoupled KLA | 2.403756 | 1.668722 | 0.620000 |
| 8 | fixed weights | 3.319488 | 1.744995 | 2.763200 |
| 8 | +structure-aware decoupled KLA | 2.337536 | 1.643854 | 0.549800 |
