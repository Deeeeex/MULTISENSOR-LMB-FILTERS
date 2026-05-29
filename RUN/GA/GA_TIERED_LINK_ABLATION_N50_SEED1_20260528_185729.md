# GA Tiered Link Ablation (2026-05-28 18:57:29)

Comparison order: fixed weights -> +structure-aware decoupled KLA

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
| 1 | 2 | +structure-aware decoupled KLA | 1.423125 | 1.183512 | 0.067500 |
| 2 | 3 | fixed weights | 1.606838 | 1.378415 | 0.061250 |
| 2 | 3 | +structure-aware decoupled KLA | 1.387470 | 1.211753 | 0.043750 |
| 3 | 4 | fixed weights | 1.582975 | 1.375559 | 0.066250 |
| 3 | 4 | +structure-aware decoupled KLA | 1.380547 | 1.200794 | 0.056250 |
| 4 | 5 | fixed weights | 1.661952 | 1.390417 | 0.095000 |
| 4 | 5 | +structure-aware decoupled KLA | 1.482511 | 1.221932 | 0.091250 |
| 5 | 6 | fixed weights | 1.606861 | 1.371722 | 0.082500 |
| 5 | 6 | +structure-aware decoupled KLA | 1.406824 | 1.193824 | 0.070000 |
| 6 | 7 | fixed weights | 1.619482 | 1.402597 | 0.082500 |
| 6 | 7 | +structure-aware decoupled KLA | 1.407321 | 1.218000 | 0.065000 |
| 7 | 8 | fixed weights | 1.609838 | 1.320463 | 0.108750 |
| 7 | 8 | +structure-aware decoupled KLA | 1.384620 | 1.150343 | 0.063750 |
| 8 | 9 | fixed weights | 1.618389 | 1.339729 | 0.112500 |
| 8 | 9 | +structure-aware decoupled KLA | 1.414068 | 1.159126 | 0.090000 |
| 9 | 10 | fixed weights | 1.615154 | 1.441848 | 0.048750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.404229 | 1.233939 | 0.040000 |
| 10 | 11 | fixed weights | 1.634175 | 1.365073 | 0.093750 |
| 10 | 11 | +structure-aware decoupled KLA | 1.416861 | 1.188414 | 0.076250 |
| 11 | 12 | fixed weights | 1.634779 | 1.436796 | 0.060000 |
| 11 | 12 | +structure-aware decoupled KLA | 1.428414 | 1.257893 | 0.040000 |
| 12 | 13 | fixed weights | 1.646082 | 1.361098 | 0.095000 |
| 12 | 13 | +structure-aware decoupled KLA | 1.417504 | 1.187319 | 0.071250 |
| 13 | 14 | fixed weights | 1.698178 | 1.442592 | 0.088750 |
| 13 | 14 | +structure-aware decoupled KLA | 1.474568 | 1.265545 | 0.068750 |
| 14 | 15 | fixed weights | 1.638514 | 1.375393 | 0.075000 |
| 14 | 15 | +structure-aware decoupled KLA | 1.442702 | 1.202483 | 0.067500 |
| 15 | 16 | fixed weights | 1.690784 | 1.388506 | 0.108750 |
| 15 | 16 | +structure-aware decoupled KLA | 1.461350 | 1.227628 | 0.066250 |
| 16 | 17 | fixed weights | 1.669561 | 1.381232 | 0.098750 |
| 16 | 17 | +structure-aware decoupled KLA | 1.420025 | 1.162603 | 0.066250 |
| 17 | 18 | fixed weights | 1.651100 | 1.411833 | 0.080000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.448598 | 1.239124 | 0.063750 |
| 18 | 19 | fixed weights | 1.593401 | 1.285643 | 0.113750 |
| 18 | 19 | +structure-aware decoupled KLA | 1.411213 | 1.123839 | 0.100000 |
| 19 | 20 | fixed weights | 1.604889 | 1.333492 | 0.075000 |
| 19 | 20 | +structure-aware decoupled KLA | 1.414734 | 1.164961 | 0.065000 |
| 20 | 21 | fixed weights | 1.720672 | 1.434714 | 0.095000 |
| 20 | 21 | +structure-aware decoupled KLA | 1.459110 | 1.224580 | 0.065000 |
| 21 | 22 | fixed weights | 1.595483 | 1.378553 | 0.073750 |
| 21 | 22 | +structure-aware decoupled KLA | 1.395513 | 1.213611 | 0.055000 |
| 22 | 23 | fixed weights | 1.612717 | 1.411055 | 0.063750 |
| 22 | 23 | +structure-aware decoupled KLA | 1.420318 | 1.210200 | 0.053750 |
| 23 | 24 | fixed weights | 1.633547 | 1.404581 | 0.090000 |
| 23 | 24 | +structure-aware decoupled KLA | 1.462224 | 1.257201 | 0.081250 |
| 24 | 25 | fixed weights | 1.590987 | 1.381488 | 0.065000 |
| 24 | 25 | +structure-aware decoupled KLA | 1.402619 | 1.210662 | 0.053750 |
| 25 | 26 | fixed weights | 1.574382 | 1.328371 | 0.086250 |
| 25 | 26 | +structure-aware decoupled KLA | 1.368118 | 1.167088 | 0.063750 |
| 26 | 27 | fixed weights | 1.583305 | 1.308135 | 0.072500 |
| 26 | 27 | +structure-aware decoupled KLA | 1.397369 | 1.176239 | 0.055000 |
| 27 | 28 | fixed weights | 1.741169 | 1.374388 | 0.121250 |
| 27 | 28 | +structure-aware decoupled KLA | 1.534901 | 1.204125 | 0.110000 |
| 28 | 29 | fixed weights | 1.700899 | 1.329079 | 0.156250 |
| 28 | 29 | +structure-aware decoupled KLA | 1.490984 | 1.160294 | 0.132500 |
| 29 | 30 | fixed weights | 1.645895 | 1.392230 | 0.080000 |
| 29 | 30 | +structure-aware decoupled KLA | 1.421890 | 1.224257 | 0.052500 |
| 30 | 31 | fixed weights | 1.607163 | 1.362216 | 0.076250 |
| 30 | 31 | +structure-aware decoupled KLA | 1.398993 | 1.201504 | 0.057500 |
| 31 | 32 | fixed weights | 1.607239 | 1.353930 | 0.085000 |
| 31 | 32 | +structure-aware decoupled KLA | 1.391331 | 1.175269 | 0.071250 |
| 32 | 33 | fixed weights | 1.664329 | 1.367188 | 0.148750 |
| 32 | 33 | +structure-aware decoupled KLA | 1.426588 | 1.188891 | 0.083750 |
| 33 | 34 | fixed weights | 1.594309 | 1.418825 | 0.061250 |
| 33 | 34 | +structure-aware decoupled KLA | 1.396445 | 1.204412 | 0.055000 |
| 34 | 35 | fixed weights | 1.710505 | 1.505018 | 0.102500 |
| 34 | 35 | +structure-aware decoupled KLA | 1.484747 | 1.249445 | 0.082500 |
| 35 | 36 | fixed weights | 1.632125 | 1.390001 | 0.098750 |
| 35 | 36 | +structure-aware decoupled KLA | 1.410947 | 1.221383 | 0.065000 |
| 36 | 37 | fixed weights | 1.619751 | 1.368349 | 0.092500 |
| 36 | 37 | +structure-aware decoupled KLA | 1.400700 | 1.194972 | 0.066250 |
| 37 | 38 | fixed weights | 1.576180 | 1.351863 | 0.080000 |
| 37 | 38 | +structure-aware decoupled KLA | 1.391133 | 1.183147 | 0.070000 |
| 38 | 39 | fixed weights | 1.602643 | 1.362600 | 0.081250 |
| 38 | 39 | +structure-aware decoupled KLA | 1.429410 | 1.197496 | 0.083750 |
| 39 | 40 | fixed weights | 1.617306 | 1.391714 | 0.071250 |
| 39 | 40 | +structure-aware decoupled KLA | 1.414807 | 1.206776 | 0.065000 |
| 40 | 41 | fixed weights | 1.600522 | 1.360449 | 0.085000 |
| 40 | 41 | +structure-aware decoupled KLA | 1.391339 | 1.175362 | 0.068750 |
| 41 | 42 | fixed weights | 1.578523 | 1.357841 | 0.086250 |
| 41 | 42 | +structure-aware decoupled KLA | 1.395976 | 1.199077 | 0.082500 |
| 42 | 43 | fixed weights | 1.679407 | 1.408957 | 0.086250 |
| 42 | 43 | +structure-aware decoupled KLA | 1.482537 | 1.228836 | 0.076250 |
| 43 | 44 | fixed weights | 1.613978 | 1.390679 | 0.086250 |
| 43 | 44 | +structure-aware decoupled KLA | 1.399202 | 1.199706 | 0.062500 |
| 44 | 45 | fixed weights | 1.609875 | 1.414953 | 0.090000 |
| 44 | 45 | +structure-aware decoupled KLA | 1.417499 | 1.171032 | 0.082500 |
| 45 | 46 | fixed weights | 1.672819 | 1.356082 | 0.115000 |
| 45 | 46 | +structure-aware decoupled KLA | 1.441686 | 1.179232 | 0.091250 |
| 46 | 47 | fixed weights | 1.667269 | 1.364952 | 0.117500 |
| 46 | 47 | +structure-aware decoupled KLA | 1.476725 | 1.209041 | 0.093750 |
| 47 | 48 | fixed weights | 1.633059 | 1.427702 | 0.081250 |
| 47 | 48 | +structure-aware decoupled KLA | 1.425806 | 1.257931 | 0.055000 |
| 48 | 49 | fixed weights | 1.660203 | 1.390574 | 0.111250 |
| 48 | 49 | +structure-aware decoupled KLA | 1.475123 | 1.200305 | 0.082500 |
| 49 | 50 | fixed weights | 1.673478 | 1.344760 | 0.106250 |
| 49 | 50 | +structure-aware decoupled KLA | 1.475809 | 1.171268 | 0.092500 |
| 50 | 51 | fixed weights | 1.660892 | 1.479252 | 0.100000 |
| 50 | 51 | +structure-aware decoupled KLA | 1.446368 | 1.235389 | 0.082500 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.634289 | 1.381434 | 0.090125 |
| +structure-aware decoupled KLA | 1.427058 | 1.201835 | 0.071300 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.634289 +/- 0.040164 | [1.623156, 1.645422] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.427058 +/- 0.035445 | [1.417233, 1.436883] | 50 |
| fixed weights | RMSE | 1.381434 +/- 0.040960 | [1.370081, 1.392788] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.201835 +/- 0.030438 | [1.193398, 1.210272] | 50 |
| fixed weights | Cardinality | 0.090125 +/- 0.020982 | [0.084309, 0.095941] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.071300 +/- 0.017489 | [0.066452, 0.076148] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.207231 +/- 0.018901 | [0.201992, 0.212470] | 12.68% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.179599 +/- 0.023434 | [0.173103, 0.186095] | 13.00% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.018825 +/- 0.011722 | [0.015576, 0.022074] | 20.89% | 49/50 | 9.059e-14 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 61.240814 +/- 8.826372 | 0.612408 | 1.000x | 50 |
| +structure-aware decoupled KLA | 66.792352 +/- 11.933422 | 0.667924 | 1.096x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 5.551538 +/- 10.202401 | 9.55% | 46/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 55.179526 | 0.551795 | 1.000x |
| 1 | 2 | +structure-aware decoupled KLA | 59.538505 | 0.595385 | 1.079x |
| 2 | 3 | fixed weights | 54.812922 | 0.548129 | 1.000x |
| 2 | 3 | +structure-aware decoupled KLA | 59.291418 | 0.592914 | 1.082x |
| 3 | 4 | fixed weights | 55.736022 | 0.557360 | 1.000x |
| 3 | 4 | +structure-aware decoupled KLA | 60.300989 | 0.603010 | 1.082x |
| 4 | 5 | fixed weights | 56.254561 | 0.562546 | 1.000x |
| 4 | 5 | +structure-aware decoupled KLA | 63.822293 | 0.638223 | 1.135x |
| 5 | 6 | fixed weights | 59.993166 | 0.599932 | 1.000x |
| 5 | 6 | +structure-aware decoupled KLA | 59.485836 | 0.594858 | 0.992x |
| 6 | 7 | fixed weights | 55.551092 | 0.555511 | 1.000x |
| 6 | 7 | +structure-aware decoupled KLA | 59.887705 | 0.598877 | 1.078x |
| 7 | 8 | fixed weights | 55.963682 | 0.559637 | 1.000x |
| 7 | 8 | +structure-aware decoupled KLA | 60.213047 | 0.602130 | 1.076x |
| 8 | 9 | fixed weights | 55.708619 | 0.557086 | 1.000x |
| 8 | 9 | +structure-aware decoupled KLA | 59.925306 | 0.599253 | 1.076x |
| 9 | 10 | fixed weights | 55.530261 | 0.555303 | 1.000x |
| 9 | 10 | +structure-aware decoupled KLA | 59.832657 | 0.598327 | 1.077x |
| 10 | 11 | fixed weights | 55.621757 | 0.556218 | 1.000x |
| 10 | 11 | +structure-aware decoupled KLA | 59.965022 | 0.599650 | 1.078x |
| 11 | 12 | fixed weights | 55.794121 | 0.557941 | 1.000x |
| 11 | 12 | +structure-aware decoupled KLA | 60.239790 | 0.602398 | 1.080x |
| 12 | 13 | fixed weights | 55.895375 | 0.558954 | 1.000x |
| 12 | 13 | +structure-aware decoupled KLA | 60.393968 | 0.603940 | 1.080x |
| 13 | 14 | fixed weights | 56.114490 | 0.561145 | 1.000x |
| 13 | 14 | +structure-aware decoupled KLA | 60.656636 | 0.606566 | 1.081x |
| 14 | 15 | fixed weights | 55.414429 | 0.554144 | 1.000x |
| 14 | 15 | +structure-aware decoupled KLA | 59.991674 | 0.599917 | 1.083x |
| 15 | 16 | fixed weights | 58.723795 | 0.587238 | 1.000x |
| 15 | 16 | +structure-aware decoupled KLA | 60.454037 | 0.604540 | 1.029x |
| 16 | 17 | fixed weights | 55.763930 | 0.557639 | 1.000x |
| 16 | 17 | +structure-aware decoupled KLA | 61.306104 | 0.613061 | 1.099x |
| 17 | 18 | fixed weights | 55.145898 | 0.551459 | 1.000x |
| 17 | 18 | +structure-aware decoupled KLA | 59.598317 | 0.595983 | 1.081x |
| 18 | 19 | fixed weights | 55.487401 | 0.554874 | 1.000x |
| 18 | 19 | +structure-aware decoupled KLA | 59.962197 | 0.599622 | 1.081x |
| 19 | 20 | fixed weights | 55.692950 | 0.556930 | 1.000x |
| 19 | 20 | +structure-aware decoupled KLA | 60.165233 | 0.601652 | 1.080x |
| 20 | 21 | fixed weights | 55.352415 | 0.553524 | 1.000x |
| 20 | 21 | +structure-aware decoupled KLA | 59.509360 | 0.595094 | 1.075x |
| 21 | 22 | fixed weights | 55.617658 | 0.556177 | 1.000x |
| 21 | 22 | +structure-aware decoupled KLA | 61.787042 | 0.617870 | 1.111x |
| 22 | 23 | fixed weights | 55.376604 | 0.553766 | 1.000x |
| 22 | 23 | +structure-aware decoupled KLA | 60.250530 | 0.602505 | 1.088x |
| 23 | 24 | fixed weights | 56.190847 | 0.561908 | 1.000x |
| 23 | 24 | +structure-aware decoupled KLA | 60.811349 | 0.608113 | 1.082x |
| 24 | 25 | fixed weights | 55.274558 | 0.552746 | 1.000x |
| 24 | 25 | +structure-aware decoupled KLA | 62.155926 | 0.621559 | 1.124x |
| 25 | 26 | fixed weights | 56.531269 | 0.565313 | 1.000x |
| 25 | 26 | +structure-aware decoupled KLA | 71.277367 | 0.712774 | 1.261x |
| 26 | 27 | fixed weights | 75.160689 | 0.751607 | 1.000x |
| 26 | 27 | +structure-aware decoupled KLA | 63.826871 | 0.638269 | 0.849x |
| 27 | 28 | fixed weights | 55.309004 | 0.553090 | 1.000x |
| 27 | 28 | +structure-aware decoupled KLA | 59.669343 | 0.596693 | 1.079x |
| 28 | 29 | fixed weights | 55.704188 | 0.557042 | 1.000x |
| 28 | 29 | +structure-aware decoupled KLA | 60.085976 | 0.600860 | 1.079x |
| 29 | 30 | fixed weights | 55.458142 | 0.554581 | 1.000x |
| 29 | 30 | +structure-aware decoupled KLA | 59.874263 | 0.598743 | 1.080x |
| 30 | 31 | fixed weights | 55.448868 | 0.554489 | 1.000x |
| 30 | 31 | +structure-aware decoupled KLA | 59.619634 | 0.596196 | 1.075x |
| 31 | 32 | fixed weights | 55.726989 | 0.557270 | 1.000x |
| 31 | 32 | +structure-aware decoupled KLA | 80.412961 | 0.804130 | 1.443x |
| 32 | 33 | fixed weights | 91.318795 | 0.913188 | 1.000x |
| 32 | 33 | +structure-aware decoupled KLA | 63.815788 | 0.638158 | 0.699x |
| 33 | 34 | fixed weights | 57.843035 | 0.578430 | 1.000x |
| 33 | 34 | +structure-aware decoupled KLA | 63.093709 | 0.630937 | 1.091x |
| 34 | 35 | fixed weights | 58.803888 | 0.588039 | 1.000x |
| 34 | 35 | +structure-aware decoupled KLA | 62.874218 | 0.628742 | 1.069x |
| 35 | 36 | fixed weights | 61.953960 | 0.619540 | 1.000x |
| 35 | 36 | +structure-aware decoupled KLA | 72.473413 | 0.724734 | 1.170x |
| 36 | 37 | fixed weights | 66.522340 | 0.665223 | 1.000x |
| 36 | 37 | +structure-aware decoupled KLA | 72.760650 | 0.727606 | 1.094x |
| 37 | 38 | fixed weights | 67.055267 | 0.670553 | 1.000x |
| 37 | 38 | +structure-aware decoupled KLA | 72.715483 | 0.727155 | 1.084x |
| 38 | 39 | fixed weights | 68.052066 | 0.680521 | 1.000x |
| 38 | 39 | +structure-aware decoupled KLA | 73.763892 | 0.737639 | 1.084x |
| 39 | 40 | fixed weights | 67.737001 | 0.677370 | 1.000x |
| 39 | 40 | +structure-aware decoupled KLA | 125.454854 | 1.254549 | 1.852x |
| 40 | 41 | fixed weights | 69.051665 | 0.690517 | 1.000x |
| 40 | 41 | +structure-aware decoupled KLA | 73.938100 | 0.739381 | 1.071x |
| 41 | 42 | fixed weights | 66.234172 | 0.662342 | 1.000x |
| 41 | 42 | +structure-aware decoupled KLA | 70.410949 | 0.704109 | 1.063x |
| 42 | 43 | fixed weights | 65.160971 | 0.651610 | 1.000x |
| 42 | 43 | +structure-aware decoupled KLA | 85.853573 | 0.858536 | 1.318x |
| 43 | 44 | fixed weights | 65.588799 | 0.655888 | 1.000x |
| 43 | 44 | +structure-aware decoupled KLA | 69.572675 | 0.695727 | 1.061x |
| 44 | 45 | fixed weights | 66.466774 | 0.664668 | 1.000x |
| 44 | 45 | +structure-aware decoupled KLA | 71.188510 | 0.711885 | 1.071x |
| 45 | 46 | fixed weights | 76.329210 | 0.763292 | 1.000x |
| 45 | 46 | +structure-aware decoupled KLA | 69.256969 | 0.692570 | 0.907x |
| 46 | 47 | fixed weights | 65.452834 | 0.654528 | 1.000x |
| 46 | 47 | +structure-aware decoupled KLA | 69.312169 | 0.693122 | 1.059x |
| 47 | 48 | fixed weights | 64.676675 | 0.646767 | 1.000x |
| 47 | 48 | +structure-aware decoupled KLA | 69.818415 | 0.698184 | 1.079x |
| 48 | 49 | fixed weights | 66.503439 | 0.665034 | 1.000x |
| 48 | 49 | +structure-aware decoupled KLA | 73.489254 | 0.734893 | 1.105x |
| 49 | 50 | fixed weights | 94.900080 | 0.949001 | 1.000x |
| 49 | 50 | +structure-aware decoupled KLA | 104.078544 | 1.040785 | 1.097x |
| 50 | 51 | fixed weights | 70.854480 | 0.708545 | 1.000x |
| 50 | 51 | +structure-aware decoupled KLA | 71.435079 | 0.714351 | 1.008x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 1.908084 | 1.441941 | 0.281625 |
| +structure-aware decoupled KLA | 1.838671 | 1.374600 | 0.270750 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 1.908084 +/- 0.054512 | [1.892974, 1.923194] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 1.838671 +/- 0.055060 | [1.823409, 1.853933] | 50 |
| fixed weights | RMSE | 1.441941 +/- 0.034792 | [1.432297, 1.451585] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.374600 +/- 0.035674 | [1.364712, 1.384489] | 50 |
| fixed weights | CardErr | 0.281625 +/- 0.032507 | [0.272614, 0.290636] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.270750 +/- 0.028531 | [0.262842, 0.278658] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.069413 +/- 0.009451 | [0.066793, 0.072033] | 3.64% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.067341 +/- 0.004899 | [0.065983, 0.068698] | 4.67% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | CardErr | 0.010875 +/- 0.011468 | [0.007696, 0.014054] | 3.86% | 43/50 | 5.728e-08 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 1.958102 | 1.448123 | 0.345600 |
| 1 | +structure-aware decoupled KLA | 1.873880 | 1.387617 | 0.297600 |
| 2 | fixed weights | 1.930062 | 1.451044 | 0.294600 |
| 2 | +structure-aware decoupled KLA | 1.870632 | 1.381892 | 0.300400 |
| 3 | fixed weights | 1.951673 | 1.443968 | 0.343800 |
| 3 | +structure-aware decoupled KLA | 1.863207 | 1.380122 | 0.292600 |
| 4 | fixed weights | 1.915241 | 1.452566 | 0.277600 |
| 4 | +structure-aware decoupled KLA | 1.852503 | 1.379788 | 0.283800 |
| 5 | fixed weights | 1.891303 | 1.442694 | 0.256200 |
| 5 | +structure-aware decoupled KLA | 1.823111 | 1.371393 | 0.255000 |
| 6 | fixed weights | 1.877376 | 1.437574 | 0.241600 |
| 6 | +structure-aware decoupled KLA | 1.805413 | 1.368606 | 0.238000 |
| 7 | fixed weights | 1.884340 | 1.435728 | 0.255200 |
| 7 | +structure-aware decoupled KLA | 1.822055 | 1.367642 | 0.258600 |
| 8 | fixed weights | 1.856573 | 1.423829 | 0.238400 |
| 8 | +structure-aware decoupled KLA | 1.798567 | 1.359744 | 0.240000 |
