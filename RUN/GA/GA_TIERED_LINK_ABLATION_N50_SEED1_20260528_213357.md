# GA Tiered Link Ablation (2026-05-28 21:33:57)

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
| 1 | 2 | fixed weights | 1.704034 | 1.400387 | 0.120000 |
| 1 | 2 | +structure-aware decoupled KLA | 1.465913 | 1.218046 | 0.078750 |
| 2 | 3 | fixed weights | 1.669301 | 1.482877 | 0.091250 |
| 2 | 3 | +structure-aware decoupled KLA | 1.428698 | 1.245738 | 0.047500 |
| 3 | 4 | fixed weights | 1.721317 | 1.426975 | 0.161250 |
| 3 | 4 | +structure-aware decoupled KLA | 1.459495 | 1.249998 | 0.081250 |
| 4 | 5 | fixed weights | 1.762046 | 1.512567 | 0.153750 |
| 4 | 5 | +structure-aware decoupled KLA | 1.534953 | 1.259580 | 0.110000 |
| 5 | 6 | fixed weights | 1.736250 | 1.499185 | 0.165000 |
| 5 | 6 | +structure-aware decoupled KLA | 1.455154 | 1.235759 | 0.073750 |
| 6 | 7 | fixed weights | 1.684778 | 1.435802 | 0.122500 |
| 6 | 7 | +structure-aware decoupled KLA | 1.476774 | 1.252566 | 0.081250 |
| 7 | 8 | fixed weights | 1.662211 | 1.369580 | 0.131250 |
| 7 | 8 | +structure-aware decoupled KLA | 1.461466 | 1.189001 | 0.088750 |
| 8 | 9 | fixed weights | 1.646555 | 1.395666 | 0.091250 |
| 8 | 9 | +structure-aware decoupled KLA | 1.493601 | 1.258971 | 0.097500 |
| 9 | 10 | fixed weights | 1.743388 | 1.529261 | 0.138750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.474900 | 1.282352 | 0.063750 |
| 10 | 11 | fixed weights | 1.696969 | 1.405778 | 0.118750 |
| 10 | 11 | +structure-aware decoupled KLA | 1.458560 | 1.223587 | 0.080000 |
| 11 | 12 | fixed weights | 1.758954 | 1.490883 | 0.155000 |
| 11 | 12 | +structure-aware decoupled KLA | 1.465742 | 1.287303 | 0.046250 |
| 12 | 13 | fixed weights | 1.769538 | 1.446828 | 0.151250 |
| 12 | 13 | +structure-aware decoupled KLA | 1.488050 | 1.234591 | 0.080000 |
| 13 | 14 | fixed weights | 1.827991 | 1.535903 | 0.173750 |
| 13 | 14 | +structure-aware decoupled KLA | 1.522112 | 1.298115 | 0.085000 |
| 14 | 15 | fixed weights | 1.693318 | 1.416277 | 0.096250 |
| 14 | 15 | +structure-aware decoupled KLA | 1.481869 | 1.238929 | 0.066250 |
| 15 | 16 | fixed weights | 1.735803 | 1.439691 | 0.137500 |
| 15 | 16 | +structure-aware decoupled KLA | 1.506899 | 1.261855 | 0.078750 |
| 16 | 17 | fixed weights | 1.729697 | 1.436034 | 0.121250 |
| 16 | 17 | +structure-aware decoupled KLA | 1.457370 | 1.189980 | 0.070000 |
| 17 | 18 | fixed weights | 1.762344 | 1.534155 | 0.142500 |
| 17 | 18 | +structure-aware decoupled KLA | 1.508107 | 1.299706 | 0.078750 |
| 18 | 19 | fixed weights | 1.640681 | 1.318779 | 0.126250 |
| 18 | 19 | +structure-aware decoupled KLA | 1.459321 | 1.157234 | 0.108750 |
| 19 | 20 | fixed weights | 1.719980 | 1.365980 | 0.161250 |
| 19 | 20 | +structure-aware decoupled KLA | 1.492308 | 1.204549 | 0.085000 |
| 20 | 21 | fixed weights | 1.756326 | 1.461872 | 0.101250 |
| 20 | 21 | +structure-aware decoupled KLA | 1.492842 | 1.252445 | 0.066250 |
| 21 | 22 | fixed weights | 1.694945 | 1.405060 | 0.158750 |
| 21 | 22 | +structure-aware decoupled KLA | 1.464634 | 1.242740 | 0.086250 |
| 22 | 23 | fixed weights | 1.754257 | 1.583100 | 0.161250 |
| 22 | 23 | +structure-aware decoupled KLA | 1.492804 | 1.281764 | 0.078750 |
| 23 | 24 | fixed weights | 1.719814 | 1.538388 | 0.125000 |
| 23 | 24 | +structure-aware decoupled KLA | 1.523917 | 1.290371 | 0.093750 |
| 24 | 25 | fixed weights | 1.639973 | 1.416416 | 0.087500 |
| 24 | 25 | +structure-aware decoupled KLA | 1.453622 | 1.247257 | 0.062500 |
| 25 | 26 | fixed weights | 1.655909 | 1.349866 | 0.145000 |
| 25 | 26 | +structure-aware decoupled KLA | 1.439574 | 1.192419 | 0.097500 |
| 26 | 27 | fixed weights | 1.718031 | 1.365479 | 0.168750 |
| 26 | 27 | +structure-aware decoupled KLA | 1.452481 | 1.212995 | 0.077500 |
| 27 | 28 | fixed weights | 1.808233 | 1.460321 | 0.150000 |
| 27 | 28 | +structure-aware decoupled KLA | 1.571752 | 1.227188 | 0.112500 |
| 28 | 29 | fixed weights | 1.778468 | 1.398685 | 0.212500 |
| 28 | 29 | +structure-aware decoupled KLA | 1.541284 | 1.199973 | 0.140000 |
| 29 | 30 | fixed weights | 1.738721 | 1.443047 | 0.151250 |
| 29 | 30 | +structure-aware decoupled KLA | 1.491633 | 1.273796 | 0.077500 |
| 30 | 31 | fixed weights | 1.637802 | 1.411981 | 0.097500 |
| 30 | 31 | +structure-aware decoupled KLA | 1.428723 | 1.222352 | 0.066250 |
| 31 | 32 | fixed weights | 1.675783 | 1.402306 | 0.132500 |
| 31 | 32 | +structure-aware decoupled KLA | 1.430967 | 1.203240 | 0.077500 |
| 32 | 33 | fixed weights | 1.704760 | 1.423710 | 0.158750 |
| 32 | 33 | +structure-aware decoupled KLA | 1.490843 | 1.226442 | 0.116250 |
| 33 | 34 | fixed weights | 1.656978 | 1.461991 | 0.088750 |
| 33 | 34 | +structure-aware decoupled KLA | 1.465145 | 1.253994 | 0.068750 |
| 34 | 35 | fixed weights | 1.806174 | 1.556450 | 0.163750 |
| 34 | 35 | +structure-aware decoupled KLA | 1.545221 | 1.281381 | 0.108750 |
| 35 | 36 | fixed weights | 1.658959 | 1.417570 | 0.102500 |
| 35 | 36 | +structure-aware decoupled KLA | 1.444435 | 1.248734 | 0.068750 |
| 36 | 37 | fixed weights | 1.735566 | 1.460606 | 0.145000 |
| 36 | 37 | +structure-aware decoupled KLA | 1.479465 | 1.243043 | 0.090000 |
| 37 | 38 | fixed weights | 1.721945 | 1.492172 | 0.168750 |
| 37 | 38 | +structure-aware decoupled KLA | 1.425875 | 1.210102 | 0.081250 |
| 38 | 39 | fixed weights | 1.684781 | 1.434889 | 0.110000 |
| 38 | 39 | +structure-aware decoupled KLA | 1.473724 | 1.231249 | 0.082500 |
| 39 | 40 | fixed weights | 1.685353 | 1.546499 | 0.100000 |
| 39 | 40 | +structure-aware decoupled KLA | 1.472518 | 1.235862 | 0.087500 |
| 40 | 41 | fixed weights | 1.718642 | 1.463728 | 0.170000 |
| 40 | 41 | +structure-aware decoupled KLA | 1.462904 | 1.307120 | 0.082500 |
| 41 | 42 | fixed weights | 1.663061 | 1.412572 | 0.143750 |
| 41 | 42 | +structure-aware decoupled KLA | 1.471015 | 1.201319 | 0.116250 |
| 42 | 43 | fixed weights | 1.770636 | 1.503401 | 0.121250 |
| 42 | 43 | +structure-aware decoupled KLA | 1.530303 | 1.275778 | 0.080000 |
| 43 | 44 | fixed weights | 1.768147 | 1.592295 | 0.190000 |
| 43 | 44 | +structure-aware decoupled KLA | 1.480749 | 1.251920 | 0.083750 |
| 44 | 45 | fixed weights | 1.697375 | 1.443631 | 0.161250 |
| 44 | 45 | +structure-aware decoupled KLA | 1.503552 | 1.219220 | 0.120000 |
| 45 | 46 | fixed weights | 1.712026 | 1.384921 | 0.128750 |
| 45 | 46 | +structure-aware decoupled KLA | 1.478176 | 1.214600 | 0.086250 |
| 46 | 47 | fixed weights | 1.726940 | 1.413485 | 0.157500 |
| 46 | 47 | +structure-aware decoupled KLA | 1.545674 | 1.244955 | 0.128750 |
| 47 | 48 | fixed weights | 1.698706 | 1.486779 | 0.122500 |
| 47 | 48 | +structure-aware decoupled KLA | 1.488804 | 1.306898 | 0.066250 |
| 48 | 49 | fixed weights | 1.726411 | 1.434958 | 0.118750 |
| 48 | 49 | +structure-aware decoupled KLA | 1.533732 | 1.245779 | 0.081250 |
| 49 | 50 | fixed weights | 1.754676 | 1.390194 | 0.156250 |
| 49 | 50 | +structure-aware decoupled KLA | 1.513845 | 1.202847 | 0.092500 |
| 50 | 51 | fixed weights | 1.710363 | 1.529444 | 0.110000 |
| 50 | 51 | +structure-aware decoupled KLA | 1.510204 | 1.263754 | 0.106250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.716898 | 1.450569 | 0.137350 |
| +structure-aware decoupled KLA | 1.483834 | 1.241988 | 0.085700 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.716898 +/- 0.045779 | [1.704209, 1.729588] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.483834 +/- 0.033936 | [1.474428, 1.493241] | 50 |
| fixed weights | RMSE | 1.450569 +/- 0.062109 | [1.433353, 1.467784] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.241988 +/- 0.033957 | [1.232575, 1.251400] | 50 |
| fixed weights | Cardinality | 0.137350 +/- 0.028452 | [0.129463, 0.145237] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.085700 +/- 0.019256 | [0.080362, 0.091038] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.233064 +/- 0.034393 | [0.223531, 0.242597] | 13.57% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.208581 +/- 0.044857 | [0.196147, 0.221014] | 14.38% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.051650 +/- 0.026112 | [0.044412, 0.058888] | 37.60% | 49/50 | 9.059e-14 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 68.176186 +/- 7.628725 | 0.681762 | 1.000x | 50 |
| +structure-aware decoupled KLA | 73.711846 +/- 9.192313 | 0.737118 | 1.085x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 5.535660 +/- 7.495283 | 8.47% | 44/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 91.445627 | 0.914456 | 1.000x |
| 1 | 2 | +structure-aware decoupled KLA | 99.386338 | 0.993863 | 1.087x |
| 2 | 3 | fixed weights | 81.552097 | 0.815521 | 1.000x |
| 2 | 3 | +structure-aware decoupled KLA | 75.866686 | 0.758667 | 0.930x |
| 3 | 4 | fixed weights | 72.128338 | 0.721283 | 1.000x |
| 3 | 4 | +structure-aware decoupled KLA | 87.605481 | 0.876055 | 1.215x |
| 4 | 5 | fixed weights | 63.667750 | 0.636677 | 1.000x |
| 4 | 5 | +structure-aware decoupled KLA | 68.505542 | 0.685055 | 1.076x |
| 5 | 6 | fixed weights | 63.451470 | 0.634515 | 1.000x |
| 5 | 6 | +structure-aware decoupled KLA | 67.451826 | 0.674518 | 1.063x |
| 6 | 7 | fixed weights | 62.863907 | 0.628639 | 1.000x |
| 6 | 7 | +structure-aware decoupled KLA | 68.013386 | 0.680134 | 1.082x |
| 7 | 8 | fixed weights | 66.417216 | 0.664172 | 1.000x |
| 7 | 8 | +structure-aware decoupled KLA | 71.817069 | 0.718171 | 1.081x |
| 8 | 9 | fixed weights | 65.273904 | 0.652739 | 1.000x |
| 8 | 9 | +structure-aware decoupled KLA | 69.149803 | 0.691498 | 1.059x |
| 9 | 10 | fixed weights | 63.939206 | 0.639392 | 1.000x |
| 9 | 10 | +structure-aware decoupled KLA | 67.803016 | 0.678030 | 1.060x |
| 10 | 11 | fixed weights | 63.999030 | 0.639990 | 1.000x |
| 10 | 11 | +structure-aware decoupled KLA | 68.830409 | 0.688304 | 1.075x |
| 11 | 12 | fixed weights | 64.650459 | 0.646505 | 1.000x |
| 11 | 12 | +structure-aware decoupled KLA | 68.638425 | 0.686384 | 1.062x |
| 12 | 13 | fixed weights | 64.069441 | 0.640694 | 1.000x |
| 12 | 13 | +structure-aware decoupled KLA | 70.170474 | 0.701705 | 1.095x |
| 13 | 14 | fixed weights | 64.991155 | 0.649912 | 1.000x |
| 13 | 14 | +structure-aware decoupled KLA | 68.829930 | 0.688299 | 1.059x |
| 14 | 15 | fixed weights | 63.094499 | 0.630945 | 1.000x |
| 14 | 15 | +structure-aware decoupled KLA | 68.140865 | 0.681409 | 1.080x |
| 15 | 16 | fixed weights | 67.264366 | 0.672644 | 1.000x |
| 15 | 16 | +structure-aware decoupled KLA | 68.860078 | 0.688601 | 1.024x |
| 16 | 17 | fixed weights | 64.212878 | 0.642129 | 1.000x |
| 16 | 17 | +structure-aware decoupled KLA | 68.821634 | 0.688216 | 1.072x |
| 17 | 18 | fixed weights | 63.738512 | 0.637385 | 1.000x |
| 17 | 18 | +structure-aware decoupled KLA | 68.387318 | 0.683873 | 1.073x |
| 18 | 19 | fixed weights | 64.864822 | 0.648648 | 1.000x |
| 18 | 19 | +structure-aware decoupled KLA | 70.144265 | 0.701443 | 1.081x |
| 19 | 20 | fixed weights | 83.680791 | 0.836808 | 1.000x |
| 19 | 20 | +structure-aware decoupled KLA | 103.522780 | 1.035228 | 1.237x |
| 20 | 21 | fixed weights | 82.199937 | 0.821999 | 1.000x |
| 20 | 21 | +structure-aware decoupled KLA | 68.215201 | 0.682152 | 0.830x |
| 21 | 22 | fixed weights | 64.382117 | 0.643821 | 1.000x |
| 21 | 22 | +structure-aware decoupled KLA | 68.956460 | 0.689565 | 1.071x |
| 22 | 23 | fixed weights | 64.733059 | 0.647331 | 1.000x |
| 22 | 23 | +structure-aware decoupled KLA | 95.438492 | 0.954385 | 1.474x |
| 23 | 24 | fixed weights | 75.579029 | 0.755790 | 1.000x |
| 23 | 24 | +structure-aware decoupled KLA | 79.895140 | 0.798951 | 1.057x |
| 24 | 25 | fixed weights | 59.866072 | 0.598661 | 1.000x |
| 24 | 25 | +structure-aware decoupled KLA | 64.793808 | 0.647938 | 1.082x |
| 25 | 26 | fixed weights | 59.762413 | 0.597624 | 1.000x |
| 25 | 26 | +structure-aware decoupled KLA | 64.595864 | 0.645959 | 1.081x |
| 26 | 27 | fixed weights | 59.603051 | 0.596031 | 1.000x |
| 26 | 27 | +structure-aware decoupled KLA | 64.138318 | 0.641383 | 1.076x |
| 27 | 28 | fixed weights | 59.267735 | 0.592677 | 1.000x |
| 27 | 28 | +structure-aware decoupled KLA | 64.261199 | 0.642612 | 1.084x |
| 28 | 29 | fixed weights | 59.650644 | 0.596506 | 1.000x |
| 28 | 29 | +structure-aware decoupled KLA | 64.853999 | 0.648540 | 1.087x |
| 29 | 30 | fixed weights | 59.561704 | 0.595617 | 1.000x |
| 29 | 30 | +structure-aware decoupled KLA | 64.246459 | 0.642465 | 1.079x |
| 30 | 31 | fixed weights | 59.991338 | 0.599913 | 1.000x |
| 30 | 31 | +structure-aware decoupled KLA | 64.204859 | 0.642049 | 1.070x |
| 31 | 32 | fixed weights | 74.466848 | 0.744668 | 1.000x |
| 31 | 32 | +structure-aware decoupled KLA | 72.907123 | 0.729071 | 0.979x |
| 32 | 33 | fixed weights | 69.326888 | 0.693269 | 1.000x |
| 32 | 33 | +structure-aware decoupled KLA | 74.393928 | 0.743939 | 1.073x |
| 33 | 34 | fixed weights | 69.517606 | 0.695176 | 1.000x |
| 33 | 34 | +structure-aware decoupled KLA | 75.132346 | 0.751323 | 1.081x |
| 34 | 35 | fixed weights | 69.153964 | 0.691540 | 1.000x |
| 34 | 35 | +structure-aware decoupled KLA | 74.791768 | 0.747918 | 1.082x |
| 35 | 36 | fixed weights | 69.160017 | 0.691600 | 1.000x |
| 35 | 36 | +structure-aware decoupled KLA | 74.580940 | 0.745809 | 1.078x |
| 36 | 37 | fixed weights | 68.370055 | 0.683701 | 1.000x |
| 36 | 37 | +structure-aware decoupled KLA | 74.844329 | 0.748443 | 1.095x |
| 37 | 38 | fixed weights | 68.449771 | 0.684498 | 1.000x |
| 37 | 38 | +structure-aware decoupled KLA | 74.047879 | 0.740479 | 1.082x |
| 38 | 39 | fixed weights | 69.897188 | 0.698972 | 1.000x |
| 38 | 39 | +structure-aware decoupled KLA | 74.977092 | 0.749771 | 1.073x |
| 39 | 40 | fixed weights | 79.983065 | 0.799831 | 1.000x |
| 39 | 40 | +structure-aware decoupled KLA | 71.383032 | 0.713830 | 0.892x |
| 40 | 41 | fixed weights | 68.014828 | 0.680148 | 1.000x |
| 40 | 41 | +structure-aware decoupled KLA | 74.810994 | 0.748110 | 1.100x |
| 41 | 42 | fixed weights | 89.107707 | 0.891077 | 1.000x |
| 41 | 42 | +structure-aware decoupled KLA | 78.988866 | 0.789889 | 0.886x |
| 42 | 43 | fixed weights | 70.128419 | 0.701284 | 1.000x |
| 42 | 43 | +structure-aware decoupled KLA | 77.162027 | 0.771620 | 1.100x |
| 43 | 44 | fixed weights | 68.140657 | 0.681407 | 1.000x |
| 43 | 44 | +structure-aware decoupled KLA | 96.721676 | 0.967217 | 1.419x |
| 44 | 45 | fixed weights | 68.971853 | 0.689719 | 1.000x |
| 44 | 45 | +structure-aware decoupled KLA | 86.466519 | 0.864665 | 1.254x |
| 45 | 46 | fixed weights | 78.812578 | 0.788126 | 1.000x |
| 45 | 46 | +structure-aware decoupled KLA | 77.972478 | 0.779725 | 0.989x |
| 46 | 47 | fixed weights | 65.921555 | 0.659216 | 1.000x |
| 46 | 47 | +structure-aware decoupled KLA | 72.773131 | 0.727731 | 1.104x |
| 47 | 48 | fixed weights | 62.009499 | 0.620095 | 1.000x |
| 47 | 48 | +structure-aware decoupled KLA | 71.263237 | 0.712632 | 1.149x |
| 48 | 49 | fixed weights | 60.957608 | 0.609576 | 1.000x |
| 48 | 49 | +structure-aware decoupled KLA | 65.080629 | 0.650806 | 1.068x |
| 49 | 50 | fixed weights | 66.070114 | 0.660701 | 1.000x |
| 49 | 50 | +structure-aware decoupled KLA | 77.031258 | 0.770313 | 1.166x |
| 50 | 51 | fixed weights | 72.446515 | 0.724465 | 1.000x |
| 50 | 51 | +structure-aware decoupled KLA | 76.717932 | 0.767179 | 1.059x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.003860 | 1.478959 | 0.397750 |
| +structure-aware decoupled KLA | 1.901547 | 1.414438 | 0.313450 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.003860 +/- 0.056069 | [1.988319, 2.019402] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 1.901547 +/- 0.054611 | [1.886409, 1.916684] | 50 |
| fixed weights | RMSE | 1.478959 +/- 0.037179 | [1.468653, 1.489264] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.414438 +/- 0.036151 | [1.404418, 1.424459] | 50 |
| fixed weights | CardErr | 0.397750 +/- 0.044679 | [0.385366, 0.410134] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.313450 +/- 0.034598 | [0.303860, 0.323040] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.102313 +/- 0.019136 | [0.097009, 0.107618] | 5.11% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.064520 +/- 0.011047 | [0.061458, 0.067582] | 4.36% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | CardErr | 0.084300 +/- 0.028409 | [0.076425, 0.092175] | 21.19% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 1.958102 | 1.448123 | 0.345600 |
| 1 | +structure-aware decoupled KLA | 1.873880 | 1.387617 | 0.297600 |
| 2 | fixed weights | 1.930001 | 1.450994 | 0.294600 |
| 2 | +structure-aware decoupled KLA | 1.870702 | 1.381841 | 0.300600 |
| 3 | fixed weights | 1.988107 | 1.450983 | 0.395800 |
| 3 | +structure-aware decoupled KLA | 1.889632 | 1.390160 | 0.321000 |
| 4 | fixed weights | 2.056793 | 1.507539 | 0.448800 |
| 4 | +structure-aware decoupled KLA | 1.975167 | 1.439846 | 0.400800 |
| 5 | fixed weights | 2.016638 | 1.507500 | 0.380200 |
| 5 | +structure-aware decoupled KLA | 1.920320 | 1.437731 | 0.314600 |
| 6 | fixed weights | 1.976905 | 1.495091 | 0.325600 |
| 6 | +structure-aware decoupled KLA | 1.889495 | 1.429858 | 0.277400 |
| 7 | fixed weights | 2.018851 | 1.488597 | 0.408400 |
| 7 | +structure-aware decoupled KLA | 1.917105 | 1.429257 | 0.319800 |
| 8 | fixed weights | 2.085483 | 1.482842 | 0.583000 |
| 8 | +structure-aware decoupled KLA | 1.876072 | 1.419197 | 0.275800 |
