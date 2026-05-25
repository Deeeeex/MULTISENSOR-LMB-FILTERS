# GA Tiered Link Ablation (2026-05-20 11:26:05)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 3
- baseSeed: 41 (fixed=1)
- trialSeeds: [42 43 44]
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

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 42 | fixed weights | 1.663061 | 1.412572 | 0.143750 |
| 1 | 42 | +structure-aware decoupled KLA | 1.471015 | 1.201319 | 0.116250 |
| 2 | 43 | fixed weights | 1.770636 | 1.503401 | 0.121250 |
| 2 | 43 | +structure-aware decoupled KLA | 1.530303 | 1.275778 | 0.080000 |
| 3 | 44 | fixed weights | 1.768147 | 1.592295 | 0.190000 |
| 3 | 44 | +structure-aware decoupled KLA | 1.480749 | 1.251920 | 0.083750 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.733948 | 1.502756 | 0.151667 |
| +structure-aware decoupled KLA | 1.494023 | 1.243006 | 0.093333 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.733948 +/- 0.061402 | [1.581404, 1.886493] | 3 |
| +structure-aware decoupled KLA | OSPA | 1.494023 +/- 0.031794 | [1.415034, 1.573011] | 3 |
| fixed weights | RMSE | 1.502756 +/- 0.089863 | [1.279505, 1.726006] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.243006 +/- 0.038021 | [1.148548, 1.337464] | 3 |
| fixed weights | Cardinality | 0.151667 +/- 0.035052 | [0.064586, 0.238748] | 3 |
| +structure-aware decoupled KLA | Cardinality | 0.093333 +/- 0.019935 | [0.043809, 0.142858] | 3 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.239926 +/- 0.047677 | [0.121480, 0.358372] | 13.84% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.259750 +/- 0.070301 | [0.085099, 0.434401] | 17.28% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | Cardinality | 0.058333 +/- 0.042063 | [-0.046165, 0.162831] | 38.46% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 85.411930 +/- 0.976067 | 0.854119 | 1.000x | 3 |
| +structure-aware decoupled KLA | 94.564224 +/- 2.870474 | 0.945642 | 1.107x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 9.152294 +/- 2.738604 | 10.72% | 3/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 42 | fixed weights | 85.137230 | 0.851372 | 1.000x |
| 1 | 42 | +structure-aware decoupled KLA | 91.255770 | 0.912558 | 1.072x |
| 2 | 43 | fixed weights | 84.602648 | 0.846026 | 1.000x |
| 2 | 43 | +structure-aware decoupled KLA | 96.044607 | 0.960446 | 1.135x |
| 3 | 44 | fixed weights | 86.495911 | 0.864959 | 1.000x |
| 3 | 44 | +structure-aware decoupled KLA | 96.392295 | 0.963923 | 1.114x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.040574 | 1.508289 | 0.434167 |
| +structure-aware decoupled KLA | 1.924013 | 1.435170 | 0.335000 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.040574 +/- 0.049533 | [1.917517, 2.163631] | 3 |
| +structure-aware decoupled KLA | E-OSPA | 1.924013 +/- 0.054076 | [1.789669, 2.058356] | 3 |
| fixed weights | RMSE | 1.508289 +/- 0.023861 | [1.449011, 1.567568] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.435170 +/- 0.018669 | [1.388790, 1.481549] | 3 |
| fixed weights | CardErr | 0.434167 +/- 0.063299 | [0.276910, 0.591423] | 3 |
| +structure-aware decoupled KLA | CardErr | 0.335000 +/- 0.046887 | [0.218516, 0.451484] | 3 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.116561 +/- 0.015007 | [0.079279, 0.153844] | 5.71% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.073120 +/- 0.007086 | [0.055516, 0.090723] | 4.85% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | CardErr | 0.099167 +/- 0.034739 | [0.012864, 0.185469] | 22.84% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 1.973168 | 1.451347 | 0.350000 |
| 1 | +structure-aware decoupled KLA | 1.898638 | 1.408946 | 0.300000 |
| 2 | fixed weights | 1.938400 | 1.488631 | 0.280000 |
| 2 | +structure-aware decoupled KLA | 1.888637 | 1.413028 | 0.300000 |
| 3 | fixed weights | 2.050858 | 1.479624 | 0.496667 |
| 3 | +structure-aware decoupled KLA | 1.923306 | 1.434082 | 0.376667 |
| 4 | fixed weights | 2.093868 | 1.522583 | 0.443333 |
| 4 | +structure-aware decoupled KLA | 2.034699 | 1.440462 | 0.453333 |
| 5 | fixed weights | 2.030719 | 1.535242 | 0.416667 |
| 5 | +structure-aware decoupled KLA | 1.919292 | 1.462430 | 0.316667 |
| 6 | fixed weights | 1.986695 | 1.490184 | 0.370000 |
| 6 | +structure-aware decoupled KLA | 1.872934 | 1.425290 | 0.280000 |
| 7 | fixed weights | 2.032908 | 1.519649 | 0.433333 |
| 7 | +structure-aware decoupled KLA | 1.922412 | 1.449266 | 0.333333 |
| 8 | fixed weights | 2.217974 | 1.579055 | 0.683333 |
| 8 | +structure-aware decoupled KLA | 1.932182 | 1.447854 | 0.320000 |
