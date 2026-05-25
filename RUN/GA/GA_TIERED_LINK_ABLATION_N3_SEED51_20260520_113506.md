# GA Tiered Link Ablation (2026-05-20 11:35:06)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 3
- baseSeed: 51 (fixed=1)
- trialSeeds: [52 53 54]
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
- Trial 1: [0.5 0.1 0.1 0.5 0.1 0 0.1 0.2]
- Trial 2: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 3: [0.5 0.5 0.1 0.2 0.1 0 0.1 0.1]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 52 | fixed weights | 2.567349 | 2.251784 | 0.708750 |
| 1 | 52 | +structure-aware decoupled KLA | 1.800623 | 1.409313 | 0.215000 |
| 2 | 53 | fixed weights | 2.478524 | 2.323377 | 0.802500 |
| 2 | 53 | +structure-aware decoupled KLA | 1.786478 | 1.431049 | 0.201250 |
| 3 | 54 | fixed weights | 2.394537 | 2.625175 | 0.547500 |
| 3 | 54 | +structure-aware decoupled KLA | 1.842379 | 1.417838 | 0.213750 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.480136 | 2.400112 | 0.686250 |
| +structure-aware decoupled KLA | 1.809827 | 1.419400 | 0.210000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.480136 +/- 0.086417 | [2.265448, 2.694825] | 3 |
| +structure-aware decoupled KLA | OSPA | 1.809827 +/- 0.029065 | [1.737620, 1.882034] | 3 |
| fixed weights | RMSE | 2.400112 +/- 0.198170 | [1.907790, 2.892433] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.419400 +/- 0.010952 | [1.392192, 1.446608] | 3 |
| fixed weights | Cardinality | 0.686250 +/- 0.128980 | [0.365819, 1.006681] | 3 |
| +structure-aware decoupled KLA | Cardinality | 0.210000 +/- 0.007603 | [0.191110, 0.228890] | 3 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.670310 +/- 0.108923 | [0.399709, 0.940910] | 27.03% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.980712 +/- 0.197840 | [0.489211, 1.472213] | 40.86% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | Cardinality | 0.476250 +/- 0.134606 | [0.141843, 0.810657] | 69.40% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 67.584022 +/- 12.721008 | 0.675840 | 1.000x | 3 |
| +structure-aware decoupled KLA | 66.931531 +/- 16.276952 | 0.669315 | 0.993x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | -0.652492 +/- 10.750381 | -0.70% | 2/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 52 | fixed weights | 78.724845 | 0.787248 | 1.000x |
| 1 | 52 | +structure-aware decoupled KLA | 85.725468 | 0.857255 | 1.089x |
| 2 | 53 | fixed weights | 70.304305 | 0.703043 | 1.000x |
| 2 | 53 | +structure-aware decoupled KLA | 57.361033 | 0.573610 | 0.816x |
| 3 | 54 | fixed weights | 53.722917 | 0.537229 | 1.000x |
| 3 | 54 | +structure-aware decoupled KLA | 57.708091 | 0.577081 | 1.074x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.776318 | 1.642188 | 1.330417 |
| +structure-aware decoupled KLA | 2.286298 | 1.586605 | 0.536667 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.776318 +/- 0.105417 | [2.514425, 3.038211] | 3 |
| +structure-aware decoupled KLA | E-OSPA | 2.286298 +/- 0.077181 | [2.094555, 2.478041] | 3 |
| fixed weights | RMSE | 1.642188 +/- 0.036391 | [1.551781, 1.732595] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.586605 +/- 0.017095 | [1.544137, 1.629074] | 3 |
| fixed weights | CardErr | 1.330417 +/- 0.011342 | [1.302239, 1.358595] | 3 |
| +structure-aware decoupled KLA | CardErr | 0.536667 +/- 0.056684 | [0.395846, 0.677488] | 3 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.490020 +/- 0.028734 | [0.418636, 0.561404] | 17.65% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.055582 +/- 0.032531 | [-0.025236, 0.136400] | 3.38% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | CardErr | 0.793750 +/- 0.045621 | [0.680413, 0.907087] | 59.66% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 3.150708 | 1.699956 | 1.676667 |
| 1 | +structure-aware decoupled KLA | 2.455296 | 1.592127 | 0.616667 |
| 2 | fixed weights | 2.907210 | 1.695153 | 1.303333 |
| 2 | +structure-aware decoupled KLA | 2.367065 | 1.621379 | 0.546667 |
| 3 | fixed weights | 3.031027 | 1.827521 | 1.343333 |
| 3 | +structure-aware decoupled KLA | 2.550221 | 1.671806 | 0.716667 |
| 4 | fixed weights | 3.647517 | 1.702638 | 2.883333 |
| 4 | +structure-aware decoupled KLA | 2.736394 | 1.745271 | 0.973333 |
| 5 | fixed weights | 2.428237 | 1.606692 | 0.836667 |
| 5 | +structure-aware decoupled KLA | 2.071387 | 1.544083 | 0.360000 |
| 6 | fixed weights | 2.218493 | 1.555435 | 0.536667 |
| 6 | +structure-aware decoupled KLA | 2.039144 | 1.515153 | 0.346667 |
| 7 | fixed weights | 2.146082 | 1.501307 | 0.563333 |
| 7 | +structure-aware decoupled KLA | 1.993358 | 1.477028 | 0.350000 |
| 8 | fixed weights | 2.681270 | 1.548799 | 1.500000 |
| 8 | +structure-aware decoupled KLA | 2.077521 | 1.525996 | 0.383333 |
