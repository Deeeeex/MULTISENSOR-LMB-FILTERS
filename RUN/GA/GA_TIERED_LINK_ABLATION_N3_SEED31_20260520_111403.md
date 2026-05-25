# GA Tiered Link Ablation (2026-05-20 11:14:03)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 3
- baseSeed: 31 (fixed=1)
- trialSeeds: [32 33 34]
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
| 1 | 32 | fixed weights | 1.607239 | 1.353930 | 0.085000 |
| 1 | 32 | +structure-aware decoupled KLA | 1.391331 | 1.175269 | 0.071250 |
| 2 | 33 | fixed weights | 1.664329 | 1.367188 | 0.148750 |
| 2 | 33 | +structure-aware decoupled KLA | 1.426588 | 1.188891 | 0.083750 |
| 3 | 34 | fixed weights | 1.594309 | 1.418825 | 0.061250 |
| 3 | 34 | +structure-aware decoupled KLA | 1.396445 | 1.204412 | 0.055000 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.621959 | 1.379981 | 0.098333 |
| +structure-aware decoupled KLA | 1.404788 | 1.189524 | 0.070000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.621959 +/- 0.037259 | [1.529395, 1.714523] | 3 |
| +structure-aware decoupled KLA | OSPA | 1.404788 +/- 0.019052 | [1.357457, 1.452120] | 3 |
| fixed weights | RMSE | 1.379981 +/- 0.034287 | [1.294801, 1.465161] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.189524 +/- 0.014582 | [1.153298, 1.225750] | 3 |
| fixed weights | Cardinality | 0.098333 +/- 0.045248 | [-0.014078, 0.210745] | 3 |
| +structure-aware decoupled KLA | Cardinality | 0.070000 +/- 0.014416 | [0.034187, 0.105813] | 3 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.217171 +/- 0.019969 | [0.167562, 0.266780] | 13.39% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.190457 +/- 0.020747 | [0.138913, 0.242000] | 13.80% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | Cardinality | 0.028333 +/- 0.031975 | [-0.051103, 0.107770] | 28.81% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 79.377634 +/- 5.636167 | 0.793776 | 1.000x | 3 |
| +structure-aware decoupled KLA | 89.182162 +/- 2.236220 | 0.891822 | 1.127x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 9.804528 +/- 6.013852 | 12.74% | 3/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 32 | fixed weights | 72.945670 | 0.729457 | 1.000x |
| 1 | 32 | +structure-aware decoupled KLA | 88.727690 | 0.887277 | 1.216x |
| 2 | 33 | fixed weights | 83.453123 | 0.834531 | 1.000x |
| 2 | 33 | +structure-aware decoupled KLA | 87.208087 | 0.872081 | 1.045x |
| 3 | 34 | fixed weights | 81.734109 | 0.817341 | 1.000x |
| 3 | 34 | +structure-aware decoupled KLA | 91.610710 | 0.916107 | 1.121x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 1.966736 | 1.477450 | 0.296667 |
| +structure-aware decoupled KLA | 1.890199 | 1.409329 | 0.273333 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 1.966736 +/- 0.011698 | [1.937676, 1.995797] | 3 |
| +structure-aware decoupled KLA | E-OSPA | 1.890199 +/- 0.020613 | [1.838990, 1.941408] | 3 |
| fixed weights | RMSE | 1.477450 +/- 0.016372 | [1.436776, 1.518124] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.409329 +/- 0.016690 | [1.367867, 1.450792] | 3 |
| fixed weights | CardErr | 0.296667 +/- 0.037088 | [0.204527, 0.388806] | 3 |
| +structure-aware decoupled KLA | CardErr | 0.273333 +/- 0.008133 | [0.253128, 0.293538] | 3 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.076537 +/- 0.016998 | [0.034309, 0.118765] | 3.89% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.068121 +/- 0.001931 | [0.063322, 0.072919] | 4.61% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | CardErr | 0.023333 +/- 0.029695 | [-0.050438, 0.097105] | 7.87% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.037353 | 1.508642 | 0.386667 |
| 1 | +structure-aware decoupled KLA | 1.966311 | 1.444481 | 0.343333 |
| 2 | fixed weights | 2.005545 | 1.474933 | 0.336667 |
| 2 | +structure-aware decoupled KLA | 1.932057 | 1.402707 | 0.303333 |
| 3 | fixed weights | 2.034127 | 1.461006 | 0.383333 |
| 3 | +structure-aware decoupled KLA | 1.889050 | 1.399378 | 0.273333 |
| 4 | fixed weights | 1.986436 | 1.507641 | 0.280000 |
| 4 | +structure-aware decoupled KLA | 1.922658 | 1.442437 | 0.280000 |
| 5 | fixed weights | 1.916879 | 1.459240 | 0.263333 |
| 5 | +structure-aware decoupled KLA | 1.823704 | 1.381957 | 0.240000 |
| 6 | fixed weights | 1.942201 | 1.471330 | 0.253333 |
| 6 | +structure-aware decoupled KLA | 1.862687 | 1.391651 | 0.256667 |
| 7 | fixed weights | 1.929066 | 1.464591 | 0.250000 |
| 7 | +structure-aware decoupled KLA | 1.877321 | 1.410315 | 0.256667 |
| 8 | fixed weights | 1.882285 | 1.472215 | 0.220000 |
| 8 | +structure-aware decoupled KLA | 1.847805 | 1.401708 | 0.233333 |
