# GA Tiered Link Ablation (2026-06-21 02:15:22)

Comparison order: fixed weights -> +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- trialSeeds: 2
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: fidFiaExistenceRefinement

## Arm Configs
### fixed weights
- enabled: 0
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useHistorySmoothedExistenceConfidence: 0
- existenceHistoryEmaAlpha: 0.800
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 0
- useStructureAwareKla: 0
- usePosteriorStructureConsistency: 1
- existenceConfidenceMinScore: 0.600
- existenceConfidencePower: 1.000
- emaAlpha: 0.700
- minWeight: 0.050
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

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.697083 | 2.122734 | 0.342500 |
| 1 | 2 | +structure-aware decoupled KLA | 1.786737 | 1.505206 | 0.091250 |
| 1 | 2 | +FID-FIA existence refinement | 1.864203 | 1.916324 | 0.078750 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.697083 | 2.122734 | 0.342500 |
| +structure-aware decoupled KLA | 1.786737 | 1.505206 | 0.091250 |
| +FID-FIA existence refinement | 1.864203 | 1.916324 | 0.078750 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.697083 +/- 0.000000 | [2.697083, 2.697083] | 1 |
| +structure-aware decoupled KLA | OSPA | 1.786737 +/- 0.000000 | [1.786737, 1.786737] | 1 |
| +FID-FIA existence refinement | OSPA | 1.864203 +/- 0.000000 | [1.864203, 1.864203] | 1 |
| fixed weights | RMSE | 2.122734 +/- 0.000000 | [2.122734, 2.122734] | 1 |
| +structure-aware decoupled KLA | RMSE | 1.505206 +/- 0.000000 | [1.505206, 1.505206] | 1 |
| +FID-FIA existence refinement | RMSE | 1.916324 +/- 0.000000 | [1.916324, 1.916324] | 1 |
| fixed weights | Cardinality | 0.342500 +/- 0.000000 | [0.342500, 0.342500] | 1 |
| +structure-aware decoupled KLA | Cardinality | 0.091250 +/- 0.000000 | [0.091250, 0.091250] | 1 |
| +FID-FIA existence refinement | Cardinality | 0.078750 +/- 0.000000 | [0.078750, 0.078750] | 1 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.910346 +/- 0.000000 | [0.910346, 0.910346] | 33.75% | 1/1 | 1 |
| +FID-FIA existence refinement | OSPA | 0.832880 +/- 0.000000 | [0.832880, 0.832880] | 30.88% | 1/1 | 1 |
| +structure-aware decoupled KLA | RMSE | 0.617527 +/- 0.000000 | [0.617527, 0.617527] | 29.09% | 1/1 | 1 |
| +FID-FIA existence refinement | RMSE | 0.206409 +/- 0.000000 | [0.206409, 0.206409] | 9.72% | 1/1 | 1 |
| +structure-aware decoupled KLA | Cardinality | 0.251250 +/- 0.000000 | [0.251250, 0.251250] | 73.36% | 1/1 | 1 |
| +FID-FIA existence refinement | Cardinality | 0.263750 +/- 0.000000 | [0.263750, 0.263750] | 77.01% | 1/1 | 1 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 42.640608 +/- 0.000000 | 1.776692 | 1.000x | 1 |
| +structure-aware decoupled KLA | 45.627532 +/- 0.000000 | 1.901147 | 1.070x | 1 |
| +FID-FIA existence refinement | 102.274239 +/- 0.000000 | 4.261427 | 2.399x | 1 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 2.986924 +/- 0.000000 | 7.00% | 1/1 |
| +FID-FIA existence refinement | 59.633631 +/- 0.000000 | 139.85% | 1/1 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 42.640608 | 1.776692 | 1.000x |
| 1 | 2 | +structure-aware decoupled KLA | 45.627532 | 1.901147 | 1.070x |
| 1 | 2 | +FID-FIA existence refinement | 102.274239 | 4.261427 | 2.399x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 3.126791 | 3.589215 | 0.860000 |
| +structure-aware decoupled KLA | 2.247748 | 4.312276 | 0.226250 |
| +FID-FIA existence refinement | 2.169716 | 4.642874 | 0.146250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 3.126791 +/- 0.000000 | [3.126791, 3.126791] | 1 |
| +structure-aware decoupled KLA | E-OSPA | 2.247748 +/- 0.000000 | [2.247748, 2.247748] | 1 |
| +FID-FIA existence refinement | E-OSPA | 2.169716 +/- 0.000000 | [2.169716, 2.169716] | 1 |
| fixed weights | RMSE | 3.589215 +/- 0.000000 | [3.589215, 3.589215] | 1 |
| +structure-aware decoupled KLA | RMSE | 4.312276 +/- 0.000000 | [4.312276, 4.312276] | 1 |
| +FID-FIA existence refinement | RMSE | 4.642874 +/- 0.000000 | [4.642874, 4.642874] | 1 |
| fixed weights | CardErr | 0.860000 +/- 0.000000 | [0.860000, 0.860000] | 1 |
| +structure-aware decoupled KLA | CardErr | 0.226250 +/- 0.000000 | [0.226250, 0.226250] | 1 |
| +FID-FIA existence refinement | CardErr | 0.146250 +/- 0.000000 | [0.146250, 0.146250] | 1 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.879043 +/- 0.000000 | [0.879043, 0.879043] | 28.11% | 1/1 | 1 |
| +FID-FIA existence refinement | E-OSPA | 0.957075 +/- 0.000000 | [0.957075, 0.957075] | 30.61% | 1/1 | 1 |
| +structure-aware decoupled KLA | RMSE | -0.723061 +/- 0.000000 | [-0.723061, -0.723061] | -20.15% | 0/1 | 1 |
| +FID-FIA existence refinement | RMSE | -1.053659 +/- 0.000000 | [-1.053659, -1.053659] | -29.36% | 0/1 | 1 |
| +structure-aware decoupled KLA | CardErr | 0.633750 +/- 0.000000 | [0.633750, 0.633750] | 73.69% | 1/1 | 1 |
| +FID-FIA existence refinement | CardErr | 0.713750 +/- 0.000000 | [0.713750, 0.713750] | 82.99% | 1/1 | 1 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 3.604575 | 4.157170 | 1.130000 |
| 1 | +structure-aware decoupled KLA | 2.109878 | 4.063810 | 0.180000 |
| 1 | +FID-FIA existence refinement | 2.093746 | 4.071429 | 0.150000 |
| 2 | fixed weights | 3.585205 | 3.159268 | 1.210000 |
| 2 | +structure-aware decoupled KLA | 2.232857 | 4.138125 | 0.220000 |
| 2 | +FID-FIA existence refinement | 2.268100 | 4.194313 | 0.200000 |
| 3 | fixed weights | 2.583889 | 3.448020 | 0.470000 |
| 3 | +structure-aware decoupled KLA | 2.287741 | 4.107045 | 0.280000 |
| 3 | +FID-FIA existence refinement | 2.228573 | 4.988769 | 0.180000 |
| 4 | fixed weights | 2.653264 | 4.021072 | 0.570000 |
| 4 | +structure-aware decoupled KLA | 2.172697 | 4.081823 | 0.230000 |
| 4 | +FID-FIA existence refinement | 2.038465 | 4.050927 | 0.140000 |
| 5 | fixed weights | 3.423206 | 3.749032 | 1.030000 |
| 5 | +structure-aware decoupled KLA | 2.412353 | 4.784164 | 0.230000 |
| 5 | +FID-FIA existence refinement | 2.330912 | 5.113371 | 0.170000 |
| 6 | fixed weights | 3.513621 | 2.340225 | 1.250000 |
| 6 | +structure-aware decoupled KLA | 2.212104 | 4.183680 | 0.220000 |
| 6 | +FID-FIA existence refinement | 2.096938 | 4.565903 | 0.090000 |
| 7 | fixed weights | 2.866240 | 4.175276 | 0.580000 |
| 7 | +structure-aware decoupled KLA | 2.240735 | 4.593743 | 0.220000 |
| 7 | +FID-FIA existence refinement | 2.112432 | 5.648779 | 0.110000 |
| 8 | fixed weights | 2.784325 | 3.663656 | 0.640000 |
| 8 | +structure-aware decoupled KLA | 2.313620 | 4.545815 | 0.230000 |
| 8 | +FID-FIA existence refinement | 2.188560 | 4.509500 | 0.130000 |
