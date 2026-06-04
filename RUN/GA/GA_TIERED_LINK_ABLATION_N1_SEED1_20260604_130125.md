# GA Tiered Link Ablation (2026-06-04 13:01:25)

Comparison order: +structure-aware decoupled KLA -> +FID-FIA existence refinement

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
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.050
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

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| 1 | 2 | +FID-FIA existence refinement | 1.714819 | 1.478188 | 0.066250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| +FID-FIA existence refinement | 1.714819 | 1.478188 | 0.066250 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | OSPA | 1.878234 +/- 0.000000 | [1.878234, 1.878234] | 1 |
| +FID-FIA existence refinement | OSPA | 1.714819 +/- 0.000000 | [1.714819, 1.714819] | 1 |
| +structure-aware decoupled KLA | RMSE | 1.439986 +/- 0.000000 | [1.439986, 1.439986] | 1 |
| +FID-FIA existence refinement | RMSE | 1.478188 +/- 0.000000 | [1.478188, 1.478188] | 1 |
| +structure-aware decoupled KLA | Cardinality | 0.240000 +/- 0.000000 | [0.240000, 0.240000] | 1 |
| +FID-FIA existence refinement | Cardinality | 0.066250 +/- 0.000000 | [0.066250, 0.066250] | 1 |

## Paired Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | OSPA | 0.163414 +/- 0.000000 | [0.163414, 0.163414] | 8.70% | 1/1 | 1 |
| +FID-FIA existence refinement | RMSE | -0.038202 +/- 0.000000 | [-0.038202, -0.038202] | -2.65% | 0/1 | 1 |
| +FID-FIA existence refinement | Cardinality | 0.173750 +/- 0.000000 | [0.173750, 0.173750] | 72.40% | 1/1 | 1 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| +structure-aware decoupled KLA | 51.037624 +/- 0.000000 | 0.510376 | 1.000x | 1 |
| +FID-FIA existence refinement | 139.686914 +/- 0.000000 | 1.396869 | 2.737x | 1 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +FID-FIA existence refinement | 88.649290 +/- 0.000000 | 173.69% | 1/1 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to +structure-aware decoupled KLA |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | +structure-aware decoupled KLA | 51.037624 | 0.510376 | 1.000x |
| 1 | 2 | +FID-FIA existence refinement | 139.686914 | 1.396869 | 2.737x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| +structure-aware decoupled KLA | 2.414068 | 1.664513 | 0.660000 |
| +FID-FIA existence refinement | 2.103998 | 1.701459 | 0.226250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| +structure-aware decoupled KLA | E-OSPA | 2.414068 +/- 0.000000 | [2.414068, 2.414068] | 1 |
| +FID-FIA existence refinement | E-OSPA | 2.103998 +/- 0.000000 | [2.103998, 2.103998] | 1 |
| +structure-aware decoupled KLA | RMSE | 1.664513 +/- 0.000000 | [1.664513, 1.664513] | 1 |
| +FID-FIA existence refinement | RMSE | 1.701459 +/- 0.000000 | [1.701459, 1.701459] | 1 |
| +structure-aware decoupled KLA | CardErr | 0.660000 +/- 0.000000 | [0.660000, 0.660000] | 1 |
| +FID-FIA existence refinement | CardErr | 0.226250 +/- 0.000000 | [0.226250, 0.226250] | 1 |

## Paired Local-Metric Improvements Relative to +structure-aware decoupled KLA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +FID-FIA existence refinement | E-OSPA | 0.310071 +/- 0.000000 | [0.310071, 0.310071] | 12.84% | 1/1 | 1 |
| +FID-FIA existence refinement | RMSE | -0.036945 +/- 0.000000 | [-0.036945, -0.036945] | -2.22% | 0/1 | 1 |
| +FID-FIA existence refinement | CardErr | 0.433750 +/- 0.000000 | [0.433750, 0.433750] | 65.72% | 1/1 | 1 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | +structure-aware decoupled KLA | 2.063496 | 1.535454 | 0.400000 |
| 1 | +FID-FIA existence refinement | 1.887565 | 1.566498 | 0.150000 |
| 2 | +structure-aware decoupled KLA | 2.405444 | 1.700100 | 0.700000 |
| 2 | +FID-FIA existence refinement | 2.080589 | 1.677235 | 0.250000 |
| 3 | +structure-aware decoupled KLA | 2.318174 | 1.567967 | 0.710000 |
| 3 | +FID-FIA existence refinement | 2.038910 | 1.630974 | 0.250000 |
| 4 | +structure-aware decoupled KLA | 2.210348 | 1.581444 | 0.550000 |
| 4 | +FID-FIA existence refinement | 1.979246 | 1.623277 | 0.200000 |
| 5 | +structure-aware decoupled KLA | 2.554959 | 1.741088 | 0.720000 |
| 5 | +FID-FIA existence refinement | 2.234557 | 1.801287 | 0.260000 |
| 6 | +structure-aware decoupled KLA | 2.495166 | 1.708043 | 0.650000 |
| 6 | +FID-FIA existence refinement | 2.187549 | 1.776103 | 0.220000 |
| 7 | +structure-aware decoupled KLA | 2.660840 | 1.769972 | 0.800000 |
| 7 | +FID-FIA existence refinement | 2.193970 | 1.764280 | 0.230000 |
| 8 | +structure-aware decoupled KLA | 2.604120 | 1.712039 | 0.750000 |
| 8 | +FID-FIA existence refinement | 2.229596 | 1.772015 | 0.250000 |
