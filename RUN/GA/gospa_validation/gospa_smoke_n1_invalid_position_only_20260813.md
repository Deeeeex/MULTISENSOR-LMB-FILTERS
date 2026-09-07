# INVALID — position-only GOSPA smoke (do not use in the paper)

This preserved diagnostic predates the complete-state correction. Its OSPA
value remains valid, but its GOSPA value was computed on position only and
must not be cited, aggregated, or copied into the manuscript.

# GA Tiered Link Ablation (2026-08-13 10:53:16)

Comparison order: fixed weights

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

- OSPA/GOSPA Euclidean cutoff c: 5
- OSPA/GOSPA order p: 2
- GOSPA alpha: 2

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
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | GOSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|------:|-----:|------------:|
| 1 | 2 | fixed weights | 2.605320 | 4.758364 | 2.258061 | 0.932500 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | GOSPA | RMSE | Cardinality |
|:----|-----:|------:|-----:|------------:|
| fixed weights | 2.605320 | 4.758364 | 2.258061 | 0.932500 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.605320 +/- 0.000000 | [2.605320, 2.605320] | 1 |
| fixed weights | GOSPA | 4.758364 +/- 0.000000 | [4.758364, 4.758364] | 1 |
| fixed weights | RMSE | 2.258061 +/- 0.000000 | [2.258061, 2.258061] | 1 |
| fixed weights | Cardinality | 0.932500 +/- 0.000000 | [0.932500, 0.932500] | 1 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 94.489590 +/- 0.000000 | 0.944896 | 1.000x | 1 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 94.489590 | 0.944896 | 1.000x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.993371 | 1.683022 | 1.760000 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.993371 +/- 0.000000 | [2.993371, 2.993371] | 1 |
| fixed weights | RMSE | 1.683022 +/- 0.000000 | [1.683022, 1.683022] | 1 |
| fixed weights | CardErr | 1.760000 +/- 0.000000 | [1.760000, 1.760000] | 1 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.315985 | 1.572866 | 0.670000 |
| 2 | fixed weights | 2.600744 | 1.795191 | 0.880000 |
| 3 | fixed weights | 2.721581 | 1.580457 | 1.090000 |
| 4 | fixed weights | 2.518571 | 1.726450 | 0.820000 |
| 5 | fixed weights | 2.986865 | 1.692057 | 1.590000 |
| 6 | fixed weights | 3.523735 | 1.717306 | 2.670000 |
| 7 | fixed weights | 3.836361 | 1.583738 | 3.410000 |
| 8 | fixed weights | 3.443131 | 1.796110 | 2.950000 |
