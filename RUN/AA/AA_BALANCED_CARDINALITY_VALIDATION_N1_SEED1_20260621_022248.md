# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:23:59

Comparison order: Balanced AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- trialSeeds: 2
- lmbParallelUpdateMode: AA
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- targetFormationLifeSpan: 24
- existenceThreshold: 0.080000
- maximumNumberOfGmComponents: 1
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Balanced AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 0
- aaStrictWeights: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced AA | 2.964614 | 3.268654 | 0.117500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced AA | 2.964614 | 3.268654 | 0.117500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | OSPA | 2.964614 +/- 0.000000 | [2.964614, 2.964614] | 1 |
| Balanced AA | Loc. disag. | 3.268654 +/- 0.000000 | [3.268654, 3.268654] | 1 |
| Balanced AA | Card. disp. | 0.117500 +/- 0.000000 | [0.117500, 0.117500] | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced AA | 3.195276 | 3.980186 | 0.190000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | E-OSPA | 3.195276 +/- 0.000000 | [3.195276, 3.195276] | 1 |
| Balanced AA | RMSE | 3.980186 +/- 0.000000 | [3.980186, 3.980186] | 1 |
| Balanced AA | CardErr | 0.190000 +/- 0.000000 | [0.190000, 0.190000] | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced AA | 62.855242 +/- 0.000000 | 2.618968 | 1.000x | 1 |
