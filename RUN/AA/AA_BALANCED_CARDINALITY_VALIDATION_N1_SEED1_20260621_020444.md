# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 02:07:55

Comparison order: Balanced AA -> Cardinality-critical AA

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
- existenceThreshold: 0.030000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 1
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
- aaStrictWeights: 1
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

### Cardinality-critical AA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useFidFiaExistence: 1
- aaStrictWeights: 1
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 1.000
- fidFiaExistenceMinScore: 0.400
- existenceMinWeight: 0.050

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced AA | 3.396263 | 4.153744 | 0.102500 |
| 1 | 2 | Cardinality-critical AA | 3.396263 | 4.153744 | 0.102500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced AA | 3.396263 | 4.153744 | 0.102500 |
| Cardinality-critical AA | 3.396263 | 4.153744 | 0.102500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | OSPA | 3.396263 +/- 0.000000 | [3.396263, 3.396263] | 1 |
| Cardinality-critical AA | OSPA | 3.396263 +/- 0.000000 | [3.396263, 3.396263] | 1 |
| Balanced AA | Loc. disag. | 4.153744 +/- 0.000000 | [4.153744, 4.153744] | 1 |
| Cardinality-critical AA | Loc. disag. | 4.153744 +/- 0.000000 | [4.153744, 4.153744] | 1 |
| Balanced AA | Card. disp. | 0.102500 +/- 0.000000 | [0.102500, 0.102500] | 1 |
| Cardinality-critical AA | Card. disp. | 0.102500 +/- 0.000000 | [0.102500, 0.102500] | 1 |

## Paired Improvements Relative to Balanced AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality-critical AA | OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |
| Cardinality-critical AA | Loc. disag. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |
| Cardinality-critical AA | Card. disp. | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced AA | 3.617960 | 4.867106 | 0.160000 |
| Cardinality-critical AA | 3.617960 | 4.867106 | 0.160000 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced AA | E-OSPA | 3.617960 +/- 0.000000 | [3.617960, 3.617960] | 1 |
| Cardinality-critical AA | E-OSPA | 3.617960 +/- 0.000000 | [3.617960, 3.617960] | 1 |
| Balanced AA | RMSE | 4.867106 +/- 0.000000 | [4.867106, 4.867106] | 1 |
| Cardinality-critical AA | RMSE | 4.867106 +/- 0.000000 | [4.867106, 4.867106] | 1 |
| Balanced AA | CardErr | 0.160000 +/- 0.000000 | [0.160000, 0.160000] | 1 |
| Cardinality-critical AA | CardErr | 0.160000 +/- 0.000000 | [0.160000, 0.160000] | 1 |

## Paired Local-Metric Improvements Relative to Balanced AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cardinality-critical AA | E-OSPA | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |
| Cardinality-critical AA | RMSE | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |
| Cardinality-critical AA | CardErr | 0.000000 +/- 0.000000 | [0.000000, 0.000000] | 0.00% | 0/1 | NaN |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced AA | 73.967902 +/- 0.000000 | 3.081996 | 1.000x | 1 |
| Cardinality-critical AA | 103.526451 +/- 0.000000 | 4.313602 | 1.400x | 1 |
