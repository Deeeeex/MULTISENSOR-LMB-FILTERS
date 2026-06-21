# AA Balanced/Cardinality Validation

Generated at: 2026-06-21 14:20:39

Comparison order: Balanced spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 10
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11]
- lmbParallelUpdateMode: AA
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- targetFormationLifeSpan: 24
- existenceThreshold: 0.050000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Balanced spatial-KLA AA
- enabled: 1
- method: factorized
- aaSpatialFusionMode: kla
- emaAlpha: 0.000
- minWeight: 0.000
- spatialEmaAlpha: 0.000
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.000
- existenceMinWeight: 0.000
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
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.2 0 0.1 0.1 0.5 0.1 0.1]
- Trial 2: [0.5 0 0.1 0.1 0.1 0.1 0.5 0.2]
- Trial 3: [0 0.1 0.5 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0 0.1 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 5: [0.1 0.1 0.1 0.5 0.2 0 0.1 0.5]
- Trial 6: [0.1 0.2 0.1 0.5 0.1 0.5 0.1 0]
- Trial 7: [0.1 0 0.5 0.1 0.1 0.1 0.2 0.5]
- Trial 8: [0 0.1 0.5 0.1 0.1 0.2 0.5 0.1]
- Trial 9: [0.5 0.1 0.1 0 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0 0.5 0.1 0.2 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 2 | Balanced spatial-KLA AA | 1.709135 | 1.498033 | 0.021250 |
| 2 | 3 | Balanced spatial-KLA AA | 1.685990 | 1.459773 | 0.035000 |
| 3 | 4 | Balanced spatial-KLA AA | 1.659577 | 1.569792 | 0.023750 |
| 4 | 5 | Balanced spatial-KLA AA | 1.629002 | 1.509168 | 0.032500 |
| 5 | 6 | Balanced spatial-KLA AA | 1.602749 | 1.628511 | 0.018750 |
| 6 | 7 | Balanced spatial-KLA AA | 1.577246 | 1.364218 | 0.018750 |
| 7 | 8 | Balanced spatial-KLA AA | 1.801845 | 1.648450 | 0.037500 |
| 8 | 9 | Balanced spatial-KLA AA | 1.641679 | 1.389084 | 0.023750 |
| 9 | 10 | Balanced spatial-KLA AA | 1.647265 | 1.850955 | 0.030000 |
| 10 | 11 | Balanced spatial-KLA AA | 1.771848 | 1.570386 | 0.033750 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Balanced spatial-KLA AA | 1.672633 | 1.548837 | 0.027500 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | OSPA | 1.672633 +/- 0.071265 | [1.621657, 1.723610] | 10 |
| Balanced spatial-KLA AA | Loc. disag. | 1.548837 +/- 0.141477 | [1.447638, 1.650037] | 10 |
| Balanced spatial-KLA AA | Card. disp. | 0.027500 +/- 0.007046 | [0.022460, 0.032540] | 10 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Balanced spatial-KLA AA | 2.002454 | 3.808155 | 0.077750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Balanced spatial-KLA AA | E-OSPA | 2.002454 +/- 0.062265 | [1.957915, 2.046993] | 10 |
| Balanced spatial-KLA AA | RMSE | 3.808155 +/- 0.266118 | [3.617799, 3.998512] | 10 |
| Balanced spatial-KLA AA | CardErr | 0.077750 +/- 0.007879 | [0.072114, 0.083386] | 10 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Balanced spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Balanced spatial-KLA AA | 48.710554 +/- 1.979556 | 2.029606 | 1.000x | 10 |
