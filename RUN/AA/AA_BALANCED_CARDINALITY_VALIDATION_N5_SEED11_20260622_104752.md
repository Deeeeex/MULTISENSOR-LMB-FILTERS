# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 10:57:07

Comparison order: Tuned spatial-KLA AA -> Bridge-aware spatial-KLA AA

## Scope
This is an AA-specific diagnostic, not a replacement for the GA/KLA paper main table. It tests whether the current Balanced and Cardinality-critical adaptive weights remain useful when consumed by AA-LMB fusion.

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
- lmbParallelUpdateMode: AA
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- targetFormationLifeSpan: 24
- existenceThreshold: 0.180000
- maximumNumberOfGmComponents: 3
- minimumTrajectoryLength: 10
- maximumNumberOfLbpIterations: 150
- lbpConvergenceTolerance: 0.0001
- aaStrictWeights: 0
- linkModel: fixed
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Arm Configs
### Tuned spatial-KLA AA
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
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.750
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- aaKlaSpatialExistencePower: 0.000
- aaKlaSpatialExistenceMinScore: 0.000
- spatialBridgeNoveltyStrength: 0.000
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

### Bridge-aware spatial-KLA AA
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
- spatialDecouplingStrength: 1.000
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.750
- existenceStructureStrength: 0.080
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- aaKlaSpatialExistencePower: 0.000
- aaKlaSpatialExistenceMinScore: 0.000
- spatialBridgeNoveltyStrength: 0.750
- captureWeightDiagnostics: 0
- existenceMinWeight: 0.000

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.5 0 0.1 0.1 0.1 0.5 0.2]
- Trial 2: [0.2 0 0.1 0.5 0.1 0.1 0.5 0.1]
- Trial 3: [0.1 0.1 0.5 0.2 0.1 0.1 0.5 0]
- Trial 4: [0.5 0.5 0 0.2 0.1 0.1 0.1 0.1]
- Trial 5: [0.5 0.5 0.1 0 0.2 0.1 0.1 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | Loc. disag. | Card. disp. |
|------:|-----:|:----|-----:|------------:|------------:|
| 1 | 12 | Tuned spatial-KLA AA | 1.673244 | 1.464816 | 0.036250 |
| 1 | 12 | Bridge-aware spatial-KLA AA | 1.683815 | 1.477663 | 0.036250 |
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 2 | 13 | Bridge-aware spatial-KLA AA | 1.808066 | 1.630884 | 0.048750 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 3 | 14 | Bridge-aware spatial-KLA AA | 1.693161 | 1.569940 | 0.023750 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 4 | 15 | Bridge-aware spatial-KLA AA | 1.738366 | 1.467087 | 0.037500 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 5 | 16 | Bridge-aware spatial-KLA AA | 1.657830 | 1.573988 | 0.022500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 |
| Bridge-aware spatial-KLA AA | 1.716247 | 1.543912 | 0.033750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.702915 +/- 0.065821 | [1.621201, 1.784630] | 5 |
| Bridge-aware spatial-KLA AA | OSPA | 1.716247 +/- 0.058986 | [1.643018, 1.789476] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.529716 +/- 0.069688 | [1.443201, 1.616231] | 5 |
| Bridge-aware spatial-KLA AA | Loc. disag. | 1.543912 +/- 0.069709 | [1.457371, 1.630453] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |
| Bridge-aware spatial-KLA AA | Card. disp. | 0.033750 +/- 0.010861 | [0.020266, 0.047234] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Bridge-aware spatial-KLA AA | OSPA | -0.013332 +/- 0.009134 | [-0.024672, -0.001992] | -0.78% | 1/5 | 0.375 |
| Bridge-aware spatial-KLA AA | Loc. disag. | -0.014196 +/- 0.002937 | [-0.017842, -0.010550] | -0.93% | 0/5 | 0.0625 |
| Bridge-aware spatial-KLA AA | Card. disp. | 0.000500 +/- 0.001425 | [-0.001269, 0.002269] | 1.46% | 2/5 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.032799 | 3.588145 | 0.086250 |
| Bridge-aware spatial-KLA AA | 2.025109 | 3.587982 | 0.085750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.032799 +/- 0.040742 | [1.982219, 2.083379] | 5 |
| Bridge-aware spatial-KLA AA | E-OSPA | 2.025109 +/- 0.032255 | [1.985065, 2.065153] | 5 |
| Tuned spatial-KLA AA | RMSE | 3.588145 +/- 0.379057 | [3.117559, 4.058731] | 5 |
| Bridge-aware spatial-KLA AA | RMSE | 3.587982 +/- 0.377422 | [3.119426, 4.056537] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |
| Bridge-aware spatial-KLA AA | CardErr | 0.085750 +/- 0.011812 | [0.071085, 0.100415] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Bridge-aware spatial-KLA AA | E-OSPA | 0.007690 +/- 0.008595 | [-0.002980, 0.018360] | 0.38% | 5/5 | 0.0625 |
| Bridge-aware spatial-KLA AA | RMSE | 0.000163 +/- 0.038290 | [-0.047373, 0.047700] | 0.00% | 4/5 | 0.375 |
| Bridge-aware spatial-KLA AA | CardErr | 0.000500 +/- 0.001425 | [-0.001269, 0.002269] | 0.58% | 2/5 | 1 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 48.112139 +/- 0.848671 | 2.004672 | 1.000x | 5 |
| Bridge-aware spatial-KLA AA | 48.855450 +/- 1.078273 | 2.035644 | 1.016x | 5 |
