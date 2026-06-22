# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 10:09:46

Comparison order: Tuned spatial-KLA AA -> Existence-gated spatial-KLA AA

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
- captureWeightDiagnostics: 1
- existenceMinWeight: 0.000

### Existence-gated spatial-KLA AA
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
- aaKlaSpatialExistencePower: 1.000
- aaKlaSpatialExistenceMinScore: 0.000
- captureWeightDiagnostics: 1
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
| 1 | 12 | Existence-gated spatial-KLA AA | 1.674255 | 1.462842 | 0.036250 |
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 2 | 13 | Existence-gated spatial-KLA AA | 1.807358 | 1.621167 | 0.048750 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 3 | 14 | Existence-gated spatial-KLA AA | 1.695126 | 1.572257 | 0.023750 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 4 | 15 | Existence-gated spatial-KLA AA | 1.716664 | 1.448378 | 0.037500 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |
| 5 | 16 | Existence-gated spatial-KLA AA | 1.648445 | 1.565402 | 0.022500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 |
| Existence-gated spatial-KLA AA | 1.708370 | 1.534009 | 0.033750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.702915 +/- 0.065821 | [1.621201, 1.784630] | 5 |
| Existence-gated spatial-KLA AA | OSPA | 1.708370 +/- 0.060823 | [1.632860, 1.783879] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.529716 +/- 0.069688 | [1.443201, 1.616231] | 5 |
| Existence-gated spatial-KLA AA | Loc. disag. | 1.534009 +/- 0.074904 | [1.441019, 1.627000] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |
| Existence-gated spatial-KLA AA | Card. disp. | 0.033750 +/- 0.010861 | [0.020266, 0.047234] | 5 |

## Paired Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Existence-gated spatial-KLA AA | OSPA | -0.005454 +/- 0.011213 | [-0.019375, 0.008466] | -0.32% | 2/5 | 1 |
| Existence-gated spatial-KLA AA | Loc. disag. | -0.004293 +/- 0.006152 | [-0.011930, 0.003344] | -0.28% | 2/5 | 1 |
| Existence-gated spatial-KLA AA | Card. disp. | 0.000500 +/- 0.001425 | [-0.001269, 0.002269] | 1.46% | 2/5 | 1 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.032799 | 3.588145 | 0.086250 |
| Existence-gated spatial-KLA AA | 2.036767 | 3.585358 | 0.085750 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.032799 +/- 0.040742 | [1.982219, 2.083379] | 5 |
| Existence-gated spatial-KLA AA | E-OSPA | 2.036767 +/- 0.036029 | [1.992038, 2.081495] | 5 |
| Tuned spatial-KLA AA | RMSE | 3.588145 +/- 0.379057 | [3.117559, 4.058731] | 5 |
| Existence-gated spatial-KLA AA | RMSE | 3.585358 +/- 0.379338 | [3.114423, 4.056292] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |
| Existence-gated spatial-KLA AA | CardErr | 0.085750 +/- 0.011812 | [0.071085, 0.100415] | 5 |

## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Existence-gated spatial-KLA AA | E-OSPA | -0.003968 +/- 0.009804 | [-0.016139, 0.008203] | -0.20% | 2/5 | 1 |
| Existence-gated spatial-KLA AA | RMSE | 0.002787 +/- 0.019552 | [-0.021486, 0.027061] | 0.08% | 2/5 | 1 |
| Existence-gated spatial-KLA AA | CardErr | 0.000500 +/- 0.001425 | [-0.001269, 0.002269] | 0.58% | 2/5 | 1 |

## Adaptive Fusion Weight Diagnostics
Aggregate over local filters, time steps, and targets. Entropy is normalized to [0, 1]; Neff is 1/sum(w^2); branch L1 is 0.5*||w_spatial-w_existence||_1.

| Arm | Spatial H | Exist H | Spatial Neff | Exist Neff | Branch L1 | Target Spatial H | Target Spatial Neff | Target Branch L1 | Target local-r spread | Target records |
|:----|----------:|--------:|-------------:|-----------:|----------:|-----------------:|--------------------:|-----------------:|----------------------:|---------------:|
| Tuned spatial-KLA AA | 0.9917 | 0.9964 | 3.934 | 3.983 | 0.0464 | 0.9919 | 3.937 | 0.0463 | 0.1331 | 50291 |
| Existence-gated spatial-KLA AA | 0.9917 | 0.9964 | 3.934 | 3.983 | 0.0464 | 0.7684 | 2.874 | 0.2677 | 0.1331 | 50276 |

| Trial | Arm | Target Spatial Neff | Target Branch L1 | Target local-r spread | Target records |
|------:|:----|--------------------:|-----------------:|----------------------:|---------------:|
| 1 | Tuned spatial-KLA AA | 3.952 | 0.0463 | 0.1359 | 10072 |
| 1 | Existence-gated spatial-KLA AA | 2.803 | 0.2843 | 0.1359 | 10062 |
| 2 | Tuned spatial-KLA AA | 3.932 | 0.0474 | 0.1388 | 10033 |
| 2 | Existence-gated spatial-KLA AA | 2.888 | 0.2659 | 0.1387 | 10032 |
| 3 | Tuned spatial-KLA AA | 3.878 | 0.0458 | 0.1338 | 10063 |
| 3 | Existence-gated spatial-KLA AA | 2.829 | 0.2668 | 0.1338 | 10063 |
| 4 | Tuned spatial-KLA AA | 3.877 | 0.0458 | 0.1278 | 10058 |
| 4 | Existence-gated spatial-KLA AA | 2.889 | 0.2558 | 0.1279 | 10059 |
| 5 | Tuned spatial-KLA AA | 4.044 | 0.0464 | 0.1293 | 10065 |
| 5 | Existence-gated spatial-KLA AA | 2.960 | 0.2654 | 0.1290 | 10060 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 59.548552 +/- 0.920577 | 2.481190 | 1.000x | 5 |
| Existence-gated spatial-KLA AA | 61.324247 +/- 0.988658 | 2.555177 | 1.030x | 5 |
