# AA Balanced/Cardinality Validation

Generated at: 2026-06-22 10:41:08

Comparison order: Tuned spatial-KLA AA

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
| 2 | 13 | Tuned spatial-KLA AA | 1.809269 | 1.614177 | 0.051250 |
| 3 | 14 | Tuned spatial-KLA AA | 1.671022 | 1.560164 | 0.022500 |
| 4 | 15 | Tuned spatial-KLA AA | 1.720003 | 1.450386 | 0.038750 |
| 5 | 16 | Tuned spatial-KLA AA | 1.641038 | 1.559038 | 0.022500 |

## Network Disagreement Metrics
| Arm | OSPA | Loc. disag. | Card. disp. |
|:----|-----:|------------:|------------:|
| Tuned spatial-KLA AA | 1.702915 | 1.529716 | 0.034250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | OSPA | 1.702915 +/- 0.065821 | [1.621201, 1.784630] | 5 |
| Tuned spatial-KLA AA | Loc. disag. | 1.529716 +/- 0.069688 | [1.443201, 1.616231] | 5 |
| Tuned spatial-KLA AA | Card. disp. | 0.034250 +/- 0.012139 | [0.019180, 0.049320] | 5 |

## Local Tracking Metrics
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| Tuned spatial-KLA AA | 2.032799 | 3.588145 | 0.086250 |

| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| Tuned spatial-KLA AA | E-OSPA | 2.032799 +/- 0.040742 | [1.982219, 2.083379] | 5 |
| Tuned spatial-KLA AA | RMSE | 3.588145 +/- 0.379057 | [3.117559, 4.058731] | 5 |
| Tuned spatial-KLA AA | CardErr | 0.086250 +/- 0.012406 | [0.070849, 0.101651] | 5 |

## Adaptive Fusion Weight Diagnostics
Aggregate over local filters, time steps, and targets. Entropy is normalized to [0, 1]; Neff is 1/sum(w^2); branch L1 is 0.5*||w_spatial-w_existence||_1.

| Arm | Spatial H | Exist H | Spatial Neff | Exist Neff | Branch L1 | Target Spatial H | Target Spatial Neff | Target Branch L1 | Target local-r spread | Target records |
|:----|----------:|--------:|-------------:|-----------:|----------:|-----------------:|--------------------:|-----------------:|----------------------:|---------------:|
| Tuned spatial-KLA AA | 0.9917 | 0.9964 | 3.934 | 3.983 | 0.0464 | 0.9919 | 3.937 | 0.0463 | 0.1331 | 50291 |

| Trial | Arm | Target Spatial Neff | Target Branch L1 | Target local-r spread | Target records |
|------:|:----|--------------------:|-----------------:|----------------------:|---------------:|
| 1 | Tuned spatial-KLA AA | 3.952 | 0.0463 | 0.1359 | 10072 |
| 2 | Tuned spatial-KLA AA | 3.932 | 0.0474 | 0.1388 | 10033 |
| 3 | Tuned spatial-KLA AA | 3.878 | 0.0458 | 0.1338 | 10063 |
| 4 | Tuned spatial-KLA AA | 3.877 | 0.0458 | 0.1278 | 10058 |
| 5 | Tuned spatial-KLA AA | 4.044 | 0.0464 | 0.1293 | 10065 |

## Consensus Loc Failure Attribution
Top rows are the largest per-time Loc disagreement values in each trial/arm. Weight columns summarize the same time step across local filters and targets when adaptive diagnostics are enabled.

| Trial | Arm | Rank | t | Truth card | Loc | OSPA | Card | Local RMSE | Counts | Worst pair | Worst pair Loc | Mean matched dist | Max matched dist | Target Spatial Neff | Target Branch L1 | Target local-r spread |
|------:|:----|-----:|--:|-----------:|----:|-----:|-----:|-----------:|:-------|:-----------|---------------:|------------------:|-----------------:|--------------------:|-----------------:|----------------------:|
| 1 | Tuned spatial-KLA AA | 1 | 81 | 2 | 5.918022 | 2.977025 | 0.250000 | 5.554761 | [2 2 3 3 2 2 2 2] | 2-7 | 17.098932 | 4.494237 | 23.934204 | 4.935 | 0.0485 | 0.1500 |
| 1 | Tuned spatial-KLA AA | 2 | 82 | 2 | 3.328339 | 2.263406 | 0.000000 | 2.448500 | [2 2 2 2 2 2 2 2] | 2-7 | 9.132080 | 2.653097 | 12.852735 | 4.930 | 0.0460 | 0.1034 |
| 1 | Tuned spatial-KLA AA | 3 | 22 | 3 | 2.675742 | 2.712099 | 0.000000 | 2.499763 | [3 3 3 3 3 3 3 3] | 4-6 | 4.520597 | 2.387813 | 6.721460 | 3.076 | 0.0447 | 0.1888 |
| 1 | Tuned spatial-KLA AA | 4 | 5 | 1 | 2.650556 | 2.810205 | 0.000000 | 2.255144 | [1 1 1 1 1 1 1 1] | 3-6 | 5.342392 | 2.650556 | 5.342392 | 3.701 | 0.0510 | 0.0679 |
| 1 | Tuned spatial-KLA AA | 5 | 6 | 1 | 2.544055 | 2.609100 | 0.000000 | 1.924421 | [1 1 1 1 1 1 1 1] | 3-6 | 5.122262 | 2.544055 | 5.122262 | 3.698 | 0.0507 | 0.0873 |
| 2 | Tuned spatial-KLA AA | 1 | 25 | 3 | 6.084822 | 2.673358 | 0.500000 | 16.951502 | [3 3 3 2 2 2 2 3] | 2-8 | 44.044478 | 4.251152 | 76.219814 | 3.709 | 0.0488 | 0.1489 |
| 2 | Tuned spatial-KLA AA | 2 | 97 | 0 | 4.146111 | 2.817239 | 0.000000 | NaN | [1 1 1 1 1 1 1 1] | 1-6 | 11.457132 | 4.146111 | 11.457132 | 4.918 | 0.0491 | 0.0975 |
| 2 | Tuned spatial-KLA AA | 3 | 98 | 0 | 3.989381 | 3.837880 | 0.500000 | 0.000000 | [1 1 1 1 0 0 0 0] | 1-4 | 13.125414 | 3.989381 | 13.125414 | 3.084 | 0.0454 | 0.0944 |
| 2 | Tuned spatial-KLA AA | 4 | 94 | 1 | 3.595032 | 3.386666 | 0.000000 | 4.779297 | [1 1 1 1 1 1 1 1] | 1-3 | 7.660793 | 3.595032 | 7.660793 | 3.706 | 0.0455 | 0.0903 |
| 2 | Tuned spatial-KLA AA | 5 | 96 | 1 | 3.538094 | 2.524078 | 0.000000 | 3.098360 | [1 1 1 1 1 1 1 1] | 1-2 | 9.740054 | 3.538094 | 9.740054 | 4.307 | 0.0505 | 0.0940 |
| 3 | Tuned spatial-KLA AA | 1 | 49 | 3 | 7.292958 | 2.792119 | 0.375000 | 23.580635 | [3 2 3 3 2 2 3 3] | 7-8 | 47.557553 | 5.209366 | 82.370571 | 3.087 | 0.0394 | 0.1172 |
| 3 | Tuned spatial-KLA AA | 2 | 74 | 3 | 7.061127 | 2.460724 | 0.125000 | 8.137760 | [3 3 3 3 2 3 3 3] | 2-7 | 15.279494 | 4.597925 | 26.418090 | 3.664 | 0.0513 | 0.1991 |
| 3 | Tuned spatial-KLA AA | 3 | 5 | 1 | 2.969134 | 2.949715 | 0.000000 | 2.261959 | [1 1 1 1 1 1 1 1] | 1-8 | 6.283110 | 2.969134 | 6.283110 | 3.089 | 0.0482 | 0.1147 |
| 3 | Tuned spatial-KLA AA | 4 | 97 | 0 | 2.722248 | 2.830188 | 0.000000 | NaN | [1 1 1 1 1 1 1 1] | 3-5 | 5.176277 | 2.722248 | 5.176277 | 4.310 | 0.0489 | 0.0851 |
| 3 | Tuned spatial-KLA AA | 5 | 36 | 3 | 2.441923 | 2.517838 | 0.000000 | 2.081142 | [3 3 3 3 3 3 3 3] | 3-7 | 4.696776 | 2.267145 | 6.309600 | 3.709 | 0.0444 | 0.1613 |
| 4 | Tuned spatial-KLA AA | 1 | 97 | 0 | 2.721577 | 2.725071 | 0.000000 | NaN | [1 1 1 1 1 1 1 1] | 3-6 | 6.415457 | 2.721577 | 6.415457 | 3.705 | 0.0498 | 0.0473 |
| 4 | Tuned spatial-KLA AA | 2 | 94 | 1 | 2.673598 | 2.658904 | 0.000000 | 2.144462 | [1 1 1 1 1 1 1 1] | 3-4 | 6.999981 | 2.673598 | 6.999981 | 3.091 | 0.0581 | 0.0621 |
| 4 | Tuned spatial-KLA AA | 3 | 93 | 1 | 2.595468 | 2.657439 | 0.000000 | 2.127294 | [1 1 1 1 1 1 1 1] | 3-6 | 4.966346 | 2.595468 | 4.966346 | 4.284 | 0.0511 | 0.1123 |
| 4 | Tuned spatial-KLA AA | 4 | 92 | 1 | 2.351258 | 2.431225 | 0.000000 | 1.908890 | [1 1 1 1 1 1 1 1] | 4-5 | 4.945253 | 2.351258 | 4.945253 | 3.696 | 0.0463 | 0.0753 |
| 4 | Tuned spatial-KLA AA | 5 | 45 | 3 | 2.161977 | 2.310759 | 0.000000 | 1.626378 | [3 3 3 3 3 3 3 3] | 2-5 | 4.287016 | 1.907962 | 5.427026 | 3.699 | 0.0450 | 0.1360 |
| 5 | Tuned spatial-KLA AA | 1 | 41 | 3 | 15.060304 | 2.362063 | 0.125000 | 25.606312 | [3 3 3 2 3 3 3 3] | 1-7 | 40.827273 | 9.133846 | 70.706883 | 4.930 | 0.0465 | 0.1078 |
| 5 | Tuned spatial-KLA AA | 2 | 6 | 1 | 2.634927 | 2.714449 | 0.000000 | 2.815941 | [1 1 1 1 1 1 1 1] | 1-5 | 5.414886 | 2.634927 | 5.414886 | 4.324 | 0.0463 | 0.0969 |
| 5 | Tuned spatial-KLA AA | 3 | 8 | 1 | 2.448889 | 2.509044 | 0.000000 | 1.985319 | [1 1 1 1 1 1 1 1] | 4-5 | 4.789132 | 2.448889 | 4.789132 | 3.692 | 0.0489 | 0.1319 |
| 5 | Tuned spatial-KLA AA | 4 | 5 | 1 | 2.304377 | 2.519765 | 0.000000 | 2.279417 | [1 1 1 1 1 1 1 1] | 1-5 | 4.488776 | 2.304377 | 4.488776 | 4.922 | 0.0464 | 0.1173 |
| 5 | Tuned spatial-KLA AA | 5 | 97 | 0 | 2.237230 | 2.277056 | 0.000000 | NaN | [1 1 1 1 1 1 1 1] | 3-4 | 4.913507 | 2.237230 | 4.913507 | 4.309 | 0.0540 | 0.1213 |

## Runtime
| Arm | Filter runtime (s) | Runtime/step (s) | Relative to Tuned spatial-KLA AA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| Tuned spatial-KLA AA | 78.899182 +/- 13.560665 | 3.287466 | 1.000x | 5 |
