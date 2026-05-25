# GA Tiered Link Ablation (2026-05-20 00:12:52)

Comparison order: PD-weighted GA -> FI-weighted GA

## Run Config
- Trials: 20
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: fiWeightedGa

## Arm Configs
### PD-weighted GA
- enabled: 1
- method: pdWeightedGa
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useFreshness: 0
- useCtFiDecay: 0
- pdWeightPower: 1.000
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

### FI-weighted GA
- enabled: 1
- method: fiTraceGa
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
- useFreshness: 0
- useCtFiDecay: 0
- fiTraceUseExistenceProbability: 0
- fiTraceExistencePower: 1.000
- fiTraceUseDetectionProbability: 0
- fiTraceUseClutterPenalty: 0
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
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]
- Trial 6: [0 0.2 0.1 0.5 0.1 0.5 0.1 0.1]
- Trial 7: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 8: [0.5 0.1 0.2 0.1 0.5 0.1 0.1 0]
- Trial 9: [0.5 0.1 0 0.1 0.1 0.5 0.2 0.1]
- Trial 10: [0.1 0.5 0.1 0 0.5 0.1 0.1 0.2]
- Trial 11: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 12: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 13: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 14: [0.1 0.5 0 0.2 0.5 0.1 0.1 0.1]
- Trial 15: [0 0.1 0.5 0.1 0.1 0.2 0.1 0.5]
- Trial 16: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 17: [0 0.1 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 18: [0.2 0 0.5 0.5 0.1 0.1 0.1 0.1]
- Trial 19: [0.2 0.1 0.1 0.1 0 0.5 0.5 0.1]
- Trial 20: [0.1 0.5 0.1 0 0.1 0.5 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | PD-weighted GA | 2.390301 | 1.842123 | 0.856250 |
| 1 | 2 | FI-weighted GA | 2.178831 | 1.763077 | 0.663750 |
| 2 | 3 | PD-weighted GA | 2.398230 | 2.372404 | 0.733750 |
| 2 | 3 | FI-weighted GA | 2.173790 | 1.992269 | 0.561250 |
| 3 | 4 | PD-weighted GA | 2.241155 | 1.772141 | 0.708750 |
| 3 | 4 | FI-weighted GA | 2.015060 | 1.615678 | 0.457500 |
| 4 | 5 | PD-weighted GA | 2.075510 | 2.427941 | 0.452500 |
| 4 | 5 | FI-weighted GA | 1.942241 | 2.533248 | 0.315000 |
| 5 | 6 | PD-weighted GA | 2.446175 | 2.081688 | 1.105000 |
| 5 | 6 | FI-weighted GA | 2.214216 | 2.203175 | 0.788750 |
| 6 | 7 | PD-weighted GA | 2.236962 | 1.829525 | 0.630000 |
| 6 | 7 | FI-weighted GA | 2.064514 | 1.902846 | 0.456250 |
| 7 | 8 | PD-weighted GA | 2.043848 | 1.716376 | 0.463750 |
| 7 | 8 | FI-weighted GA | 1.941863 | 1.595713 | 0.356250 |
| 8 | 9 | PD-weighted GA | 2.000498 | 2.058397 | 0.417500 |
| 8 | 9 | FI-weighted GA | 1.901110 | 2.026119 | 0.308750 |
| 9 | 10 | PD-weighted GA | 2.388965 | 2.058954 | 0.766250 |
| 9 | 10 | FI-weighted GA | 2.167493 | 2.283635 | 0.562500 |
| 10 | 11 | PD-weighted GA | 2.000776 | 1.888090 | 0.438750 |
| 10 | 11 | FI-weighted GA | 1.896970 | 1.864048 | 0.356250 |
| 11 | 12 | PD-weighted GA | 2.134934 | 1.948271 | 0.587500 |
| 11 | 12 | FI-weighted GA | 1.999331 | 1.717609 | 0.436250 |
| 12 | 13 | PD-weighted GA | 1.998051 | 1.625551 | 0.453750 |
| 12 | 13 | FI-weighted GA | 1.920222 | 1.523957 | 0.342500 |
| 13 | 14 | PD-weighted GA | 2.198798 | 2.049692 | 0.537500 |
| 13 | 14 | FI-weighted GA | 2.003407 | 2.000235 | 0.340000 |
| 14 | 15 | PD-weighted GA | 2.020859 | 1.665916 | 0.396250 |
| 14 | 15 | FI-weighted GA | 1.917041 | 1.617112 | 0.292500 |
| 15 | 16 | PD-weighted GA | 1.885429 | 1.853179 | 0.338750 |
| 15 | 16 | FI-weighted GA | 1.871666 | 1.849637 | 0.280000 |
| 16 | 17 | PD-weighted GA | 2.193007 | 1.782459 | 0.540000 |
| 16 | 17 | FI-weighted GA | 2.095652 | 1.964787 | 0.421250 |
| 17 | 18 | PD-weighted GA | 2.304562 | 1.892512 | 0.686250 |
| 17 | 18 | FI-weighted GA | 2.009382 | 1.808203 | 0.468750 |
| 18 | 19 | PD-weighted GA | 2.032608 | 2.010096 | 0.536250 |
| 18 | 19 | FI-weighted GA | 1.927739 | 1.946454 | 0.403750 |
| 19 | 20 | PD-weighted GA | 2.231632 | 1.884299 | 0.687500 |
| 19 | 20 | FI-weighted GA | 2.111044 | 1.767568 | 0.497500 |
| 20 | 21 | PD-weighted GA | 2.261046 | 2.156002 | 0.597500 |
| 20 | 21 | FI-weighted GA | 2.061814 | 1.940196 | 0.426250 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| PD-weighted GA | 2.174167 | 1.945781 | 0.596688 |
| FI-weighted GA | 2.020669 | 1.895778 | 0.436750 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| PD-weighted GA | OSPA | 2.174167 +/- 0.162335 | [2.098193, 2.250142] | 20 |
| FI-weighted GA | OSPA | 2.020669 +/- 0.107464 | [1.970375, 2.070963] | 20 |
| PD-weighted GA | RMSE | 1.945781 +/- 0.211596 | [1.846752, 2.044810] | 20 |
| FI-weighted GA | RMSE | 1.895778 +/- 0.246966 | [1.780196, 2.011361] | 20 |
| PD-weighted GA | Cardinality | 0.596688 +/- 0.183006 | [0.511039, 0.682336] | 20 |
| FI-weighted GA | Cardinality | 0.436750 +/- 0.130081 | [0.375871, 0.497629] | 20 |

## Paired Improvements Relative to PD-weighted GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| FI-weighted GA | OSPA | 0.153498 +/- 0.069517 | [0.120963, 0.186033] | 7.06% | 20/20 | 1.907e-06 |
| FI-weighted GA | RMSE | 0.050002 +/- 0.144382 | [-0.017570, 0.117575] | 2.57% | 15/20 | 0.04139 |
| FI-weighted GA | Cardinality | 0.159938 +/- 0.061566 | [0.131124, 0.188751] | 26.80% | 20/20 | 1.907e-06 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to PD-weighted GA | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| PD-weighted GA | 65.347590 +/- 6.613846 | 0.653476 | 1.000x | 20 |
| FI-weighted GA | 66.028109 +/- 8.124129 | 0.660281 | 1.011x | 20 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| FI-weighted GA | 0.680519 +/- 4.585829 | 1.06% | 13/20 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to PD-weighted GA |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | PD-weighted GA | 59.587375 | 0.595874 | 1.000x |
| 1 | 2 | FI-weighted GA | 61.372797 | 0.613728 | 1.030x |
| 2 | 3 | PD-weighted GA | 60.123635 | 0.601236 | 1.000x |
| 2 | 3 | FI-weighted GA | 61.175563 | 0.611756 | 1.017x |
| 3 | 4 | PD-weighted GA | 62.081942 | 0.620819 | 1.000x |
| 3 | 4 | FI-weighted GA | 60.884258 | 0.608843 | 0.981x |
| 4 | 5 | PD-weighted GA | 61.163810 | 0.611638 | 1.000x |
| 4 | 5 | FI-weighted GA | 60.805162 | 0.608052 | 0.994x |
| 5 | 6 | PD-weighted GA | 65.475095 | 0.654751 | 1.000x |
| 5 | 6 | FI-weighted GA | 66.892772 | 0.668928 | 1.022x |
| 6 | 7 | PD-weighted GA | 62.931691 | 0.629317 | 1.000x |
| 6 | 7 | FI-weighted GA | 67.842758 | 0.678428 | 1.078x |
| 7 | 8 | PD-weighted GA | 75.710164 | 0.757102 | 1.000x |
| 7 | 8 | FI-weighted GA | 71.983898 | 0.719839 | 0.951x |
| 8 | 9 | PD-weighted GA | 63.871312 | 0.638713 | 1.000x |
| 8 | 9 | FI-weighted GA | 62.586038 | 0.625860 | 0.980x |
| 9 | 10 | PD-weighted GA | 63.127182 | 0.631272 | 1.000x |
| 9 | 10 | FI-weighted GA | 65.004847 | 0.650048 | 1.030x |
| 10 | 11 | PD-weighted GA | 63.494365 | 0.634944 | 1.000x |
| 10 | 11 | FI-weighted GA | 64.426191 | 0.644262 | 1.015x |
| 11 | 12 | PD-weighted GA | 84.889822 | 0.848898 | 1.000x |
| 11 | 12 | FI-weighted GA | 98.573290 | 0.985733 | 1.161x |
| 12 | 13 | PD-weighted GA | 79.104488 | 0.791045 | 1.000x |
| 12 | 13 | FI-weighted GA | 66.618111 | 0.666181 | 0.842x |
| 13 | 14 | PD-weighted GA | 65.431236 | 0.654312 | 1.000x |
| 13 | 14 | FI-weighted GA | 65.083108 | 0.650831 | 0.995x |
| 14 | 15 | PD-weighted GA | 62.530239 | 0.625302 | 1.000x |
| 14 | 15 | FI-weighted GA | 64.621498 | 0.646215 | 1.033x |
| 15 | 16 | PD-weighted GA | 62.316309 | 0.623163 | 1.000x |
| 15 | 16 | FI-weighted GA | 63.550205 | 0.635502 | 1.020x |
| 16 | 17 | PD-weighted GA | 63.129107 | 0.631291 | 1.000x |
| 16 | 17 | FI-weighted GA | 63.244035 | 0.632440 | 1.002x |
| 17 | 18 | PD-weighted GA | 63.523298 | 0.635233 | 1.000x |
| 17 | 18 | FI-weighted GA | 65.413361 | 0.654134 | 1.030x |
| 18 | 19 | PD-weighted GA | 61.767010 | 0.617670 | 1.000x |
| 18 | 19 | FI-weighted GA | 61.853258 | 0.618533 | 1.001x |
| 19 | 20 | PD-weighted GA | 62.528970 | 0.625290 | 1.000x |
| 19 | 20 | FI-weighted GA | 64.565008 | 0.645650 | 1.033x |
| 20 | 21 | PD-weighted GA | 64.164753 | 0.641648 | 1.000x |
| 20 | 21 | FI-weighted GA | 64.066028 | 0.640660 | 0.998x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| PD-weighted GA | 2.724237 | 1.552784 | 1.255062 |
| FI-weighted GA | 2.480159 | 1.538486 | 0.979625 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| PD-weighted GA | E-OSPA | 2.724237 +/- 0.119604 | [2.668262, 2.780213] | 20 |
| FI-weighted GA | E-OSPA | 2.480159 +/- 0.114552 | [2.426548, 2.533771] | 20 |
| PD-weighted GA | RMSE | 1.552784 +/- 0.065251 | [1.522247, 1.583322] | 20 |
| FI-weighted GA | RMSE | 1.538486 +/- 0.057275 | [1.511681, 1.565291] | 20 |
| PD-weighted GA | CardErr | 1.255062 +/- 0.216179 | [1.153889, 1.356236] | 20 |
| FI-weighted GA | CardErr | 0.979625 +/- 0.176297 | [0.897116, 1.062134] | 20 |

## Paired Local-Metric Improvements Relative to PD-weighted GA
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| FI-weighted GA | E-OSPA | 0.244078 +/- 0.061476 | [0.215307, 0.272849] | 8.96% | 20/20 | 1.907e-06 |
| FI-weighted GA | RMSE | 0.014299 +/- 0.029127 | [0.000667, 0.027930] | 0.92% | 14/20 | 0.1153 |
| FI-weighted GA | CardErr | 0.275437 +/- 0.066657 | [0.244241, 0.306634] | 21.95% | 20/20 | 1.907e-06 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | PD-weighted GA | 2.458797 | 1.493420 | 0.755500 |
| 1 | FI-weighted GA | 2.322265 | 1.496265 | 0.658500 |
| 2 | PD-weighted GA | 2.547001 | 1.494479 | 0.857500 |
| 2 | FI-weighted GA | 2.356487 | 1.494289 | 0.715000 |
| 3 | PD-weighted GA | 2.534625 | 1.483640 | 0.840000 |
| 3 | FI-weighted GA | 2.333007 | 1.482697 | 0.706500 |
| 4 | PD-weighted GA | 2.728167 | 1.533072 | 1.324500 |
| 4 | FI-weighted GA | 2.492424 | 1.541633 | 1.037000 |
| 5 | PD-weighted GA | 2.921659 | 1.593906 | 1.675000 |
| 5 | FI-weighted GA | 2.641883 | 1.570669 | 1.309000 |
| 6 | PD-weighted GA | 2.810291 | 1.610238 | 1.409500 |
| 6 | FI-weighted GA | 2.507367 | 1.557416 | 1.029500 |
| 7 | PD-weighted GA | 2.877087 | 1.589842 | 1.539500 |
| 7 | FI-weighted GA | 2.594688 | 1.579725 | 1.171500 |
| 8 | PD-weighted GA | 2.916271 | 1.623678 | 1.639000 |
| 8 | FI-weighted GA | 2.593151 | 1.585193 | 1.210000 |
