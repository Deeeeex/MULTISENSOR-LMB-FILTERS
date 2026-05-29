# GA Tiered Link Ablation (2026-05-28 09:25:45)

Comparison order: fixed weights -> PD-weighted GA -> FI-weighted GA -> +link quality

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: fiWeightedGa

## Arm Configs
### fixed weights
- enabled: 0
- method: factorized
- useCovariance: 0
- useLinkQuality: 0
- useExistenceConfidence: 0
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

### +link quality
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 0
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
- Trial 21: [0.1 0.5 0.1 0.2 0 0.5 0.1 0.1]
- Trial 22: [0.1 0.2 0.5 0.5 0 0.1 0.1 0.1]
- Trial 23: [0.1 0.1 0.1 0 0.1 0.5 0.5 0.2]
- Trial 24: [0.2 0.1 0.5 0.5 0.1 0.1 0 0.1]
- Trial 25: [0.1 0.5 0.5 0.1 0.2 0.1 0.1 0]
- Trial 26: [0.2 0.1 0.5 0.1 0.1 0 0.1 0.5]
- Trial 27: [0.1 0.5 0 0.1 0.2 0.1 0.1 0.5]
- Trial 28: [0.1 0.5 0.1 0 0.2 0.5 0.1 0.1]
- Trial 29: [0.1 0.1 0.1 0.1 0.2 0.5 0.5 0]
- Trial 30: [0.1 0 0.1 0.1 0.1 0.5 0.2 0.5]
- Trial 31: [0.1 0 0.1 0.5 0.2 0.1 0.5 0.1]
- Trial 32: [0.5 0.2 0 0.1 0.1 0.1 0.1 0.5]
- Trial 33: [0.2 0.1 0.1 0.1 0.5 0.5 0 0.1]
- Trial 34: [0.1 0.1 0 0.1 0.1 0.5 0.2 0.5]
- Trial 35: [0.2 0.1 0.1 0 0.1 0.1 0.5 0.5]
- Trial 36: [0.1 0.2 0.5 0.5 0.1 0 0.1 0.1]
- Trial 37: [0.1 0.1 0.1 0.2 0 0.5 0.1 0.5]
- Trial 38: [0.1 0 0.5 0.1 0.5 0.1 0.1 0.2]
- Trial 39: [0.5 0 0.1 0.1 0.5 0.1 0.2 0.1]
- Trial 40: [0.1 0.2 0.5 0 0.5 0.1 0.1 0.1]
- Trial 41: [0.1 0.2 0.1 0.1 0.1 0 0.5 0.5]
- Trial 42: [0.5 0 0.2 0.1 0.1 0.1 0.1 0.5]
- Trial 43: [0.2 0.1 0.5 0.5 0 0.1 0.1 0.1]
- Trial 44: [0 0.5 0.5 0.1 0.1 0.2 0.1 0.1]
- Trial 45: [0.5 0.1 0.1 0.2 0.5 0.1 0.1 0]
- Trial 46: [0.5 0.1 0.1 0.1 0.5 0.1 0 0.2]
- Trial 47: [0.5 0.1 0.1 0.2 0.1 0.5 0 0.1]
- Trial 48: [0 0.5 0.1 0.1 0.5 0.1 0.2 0.1]
- Trial 49: [0.1 0 0.5 0.5 0.1 0.2 0.1 0.1]
- Trial 50: [0.1 0.5 0.5 0.1 0 0.1 0.2 0.1]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 2.605320 | 2.258061 | 0.932500 |
| 1 | 2 | PD-weighted GA | 2.390301 | 1.842123 | 0.856250 |
| 1 | 2 | FI-weighted GA | 2.178831 | 1.763077 | 0.663750 |
| 1 | 2 | +link quality | 1.894149 | 1.454645 | 0.245000 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | PD-weighted GA | 2.398230 | 2.372404 | 0.733750 |
| 2 | 3 | FI-weighted GA | 2.173790 | 1.992269 | 0.561250 |
| 2 | 3 | +link quality | 1.867780 | 1.387814 | 0.195000 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | PD-weighted GA | 2.241155 | 1.772141 | 0.708750 |
| 3 | 4 | FI-weighted GA | 2.015060 | 1.615678 | 0.457500 |
| 3 | 4 | +link quality | 1.825910 | 1.438299 | 0.220000 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | PD-weighted GA | 2.075510 | 2.427941 | 0.452500 |
| 4 | 5 | FI-weighted GA | 1.942241 | 2.533248 | 0.315000 |
| 4 | 5 | +link quality | 1.748949 | 1.913620 | 0.175000 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | PD-weighted GA | 2.446175 | 2.081688 | 1.105000 |
| 5 | 6 | FI-weighted GA | 2.214216 | 2.203175 | 0.788750 |
| 5 | 6 | +link quality | 1.872465 | 1.868450 | 0.246250 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | PD-weighted GA | 2.236962 | 1.829525 | 0.630000 |
| 6 | 7 | FI-weighted GA | 2.064514 | 1.902846 | 0.456250 |
| 6 | 7 | +link quality | 1.861180 | 1.788151 | 0.198750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | PD-weighted GA | 2.043848 | 1.716376 | 0.463750 |
| 7 | 8 | FI-weighted GA | 1.941863 | 1.595713 | 0.356250 |
| 7 | 8 | +link quality | 1.720911 | 1.373876 | 0.168750 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | PD-weighted GA | 2.000498 | 2.058397 | 0.417500 |
| 8 | 9 | FI-weighted GA | 1.901110 | 2.026119 | 0.308750 |
| 8 | 9 | +link quality | 1.755763 | 1.466729 | 0.188750 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | PD-weighted GA | 2.388965 | 2.058954 | 0.766250 |
| 9 | 10 | FI-weighted GA | 2.167493 | 2.283635 | 0.562500 |
| 9 | 10 | +link quality | 1.812550 | 2.095538 | 0.178750 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | PD-weighted GA | 2.000776 | 1.888090 | 0.438750 |
| 10 | 11 | FI-weighted GA | 1.896970 | 1.864048 | 0.356250 |
| 10 | 11 | +link quality | 1.772281 | 1.481958 | 0.176250 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | PD-weighted GA | 2.134934 | 1.948271 | 0.587500 |
| 11 | 12 | FI-weighted GA | 1.999331 | 1.717609 | 0.436250 |
| 11 | 12 | +link quality | 1.760701 | 1.535206 | 0.173750 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | PD-weighted GA | 1.998051 | 1.625551 | 0.453750 |
| 12 | 13 | FI-weighted GA | 1.920222 | 1.523957 | 0.342500 |
| 12 | 13 | +link quality | 1.724244 | 1.390455 | 0.190000 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | PD-weighted GA | 2.198798 | 2.049692 | 0.537500 |
| 13 | 14 | FI-weighted GA | 2.003407 | 2.000235 | 0.340000 |
| 13 | 14 | +link quality | 1.803248 | 1.467818 | 0.183750 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | PD-weighted GA | 2.020859 | 1.665916 | 0.396250 |
| 14 | 15 | FI-weighted GA | 1.917041 | 1.617112 | 0.292500 |
| 14 | 15 | +link quality | 1.777916 | 1.475596 | 0.188750 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | PD-weighted GA | 1.885429 | 1.853179 | 0.338750 |
| 15 | 16 | FI-weighted GA | 1.871666 | 1.849637 | 0.280000 |
| 15 | 16 | +link quality | 1.729686 | 1.524912 | 0.187500 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | PD-weighted GA | 2.193007 | 1.782459 | 0.540000 |
| 16 | 17 | FI-weighted GA | 2.095652 | 1.964787 | 0.421250 |
| 16 | 17 | +link quality | 1.816784 | 1.450195 | 0.223750 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | PD-weighted GA | 2.304562 | 1.892512 | 0.686250 |
| 17 | 18 | FI-weighted GA | 2.009382 | 1.808203 | 0.468750 |
| 17 | 18 | +link quality | 1.871102 | 1.664706 | 0.213750 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | PD-weighted GA | 2.032608 | 2.010096 | 0.536250 |
| 18 | 19 | FI-weighted GA | 1.927739 | 1.946454 | 0.403750 |
| 18 | 19 | +link quality | 1.690478 | 1.467031 | 0.163750 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | PD-weighted GA | 2.231632 | 1.884299 | 0.687500 |
| 19 | 20 | FI-weighted GA | 2.111044 | 1.767568 | 0.497500 |
| 19 | 20 | +link quality | 1.825640 | 1.482957 | 0.200000 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | PD-weighted GA | 2.261046 | 2.156002 | 0.597500 |
| 20 | 21 | FI-weighted GA | 2.061814 | 1.940196 | 0.426250 |
| 20 | 21 | +link quality | 1.770289 | 1.489944 | 0.176250 |
| 21 | 22 | fixed weights | 2.445386 | 2.063249 | 0.706250 |
| 21 | 22 | PD-weighted GA | 2.066214 | 1.780853 | 0.495000 |
| 21 | 22 | FI-weighted GA | 1.905285 | 1.616624 | 0.316250 |
| 21 | 22 | +link quality | 1.789513 | 1.553570 | 0.180000 |
| 22 | 23 | fixed weights | 2.782054 | 2.774744 | 0.917500 |
| 22 | 23 | PD-weighted GA | 2.429467 | 2.274308 | 0.661250 |
| 22 | 23 | FI-weighted GA | 2.143802 | 1.934725 | 0.456250 |
| 22 | 23 | +link quality | 1.856143 | 1.440447 | 0.211250 |
| 23 | 24 | fixed weights | 2.811135 | 1.942125 | 1.156250 |
| 23 | 24 | PD-weighted GA | 2.400164 | 1.974802 | 0.925000 |
| 23 | 24 | FI-weighted GA | 2.275572 | 2.235598 | 0.655000 |
| 23 | 24 | +link quality | 1.810456 | 1.501741 | 0.176250 |
| 24 | 25 | fixed weights | 2.569606 | 2.368173 | 0.711250 |
| 24 | 25 | PD-weighted GA | 2.215882 | 2.045332 | 0.533750 |
| 24 | 25 | FI-weighted GA | 2.065766 | 1.952977 | 0.421250 |
| 24 | 25 | +link quality | 1.879064 | 1.513682 | 0.220000 |
| 25 | 26 | fixed weights | 2.225426 | 2.278298 | 0.403750 |
| 25 | 26 | PD-weighted GA | 2.004920 | 1.843632 | 0.438750 |
| 25 | 26 | FI-weighted GA | 2.003834 | 1.783240 | 0.397500 |
| 25 | 26 | +link quality | 1.733723 | 1.395662 | 0.183750 |
| 26 | 27 | fixed weights | 2.159231 | 1.899203 | 0.517500 |
| 26 | 27 | PD-weighted GA | 2.102619 | 1.885960 | 0.517500 |
| 26 | 27 | FI-weighted GA | 1.875978 | 1.442565 | 0.370000 |
| 26 | 27 | +link quality | 1.661765 | 1.296388 | 0.148750 |
| 27 | 28 | fixed weights | 2.126003 | 1.944541 | 0.406250 |
| 27 | 28 | PD-weighted GA | 1.870453 | 1.656489 | 0.316250 |
| 27 | 28 | FI-weighted GA | 1.893724 | 1.599678 | 0.301250 |
| 27 | 28 | +link quality | 1.735363 | 1.381318 | 0.208750 |
| 28 | 29 | fixed weights | 2.917753 | 1.994773 | 1.183750 |
| 28 | 29 | PD-weighted GA | 2.405282 | 1.912411 | 0.911250 |
| 28 | 29 | FI-weighted GA | 2.300030 | 1.914871 | 0.736250 |
| 28 | 29 | +link quality | 1.886254 | 1.670214 | 0.270000 |
| 29 | 30 | fixed weights | 2.600417 | 1.944603 | 1.000000 |
| 29 | 30 | PD-weighted GA | 2.248775 | 1.896394 | 0.805000 |
| 29 | 30 | FI-weighted GA | 2.281045 | 2.239092 | 0.660000 |
| 29 | 30 | +link quality | 1.877042 | 1.641901 | 0.227500 |
| 30 | 31 | fixed weights | 2.468399 | 2.109218 | 0.737500 |
| 30 | 31 | PD-weighted GA | 2.183131 | 1.927515 | 0.615000 |
| 30 | 31 | FI-weighted GA | 1.974396 | 1.894169 | 0.407500 |
| 30 | 31 | +link quality | 1.784876 | 1.482583 | 0.162500 |
| 31 | 32 | fixed weights | 2.513279 | 1.885047 | 0.721250 |
| 31 | 32 | PD-weighted GA | 2.215388 | 1.807747 | 0.515000 |
| 31 | 32 | FI-weighted GA | 1.896271 | 1.717728 | 0.327500 |
| 31 | 32 | +link quality | 1.756308 | 1.469851 | 0.170000 |
| 32 | 33 | fixed weights | 2.019727 | 1.812822 | 0.302500 |
| 32 | 33 | PD-weighted GA | 1.808896 | 1.551333 | 0.265000 |
| 32 | 33 | FI-weighted GA | 1.758932 | 1.703870 | 0.205000 |
| 32 | 33 | +link quality | 1.698242 | 1.420990 | 0.162500 |
| 33 | 34 | fixed weights | 2.490122 | 2.173185 | 0.962500 |
| 33 | 34 | PD-weighted GA | 2.206139 | 2.316573 | 0.786250 |
| 33 | 34 | FI-weighted GA | 2.054923 | 2.063315 | 0.577500 |
| 33 | 34 | +link quality | 1.693889 | 1.581188 | 0.187500 |
| 34 | 35 | fixed weights | 2.915586 | 2.674676 | 1.071250 |
| 34 | 35 | PD-weighted GA | 2.524605 | 2.056169 | 0.880000 |
| 34 | 35 | FI-weighted GA | 2.193152 | 1.829801 | 0.628750 |
| 34 | 35 | +link quality | 1.870021 | 1.490107 | 0.186250 |
| 35 | 36 | fixed weights | 2.381312 | 2.282297 | 0.561250 |
| 35 | 36 | PD-weighted GA | 2.105833 | 1.994328 | 0.505000 |
| 35 | 36 | FI-weighted GA | 1.944990 | 1.659743 | 0.362500 |
| 35 | 36 | +link quality | 1.698205 | 1.454358 | 0.137500 |
| 36 | 37 | fixed weights | 2.794401 | 3.053361 | 0.971250 |
| 36 | 37 | PD-weighted GA | 2.393963 | 2.665813 | 0.671250 |
| 36 | 37 | FI-weighted GA | 2.190804 | 2.375313 | 0.535000 |
| 36 | 37 | +link quality | 1.858989 | 1.497741 | 0.216250 |
| 37 | 38 | fixed weights | 2.488304 | 2.240250 | 0.821250 |
| 37 | 38 | PD-weighted GA | 2.141029 | 2.030257 | 0.586250 |
| 37 | 38 | FI-weighted GA | 2.057257 | 1.772752 | 0.512500 |
| 37 | 38 | +link quality | 1.717295 | 1.543967 | 0.147500 |
| 38 | 39 | fixed weights | 2.292857 | 2.023920 | 0.538750 |
| 38 | 39 | PD-weighted GA | 2.178682 | 1.986546 | 0.518750 |
| 38 | 39 | FI-weighted GA | 1.979323 | 1.836693 | 0.372500 |
| 38 | 39 | +link quality | 1.823606 | 1.486187 | 0.177500 |
| 39 | 40 | fixed weights | 2.535165 | 2.535969 | 0.646250 |
| 39 | 40 | PD-weighted GA | 2.091245 | 1.768588 | 0.490000 |
| 39 | 40 | FI-weighted GA | 2.009122 | 1.761998 | 0.446250 |
| 39 | 40 | +link quality | 1.815326 | 1.744380 | 0.205000 |
| 40 | 41 | fixed weights | 2.257174 | 2.481788 | 0.530000 |
| 40 | 41 | PD-weighted GA | 2.094461 | 2.234327 | 0.536250 |
| 40 | 41 | FI-weighted GA | 2.042098 | 1.873312 | 0.472500 |
| 40 | 41 | +link quality | 1.789400 | 1.580266 | 0.206250 |
| 41 | 42 | fixed weights | 2.094739 | 2.021235 | 0.453750 |
| 41 | 42 | PD-weighted GA | 1.940991 | 1.879144 | 0.393750 |
| 41 | 42 | FI-weighted GA | 1.762751 | 1.776705 | 0.268750 |
| 41 | 42 | +link quality | 1.591801 | 1.288719 | 0.145000 |
| 42 | 43 | fixed weights | 2.470491 | 2.757701 | 0.583750 |
| 42 | 43 | PD-weighted GA | 2.259994 | 2.142787 | 0.570000 |
| 42 | 43 | FI-weighted GA | 2.000583 | 1.753832 | 0.395000 |
| 42 | 43 | +link quality | 1.724105 | 1.444278 | 0.095000 |
| 43 | 44 | fixed weights | 2.550641 | 2.146021 | 0.801250 |
| 43 | 44 | PD-weighted GA | 2.173870 | 2.079515 | 0.505000 |
| 43 | 44 | FI-weighted GA | 2.117925 | 1.928204 | 0.416250 |
| 43 | 44 | +link quality | 1.812060 | 1.405628 | 0.182500 |
| 44 | 45 | fixed weights | 2.216546 | 2.738044 | 0.428750 |
| 44 | 45 | PD-weighted GA | 2.055270 | 2.353187 | 0.437500 |
| 44 | 45 | FI-weighted GA | 1.952491 | 2.083624 | 0.337500 |
| 44 | 45 | +link quality | 1.721599 | 1.442912 | 0.158750 |
| 45 | 46 | fixed weights | 2.537339 | 2.539290 | 0.686250 |
| 45 | 46 | PD-weighted GA | 2.097076 | 2.150126 | 0.498750 |
| 45 | 46 | FI-weighted GA | 1.956251 | 1.707893 | 0.340000 |
| 45 | 46 | +link quality | 1.746446 | 1.436876 | 0.172500 |
| 46 | 47 | fixed weights | 2.665038 | 2.010381 | 0.693750 |
| 46 | 47 | PD-weighted GA | 2.173818 | 1.902870 | 0.442500 |
| 46 | 47 | FI-weighted GA | 1.958433 | 1.642957 | 0.342500 |
| 46 | 47 | +link quality | 1.850503 | 1.729966 | 0.158750 |
| 47 | 48 | fixed weights | 2.238065 | 2.249780 | 0.577500 |
| 47 | 48 | PD-weighted GA | 2.167693 | 1.957190 | 0.627500 |
| 47 | 48 | FI-weighted GA | 2.091681 | 1.763918 | 0.496250 |
| 47 | 48 | +link quality | 1.759599 | 1.378705 | 0.163750 |
| 48 | 49 | fixed weights | 2.529967 | 3.164643 | 0.665000 |
| 48 | 49 | PD-weighted GA | 2.259511 | 2.020426 | 0.676250 |
| 48 | 49 | FI-weighted GA | 2.258435 | 1.923475 | 0.576250 |
| 48 | 49 | +link quality | 1.901177 | 1.678296 | 0.230000 |
| 49 | 50 | fixed weights | 2.564502 | 2.102492 | 0.951250 |
| 49 | 50 | PD-weighted GA | 2.176858 | 2.159523 | 0.583750 |
| 49 | 50 | FI-weighted GA | 2.020634 | 1.762611 | 0.431250 |
| 49 | 50 | +link quality | 1.740913 | 1.432767 | 0.188750 |
| 50 | 51 | fixed weights | 2.714437 | 3.379717 | 0.801250 |
| 50 | 51 | PD-weighted GA | 2.391279 | 2.561643 | 0.760000 |
| 50 | 51 | FI-weighted GA | 2.099736 | 2.022748 | 0.542500 |
| 50 | 51 | +link quality | 1.916190 | 1.647148 | 0.233750 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.468973 | 2.326034 | 0.716025 |
| PD-weighted GA | 2.177337 | 1.994628 | 0.588025 |
| FI-weighted GA | 2.029572 | 1.869792 | 0.440850 |
| +link quality | 1.788038 | 1.524995 | 0.188150 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.468973 +/- 0.219981 | [2.407997, 2.529949] | 50 |
| PD-weighted GA | OSPA | 2.177337 +/- 0.161465 | [2.132581, 2.222093] | 50 |
| FI-weighted GA | OSPA | 2.029572 +/- 0.128282 | [1.994014, 2.065130] | 50 |
| +link quality | OSPA | 1.788038 +/- 0.071415 | [1.768243, 1.807833] | 50 |
| fixed weights | RMSE | 2.326034 +/- 0.353065 | [2.228170, 2.423899] | 50 |
| PD-weighted GA | RMSE | 1.994628 +/- 0.232503 | [1.930182, 2.059075] | 50 |
| FI-weighted GA | RMSE | 1.869792 +/- 0.220981 | [1.808539, 1.931045] | 50 |
| +link quality | RMSE | 1.524995 +/- 0.155955 | [1.481766, 1.568223] | 50 |
| fixed weights | Cardinality | 0.716025 +/- 0.223795 | [0.653992, 0.778058] | 50 |
| PD-weighted GA | Cardinality | 0.588025 +/- 0.170389 | [0.540796, 0.635254] | 50 |
| FI-weighted GA | Cardinality | 0.440850 +/- 0.126942 | [0.405663, 0.476037] | 50 |
| +link quality | Cardinality | 0.188150 +/- 0.031280 | [0.179480, 0.196820] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | OSPA | 0.291636 +/- 0.114683 | [0.259847, 0.323424] | 11.81% | 50/50 | 1.776e-15 |
| FI-weighted GA | OSPA | 0.439401 +/- 0.146377 | [0.398827, 0.479974] | 17.80% | 50/50 | 1.776e-15 |
| +link quality | OSPA | 0.680935 +/- 0.177344 | [0.631778, 0.730092] | 27.58% | 50/50 | 1.776e-15 |
| PD-weighted GA | RMSE | 0.331406 +/- 0.248797 | [0.262443, 0.400369] | 14.25% | 47/50 | 3.708e-11 |
| FI-weighted GA | RMSE | 0.456242 +/- 0.318522 | [0.367952, 0.544532] | 19.61% | 48/50 | 2.267e-12 |
| +link quality | RMSE | 0.801039 +/- 0.338775 | [0.707136, 0.894943] | 34.44% | 50/50 | 1.776e-15 |
| PD-weighted GA | Cardinality | 0.128000 +/- 0.110710 | [0.097313, 0.158687] | 17.88% | 42/50 | 3.625e-07 |
| FI-weighted GA | Cardinality | 0.275175 +/- 0.139278 | [0.236569, 0.313781] | 38.43% | 50/50 | 1.776e-15 |
| +link quality | Cardinality | 0.527875 +/- 0.209468 | [0.469813, 0.585937] | 73.72% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 50.180396 +/- 3.938005 | 0.501804 | 1.000x | 50 |
| PD-weighted GA | 61.668471 +/- 5.376481 | 0.616685 | 1.233x | 50 |
| FI-weighted GA | 62.810458 +/- 6.767913 | 0.628105 | 1.254x | 50 |
| +link quality | 53.094714 +/- 4.308752 | 0.530947 | 1.062x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| PD-weighted GA | 11.488075 +/- 5.869895 | 23.25% | 49/50 |
| FI-weighted GA | 12.630061 +/- 6.524155 | 25.41% | 49/50 |
| +link quality | 2.914317 +/- 5.707930 | 6.24% | 44/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 50.064438 | 0.500644 | 1.000x |
| 1 | 2 | PD-weighted GA | 62.068853 | 0.620689 | 1.240x |
| 1 | 2 | FI-weighted GA | 63.027923 | 0.630279 | 1.259x |
| 1 | 2 | +link quality | 52.548732 | 0.525487 | 1.050x |
| 2 | 3 | fixed weights | 48.988340 | 0.489883 | 1.000x |
| 2 | 3 | PD-weighted GA | 58.538410 | 0.585384 | 1.195x |
| 2 | 3 | FI-weighted GA | 58.917124 | 0.589171 | 1.203x |
| 2 | 3 | +link quality | 49.449413 | 0.494494 | 1.009x |
| 3 | 4 | fixed weights | 49.191234 | 0.491912 | 1.000x |
| 3 | 4 | PD-weighted GA | 60.014629 | 0.600146 | 1.220x |
| 3 | 4 | FI-weighted GA | 59.158402 | 0.591584 | 1.203x |
| 3 | 4 | +link quality | 51.140631 | 0.511406 | 1.040x |
| 4 | 5 | fixed weights | 52.142173 | 0.521422 | 1.000x |
| 4 | 5 | PD-weighted GA | 62.883963 | 0.628840 | 1.206x |
| 4 | 5 | FI-weighted GA | 62.472077 | 0.624721 | 1.198x |
| 4 | 5 | +link quality | 59.169172 | 0.591692 | 1.135x |
| 5 | 6 | fixed weights | 48.890102 | 0.488901 | 1.000x |
| 5 | 6 | PD-weighted GA | 60.574684 | 0.605747 | 1.239x |
| 5 | 6 | FI-weighted GA | 60.959331 | 0.609593 | 1.247x |
| 5 | 6 | +link quality | 50.884942 | 0.508849 | 1.041x |
| 6 | 7 | fixed weights | 45.516180 | 0.455162 | 1.000x |
| 6 | 7 | PD-weighted GA | 56.937949 | 0.569379 | 1.251x |
| 6 | 7 | FI-weighted GA | 57.472493 | 0.574725 | 1.263x |
| 6 | 7 | +link quality | 51.261523 | 0.512615 | 1.126x |
| 7 | 8 | fixed weights | 50.583799 | 0.505838 | 1.000x |
| 7 | 8 | PD-weighted GA | 61.817474 | 0.618175 | 1.222x |
| 7 | 8 | FI-weighted GA | 62.490907 | 0.624909 | 1.235x |
| 7 | 8 | +link quality | 53.997209 | 0.539972 | 1.067x |
| 8 | 9 | fixed weights | 50.711744 | 0.507117 | 1.000x |
| 8 | 9 | PD-weighted GA | 61.665220 | 0.616652 | 1.216x |
| 8 | 9 | FI-weighted GA | 61.623715 | 0.616237 | 1.215x |
| 8 | 9 | +link quality | 52.250815 | 0.522508 | 1.030x |
| 9 | 10 | fixed weights | 48.152542 | 0.481525 | 1.000x |
| 9 | 10 | PD-weighted GA | 57.834814 | 0.578348 | 1.201x |
| 9 | 10 | FI-weighted GA | 58.598855 | 0.585989 | 1.217x |
| 9 | 10 | +link quality | 50.042469 | 0.500425 | 1.039x |
| 10 | 11 | fixed weights | 49.408435 | 0.494084 | 1.000x |
| 10 | 11 | PD-weighted GA | 60.494897 | 0.604949 | 1.224x |
| 10 | 11 | FI-weighted GA | 62.569103 | 0.625691 | 1.266x |
| 10 | 11 | +link quality | 54.112915 | 0.541129 | 1.095x |
| 11 | 12 | fixed weights | 51.461031 | 0.514610 | 1.000x |
| 11 | 12 | PD-weighted GA | 64.262697 | 0.642627 | 1.249x |
| 11 | 12 | FI-weighted GA | 95.801161 | 0.958012 | 1.862x |
| 11 | 12 | +link quality | 77.403555 | 0.774036 | 1.504x |
| 12 | 13 | fixed weights | 52.760519 | 0.527605 | 1.000x |
| 12 | 13 | PD-weighted GA | 60.219638 | 0.602196 | 1.141x |
| 12 | 13 | FI-weighted GA | 60.798680 | 0.607987 | 1.152x |
| 12 | 13 | +link quality | 51.988450 | 0.519885 | 0.985x |
| 13 | 14 | fixed weights | 48.641206 | 0.486412 | 1.000x |
| 13 | 14 | PD-weighted GA | 63.243586 | 0.632436 | 1.300x |
| 13 | 14 | FI-weighted GA | 62.386059 | 0.623861 | 1.283x |
| 13 | 14 | +link quality | 54.547074 | 0.545471 | 1.121x |
| 14 | 15 | fixed weights | 49.434879 | 0.494349 | 1.000x |
| 14 | 15 | PD-weighted GA | 60.571763 | 0.605718 | 1.225x |
| 14 | 15 | FI-weighted GA | 62.566541 | 0.625665 | 1.266x |
| 14 | 15 | +link quality | 53.555036 | 0.535550 | 1.083x |
| 15 | 16 | fixed weights | 50.252281 | 0.502523 | 1.000x |
| 15 | 16 | PD-weighted GA | 61.578288 | 0.615783 | 1.225x |
| 15 | 16 | FI-weighted GA | 60.806731 | 0.608067 | 1.210x |
| 15 | 16 | +link quality | 52.265136 | 0.522651 | 1.040x |
| 16 | 17 | fixed weights | 47.804497 | 0.478045 | 1.000x |
| 16 | 17 | PD-weighted GA | 61.179737 | 0.611797 | 1.280x |
| 16 | 17 | FI-weighted GA | 61.478195 | 0.614782 | 1.286x |
| 16 | 17 | +link quality | 53.169399 | 0.531694 | 1.112x |
| 17 | 18 | fixed weights | 50.575378 | 0.505754 | 1.000x |
| 17 | 18 | PD-weighted GA | 62.735329 | 0.627353 | 1.240x |
| 17 | 18 | FI-weighted GA | 63.147101 | 0.631471 | 1.249x |
| 17 | 18 | +link quality | 55.415889 | 0.554159 | 1.096x |
| 18 | 19 | fixed weights | 51.983564 | 0.519836 | 1.000x |
| 18 | 19 | PD-weighted GA | 60.418924 | 0.604189 | 1.162x |
| 18 | 19 | FI-weighted GA | 60.315489 | 0.603155 | 1.160x |
| 18 | 19 | +link quality | 51.647153 | 0.516472 | 0.994x |
| 19 | 20 | fixed weights | 48.891639 | 0.488916 | 1.000x |
| 19 | 20 | PD-weighted GA | 59.527650 | 0.595277 | 1.218x |
| 19 | 20 | FI-weighted GA | 60.181877 | 0.601819 | 1.231x |
| 19 | 20 | +link quality | 50.678454 | 0.506785 | 1.037x |
| 20 | 21 | fixed weights | 50.570012 | 0.505700 | 1.000x |
| 20 | 21 | PD-weighted GA | 62.565830 | 0.625658 | 1.237x |
| 20 | 21 | FI-weighted GA | 62.581649 | 0.625816 | 1.238x |
| 20 | 21 | +link quality | 53.201568 | 0.532016 | 1.052x |
| 21 | 22 | fixed weights | 50.506277 | 0.505063 | 1.000x |
| 21 | 22 | PD-weighted GA | 62.836586 | 0.628366 | 1.244x |
| 21 | 22 | FI-weighted GA | 62.405682 | 0.624057 | 1.236x |
| 21 | 22 | +link quality | 53.658421 | 0.536584 | 1.062x |
| 22 | 23 | fixed weights | 51.370177 | 0.513702 | 1.000x |
| 22 | 23 | PD-weighted GA | 62.243736 | 0.622437 | 1.212x |
| 22 | 23 | FI-weighted GA | 61.606570 | 0.616066 | 1.199x |
| 22 | 23 | +link quality | 51.311514 | 0.513115 | 0.999x |
| 23 | 24 | fixed weights | 51.107690 | 0.511077 | 1.000x |
| 23 | 24 | PD-weighted GA | 64.997035 | 0.649970 | 1.272x |
| 23 | 24 | FI-weighted GA | 65.930304 | 0.659303 | 1.290x |
| 23 | 24 | +link quality | 55.108495 | 0.551085 | 1.078x |
| 24 | 25 | fixed weights | 49.746923 | 0.497469 | 1.000x |
| 24 | 25 | PD-weighted GA | 60.865853 | 0.608659 | 1.224x |
| 24 | 25 | FI-weighted GA | 61.809320 | 0.618093 | 1.242x |
| 24 | 25 | +link quality | 53.326054 | 0.533261 | 1.072x |
| 25 | 26 | fixed weights | 51.714442 | 0.517144 | 1.000x |
| 25 | 26 | PD-weighted GA | 62.306119 | 0.623061 | 1.205x |
| 25 | 26 | FI-weighted GA | 63.163268 | 0.631633 | 1.221x |
| 25 | 26 | +link quality | 52.441477 | 0.524415 | 1.014x |
| 26 | 27 | fixed weights | 49.014285 | 0.490143 | 1.000x |
| 26 | 27 | PD-weighted GA | 60.162067 | 0.601621 | 1.227x |
| 26 | 27 | FI-weighted GA | 61.028994 | 0.610290 | 1.245x |
| 26 | 27 | +link quality | 53.277043 | 0.532770 | 1.087x |
| 27 | 28 | fixed weights | 50.669617 | 0.506696 | 1.000x |
| 27 | 28 | PD-weighted GA | 61.361716 | 0.613617 | 1.211x |
| 27 | 28 | FI-weighted GA | 63.717198 | 0.637172 | 1.258x |
| 27 | 28 | +link quality | 55.070208 | 0.550702 | 1.087x |
| 28 | 29 | fixed weights | 48.907108 | 0.489071 | 1.000x |
| 28 | 29 | PD-weighted GA | 59.606992 | 0.596070 | 1.219x |
| 28 | 29 | FI-weighted GA | 61.249792 | 0.612498 | 1.252x |
| 28 | 29 | +link quality | 51.140988 | 0.511410 | 1.046x |
| 29 | 30 | fixed weights | 49.410062 | 0.494101 | 1.000x |
| 29 | 30 | PD-weighted GA | 60.680190 | 0.606802 | 1.228x |
| 29 | 30 | FI-weighted GA | 60.781662 | 0.607817 | 1.230x |
| 29 | 30 | +link quality | 53.444021 | 0.534440 | 1.082x |
| 30 | 31 | fixed weights | 49.900069 | 0.499001 | 1.000x |
| 30 | 31 | PD-weighted GA | 62.366547 | 0.623665 | 1.250x |
| 30 | 31 | FI-weighted GA | 62.240766 | 0.622408 | 1.247x |
| 30 | 31 | +link quality | 53.602994 | 0.536030 | 1.074x |
| 31 | 32 | fixed weights | 49.889680 | 0.498897 | 1.000x |
| 31 | 32 | PD-weighted GA | 61.757102 | 0.617571 | 1.238x |
| 31 | 32 | FI-weighted GA | 61.398907 | 0.613989 | 1.231x |
| 31 | 32 | +link quality | 52.897528 | 0.528975 | 1.060x |
| 32 | 33 | fixed weights | 50.695053 | 0.506951 | 1.000x |
| 32 | 33 | PD-weighted GA | 60.528583 | 0.605286 | 1.194x |
| 32 | 33 | FI-weighted GA | 63.724220 | 0.637242 | 1.257x |
| 32 | 33 | +link quality | 53.255747 | 0.532557 | 1.051x |
| 33 | 34 | fixed weights | 48.794293 | 0.487943 | 1.000x |
| 33 | 34 | PD-weighted GA | 60.419172 | 0.604192 | 1.238x |
| 33 | 34 | FI-weighted GA | 62.131179 | 0.621312 | 1.273x |
| 33 | 34 | +link quality | 53.708440 | 0.537084 | 1.101x |
| 34 | 35 | fixed weights | 50.736478 | 0.507365 | 1.000x |
| 34 | 35 | PD-weighted GA | 62.725141 | 0.627251 | 1.236x |
| 34 | 35 | FI-weighted GA | 62.639258 | 0.626393 | 1.235x |
| 34 | 35 | +link quality | 55.229112 | 0.552291 | 1.089x |
| 35 | 36 | fixed weights | 54.900791 | 0.549008 | 1.000x |
| 35 | 36 | PD-weighted GA | 92.526798 | 0.925268 | 1.685x |
| 35 | 36 | FI-weighted GA | 91.408873 | 0.914089 | 1.665x |
| 35 | 36 | +link quality | 53.888239 | 0.538882 | 0.982x |
| 36 | 37 | fixed weights | 47.495284 | 0.474953 | 1.000x |
| 36 | 37 | PD-weighted GA | 57.649662 | 0.576497 | 1.214x |
| 36 | 37 | FI-weighted GA | 59.717662 | 0.597177 | 1.257x |
| 36 | 37 | +link quality | 51.743380 | 0.517434 | 1.089x |
| 37 | 38 | fixed weights | 49.993023 | 0.499930 | 1.000x |
| 37 | 38 | PD-weighted GA | 62.257209 | 0.622572 | 1.245x |
| 37 | 38 | FI-weighted GA | 65.966206 | 0.659662 | 1.320x |
| 37 | 38 | +link quality | 54.085317 | 0.540853 | 1.082x |
| 38 | 39 | fixed weights | 50.628888 | 0.506289 | 1.000x |
| 38 | 39 | PD-weighted GA | 61.564885 | 0.615649 | 1.216x |
| 38 | 39 | FI-weighted GA | 61.194755 | 0.611948 | 1.209x |
| 38 | 39 | +link quality | 51.911143 | 0.519111 | 1.025x |
| 39 | 40 | fixed weights | 48.572216 | 0.485722 | 1.000x |
| 39 | 40 | PD-weighted GA | 76.639093 | 0.766391 | 1.578x |
| 39 | 40 | FI-weighted GA | 64.855923 | 0.648559 | 1.335x |
| 39 | 40 | +link quality | 50.886305 | 0.508863 | 1.048x |
| 40 | 41 | fixed weights | 47.136255 | 0.471363 | 1.000x |
| 40 | 41 | PD-weighted GA | 57.889604 | 0.578896 | 1.228x |
| 40 | 41 | FI-weighted GA | 58.464844 | 0.584648 | 1.240x |
| 40 | 41 | +link quality | 50.079097 | 0.500791 | 1.062x |
| 41 | 42 | fixed weights | 48.049525 | 0.480495 | 1.000x |
| 41 | 42 | PD-weighted GA | 58.837150 | 0.588371 | 1.225x |
| 41 | 42 | FI-weighted GA | 62.193712 | 0.621937 | 1.294x |
| 41 | 42 | +link quality | 63.647007 | 0.636470 | 1.325x |
| 42 | 43 | fixed weights | 74.423261 | 0.744233 | 1.000x |
| 42 | 43 | PD-weighted GA | 60.378557 | 0.603786 | 0.811x |
| 42 | 43 | FI-weighted GA | 68.182826 | 0.681828 | 0.916x |
| 42 | 43 | +link quality | 50.486630 | 0.504866 | 0.678x |
| 43 | 44 | fixed weights | 54.107066 | 0.541071 | 1.000x |
| 43 | 44 | PD-weighted GA | 65.588455 | 0.655885 | 1.212x |
| 43 | 44 | FI-weighted GA | 66.419497 | 0.664195 | 1.228x |
| 43 | 44 | +link quality | 50.451539 | 0.504515 | 0.932x |
| 44 | 45 | fixed weights | 47.791104 | 0.477911 | 1.000x |
| 44 | 45 | PD-weighted GA | 58.794608 | 0.587946 | 1.230x |
| 44 | 45 | FI-weighted GA | 59.038892 | 0.590389 | 1.235x |
| 44 | 45 | +link quality | 50.493472 | 0.504935 | 1.057x |
| 45 | 46 | fixed weights | 46.913284 | 0.469133 | 1.000x |
| 45 | 46 | PD-weighted GA | 56.772792 | 0.567728 | 1.210x |
| 45 | 46 | FI-weighted GA | 57.645903 | 0.576459 | 1.229x |
| 45 | 46 | +link quality | 49.099593 | 0.490996 | 1.047x |
| 46 | 47 | fixed weights | 48.544960 | 0.485450 | 1.000x |
| 46 | 47 | PD-weighted GA | 59.278452 | 0.592785 | 1.221x |
| 46 | 47 | FI-weighted GA | 59.260141 | 0.592601 | 1.221x |
| 46 | 47 | +link quality | 50.986010 | 0.509860 | 1.050x |
| 47 | 48 | fixed weights | 48.445768 | 0.484458 | 1.000x |
| 47 | 48 | PD-weighted GA | 58.542043 | 0.585420 | 1.208x |
| 47 | 48 | FI-weighted GA | 58.703604 | 0.587036 | 1.212x |
| 47 | 48 | +link quality | 49.732676 | 0.497327 | 1.027x |
| 48 | 49 | fixed weights | 47.431752 | 0.474318 | 1.000x |
| 48 | 49 | PD-weighted GA | 58.053279 | 0.580533 | 1.224x |
| 48 | 49 | FI-weighted GA | 59.224251 | 0.592243 | 1.249x |
| 48 | 49 | +link quality | 50.074414 | 0.500744 | 1.056x |
| 49 | 50 | fixed weights | 46.706567 | 0.467066 | 1.000x |
| 49 | 50 | PD-weighted GA | 57.791614 | 0.577916 | 1.237x |
| 49 | 50 | FI-weighted GA | 57.844249 | 0.578442 | 1.238x |
| 49 | 50 | +link quality | 50.011981 | 0.500120 | 1.071x |
| 50 | 51 | fixed weights | 49.393931 | 0.493939 | 1.000x |
| 50 | 51 | PD-weighted GA | 58.868183 | 0.588682 | 1.192x |
| 50 | 51 | FI-weighted GA | 59.221011 | 0.592210 | 1.199x |
| 50 | 51 | +link quality | 50.957296 | 0.509573 | 1.032x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.862938 | 1.649569 | 1.455125 |
| PD-weighted GA | 2.735723 | 1.562807 | 1.254925 |
| FI-weighted GA | 2.495536 | 1.547675 | 0.984750 |
| +link quality | 2.329283 | 1.600391 | 0.578800 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.862938 +/- 0.123917 | [2.828590, 2.897286] | 50 |
| PD-weighted GA | E-OSPA | 2.735723 +/- 0.116190 | [2.703517, 2.767930] | 50 |
| FI-weighted GA | E-OSPA | 2.495536 +/- 0.111833 | [2.464537, 2.526534] | 50 |
| +link quality | E-OSPA | 2.329283 +/- 0.071369 | [2.309500, 2.349066] | 50 |
| fixed weights | RMSE | 1.649569 +/- 0.078096 | [1.627922, 1.671216] | 50 |
| PD-weighted GA | RMSE | 1.562807 +/- 0.060652 | [1.545995, 1.579618] | 50 |
| FI-weighted GA | RMSE | 1.547675 +/- 0.049674 | [1.533906, 1.561444] | 50 |
| +link quality | RMSE | 1.600391 +/- 0.049191 | [1.586756, 1.614026] | 50 |
| fixed weights | CardErr | 1.455125 +/- 0.241978 | [1.388052, 1.522198] | 50 |
| PD-weighted GA | CardErr | 1.254925 +/- 0.204563 | [1.198223, 1.311627] | 50 |
| FI-weighted GA | CardErr | 0.984750 +/- 0.174928 | [0.936263, 1.033237] | 50 |
| +link quality | CardErr | 0.578800 +/- 0.066873 | [0.560264, 0.597336] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | E-OSPA | 0.127215 +/- 0.039760 | [0.116194, 0.138236] | 4.44% | 50/50 | 1.776e-15 |
| FI-weighted GA | E-OSPA | 0.367402 +/- 0.074883 | [0.346646, 0.388159] | 12.83% | 50/50 | 1.776e-15 |
| +link quality | E-OSPA | 0.533655 +/- 0.088486 | [0.509128, 0.558182] | 18.64% | 50/50 | 1.776e-15 |
| PD-weighted GA | RMSE | 0.086763 +/- 0.039207 | [0.075895, 0.097630] | 5.26% | 50/50 | 1.776e-15 |
| FI-weighted GA | RMSE | 0.101894 +/- 0.057194 | [0.086041, 0.117748] | 6.18% | 50/50 | 1.776e-15 |
| +link quality | RMSE | 0.049178 +/- 0.059811 | [0.032599, 0.065757] | 2.98% | 45/50 | 4.21e-09 |
| PD-weighted GA | CardErr | 0.200200 +/- 0.087598 | [0.175919, 0.224481] | 13.76% | 50/50 | 1.776e-15 |
| FI-weighted GA | CardErr | 0.470375 +/- 0.118081 | [0.437645, 0.503105] | 32.33% | 50/50 | 1.776e-15 |
| +link quality | CardErr | 0.876325 +/- 0.211178 | [0.817789, 0.934861] | 60.22% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.719987 | 1.619776 | 1.083600 |
| 1 | PD-weighted GA | 2.527324 | 1.515429 | 0.824800 |
| 1 | FI-weighted GA | 2.356511 | 1.510822 | 0.690200 |
| 1 | +link quality | 2.284853 | 1.555869 | 0.541200 |
| 2 | fixed weights | 2.670351 | 1.594302 | 0.999200 |
| 2 | PD-weighted GA | 2.576232 | 1.521919 | 0.873000 |
| 2 | FI-weighted GA | 2.370264 | 1.513498 | 0.720000 |
| 2 | +link quality | 2.290607 | 1.565054 | 0.546400 |
| 3 | fixed weights | 2.740374 | 1.607566 | 1.150400 |
| 3 | PD-weighted GA | 2.558967 | 1.523273 | 0.874800 |
| 3 | FI-weighted GA | 2.362868 | 1.516864 | 0.735600 |
| 3 | +link quality | 2.295211 | 1.566326 | 0.570600 |
| 4 | fixed weights | 2.883255 | 1.654573 | 1.494400 |
| 4 | PD-weighted GA | 2.827513 | 1.573511 | 1.453800 |
| 4 | FI-weighted GA | 2.578131 | 1.562216 | 1.154000 |
| 4 | +link quality | 2.411042 | 1.633281 | 0.685400 |
| 5 | fixed weights | 2.918523 | 1.691261 | 1.520000 |
| 5 | PD-weighted GA | 2.896439 | 1.583266 | 1.603000 |
| 5 | FI-weighted GA | 2.628788 | 1.567069 | 1.258400 |
| 5 | +link quality | 2.371967 | 1.629439 | 0.623600 |
| 6 | fixed weights | 2.913753 | 1.708755 | 1.535200 |
| 6 | PD-weighted GA | 2.808430 | 1.601976 | 1.443600 |
| 6 | FI-weighted GA | 2.553213 | 1.570791 | 1.104600 |
| 6 | +link quality | 2.318031 | 1.613526 | 0.540200 |
| 7 | fixed weights | 2.925998 | 1.663623 | 1.565800 |
| 7 | PD-weighted GA | 2.866429 | 1.584224 | 1.507800 |
| 7 | FI-weighted GA | 2.590203 | 1.572937 | 1.153600 |
| 7 | +link quality | 2.364495 | 1.627267 | 0.604200 |
| 8 | fixed weights | 3.131265 | 1.656698 | 2.292400 |
| 8 | PD-weighted GA | 2.824454 | 1.598854 | 1.458600 |
| 8 | FI-weighted GA | 2.524306 | 1.567202 | 1.061600 |
| 8 | +link quality | 2.298057 | 1.612369 | 0.518800 |
