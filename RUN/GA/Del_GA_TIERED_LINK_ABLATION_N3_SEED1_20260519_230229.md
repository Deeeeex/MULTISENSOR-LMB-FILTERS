# GA Tiered Link Ablation (2026-05-19 23:02:29)

Comparison order: fixed weights -> PD-weighted GA -> FI-weighted GA -> +link quality

## Run Config
- Trials: 3
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4]
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

## Per-Trial Consensus Metrics
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

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.633016 | 2.466452 | 0.875000 |
| PD-weighted GA | 2.343229 | 1.995556 | 0.766250 |
| FI-weighted GA | 2.122560 | 1.790341 | 0.560833 |
| +link quality | 1.862613 | 1.426919 | 0.220000 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.633016 +/- 0.049105 | [2.511022, 2.755010] | 3 |
| PD-weighted GA | OSPA | 2.343229 +/- 0.088487 | [2.123396, 2.563061] | 3 |
| FI-weighted GA | OSPA | 2.122560 +/- 0.093132 | [1.891188, 2.353932] | 3 |
| +link quality | OSPA | 1.862613 +/- 0.034412 | [1.777122, 1.948103] | 3 |
| fixed weights | RMSE | 2.466452 +/- 0.267891 | [1.800919, 3.131985] | 3 |
| PD-weighted GA | RMSE | 1.995556 +/- 0.328230 | [1.180121, 2.810991] | 3 |
| FI-weighted GA | RMSE | 1.790341 +/- 0.189770 | [1.318889, 2.261794] | 3 |
| +link quality | RMSE | 1.426919 +/- 0.034839 | [1.340368, 1.513470] | 3 |
| fixed weights | Cardinality | 0.875000 +/- 0.086927 | [0.659045, 1.090955] | 3 |
| PD-weighted GA | Cardinality | 0.766250 +/- 0.078938 | [0.570141, 0.962359] | 3 |
| FI-weighted GA | Cardinality | 0.560833 +/- 0.103126 | [0.304634, 0.817032] | 3 |
| +link quality | Cardinality | 0.220000 +/- 0.025000 | [0.157892, 0.282108] | 3 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | OSPA | 0.289787 +/- 0.137577 | [-0.052001, 0.631575] | 11.01% | 3/3 | 0.25 |
| FI-weighted GA | OSPA | 0.510456 +/- 0.142212 | [0.157154, 0.863757] | 19.39% | 3/3 | 0.25 |
| +link quality | OSPA | 0.770403 +/- 0.081852 | [0.567056, 0.973751] | 29.26% | 3/3 | 0.25 |
| PD-weighted GA | RMSE | 0.470896 +/- 0.112695 | [0.190922, 0.750869] | 19.09% | 3/3 | 0.25 |
| FI-weighted GA | RMSE | 0.676110 +/- 0.157159 | [0.285675, 1.066546] | 27.41% | 3/3 | 0.25 |
| +link quality | RMSE | 1.039532 +/- 0.302723 | [0.287466, 1.791599] | 42.15% | 3/3 | 0.25 |
| PD-weighted GA | Cardinality | 0.108750 +/- 0.088353 | [-0.110749, 0.328249] | 12.43% | 3/3 | 0.25 |
| FI-weighted GA | Cardinality | 0.314167 +/- 0.129255 | [-0.006946, 0.635279] | 35.90% | 3/3 | 0.25 |
| +link quality | Cardinality | 0.655000 +/- 0.065144 | [0.493160, 0.816840] | 74.86% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 48.829602 +/- 1.101080 | 0.488296 | 1.000x | 3 |
| PD-weighted GA | 60.306207 +/- 1.045282 | 0.603062 | 1.235x | 3 |
| FI-weighted GA | 60.916447 +/- 0.401217 | 0.609164 | 1.248x | 3 |
| +link quality | 52.130693 +/- 1.538702 | 0.521307 | 1.067x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| PD-weighted GA | 11.476605 +/- 0.299451 | 23.51% | 3/3 |
| FI-weighted GA | 12.086845 +/- 1.325076 | 24.80% | 3/3 |
| +link quality | 3.301091 +/- 0.469861 | 6.75% | 3/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 48.227666 | 0.482277 | 1.000x |
| 1 | 2 | PD-weighted GA | 60.045063 | 0.600451 | 1.245x |
| 1 | 2 | FI-weighted GA | 61.379253 | 0.613793 | 1.273x |
| 1 | 2 | +link quality | 51.501992 | 0.515020 | 1.068x |
| 2 | 3 | fixed weights | 48.160707 | 0.481607 | 1.000x |
| 2 | 3 | PD-weighted GA | 59.416256 | 0.594163 | 1.234x |
| 2 | 3 | FI-weighted GA | 60.666795 | 0.606668 | 1.260x |
| 2 | 3 | +link quality | 51.005892 | 0.510059 | 1.059x |
| 3 | 4 | fixed weights | 50.100432 | 0.501004 | 1.000x |
| 3 | 4 | PD-weighted GA | 61.457302 | 0.614573 | 1.227x |
| 3 | 4 | FI-weighted GA | 60.703292 | 0.607033 | 1.212x |
| 3 | 4 | +link quality | 53.884194 | 0.538842 | 1.076x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.898247 | 1.629533 | 1.598333 |
| PD-weighted GA | 2.772143 | 1.539509 | 1.382917 |
| FI-weighted GA | 2.489994 | 1.541360 | 1.068333 |
| +link quality | 2.328685 | 1.581837 | 0.580000 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.898247 +/- 0.082407 | [2.693521, 3.102973] | 3 |
| PD-weighted GA | E-OSPA | 2.772143 +/- 0.088600 | [2.552032, 2.992254] | 3 |
| FI-weighted GA | E-OSPA | 2.489994 +/- 0.135330 | [2.153788, 2.826200] | 3 |
| +link quality | E-OSPA | 2.328685 +/- 0.072518 | [2.148525, 2.508845] | 3 |
| fixed weights | RMSE | 1.629533 +/- 0.065863 | [1.465908, 1.793158] | 3 |
| PD-weighted GA | RMSE | 1.539509 +/- 0.063249 | [1.382376, 1.696641] | 3 |
| FI-weighted GA | RMSE | 1.541360 +/- 0.085525 | [1.328887, 1.753833] | 3 |
| +link quality | RMSE | 1.581837 +/- 0.074104 | [1.397738, 1.765936] | 3 |
| fixed weights | CardErr | 1.598333 +/- 0.141097 | [1.247801, 1.948866] | 3 |
| PD-weighted GA | CardErr | 1.382917 +/- 0.155007 | [0.997828, 1.768006] | 3 |
| FI-weighted GA | CardErr | 1.068333 +/- 0.189598 | [0.597308, 1.539359] | 3 |
| +link quality | CardErr | 0.580000 +/- 0.068053 | [0.410933, 0.749067] | 3 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| PD-weighted GA | E-OSPA | 0.126104 +/- 0.019549 | [0.077537, 0.174672] | 4.35% | 3/3 | 0.25 |
| FI-weighted GA | E-OSPA | 0.408253 +/- 0.056735 | [0.267303, 0.549203] | 14.09% | 3/3 | 0.25 |
| +link quality | E-OSPA | 0.569563 +/- 0.025667 | [0.505797, 0.633329] | 19.65% | 3/3 | 0.25 |
| PD-weighted GA | RMSE | 0.090024 +/- 0.017287 | [0.047079, 0.132970] | 5.52% | 3/3 | 0.25 |
| FI-weighted GA | RMSE | 0.088173 +/- 0.023319 | [0.030240, 0.146106] | 5.41% | 3/3 | 0.25 |
| +link quality | RMSE | 0.047696 +/- 0.025505 | [-0.015668, 0.111060] | 2.93% | 3/3 | 0.25 |
| PD-weighted GA | CardErr | 0.215417 +/- 0.064242 | [0.055817, 0.375016] | 13.48% | 3/3 | 0.25 |
| FI-weighted GA | CardErr | 0.530000 +/- 0.086684 | [0.314648, 0.745352] | 33.16% | 3/3 | 0.25 |
| +link quality | CardErr | 1.018333 +/- 0.073158 | [0.836584, 1.200082] | 63.71% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.808112 | 1.593222 | 1.263333 |
| 1 | PD-weighted GA | 2.481936 | 1.511165 | 0.813333 |
| 1 | FI-weighted GA | 2.335965 | 1.495615 | 0.673333 |
| 1 | +link quality | 2.314103 | 1.540642 | 0.540000 |
| 2 | fixed weights | 2.564830 | 1.596495 | 0.856667 |
| 2 | PD-weighted GA | 2.606496 | 1.471433 | 0.960000 |
| 2 | FI-weighted GA | 2.294361 | 1.509466 | 0.670000 |
| 2 | +link quality | 2.310036 | 1.547500 | 0.580000 |
| 3 | fixed weights | 2.788741 | 1.520167 | 1.313333 |
| 3 | PD-weighted GA | 2.536456 | 1.470599 | 0.843333 |
| 3 | FI-weighted GA | 2.291095 | 1.471171 | 0.713333 |
| 3 | +link quality | 2.289154 | 1.525611 | 0.546667 |
| 4 | fixed weights | 2.744554 | 1.647656 | 1.406667 |
| 4 | PD-weighted GA | 2.799110 | 1.514594 | 1.646667 |
| 4 | FI-weighted GA | 2.539650 | 1.573800 | 1.253333 |
| 4 | +link quality | 2.321449 | 1.562768 | 0.606667 |
| 5 | fixed weights | 3.253490 | 1.736795 | 2.083333 |
| 5 | PD-weighted GA | 3.118085 | 1.611255 | 1.923333 |
| 5 | FI-weighted GA | 2.769453 | 1.570672 | 1.510000 |
| 5 | +link quality | 2.437497 | 1.639589 | 0.706667 |
| 6 | fixed weights | 2.936944 | 1.678346 | 1.620000 |
| 6 | PD-weighted GA | 2.781268 | 1.600424 | 1.396667 |
| 6 | FI-weighted GA | 2.448644 | 1.539188 | 1.033333 |
| 6 | +link quality | 2.262963 | 1.609386 | 0.486667 |
| 7 | fixed weights | 3.097092 | 1.610141 | 2.013333 |
| 7 | PD-weighted GA | 3.060067 | 1.552619 | 1.936667 |
| 7 | FI-weighted GA | 2.695913 | 1.612293 | 1.480000 |
| 7 | +link quality | 2.442660 | 1.647925 | 0.660000 |
| 8 | fixed weights | 2.992216 | 1.653440 | 2.230000 |
| 8 | PD-weighted GA | 2.793727 | 1.583980 | 1.543333 |
| 8 | FI-weighted GA | 2.544872 | 1.558674 | 1.213333 |
| 8 | +link quality | 2.251615 | 1.581275 | 0.513333 |
