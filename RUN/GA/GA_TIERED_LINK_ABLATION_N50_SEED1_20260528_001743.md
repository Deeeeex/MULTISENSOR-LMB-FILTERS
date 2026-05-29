# GA Tiered Link Ablation (2026-05-28 00:17:43)

Comparison order: fixed weights -> Cao-Zhao FID-FIA baseline -> +structure-aware decoupled KLA -> +FID-FIA existence refinement

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

- finalArmMode: fidFiaExistenceRefinement

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

### Cao-Zhao FID-FIA baseline
- enabled: 1
- method: fidFia
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
- fidFiaExistenceStrength: 0.500
- fidFiaExistenceMinScore: 0.400
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

### +structure-aware decoupled KLA
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.700
- spatialMinWeight: 0.050
- existenceMinWeight: 0.050
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 0

### +FID-FIA existence refinement
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
- useFreshness: 0
- useCtFiDecay: 0
- useNIS: 0
- useDecoupledKla: 1
- useStructureAwareKla: 1
- usePosteriorStructureConsistency: 0
- existenceConfidenceMinScore: 0.850
- existenceConfidencePower: 2.000
- spatialEmaAlpha: 0.700
- existenceEmaAlpha: 0.000
- spatialMinWeight: 0.050
- existenceMinWeight: 0.000
- spatialDecouplingStrength: 0.500
- existenceDecouplingStrength: 0.150
- spatialStructureStrength: 0.450
- existenceStructureStrength: 0.080
- structureReliabilityPower: 0.300
- structureReliabilityMinScore: 0.250
- useFidFiaExistence: 1
- fidFiaExistenceStrength: 4.000
- fidFiaExistenceMinScore: 0.000
- fidFiaUseExistenceWeight: 1
- fidFiaExistencePower: 1.000
- fidFiaQuadraturePoints: 3
- fidFiaUseDetectionProbability: 1
- fidFiaUseEma: 0
- fidFiaMinWeight: 0.000

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
| 1 | 2 | Cao-Zhao FID-FIA baseline | 1.919750 | 1.703510 | 0.160000 |
| 1 | 2 | +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| 1 | 2 | +FID-FIA existence refinement | 1.714819 | 1.478188 | 0.066250 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 1.760426 | 1.579121 | 0.085000 |
| 2 | 3 | +structure-aware decoupled KLA | 1.854447 | 1.371532 | 0.191250 |
| 2 | 3 | +FID-FIA existence refinement | 1.663486 | 1.698166 | 0.052500 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 1.765981 | 1.628179 | 0.092500 |
| 3 | 4 | +structure-aware decoupled KLA | 1.808801 | 1.431204 | 0.221250 |
| 3 | 4 | +FID-FIA existence refinement | 1.636377 | 1.664398 | 0.055000 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 1.842140 | 1.761535 | 0.131250 |
| 4 | 5 | +structure-aware decoupled KLA | 1.736420 | 1.923505 | 0.170000 |
| 4 | 5 | +FID-FIA existence refinement | 1.644623 | 1.459300 | 0.048750 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 1.860653 | 1.788350 | 0.146250 |
| 5 | 6 | +structure-aware decoupled KLA | 1.852532 | 1.777241 | 0.243750 |
| 5 | 6 | +FID-FIA existence refinement | 1.679733 | 1.496193 | 0.053750 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 1.869464 | 1.794725 | 0.157500 |
| 6 | 7 | +structure-aware decoupled KLA | 1.839920 | 1.764960 | 0.193750 |
| 6 | 7 | +FID-FIA existence refinement | 1.783806 | 1.798401 | 0.068750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 1.777385 | 1.501640 | 0.125000 |
| 7 | 8 | +structure-aware decoupled KLA | 1.724528 | 1.356549 | 0.161250 |
| 7 | 8 | +FID-FIA existence refinement | 1.602056 | 1.442085 | 0.050000 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 1.765026 | 1.553181 | 0.090000 |
| 8 | 9 | +structure-aware decoupled KLA | 1.732578 | 1.434821 | 0.177500 |
| 8 | 9 | +FID-FIA existence refinement | 1.616239 | 1.433022 | 0.043750 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 1.832008 | 1.807331 | 0.133750 |
| 9 | 10 | +structure-aware decoupled KLA | 1.798485 | 2.148753 | 0.180000 |
| 9 | 10 | +FID-FIA existence refinement | 1.710576 | 1.520268 | 0.077500 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 1.791028 | 1.578027 | 0.115000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.787456 | 1.493683 | 0.182500 |
| 10 | 11 | +FID-FIA existence refinement | 1.672873 | 1.598233 | 0.070000 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 1.872682 | 1.777547 | 0.136250 |
| 11 | 12 | +structure-aware decoupled KLA | 1.753632 | 1.572377 | 0.171250 |
| 11 | 12 | +FID-FIA existence refinement | 1.708450 | 1.733906 | 0.086250 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 1.912568 | 1.624291 | 0.148750 |
| 12 | 13 | +structure-aware decoupled KLA | 1.742848 | 1.486898 | 0.197500 |
| 12 | 13 | +FID-FIA existence refinement | 1.679613 | 1.530487 | 0.068750 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 1.875715 | 1.586621 | 0.125000 |
| 13 | 14 | +structure-aware decoupled KLA | 1.790615 | 1.451047 | 0.182500 |
| 13 | 14 | +FID-FIA existence refinement | 1.704557 | 1.463948 | 0.072500 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 1.834527 | 1.599571 | 0.117500 |
| 14 | 15 | +structure-aware decoupled KLA | 1.772452 | 1.466171 | 0.191250 |
| 14 | 15 | +FID-FIA existence refinement | 1.685614 | 1.464790 | 0.095000 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 1.666159 | 1.429455 | 0.115000 |
| 15 | 16 | +structure-aware decoupled KLA | 1.704826 | 1.517027 | 0.176250 |
| 15 | 16 | +FID-FIA existence refinement | 1.543640 | 1.367411 | 0.048750 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 1.789898 | 1.591812 | 0.147500 |
| 16 | 17 | +structure-aware decoupled KLA | 1.830609 | 1.438554 | 0.228750 |
| 16 | 17 | +FID-FIA existence refinement | 1.628893 | 1.409847 | 0.072500 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 1.912720 | 1.789253 | 0.188750 |
| 17 | 18 | +structure-aware decoupled KLA | 1.863847 | 1.652217 | 0.216250 |
| 17 | 18 | +FID-FIA existence refinement | 1.733908 | 1.473600 | 0.075000 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 1.744260 | 1.643484 | 0.107500 |
| 18 | 19 | +structure-aware decoupled KLA | 1.673306 | 1.510607 | 0.161250 |
| 18 | 19 | +FID-FIA existence refinement | 1.623403 | 1.457256 | 0.036250 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 1.816670 | 1.618665 | 0.110000 |
| 19 | 20 | +structure-aware decoupled KLA | 1.812300 | 1.473501 | 0.197500 |
| 19 | 20 | +FID-FIA existence refinement | 1.685378 | 1.551796 | 0.043750 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 1.795517 | 1.591942 | 0.091250 |
| 20 | 21 | +structure-aware decoupled KLA | 1.759623 | 1.539784 | 0.175000 |
| 20 | 21 | +FID-FIA existence refinement | 1.661180 | 1.522340 | 0.036250 |
| 21 | 22 | fixed weights | 2.445386 | 2.063249 | 0.706250 |
| 21 | 22 | Cao-Zhao FID-FIA baseline | 1.765444 | 1.576817 | 0.126250 |
| 21 | 22 | +structure-aware decoupled KLA | 1.771566 | 1.591554 | 0.178750 |
| 21 | 22 | +FID-FIA existence refinement | 1.686873 | 1.456035 | 0.085000 |
| 22 | 23 | fixed weights | 2.782054 | 2.774744 | 0.917500 |
| 22 | 23 | Cao-Zhao FID-FIA baseline | 1.859706 | 1.634981 | 0.132500 |
| 22 | 23 | +structure-aware decoupled KLA | 1.828140 | 1.410398 | 0.208750 |
| 22 | 23 | +FID-FIA existence refinement | 1.717386 | 1.484844 | 0.063750 |
| 23 | 24 | fixed weights | 2.811135 | 1.942125 | 1.156250 |
| 23 | 24 | Cao-Zhao FID-FIA baseline | 1.851457 | 1.616359 | 0.110000 |
| 23 | 24 | +structure-aware decoupled KLA | 1.793788 | 1.500075 | 0.176250 |
| 23 | 24 | +FID-FIA existence refinement | 1.707424 | 1.532879 | 0.040000 |
| 24 | 25 | fixed weights | 2.569606 | 2.368173 | 0.711250 |
| 24 | 25 | Cao-Zhao FID-FIA baseline | 1.870342 | 1.672704 | 0.135000 |
| 24 | 25 | +structure-aware decoupled KLA | 1.875143 | 1.500632 | 0.221250 |
| 24 | 25 | +FID-FIA existence refinement | 1.717941 | 1.523236 | 0.085000 |
| 25 | 26 | fixed weights | 2.225426 | 2.278298 | 0.403750 |
| 25 | 26 | Cao-Zhao FID-FIA baseline | 1.802784 | 1.614028 | 0.127500 |
| 25 | 26 | +structure-aware decoupled KLA | 1.717305 | 1.374395 | 0.173750 |
| 25 | 26 | +FID-FIA existence refinement | 1.622779 | 1.421175 | 0.077500 |
| 26 | 27 | fixed weights | 2.159231 | 1.899203 | 0.517500 |
| 26 | 27 | Cao-Zhao FID-FIA baseline | 1.775112 | 1.557627 | 0.120000 |
| 26 | 27 | +structure-aware decoupled KLA | 1.646146 | 1.363300 | 0.142500 |
| 26 | 27 | +FID-FIA existence refinement | 1.652730 | 1.490183 | 0.060000 |
| 27 | 28 | fixed weights | 2.126003 | 1.944541 | 0.406250 |
| 27 | 28 | Cao-Zhao FID-FIA baseline | 1.809826 | 1.580797 | 0.132500 |
| 27 | 28 | +structure-aware decoupled KLA | 1.736613 | 1.421164 | 0.208750 |
| 27 | 28 | +FID-FIA existence refinement | 1.668913 | 1.503191 | 0.043750 |
| 28 | 29 | fixed weights | 2.917753 | 1.994773 | 1.183750 |
| 28 | 29 | Cao-Zhao FID-FIA baseline | 1.846308 | 1.759788 | 0.163750 |
| 28 | 29 | +structure-aware decoupled KLA | 1.873211 | 1.616359 | 0.268750 |
| 28 | 29 | +FID-FIA existence refinement | 1.659328 | 1.472859 | 0.086250 |
| 29 | 30 | fixed weights | 2.600417 | 1.944603 | 1.000000 |
| 29 | 30 | Cao-Zhao FID-FIA baseline | 1.942099 | 1.757728 | 0.162500 |
| 29 | 30 | +structure-aware decoupled KLA | 1.870636 | 1.691933 | 0.227500 |
| 29 | 30 | +FID-FIA existence refinement | 1.769754 | 1.490459 | 0.085000 |
| 30 | 31 | fixed weights | 2.468399 | 2.109218 | 0.737500 |
| 30 | 31 | Cao-Zhao FID-FIA baseline | 1.801926 | 2.017408 | 0.102500 |
| 30 | 31 | +structure-aware decoupled KLA | 1.770363 | 1.471423 | 0.161250 |
| 30 | 31 | +FID-FIA existence refinement | 1.671316 | 1.432130 | 0.055000 |
| 31 | 32 | fixed weights | 2.513279 | 1.885047 | 0.721250 |
| 31 | 32 | Cao-Zhao FID-FIA baseline | 1.792032 | 1.568905 | 0.131250 |
| 31 | 32 | +structure-aware decoupled KLA | 1.746697 | 1.452005 | 0.170000 |
| 31 | 32 | +FID-FIA existence refinement | 1.646013 | 1.432492 | 0.065000 |
| 32 | 33 | fixed weights | 2.019727 | 1.812822 | 0.302500 |
| 32 | 33 | Cao-Zhao FID-FIA baseline | 1.738279 | 1.463929 | 0.120000 |
| 32 | 33 | +structure-aware decoupled KLA | 1.679034 | 1.408664 | 0.165000 |
| 32 | 33 | +FID-FIA existence refinement | 1.648319 | 1.381624 | 0.061250 |
| 33 | 34 | fixed weights | 2.490122 | 2.173185 | 0.962500 |
| 33 | 34 | Cao-Zhao FID-FIA baseline | 1.847348 | 1.861965 | 0.106250 |
| 33 | 34 | +structure-aware decoupled KLA | 1.674970 | 1.535775 | 0.183750 |
| 33 | 34 | +FID-FIA existence refinement | 1.741679 | 2.128409 | 0.061250 |
| 34 | 35 | fixed weights | 2.915586 | 2.674676 | 1.071250 |
| 34 | 35 | Cao-Zhao FID-FIA baseline | 1.894216 | 1.682247 | 0.158750 |
| 34 | 35 | +structure-aware decoupled KLA | 1.864837 | 1.559731 | 0.185000 |
| 34 | 35 | +FID-FIA existence refinement | 1.781239 | 1.542392 | 0.088750 |
| 35 | 36 | fixed weights | 2.381312 | 2.282297 | 0.561250 |
| 35 | 36 | Cao-Zhao FID-FIA baseline | 1.748627 | 1.554821 | 0.093750 |
| 35 | 36 | +structure-aware decoupled KLA | 1.692204 | 1.449569 | 0.136250 |
| 35 | 36 | +FID-FIA existence refinement | 1.652536 | 2.085584 | 0.047500 |
| 36 | 37 | fixed weights | 2.794401 | 3.053361 | 0.971250 |
| 36 | 37 | Cao-Zhao FID-FIA baseline | 1.834034 | 1.652884 | 0.112500 |
| 36 | 37 | +structure-aware decoupled KLA | 1.844239 | 1.497974 | 0.215000 |
| 36 | 37 | +FID-FIA existence refinement | 1.667394 | 1.465123 | 0.067500 |
| 37 | 38 | fixed weights | 2.488304 | 2.240250 | 0.821250 |
| 37 | 38 | Cao-Zhao FID-FIA baseline | 1.788970 | 1.567237 | 0.115000 |
| 37 | 38 | +structure-aware decoupled KLA | 1.697548 | 1.538374 | 0.148750 |
| 37 | 38 | +FID-FIA existence refinement | 1.708430 | 1.923114 | 0.098750 |
| 38 | 39 | fixed weights | 2.292857 | 2.023920 | 0.538750 |
| 38 | 39 | Cao-Zhao FID-FIA baseline | 1.812445 | 1.551533 | 0.113750 |
| 38 | 39 | +structure-aware decoupled KLA | 1.826976 | 1.463676 | 0.183750 |
| 38 | 39 | +FID-FIA existence refinement | 1.699633 | 1.498570 | 0.061250 |
| 39 | 40 | fixed weights | 2.535165 | 2.535969 | 0.646250 |
| 39 | 40 | Cao-Zhao FID-FIA baseline | 1.772170 | 1.675977 | 0.117500 |
| 39 | 40 | +structure-aware decoupled KLA | 1.796344 | 1.662470 | 0.206250 |
| 39 | 40 | +FID-FIA existence refinement | 1.640831 | 1.570054 | 0.085000 |
| 40 | 41 | fixed weights | 2.257174 | 2.481788 | 0.530000 |
| 40 | 41 | Cao-Zhao FID-FIA baseline | 1.817595 | 1.692176 | 0.091250 |
| 40 | 41 | +structure-aware decoupled KLA | 1.780970 | 1.560634 | 0.201250 |
| 40 | 41 | +FID-FIA existence refinement | 1.696185 | 1.717111 | 0.050000 |
| 41 | 42 | fixed weights | 2.094739 | 2.021235 | 0.453750 |
| 41 | 42 | Cao-Zhao FID-FIA baseline | 1.731595 | 1.529723 | 0.106250 |
| 41 | 42 | +structure-aware decoupled KLA | 1.593887 | 1.271218 | 0.158750 |
| 41 | 42 | +FID-FIA existence refinement | 1.576100 | 1.581485 | 0.055000 |
| 42 | 43 | fixed weights | 2.470491 | 2.757701 | 0.583750 |
| 42 | 43 | Cao-Zhao FID-FIA baseline | 1.815590 | 1.584592 | 0.087500 |
| 42 | 43 | +structure-aware decoupled KLA | 1.717249 | 1.437860 | 0.095000 |
| 42 | 43 | +FID-FIA existence refinement | 1.703510 | 1.660994 | 0.058750 |
| 43 | 44 | fixed weights | 2.550641 | 2.146021 | 0.801250 |
| 43 | 44 | Cao-Zhao FID-FIA baseline | 1.884829 | 1.593909 | 0.137500 |
| 43 | 44 | +structure-aware decoupled KLA | 1.809049 | 1.398760 | 0.183750 |
| 43 | 44 | +FID-FIA existence refinement | 1.716981 | 1.485193 | 0.055000 |
| 44 | 45 | fixed weights | 2.216546 | 2.738044 | 0.428750 |
| 44 | 45 | Cao-Zhao FID-FIA baseline | 1.777799 | 1.507427 | 0.118750 |
| 44 | 45 | +structure-aware decoupled KLA | 1.710189 | 1.388926 | 0.165000 |
| 44 | 45 | +FID-FIA existence refinement | 1.619562 | 1.400978 | 0.047500 |
| 45 | 46 | fixed weights | 2.537339 | 2.539290 | 0.686250 |
| 45 | 46 | Cao-Zhao FID-FIA baseline | 1.768407 | 1.580512 | 0.111250 |
| 45 | 46 | +structure-aware decoupled KLA | 1.745958 | 1.425635 | 0.173750 |
| 45 | 46 | +FID-FIA existence refinement | 1.669233 | 1.489890 | 0.072500 |
| 46 | 47 | fixed weights | 2.665038 | 2.010381 | 0.693750 |
| 46 | 47 | Cao-Zhao FID-FIA baseline | 1.805441 | 1.653108 | 0.117500 |
| 46 | 47 | +structure-aware decoupled KLA | 1.871566 | 1.802479 | 0.161250 |
| 46 | 47 | +FID-FIA existence refinement | 1.632071 | 1.488300 | 0.040000 |
| 47 | 48 | fixed weights | 2.238065 | 2.249780 | 0.577500 |
| 47 | 48 | Cao-Zhao FID-FIA baseline | 1.770124 | 1.581392 | 0.086250 |
| 47 | 48 | +structure-aware decoupled KLA | 1.751635 | 1.357804 | 0.170000 |
| 47 | 48 | +FID-FIA existence refinement | 1.625713 | 1.472262 | 0.032500 |
| 48 | 49 | fixed weights | 2.529967 | 3.164643 | 0.665000 |
| 48 | 49 | Cao-Zhao FID-FIA baseline | 1.880506 | 1.716774 | 0.123750 |
| 48 | 49 | +structure-aware decoupled KLA | 1.917595 | 1.648404 | 0.238750 |
| 48 | 49 | +FID-FIA existence refinement | 1.734222 | 1.492284 | 0.092500 |
| 49 | 50 | fixed weights | 2.564502 | 2.102492 | 0.951250 |
| 49 | 50 | Cao-Zhao FID-FIA baseline | 1.830986 | 1.631762 | 0.123750 |
| 49 | 50 | +structure-aware decoupled KLA | 1.726094 | 1.431585 | 0.183750 |
| 49 | 50 | +FID-FIA existence refinement | 1.741748 | 1.524762 | 0.066250 |
| 50 | 51 | fixed weights | 2.714437 | 3.379717 | 0.801250 |
| 50 | 51 | Cao-Zhao FID-FIA baseline | 1.855060 | 1.754974 | 0.138750 |
| 50 | 51 | +structure-aware decoupled KLA | 1.913492 | 1.626347 | 0.232500 |
| 50 | 51 | +FID-FIA existence refinement | 1.721147 | 1.594127 | 0.051250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.468973 | 2.326034 | 0.716025 |
| Cao-Zhao FID-FIA baseline | 1.817913 | 1.642846 | 0.122950 |
| +structure-aware decoupled KLA | 1.779218 | 1.522191 | 0.187675 |
| +FID-FIA existence refinement | 1.677524 | 1.546107 | 0.063200 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.468973 +/- 0.219981 | [2.407997, 2.529949] | 50 |
| Cao-Zhao FID-FIA baseline | OSPA | 1.817913 +/- 0.056010 | [1.802388, 1.833438] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.779218 +/- 0.072856 | [1.759023, 1.799413] | 50 |
| +FID-FIA existence refinement | OSPA | 1.677524 +/- 0.049899 | [1.663693, 1.691355] | 50 |
| fixed weights | RMSE | 2.326034 +/- 0.353065 | [2.228170, 2.423899] | 50 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.642846 +/- 0.109008 | [1.612631, 1.673062] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.522191 +/- 0.157069 | [1.478654, 1.565728] | 50 |
| +FID-FIA existence refinement | RMSE | 1.546107 +/- 0.157649 | [1.502409, 1.589806] | 50 |
| fixed weights | Cardinality | 0.716025 +/- 0.223795 | [0.653992, 0.778058] | 50 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.122950 +/- 0.022854 | [0.116615, 0.129285] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.187675 +/- 0.031189 | [0.179030, 0.196320] | 50 |
| +FID-FIA existence refinement | Cardinality | 0.063200 +/- 0.017041 | [0.058477, 0.067923] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | OSPA | 0.651060 +/- 0.199194 | [0.595846, 0.706274] | 26.37% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | OSPA | 0.689755 +/- 0.179418 | [0.640023, 0.739487] | 27.94% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | OSPA | 0.791449 +/- 0.202183 | [0.735406, 0.847491] | 32.06% | 50/50 | 1.776e-15 |
| Cao-Zhao FID-FIA baseline | RMSE | 0.683188 +/- 0.347189 | [0.586952, 0.779424] | 29.37% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.803843 +/- 0.347451 | [0.707535, 0.900152] | 34.56% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | RMSE | 0.779927 +/- 0.370518 | [0.677224, 0.882629] | 33.53% | 50/50 | 1.776e-15 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.593075 +/- 0.217069 | [0.532906, 0.653244] | 82.83% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.528350 +/- 0.210098 | [0.470114, 0.586586] | 73.79% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | Cardinality | 0.652825 +/- 0.222628 | [0.591116, 0.714534] | 91.17% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 52.122628 +/- 7.932188 | 0.521226 | 1.000x | 50 |
| Cao-Zhao FID-FIA baseline | 147.673755 +/- 23.956635 | 1.476738 | 2.836x | 50 |
| +structure-aware decoupled KLA | 56.378137 +/- 9.626476 | 0.563781 | 1.086x | 50 |
| +FID-FIA existence refinement | 155.913438 +/- 18.219890 | 1.559134 | 3.016x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| Cao-Zhao FID-FIA baseline | 95.551127 +/- 17.559898 | 183.63% | 50/50 |
| +structure-aware decoupled KLA | 4.255510 +/- 7.139929 | 8.58% | 48/50 |
| +FID-FIA existence refinement | 103.790811 +/- 13.775987 | 201.58% | 50/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 48.180762 | 0.481808 | 1.000x |
| 1 | 2 | Cao-Zhao FID-FIA baseline | 133.884603 | 1.338846 | 2.779x |
| 1 | 2 | +structure-aware decoupled KLA | 50.530234 | 0.505302 | 1.049x |
| 1 | 2 | +FID-FIA existence refinement | 138.590939 | 1.385909 | 2.876x |
| 2 | 3 | fixed weights | 47.056640 | 0.470566 | 1.000x |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 127.149981 | 1.271500 | 2.702x |
| 2 | 3 | +structure-aware decoupled KLA | 50.013014 | 0.500130 | 1.063x |
| 2 | 3 | +FID-FIA existence refinement | 141.647137 | 1.416471 | 3.010x |
| 3 | 4 | fixed weights | 49.229909 | 0.492299 | 1.000x |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 132.158571 | 1.321586 | 2.685x |
| 3 | 4 | +structure-aware decoupled KLA | 54.391890 | 0.543919 | 1.105x |
| 3 | 4 | +FID-FIA existence refinement | 163.716416 | 1.637164 | 3.326x |
| 4 | 5 | fixed weights | 54.593785 | 0.545938 | 1.000x |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 157.201584 | 1.572016 | 2.879x |
| 4 | 5 | +structure-aware decoupled KLA | 58.651075 | 0.586511 | 1.074x |
| 4 | 5 | +FID-FIA existence refinement | 170.694717 | 1.706947 | 3.127x |
| 5 | 6 | fixed weights | 46.235613 | 0.462356 | 1.000x |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 126.051956 | 1.260520 | 2.726x |
| 5 | 6 | +structure-aware decoupled KLA | 49.873433 | 0.498734 | 1.079x |
| 5 | 6 | +FID-FIA existence refinement | 135.064145 | 1.350641 | 2.921x |
| 6 | 7 | fixed weights | 45.038359 | 0.450384 | 1.000x |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 135.751659 | 1.357517 | 3.014x |
| 6 | 7 | +structure-aware decoupled KLA | 49.747175 | 0.497472 | 1.105x |
| 6 | 7 | +FID-FIA existence refinement | 151.895681 | 1.518957 | 3.373x |
| 7 | 8 | fixed weights | 48.281316 | 0.482813 | 1.000x |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 134.845799 | 1.348458 | 2.793x |
| 7 | 8 | +structure-aware decoupled KLA | 51.051692 | 0.510517 | 1.057x |
| 7 | 8 | +FID-FIA existence refinement | 146.270490 | 1.462705 | 3.030x |
| 8 | 9 | fixed weights | 47.918355 | 0.479184 | 1.000x |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 130.053249 | 1.300532 | 2.714x |
| 8 | 9 | +structure-aware decoupled KLA | 50.534029 | 0.505340 | 1.055x |
| 8 | 9 | +FID-FIA existence refinement | 137.976007 | 1.379760 | 2.879x |
| 9 | 10 | fixed weights | 46.924984 | 0.469250 | 1.000x |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 131.913005 | 1.319130 | 2.811x |
| 9 | 10 | +structure-aware decoupled KLA | 49.706295 | 0.497063 | 1.059x |
| 9 | 10 | +FID-FIA existence refinement | 149.581378 | 1.495814 | 3.188x |
| 10 | 11 | fixed weights | 46.960360 | 0.469604 | 1.000x |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 131.087447 | 1.310874 | 2.791x |
| 10 | 11 | +structure-aware decoupled KLA | 51.174102 | 0.511741 | 1.090x |
| 10 | 11 | +FID-FIA existence refinement | 140.132790 | 1.401328 | 2.984x |
| 11 | 12 | fixed weights | 47.951324 | 0.479513 | 1.000x |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 138.034804 | 1.380348 | 2.879x |
| 11 | 12 | +structure-aware decoupled KLA | 51.033366 | 0.510334 | 1.064x |
| 11 | 12 | +FID-FIA existence refinement | 159.036340 | 1.590363 | 3.317x |
| 12 | 13 | fixed weights | 48.891786 | 0.488918 | 1.000x |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 147.776117 | 1.477761 | 3.023x |
| 12 | 13 | +structure-aware decoupled KLA | 73.874056 | 0.738741 | 1.511x |
| 12 | 13 | +FID-FIA existence refinement | 152.750249 | 1.527502 | 3.124x |
| 13 | 14 | fixed weights | 48.796066 | 0.487961 | 1.000x |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 139.620393 | 1.396204 | 2.861x |
| 13 | 14 | +structure-aware decoupled KLA | 52.888766 | 0.528888 | 1.084x |
| 13 | 14 | +FID-FIA existence refinement | 158.578449 | 1.585784 | 3.250x |
| 14 | 15 | fixed weights | 47.401393 | 0.474014 | 1.000x |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 141.911856 | 1.419119 | 2.994x |
| 14 | 15 | +structure-aware decoupled KLA | 52.402631 | 0.524026 | 1.106x |
| 14 | 15 | +FID-FIA existence refinement | 152.037000 | 1.520370 | 3.207x |
| 15 | 16 | fixed weights | 54.566321 | 0.545663 | 1.000x |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 142.603006 | 1.426030 | 2.613x |
| 15 | 16 | +structure-aware decoupled KLA | 55.199955 | 0.552000 | 1.012x |
| 15 | 16 | +FID-FIA existence refinement | 152.701396 | 1.527014 | 2.798x |
| 16 | 17 | fixed weights | 49.983753 | 0.499838 | 1.000x |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 150.332296 | 1.503323 | 3.008x |
| 16 | 17 | +structure-aware decoupled KLA | 72.703427 | 0.727034 | 1.455x |
| 16 | 17 | +FID-FIA existence refinement | 198.149715 | 1.981497 | 3.964x |
| 17 | 18 | fixed weights | 77.772698 | 0.777727 | 1.000x |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 207.396214 | 2.073962 | 2.667x |
| 17 | 18 | +structure-aware decoupled KLA | 77.851189 | 0.778512 | 1.001x |
| 17 | 18 | +FID-FIA existence refinement | 214.982530 | 2.149825 | 2.764x |
| 18 | 19 | fixed weights | 69.683795 | 0.696838 | 1.000x |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 201.316384 | 2.013164 | 2.889x |
| 18 | 19 | +structure-aware decoupled KLA | 77.274270 | 0.772743 | 1.109x |
| 18 | 19 | +FID-FIA existence refinement | 209.698873 | 2.096989 | 3.009x |
| 19 | 20 | fixed weights | 70.786405 | 0.707864 | 1.000x |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 211.681192 | 2.116812 | 2.990x |
| 19 | 20 | +structure-aware decoupled KLA | 76.971723 | 0.769717 | 1.087x |
| 19 | 20 | +FID-FIA existence refinement | 203.215586 | 2.032156 | 2.871x |
| 20 | 21 | fixed weights | 72.470741 | 0.724707 | 1.000x |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 196.766771 | 1.967668 | 2.715x |
| 20 | 21 | +structure-aware decoupled KLA | 77.291799 | 0.772918 | 1.067x |
| 20 | 21 | +FID-FIA existence refinement | 167.623024 | 1.676230 | 2.313x |
| 21 | 22 | fixed weights | 47.375905 | 0.473759 | 1.000x |
| 21 | 22 | Cao-Zhao FID-FIA baseline | 135.751917 | 1.357519 | 2.865x |
| 21 | 22 | +structure-aware decoupled KLA | 51.556785 | 0.515568 | 1.088x |
| 21 | 22 | +FID-FIA existence refinement | 145.137678 | 1.451377 | 3.064x |
| 22 | 23 | fixed weights | 58.564163 | 0.585642 | 1.000x |
| 22 | 23 | Cao-Zhao FID-FIA baseline | 180.841142 | 1.808411 | 3.088x |
| 22 | 23 | +structure-aware decoupled KLA | 51.419893 | 0.514199 | 0.878x |
| 22 | 23 | +FID-FIA existence refinement | 144.782843 | 1.447828 | 2.472x |
| 23 | 24 | fixed weights | 49.458601 | 0.494586 | 1.000x |
| 23 | 24 | Cao-Zhao FID-FIA baseline | 136.247260 | 1.362473 | 2.755x |
| 23 | 24 | +structure-aware decoupled KLA | 51.448756 | 0.514488 | 1.040x |
| 23 | 24 | +FID-FIA existence refinement | 143.570035 | 1.435700 | 2.903x |
| 24 | 25 | fixed weights | 47.095919 | 0.470959 | 1.000x |
| 24 | 25 | Cao-Zhao FID-FIA baseline | 132.092171 | 1.320922 | 2.805x |
| 24 | 25 | +structure-aware decoupled KLA | 50.581923 | 0.505819 | 1.074x |
| 24 | 25 | +FID-FIA existence refinement | 146.590031 | 1.465900 | 3.113x |
| 25 | 26 | fixed weights | 49.370371 | 0.493704 | 1.000x |
| 25 | 26 | Cao-Zhao FID-FIA baseline | 141.480565 | 1.414806 | 2.866x |
| 25 | 26 | +structure-aware decoupled KLA | 52.260086 | 0.522601 | 1.059x |
| 25 | 26 | +FID-FIA existence refinement | 150.311752 | 1.503118 | 3.045x |
| 26 | 27 | fixed weights | 48.153870 | 0.481539 | 1.000x |
| 26 | 27 | Cao-Zhao FID-FIA baseline | 129.657863 | 1.296579 | 2.693x |
| 26 | 27 | +structure-aware decoupled KLA | 51.382066 | 0.513821 | 1.067x |
| 26 | 27 | +FID-FIA existence refinement | 137.400789 | 1.374008 | 2.853x |
| 27 | 28 | fixed weights | 48.050551 | 0.480506 | 1.000x |
| 27 | 28 | Cao-Zhao FID-FIA baseline | 135.330200 | 1.353302 | 2.816x |
| 27 | 28 | +structure-aware decoupled KLA | 51.909897 | 0.519099 | 1.080x |
| 27 | 28 | +FID-FIA existence refinement | 143.797686 | 1.437977 | 2.993x |
| 28 | 29 | fixed weights | 45.644340 | 0.456443 | 1.000x |
| 28 | 29 | Cao-Zhao FID-FIA baseline | 132.381724 | 1.323817 | 2.900x |
| 28 | 29 | +structure-aware decoupled KLA | 49.919341 | 0.499193 | 1.094x |
| 28 | 29 | +FID-FIA existence refinement | 143.392786 | 1.433928 | 3.142x |
| 29 | 30 | fixed weights | 47.979662 | 0.479797 | 1.000x |
| 29 | 30 | Cao-Zhao FID-FIA baseline | 136.264337 | 1.362643 | 2.840x |
| 29 | 30 | +structure-aware decoupled KLA | 51.854640 | 0.518546 | 1.081x |
| 29 | 30 | +FID-FIA existence refinement | 149.316282 | 1.493163 | 3.112x |
| 30 | 31 | fixed weights | 47.912601 | 0.479126 | 1.000x |
| 30 | 31 | Cao-Zhao FID-FIA baseline | 135.083330 | 1.350833 | 2.819x |
| 30 | 31 | +structure-aware decoupled KLA | 51.153931 | 0.511539 | 1.068x |
| 30 | 31 | +FID-FIA existence refinement | 148.347454 | 1.483475 | 3.096x |
| 31 | 32 | fixed weights | 49.699456 | 0.496995 | 1.000x |
| 31 | 32 | Cao-Zhao FID-FIA baseline | 134.849231 | 1.348492 | 2.713x |
| 31 | 32 | +structure-aware decoupled KLA | 54.267598 | 0.542676 | 1.092x |
| 31 | 32 | +FID-FIA existence refinement | 146.796032 | 1.467960 | 2.954x |
| 32 | 33 | fixed weights | 48.407405 | 0.484074 | 1.000x |
| 32 | 33 | Cao-Zhao FID-FIA baseline | 159.046193 | 1.590462 | 3.286x |
| 32 | 33 | +structure-aware decoupled KLA | 52.251566 | 0.522516 | 1.079x |
| 32 | 33 | +FID-FIA existence refinement | 145.537491 | 1.455375 | 3.007x |
| 33 | 34 | fixed weights | 47.322797 | 0.473228 | 1.000x |
| 33 | 34 | Cao-Zhao FID-FIA baseline | 130.424443 | 1.304244 | 2.756x |
| 33 | 34 | +structure-aware decoupled KLA | 50.979596 | 0.509796 | 1.077x |
| 33 | 34 | +FID-FIA existence refinement | 142.934535 | 1.429345 | 3.020x |
| 34 | 35 | fixed weights | 48.131603 | 0.481316 | 1.000x |
| 34 | 35 | Cao-Zhao FID-FIA baseline | 132.979524 | 1.329795 | 2.763x |
| 34 | 35 | +structure-aware decoupled KLA | 51.173946 | 0.511739 | 1.063x |
| 34 | 35 | +FID-FIA existence refinement | 149.471797 | 1.494718 | 3.105x |
| 35 | 36 | fixed weights | 52.738113 | 0.527381 | 1.000x |
| 35 | 36 | Cao-Zhao FID-FIA baseline | 146.509242 | 1.465092 | 2.778x |
| 35 | 36 | +structure-aware decoupled KLA | 53.666633 | 0.536666 | 1.018x |
| 35 | 36 | +FID-FIA existence refinement | 154.835046 | 1.548350 | 2.936x |
| 36 | 37 | fixed weights | 52.018754 | 0.520188 | 1.000x |
| 36 | 37 | Cao-Zhao FID-FIA baseline | 145.147272 | 1.451473 | 2.790x |
| 36 | 37 | +structure-aware decoupled KLA | 58.647601 | 0.586476 | 1.127x |
| 36 | 37 | +FID-FIA existence refinement | 145.069154 | 1.450692 | 2.789x |
| 37 | 38 | fixed weights | 47.348644 | 0.473486 | 1.000x |
| 37 | 38 | Cao-Zhao FID-FIA baseline | 143.957742 | 1.439577 | 3.040x |
| 37 | 38 | +structure-aware decoupled KLA | 52.901119 | 0.529011 | 1.117x |
| 37 | 38 | +FID-FIA existence refinement | 155.733213 | 1.557332 | 3.289x |
| 38 | 39 | fixed weights | 47.037504 | 0.470375 | 1.000x |
| 38 | 39 | Cao-Zhao FID-FIA baseline | 131.451040 | 1.314510 | 2.795x |
| 38 | 39 | +structure-aware decoupled KLA | 51.151761 | 0.511518 | 1.087x |
| 38 | 39 | +FID-FIA existence refinement | 149.544054 | 1.495441 | 3.179x |
| 39 | 40 | fixed weights | 73.768093 | 0.737681 | 1.000x |
| 39 | 40 | Cao-Zhao FID-FIA baseline | 171.868018 | 1.718680 | 2.330x |
| 39 | 40 | +structure-aware decoupled KLA | 51.783424 | 0.517834 | 0.702x |
| 39 | 40 | +FID-FIA existence refinement | 150.341236 | 1.503412 | 2.038x |
| 40 | 41 | fixed weights | 49.585819 | 0.495858 | 1.000x |
| 40 | 41 | Cao-Zhao FID-FIA baseline | 155.120637 | 1.551206 | 3.128x |
| 40 | 41 | +structure-aware decoupled KLA | 52.165036 | 0.521650 | 1.052x |
| 40 | 41 | +FID-FIA existence refinement | 140.585716 | 1.405857 | 2.835x |
| 41 | 42 | fixed weights | 47.874450 | 0.478744 | 1.000x |
| 41 | 42 | Cao-Zhao FID-FIA baseline | 135.087737 | 1.350877 | 2.822x |
| 41 | 42 | +structure-aware decoupled KLA | 51.634996 | 0.516350 | 1.079x |
| 41 | 42 | +FID-FIA existence refinement | 146.808843 | 1.468088 | 3.067x |
| 42 | 43 | fixed weights | 49.637099 | 0.496371 | 1.000x |
| 42 | 43 | Cao-Zhao FID-FIA baseline | 131.459730 | 1.314597 | 2.648x |
| 42 | 43 | +structure-aware decoupled KLA | 50.150287 | 0.501503 | 1.010x |
| 42 | 43 | +FID-FIA existence refinement | 142.566612 | 1.425666 | 2.872x |
| 43 | 44 | fixed weights | 47.511508 | 0.475115 | 1.000x |
| 43 | 44 | Cao-Zhao FID-FIA baseline | 134.433364 | 1.344334 | 2.829x |
| 43 | 44 | +structure-aware decoupled KLA | 50.604544 | 0.506045 | 1.065x |
| 43 | 44 | +FID-FIA existence refinement | 159.879534 | 1.598795 | 3.365x |
| 44 | 45 | fixed weights | 58.343851 | 0.583439 | 1.000x |
| 44 | 45 | Cao-Zhao FID-FIA baseline | 155.445150 | 1.554451 | 2.664x |
| 44 | 45 | +structure-aware decoupled KLA | 60.886045 | 0.608860 | 1.044x |
| 44 | 45 | +FID-FIA existence refinement | 172.153542 | 1.721535 | 2.951x |
| 45 | 46 | fixed weights | 56.499437 | 0.564994 | 1.000x |
| 45 | 46 | Cao-Zhao FID-FIA baseline | 150.511819 | 1.505118 | 2.664x |
| 45 | 46 | +structure-aware decoupled KLA | 60.560859 | 0.605609 | 1.072x |
| 45 | 46 | +FID-FIA existence refinement | 168.840743 | 1.688407 | 2.988x |
| 46 | 47 | fixed weights | 60.015086 | 0.600151 | 1.000x |
| 46 | 47 | Cao-Zhao FID-FIA baseline | 167.792367 | 1.677924 | 2.796x |
| 46 | 47 | +structure-aware decoupled KLA | 63.310346 | 0.633103 | 1.055x |
| 46 | 47 | +FID-FIA existence refinement | 176.282743 | 1.762827 | 2.937x |
| 47 | 48 | fixed weights | 59.735702 | 0.597357 | 1.000x |
| 47 | 48 | Cao-Zhao FID-FIA baseline | 227.381429 | 2.273814 | 3.806x |
| 47 | 48 | +structure-aware decoupled KLA | 91.156839 | 0.911568 | 1.526x |
| 47 | 48 | +FID-FIA existence refinement | 174.155459 | 1.741555 | 2.915x |
| 48 | 49 | fixed weights | 55.181761 | 0.551818 | 1.000x |
| 48 | 49 | Cao-Zhao FID-FIA baseline | 154.873763 | 1.548738 | 2.807x |
| 48 | 49 | +structure-aware decoupled KLA | 55.513337 | 0.555133 | 1.006x |
| 48 | 49 | +FID-FIA existence refinement | 169.399121 | 1.693991 | 3.070x |
| 49 | 50 | fixed weights | 47.926964 | 0.479270 | 1.000x |
| 49 | 50 | Cao-Zhao FID-FIA baseline | 130.559859 | 1.305599 | 2.724x |
| 49 | 50 | +structure-aware decoupled KLA | 58.257853 | 0.582579 | 1.216x |
| 49 | 50 | +FID-FIA existence refinement | 155.684333 | 1.556843 | 3.248x |
| 50 | 51 | fixed weights | 49.016995 | 0.490170 | 1.000x |
| 50 | 51 | Cao-Zhao FID-FIA baseline | 132.121800 | 1.321218 | 2.695x |
| 50 | 51 | +structure-aware decoupled KLA | 51.192050 | 0.511920 | 1.044x |
| 50 | 51 | +FID-FIA existence refinement | 152.062521 | 1.520625 | 3.102x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.862938 | 1.649569 | 1.455125 |
| Cao-Zhao FID-FIA baseline | 2.184698 | 1.734381 | 0.388050 |
| +structure-aware decoupled KLA | 2.334915 | 1.605910 | 0.578775 |
| +FID-FIA existence refinement | 2.019842 | 1.720931 | 0.223700 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.862938 +/- 0.123917 | [2.828590, 2.897286] | 50 |
| Cao-Zhao FID-FIA baseline | E-OSPA | 2.184698 +/- 0.050316 | [2.170751, 2.198645] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 2.334915 +/- 0.071651 | [2.315054, 2.354776] | 50 |
| +FID-FIA existence refinement | E-OSPA | 2.019842 +/- 0.047164 | [2.006769, 2.032915] | 50 |
| fixed weights | RMSE | 1.649569 +/- 0.078096 | [1.627922, 1.671216] | 50 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.734381 +/- 0.094493 | [1.708189, 1.760573] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.605910 +/- 0.049506 | [1.592188, 1.619633] | 50 |
| +FID-FIA existence refinement | RMSE | 1.720931 +/- 0.161413 | [1.676189, 1.765672] | 50 |
| fixed weights | CardErr | 1.455125 +/- 0.241978 | [1.388052, 1.522198] | 50 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.388050 +/- 0.046955 | [0.375035, 0.401065] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.578775 +/- 0.066558 | [0.560326, 0.597224] | 50 |
| +FID-FIA existence refinement | CardErr | 0.223700 +/- 0.029320 | [0.215573, 0.231827] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | E-OSPA | 0.678240 +/- 0.109771 | [0.647813, 0.708667] | 23.69% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | E-OSPA | 0.528023 +/- 0.089961 | [0.503087, 0.552959] | 18.44% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | E-OSPA | 0.843096 +/- 0.113871 | [0.811532, 0.874660] | 29.45% | 50/50 | 1.776e-15 |
| Cao-Zhao FID-FIA baseline | RMSE | -0.084812 +/- 0.106093 | [-0.114219, -0.055404] | -5.14% | 4/50 | 4.462e-10 |
| +structure-aware decoupled KLA | RMSE | 0.043659 +/- 0.059480 | [0.027172, 0.060146] | 2.65% | 43/50 | 2.099e-07 |
| +FID-FIA existence refinement | RMSE | -0.071362 +/- 0.170415 | [-0.118598, -0.024125] | -4.33% | 16/50 | 0.01535 |
| Cao-Zhao FID-FIA baseline | CardErr | 1.067075 +/- 0.223978 | [1.004991, 1.129159] | 73.33% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | CardErr | 0.876350 +/- 0.211610 | [0.817695, 0.935005] | 60.23% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | CardErr | 1.231425 +/- 0.239992 | [1.164903, 1.297947] | 84.63% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.719987 | 1.619776 | 1.083600 |
| 1 | Cao-Zhao FID-FIA baseline | 2.131489 | 1.673484 | 0.346200 |
| 1 | +structure-aware decoupled KLA | 2.289633 | 1.559677 | 0.542000 |
| 1 | +FID-FIA existence refinement | 1.982109 | 1.642265 | 0.227800 |
| 2 | fixed weights | 2.670351 | 1.594302 | 0.999200 |
| 2 | Cao-Zhao FID-FIA baseline | 2.149580 | 1.708237 | 0.362000 |
| 2 | +structure-aware decoupled KLA | 2.297153 | 1.572246 | 0.546000 |
| 2 | +FID-FIA existence refinement | 1.989426 | 1.728724 | 0.226000 |
| 3 | fixed weights | 2.740374 | 1.607566 | 1.150400 |
| 3 | Cao-Zhao FID-FIA baseline | 2.164549 | 1.733686 | 0.389600 |
| 3 | +structure-aware decoupled KLA | 2.304960 | 1.573270 | 0.573600 |
| 3 | +FID-FIA existence refinement | 1.983219 | 1.673551 | 0.226600 |
| 4 | fixed weights | 2.883255 | 1.654573 | 1.494400 |
| 4 | Cao-Zhao FID-FIA baseline | 2.261465 | 1.787437 | 0.506000 |
| 4 | +structure-aware decoupled KLA | 2.413320 | 1.639044 | 0.681200 |
| 4 | +FID-FIA existence refinement | 2.061510 | 1.779754 | 0.248000 |
| 5 | fixed weights | 2.918523 | 1.691261 | 1.520000 |
| 5 | Cao-Zhao FID-FIA baseline | 2.218486 | 1.761091 | 0.414800 |
| 5 | +structure-aware decoupled KLA | 2.375721 | 1.635650 | 0.619400 |
| 5 | +FID-FIA existence refinement | 2.048466 | 1.766530 | 0.226000 |
| 6 | fixed weights | 2.913753 | 1.708755 | 1.535200 |
| 6 | Cao-Zhao FID-FIA baseline | 2.177567 | 1.727532 | 0.348000 |
| 6 | +structure-aware decoupled KLA | 2.327530 | 1.618789 | 0.542600 |
| 6 | +FID-FIA existence refinement | 2.029988 | 1.694478 | 0.208400 |
| 7 | fixed weights | 2.925998 | 1.663623 | 1.565800 |
| 7 | Cao-Zhao FID-FIA baseline | 2.209635 | 1.767064 | 0.396800 |
| 7 | +structure-aware decoupled KLA | 2.370662 | 1.633665 | 0.605400 |
| 7 | +FID-FIA existence refinement | 2.041801 | 1.731545 | 0.222000 |
| 8 | fixed weights | 3.131265 | 1.656698 | 2.292400 |
| 8 | Cao-Zhao FID-FIA baseline | 2.164811 | 1.716518 | 0.341000 |
| 8 | +structure-aware decoupled KLA | 2.300340 | 1.614939 | 0.520000 |
| 8 | +FID-FIA existence refinement | 2.022219 | 1.750601 | 0.204800 |
