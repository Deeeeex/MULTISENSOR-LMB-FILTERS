# GA Tiered Link Ablation (2026-05-28 05:16:36)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA

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

- finalArmMode: structureAware

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

### +covariance
- enabled: 1
- method: factorized
- useCovariance: 1
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

### +existence confidence
- enabled: 1
- method: factorized
- useCovariance: 1
- useLinkQuality: 1
- useExistenceConfidence: 1
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
| 1 | 2 | +covariance | 2.256340 | 1.835282 | 0.663750 |
| 1 | 2 | +link quality | 1.894149 | 1.454645 | 0.245000 |
| 1 | 2 | +existence confidence | 1.898974 | 1.505315 | 0.245000 |
| 1 | 2 | +structure-aware decoupled KLA | 1.878234 | 1.439986 | 0.240000 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | +covariance | 2.250649 | 2.047509 | 0.613750 |
| 2 | 3 | +link quality | 1.867780 | 1.387814 | 0.195000 |
| 2 | 3 | +existence confidence | 1.881834 | 1.380769 | 0.196250 |
| 2 | 3 | +structure-aware decoupled KLA | 1.854447 | 1.371532 | 0.191250 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | +covariance | 2.171057 | 1.775920 | 0.582500 |
| 3 | 4 | +link quality | 1.825910 | 1.438299 | 0.220000 |
| 3 | 4 | +existence confidence | 1.818312 | 1.428127 | 0.218750 |
| 3 | 4 | +structure-aware decoupled KLA | 1.808801 | 1.431204 | 0.221250 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | +covariance | 1.999058 | 2.768107 | 0.352500 |
| 4 | 5 | +link quality | 1.748949 | 1.913620 | 0.175000 |
| 4 | 5 | +existence confidence | 1.749996 | 1.932214 | 0.173750 |
| 4 | 5 | +structure-aware decoupled KLA | 1.736420 | 1.923505 | 0.170000 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | +covariance | 2.381920 | 2.160746 | 0.878750 |
| 5 | 6 | +link quality | 1.872465 | 1.868450 | 0.246250 |
| 5 | 6 | +existence confidence | 1.868894 | 1.826150 | 0.246250 |
| 5 | 6 | +structure-aware decoupled KLA | 1.852532 | 1.777241 | 0.243750 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | +covariance | 2.130087 | 1.983686 | 0.480000 |
| 6 | 7 | +link quality | 1.861180 | 1.788151 | 0.198750 |
| 6 | 7 | +existence confidence | 1.841840 | 1.757509 | 0.195000 |
| 6 | 7 | +structure-aware decoupled KLA | 1.839920 | 1.764960 | 0.193750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | +covariance | 1.997424 | 1.786795 | 0.383750 |
| 7 | 8 | +link quality | 1.720911 | 1.373876 | 0.168750 |
| 7 | 8 | +existence confidence | 1.740739 | 1.371951 | 0.163750 |
| 7 | 8 | +structure-aware decoupled KLA | 1.724528 | 1.356549 | 0.161250 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | +covariance | 1.971638 | 1.988356 | 0.366250 |
| 8 | 9 | +link quality | 1.755763 | 1.466729 | 0.188750 |
| 8 | 9 | +existence confidence | 1.761537 | 1.462152 | 0.181250 |
| 8 | 9 | +structure-aware decoupled KLA | 1.732578 | 1.434821 | 0.177500 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | +covariance | 2.240500 | 1.859515 | 0.631250 |
| 9 | 10 | +link quality | 1.812550 | 2.095538 | 0.178750 |
| 9 | 10 | +existence confidence | 1.819496 | 2.120905 | 0.180000 |
| 9 | 10 | +structure-aware decoupled KLA | 1.798485 | 2.148753 | 0.180000 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | +covariance | 1.936075 | 1.936313 | 0.357500 |
| 10 | 11 | +link quality | 1.772281 | 1.481958 | 0.176250 |
| 10 | 11 | +existence confidence | 1.775678 | 1.480443 | 0.175000 |
| 10 | 11 | +structure-aware decoupled KLA | 1.787456 | 1.493683 | 0.182500 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | +covariance | 2.028315 | 1.907117 | 0.447500 |
| 11 | 12 | +link quality | 1.760701 | 1.535206 | 0.173750 |
| 11 | 12 | +existence confidence | 1.760747 | 1.563510 | 0.167500 |
| 11 | 12 | +structure-aware decoupled KLA | 1.753632 | 1.572377 | 0.171250 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | +covariance | 1.938693 | 1.584191 | 0.367500 |
| 12 | 13 | +link quality | 1.724244 | 1.390455 | 0.190000 |
| 12 | 13 | +existence confidence | 1.723185 | 1.436199 | 0.186250 |
| 12 | 13 | +structure-aware decoupled KLA | 1.742848 | 1.486898 | 0.197500 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | +covariance | 2.071381 | 2.219613 | 0.393750 |
| 13 | 14 | +link quality | 1.803248 | 1.467818 | 0.183750 |
| 13 | 14 | +existence confidence | 1.800903 | 1.472665 | 0.182500 |
| 13 | 14 | +structure-aware decoupled KLA | 1.790615 | 1.451047 | 0.182500 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | +covariance | 1.947294 | 1.644533 | 0.307500 |
| 14 | 15 | +link quality | 1.777916 | 1.475596 | 0.188750 |
| 14 | 15 | +existence confidence | 1.772918 | 1.476034 | 0.187500 |
| 14 | 15 | +structure-aware decoupled KLA | 1.772452 | 1.466171 | 0.191250 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | +covariance | 1.892142 | 1.980432 | 0.292500 |
| 15 | 16 | +link quality | 1.729686 | 1.524912 | 0.187500 |
| 15 | 16 | +existence confidence | 1.718018 | 1.489085 | 0.185000 |
| 15 | 16 | +structure-aware decoupled KLA | 1.704826 | 1.517027 | 0.176250 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | +covariance | 2.094888 | 1.852252 | 0.457500 |
| 16 | 17 | +link quality | 1.816784 | 1.450195 | 0.223750 |
| 16 | 17 | +existence confidence | 1.839367 | 1.425175 | 0.226250 |
| 16 | 17 | +structure-aware decoupled KLA | 1.830609 | 1.438554 | 0.228750 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | +covariance | 2.252858 | 1.989501 | 0.591250 |
| 17 | 18 | +link quality | 1.871102 | 1.664706 | 0.213750 |
| 17 | 18 | +existence confidence | 1.870496 | 1.675677 | 0.210000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.863847 | 1.652217 | 0.216250 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | +covariance | 1.958102 | 1.887362 | 0.430000 |
| 18 | 19 | +link quality | 1.690478 | 1.467031 | 0.163750 |
| 18 | 19 | +existence confidence | 1.680623 | 1.498007 | 0.161250 |
| 18 | 19 | +structure-aware decoupled KLA | 1.673306 | 1.510607 | 0.161250 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | +covariance | 2.133150 | 1.659939 | 0.527500 |
| 19 | 20 | +link quality | 1.825640 | 1.482957 | 0.200000 |
| 19 | 20 | +existence confidence | 1.800486 | 1.461269 | 0.195000 |
| 19 | 20 | +structure-aware decoupled KLA | 1.812300 | 1.473501 | 0.197500 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | +covariance | 2.142521 | 2.085485 | 0.483750 |
| 20 | 21 | +link quality | 1.770289 | 1.489944 | 0.176250 |
| 20 | 21 | +existence confidence | 1.770796 | 1.545295 | 0.180000 |
| 20 | 21 | +structure-aware decoupled KLA | 1.759623 | 1.539784 | 0.175000 |
| 21 | 22 | fixed weights | 2.445386 | 2.063249 | 0.706250 |
| 21 | 22 | +covariance | 1.978431 | 1.667781 | 0.361250 |
| 21 | 22 | +link quality | 1.789513 | 1.553570 | 0.180000 |
| 21 | 22 | +existence confidence | 1.820807 | 1.736132 | 0.190000 |
| 21 | 22 | +structure-aware decoupled KLA | 1.771566 | 1.591554 | 0.178750 |
| 22 | 23 | fixed weights | 2.782054 | 2.774744 | 0.917500 |
| 22 | 23 | +covariance | 2.330270 | 2.027310 | 0.550000 |
| 22 | 23 | +link quality | 1.856143 | 1.440447 | 0.211250 |
| 22 | 23 | +existence confidence | 1.818080 | 1.412198 | 0.207500 |
| 22 | 23 | +structure-aware decoupled KLA | 1.828140 | 1.410398 | 0.208750 |
| 23 | 24 | fixed weights | 2.811135 | 1.942125 | 1.156250 |
| 23 | 24 | +covariance | 2.276285 | 1.916957 | 0.637500 |
| 23 | 24 | +link quality | 1.810456 | 1.501741 | 0.176250 |
| 23 | 24 | +existence confidence | 1.805832 | 1.498926 | 0.171250 |
| 23 | 24 | +structure-aware decoupled KLA | 1.793788 | 1.500075 | 0.176250 |
| 24 | 25 | fixed weights | 2.569606 | 2.368173 | 0.711250 |
| 24 | 25 | +covariance | 2.178258 | 2.022457 | 0.480000 |
| 24 | 25 | +link quality | 1.879064 | 1.513682 | 0.220000 |
| 24 | 25 | +existence confidence | 1.890222 | 1.515365 | 0.221250 |
| 24 | 25 | +structure-aware decoupled KLA | 1.875143 | 1.500632 | 0.221250 |
| 25 | 26 | fixed weights | 2.225426 | 2.278298 | 0.403750 |
| 25 | 26 | +covariance | 2.057762 | 1.901565 | 0.391250 |
| 25 | 26 | +link quality | 1.733723 | 1.395662 | 0.183750 |
| 25 | 26 | +existence confidence | 1.752516 | 1.348083 | 0.183750 |
| 25 | 26 | +structure-aware decoupled KLA | 1.717305 | 1.374395 | 0.173750 |
| 26 | 27 | fixed weights | 2.159231 | 1.899203 | 0.517500 |
| 26 | 27 | +covariance | 1.918019 | 1.543218 | 0.388750 |
| 26 | 27 | +link quality | 1.661765 | 1.296388 | 0.148750 |
| 26 | 27 | +existence confidence | 1.645385 | 1.372653 | 0.138750 |
| 26 | 27 | +structure-aware decoupled KLA | 1.646146 | 1.363300 | 0.142500 |
| 27 | 28 | fixed weights | 2.126003 | 1.944541 | 0.406250 |
| 27 | 28 | +covariance | 1.907666 | 1.792025 | 0.302500 |
| 27 | 28 | +link quality | 1.735363 | 1.381318 | 0.208750 |
| 27 | 28 | +existence confidence | 1.754866 | 1.442559 | 0.211250 |
| 27 | 28 | +structure-aware decoupled KLA | 1.736613 | 1.421164 | 0.208750 |
| 28 | 29 | fixed weights | 2.917753 | 1.994773 | 1.183750 |
| 28 | 29 | +covariance | 2.286019 | 1.687519 | 0.767500 |
| 28 | 29 | +link quality | 1.886254 | 1.670214 | 0.270000 |
| 28 | 29 | +existence confidence | 1.887608 | 1.670292 | 0.266250 |
| 28 | 29 | +structure-aware decoupled KLA | 1.873211 | 1.616359 | 0.268750 |
| 29 | 30 | fixed weights | 2.600417 | 1.944603 | 1.000000 |
| 29 | 30 | +covariance | 2.219677 | 1.916290 | 0.642500 |
| 29 | 30 | +link quality | 1.877042 | 1.641901 | 0.227500 |
| 29 | 30 | +existence confidence | 1.872650 | 1.638620 | 0.223750 |
| 29 | 30 | +structure-aware decoupled KLA | 1.870636 | 1.691933 | 0.227500 |
| 30 | 31 | fixed weights | 2.468399 | 2.109218 | 0.737500 |
| 30 | 31 | +covariance | 2.155426 | 2.073726 | 0.460000 |
| 30 | 31 | +link quality | 1.784876 | 1.482583 | 0.162500 |
| 30 | 31 | +existence confidence | 1.790957 | 1.495661 | 0.163750 |
| 30 | 31 | +structure-aware decoupled KLA | 1.770363 | 1.471423 | 0.161250 |
| 31 | 32 | fixed weights | 2.513279 | 1.885047 | 0.721250 |
| 31 | 32 | +covariance | 2.100438 | 1.921725 | 0.401250 |
| 31 | 32 | +link quality | 1.756308 | 1.469851 | 0.170000 |
| 31 | 32 | +existence confidence | 1.765873 | 1.473782 | 0.167500 |
| 31 | 32 | +structure-aware decoupled KLA | 1.746697 | 1.452005 | 0.170000 |
| 32 | 33 | fixed weights | 2.019727 | 1.812822 | 0.302500 |
| 32 | 33 | +covariance | 1.720321 | 1.474372 | 0.191250 |
| 32 | 33 | +link quality | 1.698242 | 1.420990 | 0.162500 |
| 32 | 33 | +existence confidence | 1.684057 | 1.418988 | 0.163750 |
| 32 | 33 | +structure-aware decoupled KLA | 1.679034 | 1.408664 | 0.165000 |
| 33 | 34 | fixed weights | 2.490122 | 2.173185 | 0.962500 |
| 33 | 34 | +covariance | 2.050168 | 1.978783 | 0.586250 |
| 33 | 34 | +link quality | 1.693889 | 1.581188 | 0.187500 |
| 33 | 34 | +existence confidence | 1.701316 | 1.570137 | 0.185000 |
| 33 | 34 | +structure-aware decoupled KLA | 1.674970 | 1.535775 | 0.183750 |
| 34 | 35 | fixed weights | 2.915586 | 2.674676 | 1.071250 |
| 34 | 35 | +covariance | 2.329375 | 1.779998 | 0.666250 |
| 34 | 35 | +link quality | 1.870021 | 1.490107 | 0.186250 |
| 34 | 35 | +existence confidence | 1.864613 | 1.477854 | 0.182500 |
| 34 | 35 | +structure-aware decoupled KLA | 1.864837 | 1.559731 | 0.185000 |
| 35 | 36 | fixed weights | 2.381312 | 2.282297 | 0.561250 |
| 35 | 36 | +covariance | 1.982301 | 1.678594 | 0.373750 |
| 35 | 36 | +link quality | 1.698205 | 1.454358 | 0.137500 |
| 35 | 36 | +existence confidence | 1.699239 | 1.449212 | 0.140000 |
| 35 | 36 | +structure-aware decoupled KLA | 1.692204 | 1.449569 | 0.136250 |
| 36 | 37 | fixed weights | 2.794401 | 3.053361 | 0.971250 |
| 36 | 37 | +covariance | 2.348293 | 2.697460 | 0.602500 |
| 36 | 37 | +link quality | 1.858989 | 1.497741 | 0.216250 |
| 36 | 37 | +existence confidence | 1.855797 | 1.507012 | 0.212500 |
| 36 | 37 | +structure-aware decoupled KLA | 1.844239 | 1.497974 | 0.215000 |
| 37 | 38 | fixed weights | 2.488304 | 2.240250 | 0.821250 |
| 37 | 38 | +covariance | 2.020268 | 2.159954 | 0.396250 |
| 37 | 38 | +link quality | 1.717295 | 1.543967 | 0.147500 |
| 37 | 38 | +existence confidence | 1.730559 | 1.545827 | 0.155000 |
| 37 | 38 | +structure-aware decoupled KLA | 1.697548 | 1.538374 | 0.148750 |
| 38 | 39 | fixed weights | 2.292857 | 2.023920 | 0.538750 |
| 38 | 39 | +covariance | 2.014115 | 2.029145 | 0.383750 |
| 38 | 39 | +link quality | 1.823606 | 1.486187 | 0.177500 |
| 38 | 39 | +existence confidence | 1.820957 | 1.488461 | 0.173750 |
| 38 | 39 | +structure-aware decoupled KLA | 1.826976 | 1.463676 | 0.183750 |
| 39 | 40 | fixed weights | 2.535165 | 2.535969 | 0.646250 |
| 39 | 40 | +covariance | 2.081385 | 1.743317 | 0.407500 |
| 39 | 40 | +link quality | 1.815326 | 1.744380 | 0.205000 |
| 39 | 40 | +existence confidence | 1.808320 | 1.694099 | 0.207500 |
| 39 | 40 | +structure-aware decoupled KLA | 1.796344 | 1.662470 | 0.206250 |
| 40 | 41 | fixed weights | 2.257174 | 2.481788 | 0.530000 |
| 40 | 41 | +covariance | 2.010449 | 2.118700 | 0.421250 |
| 40 | 41 | +link quality | 1.789400 | 1.580266 | 0.206250 |
| 40 | 41 | +existence confidence | 1.799584 | 1.566194 | 0.212500 |
| 40 | 41 | +structure-aware decoupled KLA | 1.780970 | 1.560634 | 0.201250 |
| 41 | 42 | fixed weights | 2.094739 | 2.021235 | 0.453750 |
| 41 | 42 | +covariance | 1.854092 | 1.827047 | 0.297500 |
| 41 | 42 | +link quality | 1.591801 | 1.288719 | 0.145000 |
| 41 | 42 | +existence confidence | 1.593716 | 1.288881 | 0.148750 |
| 41 | 42 | +structure-aware decoupled KLA | 1.593887 | 1.271218 | 0.158750 |
| 42 | 43 | fixed weights | 2.470491 | 2.757701 | 0.583750 |
| 42 | 43 | +covariance | 2.101099 | 1.866587 | 0.396250 |
| 42 | 43 | +link quality | 1.724105 | 1.444278 | 0.095000 |
| 42 | 43 | +existence confidence | 1.725713 | 1.429180 | 0.093750 |
| 42 | 43 | +structure-aware decoupled KLA | 1.717249 | 1.437860 | 0.095000 |
| 43 | 44 | fixed weights | 2.550641 | 2.146021 | 0.801250 |
| 43 | 44 | +covariance | 2.137172 | 2.342524 | 0.412500 |
| 43 | 44 | +link quality | 1.812060 | 1.405628 | 0.182500 |
| 43 | 44 | +existence confidence | 1.815124 | 1.411813 | 0.181250 |
| 43 | 44 | +structure-aware decoupled KLA | 1.809049 | 1.398760 | 0.183750 |
| 44 | 45 | fixed weights | 2.216546 | 2.738044 | 0.428750 |
| 44 | 45 | +covariance | 1.948670 | 2.079577 | 0.368750 |
| 44 | 45 | +link quality | 1.721599 | 1.442912 | 0.158750 |
| 44 | 45 | +existence confidence | 1.715353 | 1.404916 | 0.160000 |
| 44 | 45 | +structure-aware decoupled KLA | 1.710189 | 1.388926 | 0.165000 |
| 45 | 46 | fixed weights | 2.537339 | 2.539290 | 0.686250 |
| 45 | 46 | +covariance | 2.003436 | 2.104338 | 0.381250 |
| 45 | 46 | +link quality | 1.746446 | 1.436876 | 0.172500 |
| 45 | 46 | +existence confidence | 1.756148 | 1.439061 | 0.172500 |
| 45 | 46 | +structure-aware decoupled KLA | 1.745958 | 1.425635 | 0.173750 |
| 46 | 47 | fixed weights | 2.665038 | 2.010381 | 0.693750 |
| 46 | 47 | +covariance | 2.072833 | 1.802736 | 0.347500 |
| 46 | 47 | +link quality | 1.850503 | 1.729966 | 0.158750 |
| 46 | 47 | +existence confidence | 1.899384 | 1.836969 | 0.168750 |
| 46 | 47 | +structure-aware decoupled KLA | 1.871566 | 1.802479 | 0.161250 |
| 47 | 48 | fixed weights | 2.238065 | 2.249780 | 0.577500 |
| 47 | 48 | +covariance | 2.083497 | 1.891447 | 0.476250 |
| 47 | 48 | +link quality | 1.759599 | 1.378705 | 0.163750 |
| 47 | 48 | +existence confidence | 1.769180 | 1.378451 | 0.168750 |
| 47 | 48 | +structure-aware decoupled KLA | 1.751635 | 1.357804 | 0.170000 |
| 48 | 49 | fixed weights | 2.529967 | 3.164643 | 0.665000 |
| 48 | 49 | +covariance | 2.260764 | 2.464590 | 0.536250 |
| 48 | 49 | +link quality | 1.901177 | 1.678296 | 0.230000 |
| 48 | 49 | +existence confidence | 1.934744 | 1.742508 | 0.237500 |
| 48 | 49 | +structure-aware decoupled KLA | 1.917595 | 1.648404 | 0.238750 |
| 49 | 50 | fixed weights | 2.564502 | 2.102492 | 0.951250 |
| 49 | 50 | +covariance | 2.067135 | 2.100314 | 0.452500 |
| 49 | 50 | +link quality | 1.740913 | 1.432767 | 0.188750 |
| 49 | 50 | +existence confidence | 1.765561 | 1.485071 | 0.193750 |
| 49 | 50 | +structure-aware decoupled KLA | 1.726094 | 1.431585 | 0.183750 |
| 50 | 51 | fixed weights | 2.714437 | 3.379717 | 0.801250 |
| 50 | 51 | +covariance | 2.244713 | 1.981760 | 0.603750 |
| 50 | 51 | +link quality | 1.916190 | 1.647148 | 0.233750 |
| 50 | 51 | +existence confidence | 1.922974 | 1.658589 | 0.235000 |
| 50 | 51 | +structure-aware decoupled KLA | 1.913492 | 1.626347 | 0.232500 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.468973 | 2.326034 | 0.716025 |
| +covariance | 2.090649 | 1.950889 | 0.465850 |
| +link quality | 1.788038 | 1.524995 | 0.188150 |
| +existence confidence | 1.791239 | 1.534119 | 0.187875 |
| +structure-aware decoupled KLA | 1.779218 | 1.522191 | 0.187675 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.468973 +/- 0.219981 | [2.407997, 2.529949] | 50 |
| +covariance | OSPA | 2.090649 +/- 0.144677 | [2.050546, 2.130751] | 50 |
| +link quality | OSPA | 1.788038 +/- 0.071415 | [1.768243, 1.807833] | 50 |
| +existence confidence | OSPA | 1.791239 +/- 0.073462 | [1.770877, 1.811602] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.779218 +/- 0.072856 | [1.759023, 1.799413] | 50 |
| fixed weights | RMSE | 2.326034 +/- 0.353065 | [2.228170, 2.423899] | 50 |
| +covariance | RMSE | 1.950889 +/- 0.252957 | [1.880773, 2.021005] | 50 |
| +link quality | RMSE | 1.524995 +/- 0.155955 | [1.481766, 1.568223] | 50 |
| +existence confidence | RMSE | 1.534119 +/- 0.161038 | [1.489481, 1.578756] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.522191 +/- 0.157069 | [1.478654, 1.565728] | 50 |
| fixed weights | Cardinality | 0.716025 +/- 0.223795 | [0.653992, 0.778058] | 50 |
| +covariance | Cardinality | 0.465850 +/- 0.134228 | [0.428644, 0.503056] | 50 |
| +link quality | Cardinality | 0.188150 +/- 0.031280 | [0.179480, 0.196820] | 50 |
| +existence confidence | Cardinality | 0.187875 +/- 0.031268 | [0.179208, 0.196542] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.187675 +/- 0.031189 | [0.179030, 0.196320] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.378324 +/- 0.118509 | [0.345475, 0.411174] | 15.32% | 50/50 | 1.776e-15 |
| +link quality | OSPA | 0.680935 +/- 0.177344 | [0.631778, 0.730092] | 27.58% | 50/50 | 1.776e-15 |
| +existence confidence | OSPA | 0.677733 +/- 0.178025 | [0.628387, 0.727080] | 27.45% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | OSPA | 0.689755 +/- 0.179418 | [0.640023, 0.739487] | 27.94% | 50/50 | 1.776e-15 |
| +covariance | RMSE | 0.375145 +/- 0.307793 | [0.289829, 0.460461] | 16.13% | 45/50 | 4.21e-09 |
| +link quality | RMSE | 0.801039 +/- 0.338775 | [0.707136, 0.894943] | 34.44% | 50/50 | 1.776e-15 |
| +existence confidence | RMSE | 0.791915 +/- 0.348480 | [0.695321, 0.888509] | 34.05% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | RMSE | 0.803843 +/- 0.347451 | [0.707535, 0.900152] | 34.56% | 50/50 | 1.776e-15 |
| +covariance | Cardinality | 0.250175 +/- 0.127169 | [0.214926, 0.285424] | 34.94% | 50/50 | 1.776e-15 |
| +link quality | Cardinality | 0.527875 +/- 0.209468 | [0.469813, 0.585937] | 73.72% | 50/50 | 1.776e-15 |
| +existence confidence | Cardinality | 0.528150 +/- 0.210173 | [0.469893, 0.586407] | 73.76% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | Cardinality | 0.528350 +/- 0.210098 | [0.470114, 0.586586] | 73.79% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 51.556826 +/- 6.590342 | 0.515568 | 1.000x | 50 |
| +covariance | 54.258784 +/- 4.891548 | 0.542588 | 1.059x | 50 |
| +link quality | 52.574732 +/- 2.068191 | 0.525747 | 1.031x | 50 |
| +existence confidence | 54.417897 +/- 10.175745 | 0.544179 | 1.061x | 50 |
| +structure-aware decoupled KLA | 54.800427 +/- 8.005700 | 0.548004 | 1.077x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +covariance | 2.701958 +/- 4.937398 | 5.93% | 45/50 |
| +link quality | 1.017906 +/- 5.917394 | 3.06% | 44/50 |
| +existence confidence | 2.861071 +/- 9.554557 | 6.10% | 43/50 |
| +structure-aware decoupled KLA | 3.243600 +/- 10.697324 | 7.72% | 44/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 47.611748 | 0.476117 | 1.000x |
| 1 | 2 | +covariance | 52.068949 | 0.520689 | 1.094x |
| 1 | 2 | +link quality | 50.183478 | 0.501835 | 1.054x |
| 1 | 2 | +existence confidence | 50.089644 | 0.500896 | 1.052x |
| 1 | 2 | +structure-aware decoupled KLA | 50.733887 | 0.507339 | 1.066x |
| 2 | 3 | fixed weights | 47.016092 | 0.470161 | 1.000x |
| 2 | 3 | +covariance | 51.022032 | 0.510220 | 1.085x |
| 2 | 3 | +link quality | 52.121902 | 0.521219 | 1.109x |
| 2 | 3 | +existence confidence | 51.041151 | 0.510412 | 1.086x |
| 2 | 3 | +structure-aware decoupled KLA | 50.350501 | 0.503505 | 1.071x |
| 3 | 4 | fixed weights | 49.186557 | 0.491866 | 1.000x |
| 3 | 4 | +covariance | 52.884658 | 0.528847 | 1.075x |
| 3 | 4 | +link quality | 51.168815 | 0.511688 | 1.040x |
| 3 | 4 | +existence confidence | 51.259278 | 0.512593 | 1.042x |
| 3 | 4 | +structure-aware decoupled KLA | 55.779409 | 0.557794 | 1.134x |
| 4 | 5 | fixed weights | 72.961123 | 0.729611 | 1.000x |
| 4 | 5 | +covariance | 65.459313 | 0.654593 | 0.897x |
| 4 | 5 | +link quality | 61.421279 | 0.614213 | 0.842x |
| 4 | 5 | +existence confidence | 116.995197 | 1.169952 | 1.604x |
| 4 | 5 | +structure-aware decoupled KLA | 52.744915 | 0.527449 | 0.723x |
| 5 | 6 | fixed weights | 55.609136 | 0.556091 | 1.000x |
| 5 | 6 | +covariance | 53.964209 | 0.539642 | 0.970x |
| 5 | 6 | +link quality | 51.324521 | 0.513245 | 0.923x |
| 5 | 6 | +existence confidence | 49.856671 | 0.498567 | 0.897x |
| 5 | 6 | +structure-aware decoupled KLA | 56.630818 | 0.566308 | 1.018x |
| 6 | 7 | fixed weights | 70.631355 | 0.706314 | 1.000x |
| 6 | 7 | +covariance | 73.872925 | 0.738729 | 1.046x |
| 6 | 7 | +link quality | 52.277600 | 0.522776 | 0.740x |
| 6 | 7 | +existence confidence | 49.755128 | 0.497551 | 0.704x |
| 6 | 7 | +structure-aware decoupled KLA | 50.086418 | 0.500864 | 0.709x |
| 7 | 8 | fixed weights | 48.350635 | 0.483506 | 1.000x |
| 7 | 8 | +covariance | 51.617584 | 0.516176 | 1.068x |
| 7 | 8 | +link quality | 50.966583 | 0.509666 | 1.054x |
| 7 | 8 | +existence confidence | 50.947188 | 0.509472 | 1.054x |
| 7 | 8 | +structure-aware decoupled KLA | 51.373296 | 0.513733 | 1.063x |
| 8 | 9 | fixed weights | 48.319302 | 0.483193 | 1.000x |
| 8 | 9 | +covariance | 51.772177 | 0.517722 | 1.071x |
| 8 | 9 | +link quality | 50.297890 | 0.502979 | 1.041x |
| 8 | 9 | +existence confidence | 50.259423 | 0.502594 | 1.040x |
| 8 | 9 | +structure-aware decoupled KLA | 50.524641 | 0.505246 | 1.046x |
| 9 | 10 | fixed weights | 46.790019 | 0.467900 | 1.000x |
| 9 | 10 | +covariance | 50.268480 | 0.502685 | 1.074x |
| 9 | 10 | +link quality | 49.492704 | 0.494927 | 1.058x |
| 9 | 10 | +existence confidence | 49.737672 | 0.497377 | 1.063x |
| 9 | 10 | +structure-aware decoupled KLA | 49.899927 | 0.498999 | 1.066x |
| 10 | 11 | fixed weights | 47.243999 | 0.472440 | 1.000x |
| 10 | 11 | +covariance | 50.714168 | 0.507142 | 1.073x |
| 10 | 11 | +link quality | 50.720298 | 0.507203 | 1.074x |
| 10 | 11 | +existence confidence | 50.637714 | 0.506377 | 1.072x |
| 10 | 11 | +structure-aware decoupled KLA | 51.134592 | 0.511346 | 1.082x |
| 11 | 12 | fixed weights | 48.090643 | 0.480906 | 1.000x |
| 11 | 12 | +covariance | 51.571106 | 0.515711 | 1.072x |
| 11 | 12 | +link quality | 50.657502 | 0.506575 | 1.053x |
| 11 | 12 | +existence confidence | 50.788799 | 0.507888 | 1.056x |
| 11 | 12 | +structure-aware decoupled KLA | 51.145112 | 0.511451 | 1.064x |
| 12 | 13 | fixed weights | 48.874541 | 0.488745 | 1.000x |
| 12 | 13 | +covariance | 52.409050 | 0.524090 | 1.072x |
| 12 | 13 | +link quality | 51.696518 | 0.516965 | 1.058x |
| 12 | 13 | +existence confidence | 52.030912 | 0.520309 | 1.065x |
| 12 | 13 | +structure-aware decoupled KLA | 52.411872 | 0.524119 | 1.072x |
| 13 | 14 | fixed weights | 48.695721 | 0.486957 | 1.000x |
| 13 | 14 | +covariance | 52.127779 | 0.521278 | 1.070x |
| 13 | 14 | +link quality | 51.709433 | 0.517094 | 1.062x |
| 13 | 14 | +existence confidence | 51.774957 | 0.517750 | 1.063x |
| 13 | 14 | +structure-aware decoupled KLA | 52.037896 | 0.520379 | 1.069x |
| 14 | 15 | fixed weights | 46.923637 | 0.469236 | 1.000x |
| 14 | 15 | +covariance | 50.590800 | 0.505908 | 1.078x |
| 14 | 15 | +link quality | 50.325350 | 0.503254 | 1.072x |
| 14 | 15 | +existence confidence | 50.226250 | 0.502262 | 1.070x |
| 14 | 15 | +structure-aware decoupled KLA | 50.876760 | 0.508768 | 1.084x |
| 15 | 16 | fixed weights | 54.401741 | 0.544017 | 1.000x |
| 15 | 16 | +covariance | 74.301421 | 0.743014 | 1.366x |
| 15 | 16 | +link quality | 58.119959 | 0.581200 | 1.068x |
| 15 | 16 | +existence confidence | 52.123951 | 0.521240 | 0.958x |
| 15 | 16 | +structure-aware decoupled KLA | 52.495401 | 0.524954 | 0.965x |
| 16 | 17 | fixed weights | 47.686278 | 0.476863 | 1.000x |
| 16 | 17 | +covariance | 51.364878 | 0.513649 | 1.077x |
| 16 | 17 | +link quality | 50.709440 | 0.507094 | 1.063x |
| 16 | 17 | +existence confidence | 50.727745 | 0.507277 | 1.064x |
| 16 | 17 | +structure-aware decoupled KLA | 51.044190 | 0.510442 | 1.070x |
| 17 | 18 | fixed weights | 47.833391 | 0.478334 | 1.000x |
| 17 | 18 | +covariance | 52.338905 | 0.523389 | 1.094x |
| 17 | 18 | +link quality | 52.688744 | 0.526887 | 1.102x |
| 17 | 18 | +existence confidence | 52.616033 | 0.526160 | 1.100x |
| 17 | 18 | +structure-aware decoupled KLA | 52.918473 | 0.529185 | 1.106x |
| 18 | 19 | fixed weights | 47.315325 | 0.473153 | 1.000x |
| 18 | 19 | +covariance | 49.975918 | 0.499759 | 1.056x |
| 18 | 19 | +link quality | 51.082719 | 0.510827 | 1.080x |
| 18 | 19 | +existence confidence | 49.890318 | 0.498903 | 1.054x |
| 18 | 19 | +structure-aware decoupled KLA | 50.197604 | 0.501976 | 1.061x |
| 19 | 20 | fixed weights | 47.409187 | 0.474092 | 1.000x |
| 19 | 20 | +covariance | 51.461831 | 0.514618 | 1.085x |
| 19 | 20 | +link quality | 50.311280 | 0.503113 | 1.061x |
| 19 | 20 | +existence confidence | 50.196795 | 0.501968 | 1.059x |
| 19 | 20 | +structure-aware decoupled KLA | 50.600850 | 0.506008 | 1.067x |
| 20 | 21 | fixed weights | 49.153837 | 0.491538 | 1.000x |
| 20 | 21 | +covariance | 52.025107 | 0.520251 | 1.058x |
| 20 | 21 | +link quality | 52.778924 | 0.527789 | 1.074x |
| 20 | 21 | +existence confidence | 59.820023 | 0.598200 | 1.217x |
| 20 | 21 | +structure-aware decoupled KLA | 91.321159 | 0.913212 | 1.858x |
| 21 | 22 | fixed weights | 51.428955 | 0.514290 | 1.000x |
| 21 | 22 | +covariance | 54.127852 | 0.541279 | 1.052x |
| 21 | 22 | +link quality | 54.292988 | 0.542930 | 1.056x |
| 21 | 22 | +existence confidence | 54.364834 | 0.543648 | 1.057x |
| 21 | 22 | +structure-aware decoupled KLA | 55.128376 | 0.551284 | 1.072x |
| 22 | 23 | fixed weights | 51.480491 | 0.514805 | 1.000x |
| 22 | 23 | +covariance | 54.229919 | 0.542299 | 1.053x |
| 22 | 23 | +link quality | 53.129275 | 0.531293 | 1.032x |
| 22 | 23 | +existence confidence | 53.674007 | 0.536740 | 1.043x |
| 22 | 23 | +structure-aware decoupled KLA | 53.148808 | 0.531488 | 1.032x |
| 23 | 24 | fixed weights | 51.665596 | 0.516656 | 1.000x |
| 23 | 24 | +covariance | 54.400916 | 0.544009 | 1.053x |
| 23 | 24 | +link quality | 53.186038 | 0.531860 | 1.029x |
| 23 | 24 | +existence confidence | 54.075216 | 0.540752 | 1.047x |
| 23 | 24 | +structure-aware decoupled KLA | 54.288391 | 0.542884 | 1.051x |
| 24 | 25 | fixed weights | 50.131454 | 0.501315 | 1.000x |
| 24 | 25 | +covariance | 55.310152 | 0.553102 | 1.103x |
| 24 | 25 | +link quality | 53.064482 | 0.530645 | 1.059x |
| 24 | 25 | +existence confidence | 53.239459 | 0.532395 | 1.062x |
| 24 | 25 | +structure-aware decoupled KLA | 52.928627 | 0.529286 | 1.056x |
| 25 | 26 | fixed weights | 50.767209 | 0.507672 | 1.000x |
| 25 | 26 | +covariance | 56.122058 | 0.561221 | 1.105x |
| 25 | 26 | +link quality | 52.908232 | 0.529082 | 1.042x |
| 25 | 26 | +existence confidence | 53.602841 | 0.536028 | 1.056x |
| 25 | 26 | +structure-aware decoupled KLA | 52.912018 | 0.529120 | 1.042x |
| 26 | 27 | fixed weights | 50.124928 | 0.501249 | 1.000x |
| 26 | 27 | +covariance | 54.080052 | 0.540801 | 1.079x |
| 26 | 27 | +link quality | 53.376395 | 0.533764 | 1.065x |
| 26 | 27 | +existence confidence | 53.082694 | 0.530827 | 1.059x |
| 26 | 27 | +structure-aware decoupled KLA | 53.066017 | 0.530660 | 1.059x |
| 27 | 28 | fixed weights | 50.939181 | 0.509392 | 1.000x |
| 27 | 28 | +covariance | 54.532779 | 0.545328 | 1.071x |
| 27 | 28 | +link quality | 54.414868 | 0.544149 | 1.068x |
| 27 | 28 | +existence confidence | 54.622580 | 0.546226 | 1.072x |
| 27 | 28 | +structure-aware decoupled KLA | 54.581786 | 0.545818 | 1.072x |
| 28 | 29 | fixed weights | 47.301330 | 0.473013 | 1.000x |
| 28 | 29 | +covariance | 50.703650 | 0.507036 | 1.072x |
| 28 | 29 | +link quality | 50.991341 | 0.509913 | 1.078x |
| 28 | 29 | +existence confidence | 51.802518 | 0.518025 | 1.095x |
| 28 | 29 | +structure-aware decoupled KLA | 75.837983 | 0.758380 | 1.603x |
| 29 | 30 | fixed weights | 72.657551 | 0.726576 | 1.000x |
| 29 | 30 | +covariance | 62.995515 | 0.629955 | 0.867x |
| 29 | 30 | +link quality | 54.003530 | 0.540035 | 0.743x |
| 29 | 30 | +existence confidence | 54.022564 | 0.540226 | 0.744x |
| 29 | 30 | +structure-aware decoupled KLA | 54.637247 | 0.546372 | 0.752x |
| 30 | 31 | fixed weights | 50.452617 | 0.504526 | 1.000x |
| 30 | 31 | +covariance | 55.365546 | 0.553655 | 1.097x |
| 30 | 31 | +link quality | 53.693412 | 0.536934 | 1.064x |
| 30 | 31 | +existence confidence | 53.154433 | 0.531544 | 1.054x |
| 30 | 31 | +structure-aware decoupled KLA | 52.743493 | 0.527435 | 1.045x |
| 31 | 32 | fixed weights | 47.824367 | 0.478244 | 1.000x |
| 31 | 32 | +covariance | 51.920286 | 0.519203 | 1.086x |
| 31 | 32 | +link quality | 51.605723 | 0.516057 | 1.079x |
| 31 | 32 | +existence confidence | 52.133288 | 0.521333 | 1.090x |
| 31 | 32 | +structure-aware decoupled KLA | 53.769139 | 0.537691 | 1.124x |
| 32 | 33 | fixed weights | 51.043175 | 0.510432 | 1.000x |
| 32 | 33 | +covariance | 54.358632 | 0.543586 | 1.065x |
| 32 | 33 | +link quality | 54.899251 | 0.548993 | 1.076x |
| 32 | 33 | +existence confidence | 55.508476 | 0.555085 | 1.087x |
| 32 | 33 | +structure-aware decoupled KLA | 55.146500 | 0.551465 | 1.080x |
| 33 | 34 | fixed weights | 49.848749 | 0.498487 | 1.000x |
| 33 | 34 | +covariance | 52.775978 | 0.527760 | 1.059x |
| 33 | 34 | +link quality | 52.644647 | 0.526446 | 1.056x |
| 33 | 34 | +existence confidence | 52.419960 | 0.524200 | 1.052x |
| 33 | 34 | +structure-aware decoupled KLA | 53.080270 | 0.530803 | 1.065x |
| 34 | 35 | fixed weights | 48.730263 | 0.487303 | 1.000x |
| 34 | 35 | +covariance | 51.808480 | 0.518085 | 1.063x |
| 34 | 35 | +link quality | 52.338326 | 0.523383 | 1.074x |
| 34 | 35 | +existence confidence | 53.037191 | 0.530372 | 1.088x |
| 34 | 35 | +structure-aware decoupled KLA | 54.384083 | 0.543841 | 1.116x |
| 35 | 36 | fixed weights | 52.512920 | 0.525129 | 1.000x |
| 35 | 36 | +covariance | 55.315099 | 0.553151 | 1.053x |
| 35 | 36 | +link quality | 53.942305 | 0.539423 | 1.027x |
| 35 | 36 | +existence confidence | 53.523625 | 0.535236 | 1.019x |
| 35 | 36 | +structure-aware decoupled KLA | 53.669658 | 0.536697 | 1.022x |
| 36 | 37 | fixed weights | 49.462417 | 0.494624 | 1.000x |
| 36 | 37 | +covariance | 51.894924 | 0.518949 | 1.049x |
| 36 | 37 | +link quality | 49.735429 | 0.497354 | 1.006x |
| 36 | 37 | +existence confidence | 49.329001 | 0.493290 | 0.997x |
| 36 | 37 | +structure-aware decoupled KLA | 49.521968 | 0.495220 | 1.001x |
| 37 | 38 | fixed weights | 48.090630 | 0.480906 | 1.000x |
| 37 | 38 | +covariance | 53.765414 | 0.537654 | 1.118x |
| 37 | 38 | +link quality | 53.542870 | 0.535429 | 1.113x |
| 37 | 38 | +existence confidence | 53.178485 | 0.531785 | 1.106x |
| 37 | 38 | +structure-aware decoupled KLA | 53.590249 | 0.535902 | 1.114x |
| 38 | 39 | fixed weights | 49.714878 | 0.497149 | 1.000x |
| 38 | 39 | +covariance | 54.072745 | 0.540727 | 1.088x |
| 38 | 39 | +link quality | 54.303706 | 0.543037 | 1.092x |
| 38 | 39 | +existence confidence | 71.916023 | 0.719160 | 1.447x |
| 38 | 39 | +structure-aware decoupled KLA | 78.200341 | 0.782003 | 1.573x |
| 39 | 40 | fixed weights | 68.704136 | 0.687041 | 1.000x |
| 39 | 40 | +covariance | 53.638137 | 0.536381 | 0.781x |
| 39 | 40 | +link quality | 52.070110 | 0.520701 | 0.758x |
| 39 | 40 | +existence confidence | 52.746253 | 0.527463 | 0.768x |
| 39 | 40 | +structure-aware decoupled KLA | 53.387339 | 0.533873 | 0.777x |
| 40 | 41 | fixed weights | 50.046497 | 0.500465 | 1.000x |
| 40 | 41 | +covariance | 53.039844 | 0.530398 | 1.060x |
| 40 | 41 | +link quality | 53.166526 | 0.531665 | 1.062x |
| 40 | 41 | +existence confidence | 52.609692 | 0.526097 | 1.051x |
| 40 | 41 | +structure-aware decoupled KLA | 52.940932 | 0.529409 | 1.058x |
| 41 | 42 | fixed weights | 50.418246 | 0.504182 | 1.000x |
| 41 | 42 | +covariance | 53.708325 | 0.537083 | 1.065x |
| 41 | 42 | +link quality | 53.571017 | 0.535710 | 1.063x |
| 41 | 42 | +existence confidence | 51.973970 | 0.519740 | 1.031x |
| 41 | 42 | +structure-aware decoupled KLA | 51.921230 | 0.519212 | 1.030x |
| 42 | 43 | fixed weights | 51.990974 | 0.519910 | 1.000x |
| 42 | 43 | +covariance | 53.974485 | 0.539745 | 1.038x |
| 42 | 43 | +link quality | 52.142545 | 0.521425 | 1.003x |
| 42 | 43 | +existence confidence | 52.881901 | 0.528819 | 1.017x |
| 42 | 43 | +structure-aware decoupled KLA | 53.040562 | 0.530406 | 1.020x |
| 43 | 44 | fixed weights | 49.989113 | 0.499891 | 1.000x |
| 43 | 44 | +covariance | 53.337795 | 0.533378 | 1.067x |
| 43 | 44 | +link quality | 53.267638 | 0.532676 | 1.066x |
| 43 | 44 | +existence confidence | 53.427918 | 0.534279 | 1.069x |
| 43 | 44 | +structure-aware decoupled KLA | 53.421912 | 0.534219 | 1.069x |
| 44 | 45 | fixed weights | 49.871382 | 0.498714 | 1.000x |
| 44 | 45 | +covariance | 54.831124 | 0.548311 | 1.099x |
| 44 | 45 | +link quality | 52.110736 | 0.521107 | 1.045x |
| 44 | 45 | +existence confidence | 51.243133 | 0.512431 | 1.028x |
| 44 | 45 | +structure-aware decoupled KLA | 51.151634 | 0.511516 | 1.026x |
| 45 | 46 | fixed weights | 49.499374 | 0.494994 | 1.000x |
| 45 | 46 | +covariance | 52.537958 | 0.525380 | 1.061x |
| 45 | 46 | +link quality | 54.126948 | 0.541269 | 1.093x |
| 45 | 46 | +existence confidence | 52.399558 | 0.523996 | 1.059x |
| 45 | 46 | +structure-aware decoupled KLA | 52.346065 | 0.523461 | 1.058x |
| 46 | 47 | fixed weights | 51.070500 | 0.510705 | 1.000x |
| 46 | 47 | +covariance | 55.080654 | 0.550807 | 1.079x |
| 46 | 47 | +link quality | 54.181768 | 0.541818 | 1.061x |
| 46 | 47 | +existence confidence | 54.255325 | 0.542553 | 1.062x |
| 46 | 47 | +structure-aware decoupled KLA | 53.219209 | 0.532192 | 1.042x |
| 47 | 48 | fixed weights | 48.489990 | 0.484900 | 1.000x |
| 47 | 48 | +covariance | 51.721565 | 0.517216 | 1.067x |
| 47 | 48 | +link quality | 49.871047 | 0.498710 | 1.028x |
| 47 | 48 | +existence confidence | 49.913596 | 0.499136 | 1.029x |
| 47 | 48 | +structure-aware decoupled KLA | 50.049742 | 0.500497 | 1.032x |
| 48 | 49 | fixed weights | 48.189633 | 0.481896 | 1.000x |
| 48 | 49 | +covariance | 54.017478 | 0.540175 | 1.121x |
| 48 | 49 | +link quality | 52.906313 | 0.529063 | 1.098x |
| 48 | 49 | +existence confidence | 52.811284 | 0.528113 | 1.096x |
| 48 | 49 | +structure-aware decoupled KLA | 53.270567 | 0.532706 | 1.105x |
| 49 | 50 | fixed weights | 49.242926 | 0.492429 | 1.000x |
| 49 | 50 | +covariance | 53.965327 | 0.539653 | 1.096x |
| 49 | 50 | +link quality | 52.926465 | 0.529265 | 1.075x |
| 49 | 50 | +existence confidence | 75.932926 | 0.759329 | 1.542x |
| 49 | 50 | +structure-aware decoupled KLA | 76.241469 | 0.762415 | 1.548x |
| 50 | 51 | fixed weights | 66.047559 | 0.660476 | 1.000x |
| 50 | 51 | +covariance | 53.495239 | 0.534952 | 0.810x |
| 50 | 51 | +link quality | 52.267742 | 0.522677 | 0.791x |
| 50 | 51 | +existence confidence | 53.237267 | 0.532373 | 0.806x |
| 50 | 51 | +structure-aware decoupled KLA | 54.087992 | 0.540880 | 0.819x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.862938 | 1.649569 | 1.455125 |
| +covariance | 2.685775 | 1.560762 | 1.154400 |
| +link quality | 2.329283 | 1.600391 | 0.578800 |
| +existence confidence | 2.332596 | 1.601376 | 0.578225 |
| +structure-aware decoupled KLA | 2.334915 | 1.605910 | 0.578775 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.862938 +/- 0.123917 | [2.828590, 2.897286] | 50 |
| +covariance | E-OSPA | 2.685775 +/- 0.102619 | [2.657330, 2.714219] | 50 |
| +link quality | E-OSPA | 2.329283 +/- 0.071369 | [2.309500, 2.349066] | 50 |
| +existence confidence | E-OSPA | 2.332596 +/- 0.071389 | [2.312808, 2.352384] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 2.334915 +/- 0.071651 | [2.315054, 2.354776] | 50 |
| fixed weights | RMSE | 1.649569 +/- 0.078096 | [1.627922, 1.671216] | 50 |
| +covariance | RMSE | 1.560762 +/- 0.052315 | [1.546261, 1.575263] | 50 |
| +link quality | RMSE | 1.600391 +/- 0.049191 | [1.586756, 1.614026] | 50 |
| +existence confidence | RMSE | 1.601376 +/- 0.049157 | [1.587750, 1.615001] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.605910 +/- 0.049506 | [1.592188, 1.619633] | 50 |
| fixed weights | CardErr | 1.455125 +/- 0.241978 | [1.388052, 1.522198] | 50 |
| +covariance | CardErr | 1.154400 +/- 0.165774 | [1.108450, 1.200350] | 50 |
| +link quality | CardErr | 0.578800 +/- 0.066873 | [0.560264, 0.597336] | 50 |
| +existence confidence | CardErr | 0.578225 +/- 0.066600 | [0.559764, 0.596686] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.578775 +/- 0.066558 | [0.560326, 0.597224] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.177164 +/- 0.046469 | [0.164283, 0.190044] | 6.19% | 50/50 | 1.776e-15 |
| +link quality | E-OSPA | 0.533655 +/- 0.088486 | [0.509128, 0.558182] | 18.64% | 50/50 | 1.776e-15 |
| +existence confidence | E-OSPA | 0.530342 +/- 0.088389 | [0.505842, 0.554842] | 18.52% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | E-OSPA | 0.528023 +/- 0.089961 | [0.503087, 0.552959] | 18.44% | 50/50 | 1.776e-15 |
| +covariance | RMSE | 0.088807 +/- 0.055862 | [0.073323, 0.104292] | 5.38% | 50/50 | 1.776e-15 |
| +link quality | RMSE | 0.049178 +/- 0.059811 | [0.032599, 0.065757] | 2.98% | 45/50 | 4.21e-09 |
| +existence confidence | RMSE | 0.048194 +/- 0.059659 | [0.031657, 0.064730] | 2.92% | 44/50 | 3.244e-08 |
| +structure-aware decoupled KLA | RMSE | 0.043659 +/- 0.059480 | [0.027172, 0.060146] | 2.65% | 43/50 | 2.099e-07 |
| +covariance | CardErr | 0.300725 +/- 0.106776 | [0.271128, 0.330322] | 20.67% | 50/50 | 1.776e-15 |
| +link quality | CardErr | 0.876325 +/- 0.211178 | [0.817789, 0.934861] | 60.22% | 50/50 | 1.776e-15 |
| +existence confidence | CardErr | 0.876900 +/- 0.211096 | [0.818387, 0.935413] | 60.26% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | CardErr | 0.876350 +/- 0.211610 | [0.817695, 0.935005] | 60.23% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.719987 | 1.619776 | 1.083600 |
| 1 | +covariance | 2.542184 | 1.512042 | 0.865400 |
| 1 | +link quality | 2.284853 | 1.555869 | 0.541200 |
| 1 | +existence confidence | 2.285195 | 1.556426 | 0.540800 |
| 1 | +structure-aware decoupled KLA | 2.289633 | 1.559677 | 0.542000 |
| 2 | fixed weights | 2.670351 | 1.594302 | 0.999200 |
| 2 | +covariance | 2.580268 | 1.520449 | 0.906600 |
| 2 | +link quality | 2.290607 | 1.565054 | 0.546400 |
| 2 | +existence confidence | 2.299175 | 1.565013 | 0.551200 |
| 2 | +structure-aware decoupled KLA | 2.297153 | 1.572246 | 0.546000 |
| 3 | fixed weights | 2.740374 | 1.607566 | 1.150400 |
| 3 | +covariance | 2.556387 | 1.522307 | 0.906600 |
| 3 | +link quality | 2.295211 | 1.566326 | 0.570600 |
| 3 | +existence confidence | 2.299423 | 1.566761 | 0.571600 |
| 3 | +structure-aware decoupled KLA | 2.304960 | 1.573270 | 0.573600 |
| 4 | fixed weights | 2.883255 | 1.654573 | 1.494400 |
| 4 | +covariance | 2.767406 | 1.581788 | 1.326400 |
| 4 | +link quality | 2.411042 | 1.633281 | 0.685400 |
| 4 | +existence confidence | 2.413783 | 1.634597 | 0.683800 |
| 4 | +structure-aware decoupled KLA | 2.413320 | 1.639044 | 0.681200 |
| 5 | fixed weights | 2.918523 | 1.691261 | 1.520000 |
| 5 | +covariance | 2.813252 | 1.582814 | 1.422400 |
| 5 | +link quality | 2.371967 | 1.629439 | 0.623600 |
| 5 | +existence confidence | 2.372498 | 1.631904 | 0.618800 |
| 5 | +structure-aware decoupled KLA | 2.375721 | 1.635650 | 0.619400 |
| 6 | fixed weights | 2.913753 | 1.708755 | 1.535200 |
| 6 | +covariance | 2.740251 | 1.585576 | 1.270600 |
| 6 | +link quality | 2.318031 | 1.613526 | 0.540200 |
| 6 | +existence confidence | 2.322120 | 1.614960 | 0.538800 |
| 6 | +structure-aware decoupled KLA | 2.327530 | 1.618789 | 0.542600 |
| 7 | fixed weights | 2.925998 | 1.663623 | 1.565800 |
| 7 | +covariance | 2.774495 | 1.594569 | 1.315200 |
| 7 | +link quality | 2.364495 | 1.627267 | 0.604200 |
| 7 | +existence confidence | 2.367062 | 1.627983 | 0.601200 |
| 7 | +structure-aware decoupled KLA | 2.370662 | 1.633665 | 0.605400 |
| 8 | fixed weights | 3.131265 | 1.656698 | 2.292400 |
| 8 | +covariance | 2.711954 | 1.586550 | 1.222000 |
| 8 | +link quality | 2.298057 | 1.612369 | 0.518800 |
| 8 | +existence confidence | 2.301515 | 1.613361 | 0.519600 |
| 8 | +structure-aware decoupled KLA | 2.300340 | 1.614939 | 0.520000 |
