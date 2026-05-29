# GA Tiered Link Ablation (2026-05-29 12:06:02)

Comparison order: fixed weights -> +covariance -> +link quality -> +cardinality consensus

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

- finalArmMode: cardinality

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

### +cardinality consensus
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
| 1 | 2 | +covariance | 2.256340 | 1.835282 | 0.663750 |
| 1 | 2 | +link quality | 1.894149 | 1.454645 | 0.245000 |
| 1 | 2 | +cardinality consensus | 1.913290 | 1.535032 | 0.233750 |
| 2 | 3 | fixed weights | 2.604015 | 2.768626 | 0.775000 |
| 2 | 3 | +covariance | 2.250649 | 2.047509 | 0.613750 |
| 2 | 3 | +link quality | 1.867780 | 1.387814 | 0.195000 |
| 2 | 3 | +cardinality consensus | 1.873416 | 1.390854 | 0.192500 |
| 3 | 4 | fixed weights | 2.689713 | 2.372668 | 0.917500 |
| 3 | 4 | +covariance | 2.171057 | 1.775920 | 0.582500 |
| 3 | 4 | +link quality | 1.825910 | 1.438299 | 0.220000 |
| 3 | 4 | +cardinality consensus | 1.797819 | 1.448949 | 0.228750 |
| 4 | 5 | fixed weights | 2.388070 | 2.724116 | 0.645000 |
| 4 | 5 | +covariance | 1.999058 | 2.768107 | 0.352500 |
| 4 | 5 | +link quality | 1.748949 | 1.913620 | 0.175000 |
| 4 | 5 | +cardinality consensus | 1.776937 | 1.858335 | 0.182500 |
| 5 | 6 | fixed weights | 2.827419 | 2.356402 | 1.322500 |
| 5 | 6 | +covariance | 2.381920 | 2.160746 | 0.878750 |
| 5 | 6 | +link quality | 1.872465 | 1.868450 | 0.246250 |
| 5 | 6 | +cardinality consensus | 1.900147 | 1.820273 | 0.268750 |
| 6 | 7 | fixed weights | 2.564167 | 2.619153 | 0.798750 |
| 6 | 7 | +covariance | 2.130087 | 1.983686 | 0.480000 |
| 6 | 7 | +link quality | 1.861180 | 1.788151 | 0.198750 |
| 6 | 7 | +cardinality consensus | 1.855523 | 1.842989 | 0.188750 |
| 7 | 8 | fixed weights | 2.392370 | 2.197932 | 0.583750 |
| 7 | 8 | +covariance | 1.997424 | 1.786795 | 0.383750 |
| 7 | 8 | +link quality | 1.720911 | 1.373876 | 0.168750 |
| 7 | 8 | +cardinality consensus | 1.735528 | 1.376218 | 0.155000 |
| 8 | 9 | fixed weights | 2.465687 | 2.549449 | 0.686250 |
| 8 | 9 | +covariance | 1.971638 | 1.988356 | 0.366250 |
| 8 | 9 | +link quality | 1.755763 | 1.466729 | 0.188750 |
| 8 | 9 | +cardinality consensus | 1.787793 | 1.541219 | 0.203750 |
| 9 | 10 | fixed weights | 2.561884 | 2.756900 | 0.791250 |
| 9 | 10 | +covariance | 2.240500 | 1.859515 | 0.631250 |
| 9 | 10 | +link quality | 1.812550 | 2.095538 | 0.178750 |
| 9 | 10 | +cardinality consensus | 1.835108 | 2.282982 | 0.195000 |
| 10 | 11 | fixed weights | 2.179530 | 2.304817 | 0.425000 |
| 10 | 11 | +covariance | 1.936075 | 1.936313 | 0.357500 |
| 10 | 11 | +link quality | 1.772281 | 1.481958 | 0.176250 |
| 10 | 11 | +cardinality consensus | 1.817757 | 1.543453 | 0.198750 |
| 11 | 12 | fixed weights | 2.267598 | 2.220642 | 0.522500 |
| 11 | 12 | +covariance | 2.028315 | 1.907117 | 0.447500 |
| 11 | 12 | +link quality | 1.760701 | 1.535206 | 0.173750 |
| 11 | 12 | +cardinality consensus | 1.738257 | 1.653499 | 0.166250 |
| 12 | 13 | fixed weights | 2.391225 | 1.945551 | 0.696250 |
| 12 | 13 | +covariance | 1.938693 | 1.584191 | 0.367500 |
| 12 | 13 | +link quality | 1.724244 | 1.390455 | 0.190000 |
| 12 | 13 | +cardinality consensus | 1.762547 | 1.520260 | 0.198750 |
| 13 | 14 | fixed weights | 2.380845 | 2.354441 | 0.607500 |
| 13 | 14 | +covariance | 2.071381 | 2.219613 | 0.393750 |
| 13 | 14 | +link quality | 1.803248 | 1.467818 | 0.183750 |
| 13 | 14 | +cardinality consensus | 1.815067 | 1.487453 | 0.191250 |
| 14 | 15 | fixed weights | 2.162331 | 1.975854 | 0.408750 |
| 14 | 15 | +covariance | 1.947294 | 1.644533 | 0.307500 |
| 14 | 15 | +link quality | 1.777916 | 1.475596 | 0.188750 |
| 14 | 15 | +cardinality consensus | 1.770675 | 1.465916 | 0.190000 |
| 15 | 16 | fixed weights | 2.043693 | 1.953721 | 0.350000 |
| 15 | 16 | +covariance | 1.892142 | 1.980432 | 0.292500 |
| 15 | 16 | +link quality | 1.729686 | 1.524912 | 0.187500 |
| 15 | 16 | +cardinality consensus | 1.756600 | 1.536643 | 0.190000 |
| 16 | 17 | fixed weights | 2.576292 | 2.288784 | 0.770000 |
| 16 | 17 | +covariance | 2.094888 | 1.852252 | 0.457500 |
| 16 | 17 | +link quality | 1.816784 | 1.450195 | 0.223750 |
| 16 | 17 | +cardinality consensus | 1.822203 | 1.442535 | 0.228750 |
| 17 | 18 | fixed weights | 2.522517 | 2.040866 | 0.775000 |
| 17 | 18 | +covariance | 2.252858 | 1.989501 | 0.591250 |
| 17 | 18 | +link quality | 1.871102 | 1.664706 | 0.213750 |
| 17 | 18 | +cardinality consensus | 1.860529 | 1.640118 | 0.217500 |
| 18 | 19 | fixed weights | 2.438055 | 2.267740 | 0.782500 |
| 18 | 19 | +covariance | 1.958102 | 1.887362 | 0.430000 |
| 18 | 19 | +link quality | 1.690478 | 1.467031 | 0.163750 |
| 18 | 19 | +cardinality consensus | 1.679391 | 1.536599 | 0.175000 |
| 19 | 20 | fixed weights | 2.524133 | 2.218863 | 0.863750 |
| 19 | 20 | +covariance | 2.133150 | 1.659939 | 0.527500 |
| 19 | 20 | +link quality | 1.825640 | 1.482957 | 0.200000 |
| 19 | 20 | +cardinality consensus | 1.839792 | 1.458987 | 0.217500 |
| 20 | 21 | fixed weights | 2.488680 | 2.535585 | 0.638750 |
| 20 | 21 | +covariance | 2.142521 | 2.085485 | 0.483750 |
| 20 | 21 | +link quality | 1.770289 | 1.489944 | 0.176250 |
| 20 | 21 | +cardinality consensus | 1.789291 | 1.559933 | 0.192500 |
| 21 | 22 | fixed weights | 2.445386 | 2.063249 | 0.706250 |
| 21 | 22 | +covariance | 1.978431 | 1.667781 | 0.361250 |
| 21 | 22 | +link quality | 1.789513 | 1.553570 | 0.180000 |
| 21 | 22 | +cardinality consensus | 1.785550 | 1.549568 | 0.187500 |
| 22 | 23 | fixed weights | 2.782054 | 2.774744 | 0.917500 |
| 22 | 23 | +covariance | 2.330270 | 2.027310 | 0.550000 |
| 22 | 23 | +link quality | 1.856143 | 1.440447 | 0.211250 |
| 22 | 23 | +cardinality consensus | 1.812440 | 1.390416 | 0.218750 |
| 23 | 24 | fixed weights | 2.811135 | 1.942125 | 1.156250 |
| 23 | 24 | +covariance | 2.276285 | 1.916957 | 0.637500 |
| 23 | 24 | +link quality | 1.810456 | 1.501741 | 0.176250 |
| 23 | 24 | +cardinality consensus | 1.833746 | 1.579707 | 0.198750 |
| 24 | 25 | fixed weights | 2.569606 | 2.368173 | 0.711250 |
| 24 | 25 | +covariance | 2.178258 | 2.022457 | 0.480000 |
| 24 | 25 | +link quality | 1.879064 | 1.513682 | 0.220000 |
| 24 | 25 | +cardinality consensus | 1.874286 | 1.524445 | 0.223750 |
| 25 | 26 | fixed weights | 2.225426 | 2.278298 | 0.403750 |
| 25 | 26 | +covariance | 2.057762 | 1.901565 | 0.391250 |
| 25 | 26 | +link quality | 1.733723 | 1.395662 | 0.183750 |
| 25 | 26 | +cardinality consensus | 1.756321 | 1.442242 | 0.186250 |
| 26 | 27 | fixed weights | 2.159231 | 1.899203 | 0.517500 |
| 26 | 27 | +covariance | 1.918019 | 1.543218 | 0.388750 |
| 26 | 27 | +link quality | 1.661765 | 1.296388 | 0.148750 |
| 26 | 27 | +cardinality consensus | 1.681919 | 1.478766 | 0.152500 |
| 27 | 28 | fixed weights | 2.126003 | 1.944541 | 0.406250 |
| 27 | 28 | +covariance | 1.907666 | 1.792025 | 0.302500 |
| 27 | 28 | +link quality | 1.735363 | 1.381318 | 0.208750 |
| 27 | 28 | +cardinality consensus | 1.755379 | 1.385605 | 0.218750 |
| 28 | 29 | fixed weights | 2.917753 | 1.994773 | 1.183750 |
| 28 | 29 | +covariance | 2.286019 | 1.687519 | 0.767500 |
| 28 | 29 | +link quality | 1.886254 | 1.670214 | 0.270000 |
| 28 | 29 | +cardinality consensus | 1.912188 | 1.741899 | 0.292500 |
| 29 | 30 | fixed weights | 2.600417 | 1.944603 | 1.000000 |
| 29 | 30 | +covariance | 2.219677 | 1.916290 | 0.642500 |
| 29 | 30 | +link quality | 1.877042 | 1.641901 | 0.227500 |
| 29 | 30 | +cardinality consensus | 1.907699 | 1.749049 | 0.242500 |
| 30 | 31 | fixed weights | 2.468399 | 2.109218 | 0.737500 |
| 30 | 31 | +covariance | 2.155426 | 2.073726 | 0.460000 |
| 30 | 31 | +link quality | 1.784876 | 1.482583 | 0.162500 |
| 30 | 31 | +cardinality consensus | 1.784506 | 1.458423 | 0.170000 |
| 31 | 32 | fixed weights | 2.513279 | 1.885047 | 0.721250 |
| 31 | 32 | +covariance | 2.100438 | 1.921725 | 0.401250 |
| 31 | 32 | +link quality | 1.756308 | 1.469851 | 0.170000 |
| 31 | 32 | +cardinality consensus | 1.781838 | 1.492019 | 0.176250 |
| 32 | 33 | fixed weights | 2.019727 | 1.812822 | 0.302500 |
| 32 | 33 | +covariance | 1.720321 | 1.474372 | 0.191250 |
| 32 | 33 | +link quality | 1.698242 | 1.420990 | 0.162500 |
| 32 | 33 | +cardinality consensus | 1.688868 | 1.413970 | 0.166250 |
| 33 | 34 | fixed weights | 2.490122 | 2.173185 | 0.962500 |
| 33 | 34 | +covariance | 2.050168 | 1.978783 | 0.586250 |
| 33 | 34 | +link quality | 1.693889 | 1.581188 | 0.187500 |
| 33 | 34 | +cardinality consensus | 1.726122 | 1.606654 | 0.200000 |
| 34 | 35 | fixed weights | 2.915586 | 2.674676 | 1.071250 |
| 34 | 35 | +covariance | 2.329375 | 1.779998 | 0.666250 |
| 34 | 35 | +link quality | 1.870021 | 1.490107 | 0.186250 |
| 34 | 35 | +cardinality consensus | 1.863042 | 1.472549 | 0.185000 |
| 35 | 36 | fixed weights | 2.381312 | 2.282297 | 0.561250 |
| 35 | 36 | +covariance | 1.982301 | 1.678594 | 0.373750 |
| 35 | 36 | +link quality | 1.698205 | 1.454358 | 0.137500 |
| 35 | 36 | +cardinality consensus | 1.707167 | 1.464277 | 0.147500 |
| 36 | 37 | fixed weights | 2.794401 | 3.053361 | 0.971250 |
| 36 | 37 | +covariance | 2.348293 | 2.697460 | 0.602500 |
| 36 | 37 | +link quality | 1.858989 | 1.497741 | 0.216250 |
| 36 | 37 | +cardinality consensus | 1.870991 | 1.515206 | 0.223750 |
| 37 | 38 | fixed weights | 2.488304 | 2.240250 | 0.821250 |
| 37 | 38 | +covariance | 2.020268 | 2.159954 | 0.396250 |
| 37 | 38 | +link quality | 1.717295 | 1.543967 | 0.147500 |
| 37 | 38 | +cardinality consensus | 1.709254 | 1.590687 | 0.153750 |
| 38 | 39 | fixed weights | 2.292857 | 2.023920 | 0.538750 |
| 38 | 39 | +covariance | 2.014115 | 2.029145 | 0.383750 |
| 38 | 39 | +link quality | 1.823606 | 1.486187 | 0.177500 |
| 38 | 39 | +cardinality consensus | 1.893493 | 1.824948 | 0.202500 |
| 39 | 40 | fixed weights | 2.535165 | 2.535969 | 0.646250 |
| 39 | 40 | +covariance | 2.081385 | 1.743317 | 0.407500 |
| 39 | 40 | +link quality | 1.815326 | 1.744380 | 0.205000 |
| 39 | 40 | +cardinality consensus | 1.844938 | 1.704724 | 0.226250 |
| 40 | 41 | fixed weights | 2.257174 | 2.481788 | 0.530000 |
| 40 | 41 | +covariance | 2.010449 | 2.118700 | 0.421250 |
| 40 | 41 | +link quality | 1.789400 | 1.580266 | 0.206250 |
| 40 | 41 | +cardinality consensus | 1.819050 | 1.597491 | 0.227500 |
| 41 | 42 | fixed weights | 2.094739 | 2.021235 | 0.453750 |
| 41 | 42 | +covariance | 1.854092 | 1.827047 | 0.297500 |
| 41 | 42 | +link quality | 1.591801 | 1.288719 | 0.145000 |
| 41 | 42 | +cardinality consensus | 1.591852 | 1.286742 | 0.132500 |
| 42 | 43 | fixed weights | 2.470491 | 2.757701 | 0.583750 |
| 42 | 43 | +covariance | 2.101099 | 1.866587 | 0.396250 |
| 42 | 43 | +link quality | 1.724105 | 1.444278 | 0.095000 |
| 42 | 43 | +cardinality consensus | 1.733354 | 1.440620 | 0.103750 |
| 43 | 44 | fixed weights | 2.550641 | 2.146021 | 0.801250 |
| 43 | 44 | +covariance | 2.137172 | 2.342524 | 0.412500 |
| 43 | 44 | +link quality | 1.812060 | 1.405628 | 0.182500 |
| 43 | 44 | +cardinality consensus | 1.833237 | 1.411426 | 0.190000 |
| 44 | 45 | fixed weights | 2.216546 | 2.738044 | 0.428750 |
| 44 | 45 | +covariance | 1.948670 | 2.079577 | 0.368750 |
| 44 | 45 | +link quality | 1.721599 | 1.442912 | 0.158750 |
| 44 | 45 | +cardinality consensus | 1.731016 | 1.599384 | 0.166250 |
| 45 | 46 | fixed weights | 2.537339 | 2.539290 | 0.686250 |
| 45 | 46 | +covariance | 2.003436 | 2.104338 | 0.381250 |
| 45 | 46 | +link quality | 1.746446 | 1.436876 | 0.172500 |
| 45 | 46 | +cardinality consensus | 1.784607 | 1.442033 | 0.186250 |
| 46 | 47 | fixed weights | 2.665038 | 2.010381 | 0.693750 |
| 46 | 47 | +covariance | 2.072833 | 1.802736 | 0.347500 |
| 46 | 47 | +link quality | 1.850503 | 1.729966 | 0.158750 |
| 46 | 47 | +cardinality consensus | 1.825842 | 1.621470 | 0.153750 |
| 47 | 48 | fixed weights | 2.238065 | 2.249780 | 0.577500 |
| 47 | 48 | +covariance | 2.083497 | 1.891447 | 0.476250 |
| 47 | 48 | +link quality | 1.759599 | 1.378705 | 0.163750 |
| 47 | 48 | +cardinality consensus | 1.805237 | 1.429035 | 0.188750 |
| 48 | 49 | fixed weights | 2.529967 | 3.164643 | 0.665000 |
| 48 | 49 | +covariance | 2.260764 | 2.464590 | 0.536250 |
| 48 | 49 | +link quality | 1.901177 | 1.678296 | 0.230000 |
| 48 | 49 | +cardinality consensus | 1.883457 | 1.603479 | 0.226250 |
| 49 | 50 | fixed weights | 2.564502 | 2.102492 | 0.951250 |
| 49 | 50 | +covariance | 2.067135 | 2.100314 | 0.452500 |
| 49 | 50 | +link quality | 1.740913 | 1.432767 | 0.188750 |
| 49 | 50 | +cardinality consensus | 1.750642 | 1.542026 | 0.180000 |
| 50 | 51 | fixed weights | 2.714437 | 3.379717 | 0.801250 |
| 50 | 51 | +covariance | 2.244713 | 1.981760 | 0.603750 |
| 50 | 51 | +link quality | 1.916190 | 1.647148 | 0.233750 |
| 50 | 51 | +cardinality consensus | 1.903052 | 1.654031 | 0.231250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.468973 | 2.326034 | 0.716025 |
| +covariance | 2.090649 | 1.950889 | 0.465850 |
| +link quality | 1.788038 | 1.524995 | 0.188150 |
| +cardinality consensus | 1.799575 | 1.559103 | 0.195475 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.468973 +/- 0.219981 | [2.407997, 2.529949] | 50 |
| +covariance | OSPA | 2.090649 +/- 0.144677 | [2.050546, 2.130751] | 50 |
| +link quality | OSPA | 1.788038 +/- 0.071415 | [1.768243, 1.807833] | 50 |
| +cardinality consensus | OSPA | 1.799575 +/- 0.070755 | [1.779963, 1.819187] | 50 |
| fixed weights | RMSE | 2.326034 +/- 0.353065 | [2.228170, 2.423899] | 50 |
| +covariance | RMSE | 1.950889 +/- 0.252957 | [1.880773, 2.021005] | 50 |
| +link quality | RMSE | 1.524995 +/- 0.155955 | [1.481766, 1.568223] | 50 |
| +cardinality consensus | RMSE | 1.559103 +/- 0.166036 | [1.513080, 1.605126] | 50 |
| fixed weights | Cardinality | 0.716025 +/- 0.223795 | [0.653992, 0.778058] | 50 |
| +covariance | Cardinality | 0.465850 +/- 0.134228 | [0.428644, 0.503056] | 50 |
| +link quality | Cardinality | 0.188150 +/- 0.031280 | [0.179480, 0.196820] | 50 |
| +cardinality consensus | Cardinality | 0.195475 +/- 0.033598 | [0.186162, 0.204788] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | OSPA | 0.378324 +/- 0.118509 | [0.345475, 0.411174] | 15.32% | 50/50 | 1.776e-15 |
| +link quality | OSPA | 0.680935 +/- 0.177344 | [0.631778, 0.730092] | 27.58% | 50/50 | 1.776e-15 |
| +cardinality consensus | OSPA | 0.669398 +/- 0.182953 | [0.618686, 0.720110] | 27.11% | 50/50 | 1.776e-15 |
| +covariance | RMSE | 0.375145 +/- 0.307793 | [0.289829, 0.460461] | 16.13% | 45/50 | 4.21e-09 |
| +link quality | RMSE | 0.801039 +/- 0.338775 | [0.707136, 0.894943] | 34.44% | 50/50 | 1.776e-15 |
| +cardinality consensus | RMSE | 0.766931 +/- 0.358124 | [0.667664, 0.866198] | 32.97% | 50/50 | 1.776e-15 |
| +covariance | Cardinality | 0.250175 +/- 0.127169 | [0.214926, 0.285424] | 34.94% | 50/50 | 1.776e-15 |
| +link quality | Cardinality | 0.527875 +/- 0.209468 | [0.469813, 0.585937] | 73.72% | 50/50 | 1.776e-15 |
| +cardinality consensus | Cardinality | 0.520550 +/- 0.208075 | [0.462875, 0.578225] | 72.70% | 50/50 | 1.776e-15 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 51.974590 +/- 6.337644 | 0.519746 | 1.000x | 50 |
| +covariance | 116.123708 +/- 432.495113 | 1.161237 | 2.359x | 50 |
| +link quality | 53.890860 +/- 3.989591 | 0.538909 | 1.046x | 50 |
| +cardinality consensus | 54.169645 +/- 5.161610 | 0.541696 | 1.054x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +covariance | 64.149118 +/- 433.174522 | 135.86% | 46/50 |
| +link quality | 1.916271 +/- 6.006598 | 4.58% | 43/50 |
| +cardinality consensus | 2.195056 +/- 8.139138 | 5.37% | 42/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 72.588809 | 0.725888 | 1.000x |
| 1 | 2 | +covariance | 78.539590 | 0.785396 | 1.082x |
| 1 | 2 | +link quality | 64.951850 | 0.649518 | 0.895x |
| 1 | 2 | +cardinality consensus | 53.855504 | 0.538555 | 0.742x |
| 2 | 3 | fixed weights | 47.360623 | 0.473606 | 1.000x |
| 2 | 3 | +covariance | 51.526166 | 0.515262 | 1.088x |
| 2 | 3 | +link quality | 49.523101 | 0.495231 | 1.046x |
| 2 | 3 | +cardinality consensus | 49.707587 | 0.497076 | 1.050x |
| 3 | 4 | fixed weights | 48.952997 | 0.489530 | 1.000x |
| 3 | 4 | +covariance | 54.736336 | 0.547363 | 1.118x |
| 3 | 4 | +link quality | 53.844583 | 0.538446 | 1.100x |
| 3 | 4 | +cardinality consensus | 53.915133 | 0.539151 | 1.101x |
| 4 | 5 | fixed weights | 50.879082 | 0.508791 | 1.000x |
| 4 | 5 | +covariance | 54.511782 | 0.545118 | 1.071x |
| 4 | 5 | +link quality | 53.763918 | 0.537639 | 1.057x |
| 4 | 5 | +cardinality consensus | 54.329238 | 0.543292 | 1.068x |
| 5 | 6 | fixed weights | 48.307324 | 0.483073 | 1.000x |
| 5 | 6 | +covariance | 51.985895 | 0.519859 | 1.076x |
| 5 | 6 | +link quality | 50.293725 | 0.502937 | 1.041x |
| 5 | 6 | +cardinality consensus | 50.975506 | 0.509755 | 1.055x |
| 6 | 7 | fixed weights | 45.712342 | 0.457123 | 1.000x |
| 6 | 7 | +covariance | 49.967933 | 0.499679 | 1.093x |
| 6 | 7 | +link quality | 50.458812 | 0.504588 | 1.104x |
| 6 | 7 | +cardinality consensus | 51.895611 | 0.518956 | 1.135x |
| 7 | 8 | fixed weights | 50.828661 | 0.508287 | 1.000x |
| 7 | 8 | +covariance | 53.957627 | 0.539576 | 1.062x |
| 7 | 8 | +link quality | 53.414088 | 0.534141 | 1.051x |
| 7 | 8 | +cardinality consensus | 53.466986 | 0.534670 | 1.052x |
| 8 | 9 | fixed weights | 50.571091 | 0.505711 | 1.000x |
| 8 | 9 | +covariance | 53.957005 | 0.539570 | 1.067x |
| 8 | 9 | +link quality | 52.352516 | 0.523525 | 1.035x |
| 8 | 9 | +cardinality consensus | 53.044682 | 0.530447 | 1.049x |
| 9 | 10 | fixed weights | 48.864807 | 0.488648 | 1.000x |
| 9 | 10 | +covariance | 51.957839 | 0.519578 | 1.063x |
| 9 | 10 | +link quality | 62.554718 | 0.625547 | 1.280x |
| 9 | 10 | +cardinality consensus | 74.923324 | 0.749233 | 1.533x |
| 10 | 11 | fixed weights | 71.254872 | 0.712549 | 1.000x |
| 10 | 11 | +covariance | 53.410880 | 0.534109 | 0.750x |
| 10 | 11 | +link quality | 51.990228 | 0.519902 | 0.730x |
| 10 | 11 | +cardinality consensus | 52.971645 | 0.529716 | 0.743x |
| 11 | 12 | fixed weights | 50.453474 | 0.504535 | 1.000x |
| 11 | 12 | +covariance | 55.398452 | 0.553985 | 1.098x |
| 11 | 12 | +link quality | 53.584635 | 0.535846 | 1.062x |
| 11 | 12 | +cardinality consensus | 53.612699 | 0.536127 | 1.063x |
| 12 | 13 | fixed weights | 51.393820 | 0.513938 | 1.000x |
| 12 | 13 | +covariance | 54.504839 | 0.545048 | 1.061x |
| 12 | 13 | +link quality | 54.216023 | 0.542160 | 1.055x |
| 12 | 13 | +cardinality consensus | 54.353603 | 0.543536 | 1.058x |
| 13 | 14 | fixed weights | 49.546888 | 0.495469 | 1.000x |
| 13 | 14 | +covariance | 52.485567 | 0.524856 | 1.059x |
| 13 | 14 | +link quality | 51.765129 | 0.517651 | 1.045x |
| 13 | 14 | +cardinality consensus | 52.125974 | 0.521260 | 1.052x |
| 14 | 15 | fixed weights | 47.872608 | 0.478726 | 1.000x |
| 14 | 15 | +covariance | 53.746869 | 0.537469 | 1.123x |
| 14 | 15 | +link quality | 53.210280 | 0.532103 | 1.111x |
| 14 | 15 | +cardinality consensus | 52.607760 | 0.526078 | 1.099x |
| 15 | 16 | fixed weights | 50.625974 | 0.506260 | 1.000x |
| 15 | 16 | +covariance | 55.454716 | 0.554547 | 1.095x |
| 15 | 16 | +link quality | 54.586449 | 0.545864 | 1.078x |
| 15 | 16 | +cardinality consensus | 56.030021 | 0.560300 | 1.107x |
| 16 | 17 | fixed weights | 50.939816 | 0.509398 | 1.000x |
| 16 | 17 | +covariance | 53.528501 | 0.535285 | 1.051x |
| 16 | 17 | +link quality | 52.380551 | 0.523806 | 1.028x |
| 16 | 17 | +cardinality consensus | 52.886744 | 0.528867 | 1.038x |
| 17 | 18 | fixed weights | 50.065300 | 0.500653 | 1.000x |
| 17 | 18 | +covariance | 53.201859 | 0.532019 | 1.063x |
| 17 | 18 | +link quality | 53.179431 | 0.531794 | 1.062x |
| 17 | 18 | +cardinality consensus | 54.773423 | 0.547734 | 1.094x |
| 18 | 19 | fixed weights | 49.415115 | 0.494151 | 1.000x |
| 18 | 19 | +covariance | 52.326994 | 0.523270 | 1.059x |
| 18 | 19 | +link quality | 52.156028 | 0.521560 | 1.055x |
| 18 | 19 | +cardinality consensus | 51.897406 | 0.518974 | 1.050x |
| 19 | 20 | fixed weights | 49.865260 | 0.498653 | 1.000x |
| 19 | 20 | +covariance | 54.667210 | 0.546672 | 1.096x |
| 19 | 20 | +link quality | 53.049845 | 0.530498 | 1.064x |
| 19 | 20 | +cardinality consensus | 52.382497 | 0.523825 | 1.050x |
| 20 | 21 | fixed weights | 50.977495 | 0.509775 | 1.000x |
| 20 | 21 | +covariance | 53.704400 | 0.537044 | 1.053x |
| 20 | 21 | +link quality | 51.896200 | 0.518962 | 1.018x |
| 20 | 21 | +cardinality consensus | 54.067457 | 0.540675 | 1.061x |
| 21 | 22 | fixed weights | 50.179171 | 0.501792 | 1.000x |
| 21 | 22 | +covariance | 53.352425 | 0.533524 | 1.063x |
| 21 | 22 | +link quality | 53.261555 | 0.532616 | 1.061x |
| 21 | 22 | +cardinality consensus | 54.637938 | 0.546379 | 1.089x |
| 22 | 23 | fixed weights | 51.273638 | 0.512736 | 1.000x |
| 22 | 23 | +covariance | 54.325236 | 0.543252 | 1.060x |
| 22 | 23 | +link quality | 53.675856 | 0.536759 | 1.047x |
| 22 | 23 | +cardinality consensus | 53.899904 | 0.538999 | 1.051x |
| 23 | 24 | fixed weights | 53.240878 | 0.532409 | 1.000x |
| 23 | 24 | +covariance | 56.912678 | 0.569127 | 1.069x |
| 23 | 24 | +link quality | 68.357882 | 0.683579 | 1.284x |
| 23 | 24 | +cardinality consensus | 77.563337 | 0.775633 | 1.457x |
| 24 | 25 | fixed weights | 71.265055 | 0.712651 | 1.000x |
| 24 | 25 | +covariance | 54.481417 | 0.544814 | 0.764x |
| 24 | 25 | +link quality | 50.619957 | 0.506200 | 0.710x |
| 24 | 25 | +cardinality consensus | 50.271506 | 0.502715 | 0.705x |
| 25 | 26 | fixed weights | 49.431082 | 0.494311 | 1.000x |
| 25 | 26 | +covariance | 53.153457 | 0.531535 | 1.075x |
| 25 | 26 | +link quality | 52.614965 | 0.526150 | 1.064x |
| 25 | 26 | +cardinality consensus | 54.801182 | 0.548012 | 1.109x |
| 26 | 27 | fixed weights | 50.562165 | 0.505622 | 1.000x |
| 26 | 27 | +covariance | 54.725810 | 0.547258 | 1.082x |
| 26 | 27 | +link quality | 53.368026 | 0.533680 | 1.055x |
| 26 | 27 | +cardinality consensus | 53.795367 | 0.537954 | 1.064x |
| 27 | 28 | fixed weights | 51.056690 | 0.510567 | 1.000x |
| 27 | 28 | +covariance | 55.615610 | 0.556156 | 1.089x |
| 27 | 28 | +link quality | 54.234761 | 0.542348 | 1.062x |
| 27 | 28 | +cardinality consensus | 54.113535 | 0.541135 | 1.060x |
| 28 | 29 | fixed weights | 47.448806 | 0.474488 | 1.000x |
| 28 | 29 | +covariance | 50.304540 | 0.503045 | 1.060x |
| 28 | 29 | +link quality | 49.618657 | 0.496187 | 1.046x |
| 28 | 29 | +cardinality consensus | 50.186251 | 0.501863 | 1.058x |
| 29 | 30 | fixed weights | 48.374979 | 0.483750 | 1.000x |
| 29 | 30 | +covariance | 52.878136 | 0.528781 | 1.093x |
| 29 | 30 | +link quality | 53.480627 | 0.534806 | 1.106x |
| 29 | 30 | +cardinality consensus | 54.853054 | 0.548531 | 1.134x |
| 30 | 31 | fixed weights | 49.936907 | 0.499369 | 1.000x |
| 30 | 31 | +covariance | 53.780047 | 0.537800 | 1.077x |
| 30 | 31 | +link quality | 53.438584 | 0.534386 | 1.070x |
| 30 | 31 | +cardinality consensus | 54.293037 | 0.542930 | 1.087x |
| 31 | 32 | fixed weights | 49.521578 | 0.495216 | 1.000x |
| 31 | 32 | +covariance | 53.350846 | 0.533508 | 1.077x |
| 31 | 32 | +link quality | 53.132867 | 0.531329 | 1.073x |
| 31 | 32 | +cardinality consensus | 52.959119 | 0.529591 | 1.069x |
| 32 | 33 | fixed weights | 49.515959 | 0.495160 | 1.000x |
| 32 | 33 | +covariance | 53.552331 | 0.535523 | 1.082x |
| 32 | 33 | +link quality | 52.630891 | 0.526309 | 1.063x |
| 32 | 33 | +cardinality consensus | 52.961144 | 0.529611 | 1.070x |
| 33 | 34 | fixed weights | 49.186429 | 0.491864 | 1.000x |
| 33 | 34 | +covariance | 54.675348 | 0.546753 | 1.112x |
| 33 | 34 | +link quality | 53.135464 | 0.531355 | 1.080x |
| 33 | 34 | +cardinality consensus | 53.973388 | 0.539734 | 1.097x |
| 34 | 35 | fixed weights | 51.007641 | 0.510076 | 1.000x |
| 34 | 35 | +covariance | 54.184793 | 0.541848 | 1.062x |
| 34 | 35 | +link quality | 53.788702 | 0.537887 | 1.055x |
| 34 | 35 | +cardinality consensus | 53.292173 | 0.532922 | 1.045x |
| 35 | 36 | fixed weights | 51.582648 | 0.515826 | 1.000x |
| 35 | 36 | +covariance | 54.936446 | 0.549364 | 1.065x |
| 35 | 36 | +link quality | 53.478135 | 0.534781 | 1.037x |
| 35 | 36 | +cardinality consensus | 52.857701 | 0.528577 | 1.025x |
| 36 | 37 | fixed weights | 49.520267 | 0.495203 | 1.000x |
| 36 | 37 | +covariance | 52.303163 | 0.523032 | 1.056x |
| 36 | 37 | +link quality | 50.068750 | 0.500687 | 1.011x |
| 36 | 37 | +cardinality consensus | 52.179396 | 0.521794 | 1.054x |
| 37 | 38 | fixed weights | 72.479981 | 0.724800 | 1.000x |
| 37 | 38 | +covariance | 78.193427 | 0.781934 | 1.079x |
| 37 | 38 | +link quality | 63.869622 | 0.638696 | 0.881x |
| 37 | 38 | +cardinality consensus | 52.899065 | 0.528991 | 0.730x |
| 38 | 39 | fixed weights | 49.806707 | 0.498067 | 1.000x |
| 38 | 39 | +covariance | 53.753147 | 0.537531 | 1.079x |
| 38 | 39 | +link quality | 53.206077 | 0.532061 | 1.068x |
| 38 | 39 | +cardinality consensus | 53.733298 | 0.537333 | 1.079x |
| 39 | 40 | fixed weights | 50.923635 | 0.509236 | 1.000x |
| 39 | 40 | +covariance | 53.988171 | 0.539882 | 1.060x |
| 39 | 40 | +link quality | 53.018855 | 0.530189 | 1.041x |
| 39 | 40 | +cardinality consensus | 52.257363 | 0.522574 | 1.026x |
| 40 | 41 | fixed weights | 48.127763 | 0.481278 | 1.000x |
| 40 | 41 | +covariance | 51.345868 | 0.513459 | 1.067x |
| 40 | 41 | +link quality | 50.256219 | 0.502562 | 1.044x |
| 40 | 41 | +cardinality consensus | 51.357081 | 0.513571 | 1.067x |
| 41 | 42 | fixed weights | 56.394036 | 0.563940 | 1.000x |
| 41 | 42 | +covariance | 51.941931 | 0.519419 | 0.921x |
| 41 | 42 | +link quality | 51.850226 | 0.518502 | 0.919x |
| 41 | 42 | +cardinality consensus | 51.879429 | 0.518794 | 0.920x |
| 42 | 43 | fixed weights | 50.281911 | 0.502819 | 1.000x |
| 42 | 43 | +covariance | 52.605425 | 0.526054 | 1.046x |
| 42 | 43 | +link quality | 50.296947 | 0.502969 | 1.000x |
| 42 | 43 | +cardinality consensus | 50.143636 | 0.501436 | 0.997x |
| 43 | 44 | fixed weights | 47.191588 | 0.471916 | 1.000x |
| 43 | 44 | +covariance | 3112.923201 | 31.129232 | 65.964x |
| 43 | 44 | +link quality | 61.327875 | 0.613279 | 1.300x |
| 43 | 44 | +cardinality consensus | 51.416305 | 0.514163 | 1.090x |
| 44 | 45 | fixed weights | 47.802044 | 0.478020 | 1.000x |
| 44 | 45 | +covariance | 52.372656 | 0.523727 | 1.096x |
| 44 | 45 | +link quality | 51.703583 | 0.517036 | 1.082x |
| 44 | 45 | +cardinality consensus | 51.157017 | 0.511570 | 1.070x |
| 45 | 46 | fixed weights | 47.288121 | 0.472881 | 1.000x |
| 45 | 46 | +covariance | 50.913267 | 0.509133 | 1.077x |
| 45 | 46 | +link quality | 49.883325 | 0.498833 | 1.055x |
| 45 | 46 | +cardinality consensus | 51.464321 | 0.514643 | 1.088x |
| 46 | 47 | fixed weights | 54.157439 | 0.541574 | 1.000x |
| 46 | 47 | +covariance | 60.980898 | 0.609809 | 1.126x |
| 46 | 47 | +link quality | 57.443183 | 0.574432 | 1.061x |
| 46 | 47 | +cardinality consensus | 63.235246 | 0.632352 | 1.168x |
| 47 | 48 | fixed weights | 57.007878 | 0.570079 | 1.000x |
| 47 | 48 | +covariance | 60.261646 | 0.602616 | 1.057x |
| 47 | 48 | +link quality | 55.701005 | 0.557010 | 0.977x |
| 47 | 48 | +cardinality consensus | 50.320071 | 0.503201 | 0.883x |
| 48 | 49 | fixed weights | 50.608106 | 0.506081 | 1.000x |
| 48 | 49 | +covariance | 55.341040 | 0.553410 | 1.094x |
| 48 | 49 | +link quality | 54.969463 | 0.549695 | 1.086x |
| 48 | 49 | +cardinality consensus | 55.984598 | 0.559846 | 1.106x |
| 49 | 50 | fixed weights | 50.673603 | 0.506736 | 1.000x |
| 49 | 50 | +covariance | 50.673201 | 0.506732 | 1.000x |
| 49 | 50 | +link quality | 50.421676 | 0.504217 | 0.995x |
| 49 | 50 | +cardinality consensus | 50.620093 | 0.506201 | 0.999x |
| 50 | 51 | fixed weights | 56.406422 | 0.564064 | 1.000x |
| 50 | 51 | +covariance | 64.788775 | 0.647888 | 1.149x |
| 50 | 51 | +link quality | 58.517179 | 0.585172 | 1.037x |
| 50 | 51 | +cardinality consensus | 61.553905 | 0.615539 | 1.091x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.862938 | 1.649569 | 1.455125 |
| +covariance | 2.685775 | 1.560762 | 1.154400 |
| +link quality | 2.329283 | 1.600391 | 0.578800 |
| +cardinality consensus | 2.334984 | 1.602751 | 0.590575 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.862938 +/- 0.123917 | [2.828590, 2.897286] | 50 |
| +covariance | E-OSPA | 2.685775 +/- 0.102619 | [2.657330, 2.714219] | 50 |
| +link quality | E-OSPA | 2.329283 +/- 0.071369 | [2.309500, 2.349066] | 50 |
| +cardinality consensus | E-OSPA | 2.334984 +/- 0.071566 | [2.315147, 2.354821] | 50 |
| fixed weights | RMSE | 1.649569 +/- 0.078096 | [1.627922, 1.671216] | 50 |
| +covariance | RMSE | 1.560762 +/- 0.052315 | [1.546261, 1.575263] | 50 |
| +link quality | RMSE | 1.600391 +/- 0.049191 | [1.586756, 1.614026] | 50 |
| +cardinality consensus | RMSE | 1.602751 +/- 0.051650 | [1.588435, 1.617068] | 50 |
| fixed weights | CardErr | 1.455125 +/- 0.241978 | [1.388052, 1.522198] | 50 |
| +covariance | CardErr | 1.154400 +/- 0.165774 | [1.108450, 1.200350] | 50 |
| +link quality | CardErr | 0.578800 +/- 0.066873 | [0.560264, 0.597336] | 50 |
| +cardinality consensus | CardErr | 0.590575 +/- 0.069320 | [0.571361, 0.609789] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +covariance | E-OSPA | 0.177164 +/- 0.046469 | [0.164283, 0.190044] | 6.19% | 50/50 | 1.776e-15 |
| +link quality | E-OSPA | 0.533655 +/- 0.088486 | [0.509128, 0.558182] | 18.64% | 50/50 | 1.776e-15 |
| +cardinality consensus | E-OSPA | 0.527954 +/- 0.089363 | [0.503184, 0.552724] | 18.44% | 50/50 | 1.776e-15 |
| +covariance | RMSE | 0.088807 +/- 0.055862 | [0.073323, 0.104292] | 5.38% | 50/50 | 1.776e-15 |
| +link quality | RMSE | 0.049178 +/- 0.059811 | [0.032599, 0.065757] | 2.98% | 45/50 | 4.21e-09 |
| +cardinality consensus | RMSE | 0.046818 +/- 0.060465 | [0.030058, 0.063578] | 2.84% | 45/50 | 4.21e-09 |
| +covariance | CardErr | 0.300725 +/- 0.106776 | [0.271128, 0.330322] | 20.67% | 50/50 | 1.776e-15 |
| +link quality | CardErr | 0.876325 +/- 0.211178 | [0.817789, 0.934861] | 60.22% | 50/50 | 1.776e-15 |
| +cardinality consensus | CardErr | 0.864550 +/- 0.209269 | [0.806543, 0.922557] | 59.41% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.719987 | 1.619776 | 1.083600 |
| 1 | +covariance | 2.542184 | 1.512042 | 0.865400 |
| 1 | +link quality | 2.284853 | 1.555869 | 0.541200 |
| 1 | +cardinality consensus | 2.290218 | 1.560303 | 0.551400 |
| 2 | fixed weights | 2.670351 | 1.594302 | 0.999200 |
| 2 | +covariance | 2.580268 | 1.520449 | 0.906600 |
| 2 | +link quality | 2.290607 | 1.565054 | 0.546400 |
| 2 | +cardinality consensus | 2.295793 | 1.568575 | 0.551000 |
| 3 | fixed weights | 2.740374 | 1.607566 | 1.150400 |
| 3 | +covariance | 2.556387 | 1.522307 | 0.906600 |
| 3 | +link quality | 2.295211 | 1.566326 | 0.570600 |
| 3 | +cardinality consensus | 2.297482 | 1.568329 | 0.576200 |
| 4 | fixed weights | 2.883255 | 1.654573 | 1.494400 |
| 4 | +covariance | 2.767406 | 1.581788 | 1.326400 |
| 4 | +link quality | 2.411042 | 1.633281 | 0.685400 |
| 4 | +cardinality consensus | 2.410082 | 1.633002 | 0.697200 |
| 5 | fixed weights | 2.918523 | 1.691261 | 1.520000 |
| 5 | +covariance | 2.813252 | 1.582814 | 1.422400 |
| 5 | +link quality | 2.371967 | 1.629439 | 0.623600 |
| 5 | +cardinality consensus | 2.383281 | 1.635184 | 0.645000 |
| 6 | fixed weights | 2.913753 | 1.708755 | 1.535200 |
| 6 | +covariance | 2.740251 | 1.585576 | 1.270600 |
| 6 | +link quality | 2.318031 | 1.613526 | 0.540200 |
| 6 | +cardinality consensus | 2.333067 | 1.617278 | 0.561400 |
| 7 | fixed weights | 2.925998 | 1.663623 | 1.565800 |
| 7 | +covariance | 2.774495 | 1.594569 | 1.315200 |
| 7 | +link quality | 2.364495 | 1.627267 | 0.604200 |
| 7 | +cardinality consensus | 2.365711 | 1.627874 | 0.612400 |
| 8 | fixed weights | 3.131265 | 1.656698 | 2.292400 |
| 8 | +covariance | 2.711954 | 1.586550 | 1.222000 |
| 8 | +link quality | 2.298057 | 1.612369 | 0.518800 |
| 8 | +cardinality consensus | 2.304240 | 1.611465 | 0.530000 |
