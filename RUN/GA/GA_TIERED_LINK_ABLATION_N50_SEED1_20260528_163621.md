# GA Tiered Link Ablation (2026-05-28 16:36:21)

Comparison order: fixed weights -> Cao-Zhao FID-FIA baseline -> +structure-aware decoupled KLA -> +FID-FIA existence refinement

## Run Config
- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.000
- pDropLevels: []
- pDropLevelCounts: []

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
- Trial 1: [0 0 0 0 0 0 0 0]
- Trial 2: [0 0 0 0 0 0 0 0]
- Trial 3: [0 0 0 0 0 0 0 0]
- Trial 4: [0 0 0 0 0 0 0 0]
- Trial 5: [0 0 0 0 0 0 0 0]
- Trial 6: [0 0 0 0 0 0 0 0]
- Trial 7: [0 0 0 0 0 0 0 0]
- Trial 8: [0 0 0 0 0 0 0 0]
- Trial 9: [0 0 0 0 0 0 0 0]
- Trial 10: [0 0 0 0 0 0 0 0]
- Trial 11: [0 0 0 0 0 0 0 0]
- Trial 12: [0 0 0 0 0 0 0 0]
- Trial 13: [0 0 0 0 0 0 0 0]
- Trial 14: [0 0 0 0 0 0 0 0]
- Trial 15: [0 0 0 0 0 0 0 0]
- Trial 16: [0 0 0 0 0 0 0 0]
- Trial 17: [0 0 0 0 0 0 0 0]
- Trial 18: [0 0 0 0 0 0 0 0]
- Trial 19: [0 0 0 0 0 0 0 0]
- Trial 20: [0 0 0 0 0 0 0 0]
- Trial 21: [0 0 0 0 0 0 0 0]
- Trial 22: [0 0 0 0 0 0 0 0]
- Trial 23: [0 0 0 0 0 0 0 0]
- Trial 24: [0 0 0 0 0 0 0 0]
- Trial 25: [0 0 0 0 0 0 0 0]
- Trial 26: [0 0 0 0 0 0 0 0]
- Trial 27: [0 0 0 0 0 0 0 0]
- Trial 28: [0 0 0 0 0 0 0 0]
- Trial 29: [0 0 0 0 0 0 0 0]
- Trial 30: [0 0 0 0 0 0 0 0]
- Trial 31: [0 0 0 0 0 0 0 0]
- Trial 32: [0 0 0 0 0 0 0 0]
- Trial 33: [0 0 0 0 0 0 0 0]
- Trial 34: [0 0 0 0 0 0 0 0]
- Trial 35: [0 0 0 0 0 0 0 0]
- Trial 36: [0 0 0 0 0 0 0 0]
- Trial 37: [0 0 0 0 0 0 0 0]
- Trial 38: [0 0 0 0 0 0 0 0]
- Trial 39: [0 0 0 0 0 0 0 0]
- Trial 40: [0 0 0 0 0 0 0 0]
- Trial 41: [0 0 0 0 0 0 0 0]
- Trial 42: [0 0 0 0 0 0 0 0]
- Trial 43: [0 0 0 0 0 0 0 0]
- Trial 44: [0 0 0 0 0 0 0 0]
- Trial 45: [0 0 0 0 0 0 0 0]
- Trial 46: [0 0 0 0 0 0 0 0]
- Trial 47: [0 0 0 0 0 0 0 0]
- Trial 48: [0 0 0 0 0 0 0 0]
- Trial 49: [0 0 0 0 0 0 0 0]
- Trial 50: [0 0 0 0 0 0 0 0]

## Per-Trial Network Disagreement Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | fixed weights | 1.650878 | 1.358804 | 0.093750 |
| 1 | 2 | Cao-Zhao FID-FIA baseline | 1.576684 | 1.359953 | 0.061250 |
| 1 | 2 | +structure-aware decoupled KLA | 1.423125 | 1.183512 | 0.067500 |
| 1 | 2 | +FID-FIA existence refinement | 1.454868 | 1.430767 | 0.055000 |
| 2 | 3 | fixed weights | 1.606838 | 1.378415 | 0.061250 |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 1.491200 | 1.328864 | 0.035000 |
| 2 | 3 | +structure-aware decoupled KLA | 1.387470 | 1.211753 | 0.043750 |
| 2 | 3 | +FID-FIA existence refinement | 1.401287 | 1.438796 | 0.040000 |
| 3 | 4 | fixed weights | 1.582975 | 1.375559 | 0.066250 |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 1.518703 | 1.340718 | 0.048750 |
| 3 | 4 | +structure-aware decoupled KLA | 1.380547 | 1.200794 | 0.056250 |
| 3 | 4 | +FID-FIA existence refinement | 1.440262 | 1.249647 | 0.050000 |
| 4 | 5 | fixed weights | 1.661952 | 1.390417 | 0.095000 |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 1.552154 | 1.357068 | 0.050000 |
| 4 | 5 | +structure-aware decoupled KLA | 1.482511 | 1.221932 | 0.091250 |
| 4 | 5 | +FID-FIA existence refinement | 1.421485 | 1.243866 | 0.040000 |
| 5 | 6 | fixed weights | 1.606861 | 1.371722 | 0.082500 |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 1.508649 | 1.322932 | 0.045000 |
| 5 | 6 | +structure-aware decoupled KLA | 1.406824 | 1.193824 | 0.070000 |
| 5 | 6 | +FID-FIA existence refinement | 1.403211 | 1.231325 | 0.030000 |
| 6 | 7 | fixed weights | 1.619482 | 1.402597 | 0.082500 |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 1.515673 | 1.325429 | 0.058750 |
| 6 | 7 | +structure-aware decoupled KLA | 1.407321 | 1.218000 | 0.065000 |
| 6 | 7 | +FID-FIA existence refinement | 1.424927 | 1.376171 | 0.048750 |
| 7 | 8 | fixed weights | 1.609838 | 1.320463 | 0.108750 |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 1.503562 | 1.273071 | 0.062500 |
| 7 | 8 | +structure-aware decoupled KLA | 1.384620 | 1.150343 | 0.063750 |
| 7 | 8 | +FID-FIA existence refinement | 1.382398 | 1.218607 | 0.043750 |
| 8 | 9 | fixed weights | 1.618389 | 1.339729 | 0.112500 |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 1.521106 | 1.301308 | 0.068750 |
| 8 | 9 | +structure-aware decoupled KLA | 1.414068 | 1.159126 | 0.090000 |
| 8 | 9 | +FID-FIA existence refinement | 1.411570 | 1.189067 | 0.061250 |
| 9 | 10 | fixed weights | 1.615154 | 1.441848 | 0.048750 |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 1.529979 | 1.339836 | 0.037500 |
| 9 | 10 | +structure-aware decoupled KLA | 1.404229 | 1.233939 | 0.040000 |
| 9 | 10 | +FID-FIA existence refinement | 1.403631 | 1.244247 | 0.027500 |
| 10 | 11 | fixed weights | 1.634175 | 1.365073 | 0.093750 |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 1.492009 | 1.285953 | 0.058750 |
| 10 | 11 | +structure-aware decoupled KLA | 1.416861 | 1.188414 | 0.076250 |
| 10 | 11 | +FID-FIA existence refinement | 1.444671 | 1.303870 | 0.075000 |
| 11 | 12 | fixed weights | 1.634779 | 1.436796 | 0.060000 |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 1.556432 | 1.371657 | 0.045000 |
| 11 | 12 | +structure-aware decoupled KLA | 1.428414 | 1.257893 | 0.040000 |
| 11 | 12 | +FID-FIA existence refinement | 1.530914 | 1.744707 | 0.065000 |
| 12 | 13 | fixed weights | 1.646082 | 1.361098 | 0.095000 |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 1.574093 | 1.374612 | 0.068750 |
| 12 | 13 | +structure-aware decoupled KLA | 1.417504 | 1.187319 | 0.071250 |
| 12 | 13 | +FID-FIA existence refinement | 1.430405 | 1.229836 | 0.052500 |
| 13 | 14 | fixed weights | 1.698178 | 1.442592 | 0.088750 |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 1.658929 | 1.400320 | 0.067500 |
| 13 | 14 | +structure-aware decoupled KLA | 1.474568 | 1.265545 | 0.068750 |
| 13 | 14 | +FID-FIA existence refinement | 1.504244 | 1.268134 | 0.066250 |
| 14 | 15 | fixed weights | 1.638514 | 1.375393 | 0.075000 |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 1.550249 | 1.328376 | 0.052500 |
| 14 | 15 | +structure-aware decoupled KLA | 1.442702 | 1.202483 | 0.067500 |
| 14 | 15 | +FID-FIA existence refinement | 1.464468 | 1.224987 | 0.053750 |
| 15 | 16 | fixed weights | 1.690784 | 1.388506 | 0.108750 |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 1.480138 | 1.286621 | 0.055000 |
| 15 | 16 | +structure-aware decoupled KLA | 1.461350 | 1.227628 | 0.066250 |
| 15 | 16 | +FID-FIA existence refinement | 1.441616 | 1.243822 | 0.048750 |
| 16 | 17 | fixed weights | 1.669561 | 1.381232 | 0.098750 |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 1.529579 | 1.290892 | 0.061250 |
| 16 | 17 | +structure-aware decoupled KLA | 1.420025 | 1.162603 | 0.066250 |
| 16 | 17 | +FID-FIA existence refinement | 1.399719 | 1.222217 | 0.058750 |
| 17 | 18 | fixed weights | 1.651100 | 1.411833 | 0.080000 |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 1.564532 | 1.364642 | 0.055000 |
| 17 | 18 | +structure-aware decoupled KLA | 1.448598 | 1.239124 | 0.063750 |
| 17 | 18 | +FID-FIA existence refinement | 1.510583 | 1.686997 | 0.063750 |
| 18 | 19 | fixed weights | 1.593401 | 1.285643 | 0.113750 |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 1.463397 | 1.268650 | 0.065000 |
| 18 | 19 | +structure-aware decoupled KLA | 1.411213 | 1.123839 | 0.100000 |
| 18 | 19 | +FID-FIA existence refinement | 1.373100 | 1.184614 | 0.042500 |
| 19 | 20 | fixed weights | 1.604889 | 1.333492 | 0.075000 |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 1.523979 | 1.343827 | 0.040000 |
| 19 | 20 | +structure-aware decoupled KLA | 1.414734 | 1.164961 | 0.065000 |
| 19 | 20 | +FID-FIA existence refinement | 1.390336 | 1.198149 | 0.043750 |
| 20 | 21 | fixed weights | 1.720672 | 1.434714 | 0.095000 |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 1.532123 | 1.374982 | 0.046250 |
| 20 | 21 | +structure-aware decoupled KLA | 1.459110 | 1.224580 | 0.065000 |
| 20 | 21 | +FID-FIA existence refinement | 1.423206 | 1.258635 | 0.031250 |
| 21 | 22 | fixed weights | 1.595483 | 1.378553 | 0.073750 |
| 21 | 22 | Cao-Zhao FID-FIA baseline | 1.433194 | 1.256580 | 0.041250 |
| 21 | 22 | +structure-aware decoupled KLA | 1.395513 | 1.213611 | 0.055000 |
| 21 | 22 | +FID-FIA existence refinement | 1.412158 | 1.229824 | 0.033750 |
| 22 | 23 | fixed weights | 1.612717 | 1.411055 | 0.063750 |
| 22 | 23 | Cao-Zhao FID-FIA baseline | 1.567765 | 1.381262 | 0.042500 |
| 22 | 23 | +structure-aware decoupled KLA | 1.420318 | 1.210200 | 0.053750 |
| 22 | 23 | +FID-FIA existence refinement | 1.414425 | 1.369089 | 0.023750 |
| 23 | 24 | fixed weights | 1.633547 | 1.404581 | 0.090000 |
| 23 | 24 | Cao-Zhao FID-FIA baseline | 1.565636 | 1.358835 | 0.056250 |
| 23 | 24 | +structure-aware decoupled KLA | 1.462224 | 1.257201 | 0.081250 |
| 23 | 24 | +FID-FIA existence refinement | 1.445199 | 1.268518 | 0.043750 |
| 24 | 25 | fixed weights | 1.590987 | 1.381488 | 0.065000 |
| 24 | 25 | Cao-Zhao FID-FIA baseline | 1.544140 | 1.353452 | 0.042500 |
| 24 | 25 | +structure-aware decoupled KLA | 1.402619 | 1.210662 | 0.053750 |
| 24 | 25 | +FID-FIA existence refinement | 1.422630 | 1.226904 | 0.047500 |
| 25 | 26 | fixed weights | 1.574382 | 1.328371 | 0.086250 |
| 25 | 26 | Cao-Zhao FID-FIA baseline | 1.526398 | 1.309294 | 0.070000 |
| 25 | 26 | +structure-aware decoupled KLA | 1.368118 | 1.167088 | 0.063750 |
| 25 | 26 | +FID-FIA existence refinement | 1.441630 | 1.249286 | 0.056250 |
| 26 | 27 | fixed weights | 1.583305 | 1.308135 | 0.072500 |
| 26 | 27 | Cao-Zhao FID-FIA baseline | 1.510275 | 1.321139 | 0.040000 |
| 26 | 27 | +structure-aware decoupled KLA | 1.397369 | 1.176239 | 0.055000 |
| 26 | 27 | +FID-FIA existence refinement | 1.417786 | 1.207076 | 0.046250 |
| 27 | 28 | fixed weights | 1.741169 | 1.374388 | 0.121250 |
| 27 | 28 | Cao-Zhao FID-FIA baseline | 1.553181 | 1.307568 | 0.067500 |
| 27 | 28 | +structure-aware decoupled KLA | 1.534901 | 1.204125 | 0.110000 |
| 27 | 28 | +FID-FIA existence refinement | 1.459875 | 1.252190 | 0.047500 |
| 28 | 29 | fixed weights | 1.700899 | 1.329079 | 0.156250 |
| 28 | 29 | Cao-Zhao FID-FIA baseline | 1.535576 | 1.346344 | 0.072500 |
| 28 | 29 | +structure-aware decoupled KLA | 1.490984 | 1.160294 | 0.132500 |
| 28 | 29 | +FID-FIA existence refinement | 1.408207 | 1.219146 | 0.053750 |
| 29 | 30 | fixed weights | 1.645895 | 1.392230 | 0.080000 |
| 29 | 30 | Cao-Zhao FID-FIA baseline | 1.572072 | 1.365956 | 0.052500 |
| 29 | 30 | +structure-aware decoupled KLA | 1.421890 | 1.224257 | 0.052500 |
| 29 | 30 | +FID-FIA existence refinement | 1.498842 | 1.270119 | 0.053750 |
| 30 | 31 | fixed weights | 1.607163 | 1.362216 | 0.076250 |
| 30 | 31 | Cao-Zhao FID-FIA baseline | 1.545748 | 1.313625 | 0.056250 |
| 30 | 31 | +structure-aware decoupled KLA | 1.398993 | 1.201504 | 0.057500 |
| 30 | 31 | +FID-FIA existence refinement | 1.461772 | 1.216251 | 0.056250 |
| 31 | 32 | fixed weights | 1.607239 | 1.353930 | 0.085000 |
| 31 | 32 | Cao-Zhao FID-FIA baseline | 1.482882 | 1.297056 | 0.050000 |
| 31 | 32 | +structure-aware decoupled KLA | 1.391331 | 1.175269 | 0.071250 |
| 31 | 32 | +FID-FIA existence refinement | 1.386492 | 1.196584 | 0.041250 |
| 32 | 33 | fixed weights | 1.664329 | 1.367188 | 0.148750 |
| 32 | 33 | Cao-Zhao FID-FIA baseline | 1.553168 | 1.352174 | 0.067500 |
| 32 | 33 | +structure-aware decoupled KLA | 1.426588 | 1.188891 | 0.083750 |
| 32 | 33 | +FID-FIA existence refinement | 1.405462 | 1.217345 | 0.036250 |
| 33 | 34 | fixed weights | 1.594309 | 1.418825 | 0.061250 |
| 33 | 34 | Cao-Zhao FID-FIA baseline | 1.560398 | 1.339881 | 0.046250 |
| 33 | 34 | +structure-aware decoupled KLA | 1.396445 | 1.204412 | 0.055000 |
| 33 | 34 | +FID-FIA existence refinement | 1.507082 | 1.643424 | 0.041250 |
| 34 | 35 | fixed weights | 1.710505 | 1.505018 | 0.102500 |
| 34 | 35 | Cao-Zhao FID-FIA baseline | 1.585460 | 1.397058 | 0.077500 |
| 34 | 35 | +structure-aware decoupled KLA | 1.484747 | 1.249445 | 0.082500 |
| 34 | 35 | +FID-FIA existence refinement | 1.458553 | 1.295854 | 0.053750 |
| 35 | 36 | fixed weights | 1.632125 | 1.390001 | 0.098750 |
| 35 | 36 | Cao-Zhao FID-FIA baseline | 1.509122 | 1.316113 | 0.056250 |
| 35 | 36 | +structure-aware decoupled KLA | 1.410947 | 1.221383 | 0.065000 |
| 35 | 36 | +FID-FIA existence refinement | 1.448184 | 1.907871 | 0.041250 |
| 36 | 37 | fixed weights | 1.619751 | 1.368349 | 0.092500 |
| 36 | 37 | Cao-Zhao FID-FIA baseline | 1.516538 | 1.346957 | 0.047500 |
| 36 | 37 | +structure-aware decoupled KLA | 1.400700 | 1.194972 | 0.066250 |
| 36 | 37 | +FID-FIA existence refinement | 1.408550 | 1.225794 | 0.041250 |
| 37 | 38 | fixed weights | 1.576180 | 1.351863 | 0.080000 |
| 37 | 38 | Cao-Zhao FID-FIA baseline | 1.509673 | 1.301056 | 0.067500 |
| 37 | 38 | +structure-aware decoupled KLA | 1.391133 | 1.183147 | 0.070000 |
| 37 | 38 | +FID-FIA existence refinement | 1.411646 | 1.228246 | 0.057500 |
| 38 | 39 | fixed weights | 1.602643 | 1.362600 | 0.081250 |
| 38 | 39 | Cao-Zhao FID-FIA baseline | 1.528854 | 1.293549 | 0.082500 |
| 38 | 39 | +structure-aware decoupled KLA | 1.429410 | 1.197496 | 0.083750 |
| 38 | 39 | +FID-FIA existence refinement | 1.434415 | 1.230313 | 0.057500 |
| 39 | 40 | fixed weights | 1.617306 | 1.391714 | 0.071250 |
| 39 | 40 | Cao-Zhao FID-FIA baseline | 1.553068 | 1.301783 | 0.093750 |
| 39 | 40 | +structure-aware decoupled KLA | 1.414807 | 1.206776 | 0.065000 |
| 39 | 40 | +FID-FIA existence refinement | 1.433304 | 1.247198 | 0.060000 |
| 40 | 41 | fixed weights | 1.600522 | 1.360449 | 0.085000 |
| 40 | 41 | Cao-Zhao FID-FIA baseline | 1.567916 | 1.366317 | 0.052500 |
| 40 | 41 | +structure-aware decoupled KLA | 1.391339 | 1.175362 | 0.068750 |
| 40 | 41 | +FID-FIA existence refinement | 1.414574 | 1.328816 | 0.047500 |
| 41 | 42 | fixed weights | 1.578523 | 1.357841 | 0.086250 |
| 41 | 42 | Cao-Zhao FID-FIA baseline | 1.519996 | 1.351283 | 0.066250 |
| 41 | 42 | +structure-aware decoupled KLA | 1.395976 | 1.199077 | 0.082500 |
| 41 | 42 | +FID-FIA existence refinement | 1.423741 | 1.815408 | 0.053750 |
| 42 | 43 | fixed weights | 1.679407 | 1.408957 | 0.086250 |
| 42 | 43 | Cao-Zhao FID-FIA baseline | 1.566709 | 1.374036 | 0.051250 |
| 42 | 43 | +structure-aware decoupled KLA | 1.482537 | 1.228836 | 0.076250 |
| 42 | 43 | +FID-FIA existence refinement | 1.437433 | 1.229739 | 0.057500 |
| 43 | 44 | fixed weights | 1.613978 | 1.390679 | 0.086250 |
| 43 | 44 | Cao-Zhao FID-FIA baseline | 1.504940 | 1.304688 | 0.053750 |
| 43 | 44 | +structure-aware decoupled KLA | 1.399202 | 1.199706 | 0.062500 |
| 43 | 44 | +FID-FIA existence refinement | 1.426997 | 1.216707 | 0.058750 |
| 44 | 45 | fixed weights | 1.609875 | 1.414953 | 0.090000 |
| 44 | 45 | Cao-Zhao FID-FIA baseline | 1.534694 | 1.320749 | 0.070000 |
| 44 | 45 | +structure-aware decoupled KLA | 1.417499 | 1.171032 | 0.082500 |
| 44 | 45 | +FID-FIA existence refinement | 1.410519 | 1.191218 | 0.051250 |
| 45 | 46 | fixed weights | 1.672819 | 1.356082 | 0.115000 |
| 45 | 46 | Cao-Zhao FID-FIA baseline | 1.544371 | 1.323729 | 0.081250 |
| 45 | 46 | +structure-aware decoupled KLA | 1.441686 | 1.179232 | 0.091250 |
| 45 | 46 | +FID-FIA existence refinement | 1.422938 | 1.254601 | 0.061250 |
| 46 | 47 | fixed weights | 1.667269 | 1.364952 | 0.117500 |
| 46 | 47 | Cao-Zhao FID-FIA baseline | 1.562726 | 1.326980 | 0.071250 |
| 46 | 47 | +structure-aware decoupled KLA | 1.476725 | 1.209041 | 0.093750 |
| 46 | 47 | +FID-FIA existence refinement | 1.432847 | 1.223036 | 0.050000 |
| 47 | 48 | fixed weights | 1.633059 | 1.427702 | 0.081250 |
| 47 | 48 | Cao-Zhao FID-FIA baseline | 1.536472 | 1.387811 | 0.040000 |
| 47 | 48 | +structure-aware decoupled KLA | 1.425806 | 1.257931 | 0.055000 |
| 47 | 48 | +FID-FIA existence refinement | 1.442628 | 1.293220 | 0.027500 |
| 48 | 49 | fixed weights | 1.660203 | 1.390574 | 0.111250 |
| 48 | 49 | Cao-Zhao FID-FIA baseline | 1.544122 | 1.345103 | 0.060000 |
| 48 | 49 | +structure-aware decoupled KLA | 1.475123 | 1.200305 | 0.082500 |
| 48 | 49 | +FID-FIA existence refinement | 1.421857 | 1.209958 | 0.056250 |
| 49 | 50 | fixed weights | 1.673478 | 1.344760 | 0.106250 |
| 49 | 50 | Cao-Zhao FID-FIA baseline | 1.519035 | 1.320362 | 0.056250 |
| 49 | 50 | +structure-aware decoupled KLA | 1.475809 | 1.171268 | 0.092500 |
| 49 | 50 | +FID-FIA existence refinement | 1.458390 | 1.232758 | 0.047500 |
| 50 | 51 | fixed weights | 1.660892 | 1.479252 | 0.100000 |
| 50 | 51 | Cao-Zhao FID-FIA baseline | 1.514452 | 1.322770 | 0.055000 |
| 50 | 51 | +structure-aware decoupled KLA | 1.446368 | 1.235389 | 0.082500 |
| 50 | 51 | +FID-FIA existence refinement | 1.418195 | 1.249353 | 0.031250 |

## Network Disagreement Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 1.634289 | 1.381434 | 0.090125 |
| Cao-Zhao FID-FIA baseline | 1.534235 | 1.332864 | 0.057400 |
| +structure-aware decoupled KLA | 1.427058 | 1.201835 | 0.071300 |
| +FID-FIA existence refinement | 1.432865 | 1.302686 | 0.048650 |

## Network Disagreement Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 1.634289 +/- 0.040164 | [1.623156, 1.645422] | 50 |
| Cao-Zhao FID-FIA baseline | OSPA | 1.534235 +/- 0.035889 | [1.524287, 1.544183] | 50 |
| +structure-aware decoupled KLA | OSPA | 1.427058 +/- 0.035445 | [1.417233, 1.436883] | 50 |
| +FID-FIA existence refinement | OSPA | 1.432865 +/- 0.033613 | [1.423547, 1.442182] | 50 |
| fixed weights | RMSE | 1.381434 +/- 0.040960 | [1.370081, 1.392788] | 50 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.332864 +/- 0.034054 | [1.323425, 1.342304] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.201835 +/- 0.030438 | [1.193398, 1.210272] | 50 |
| +FID-FIA existence refinement | RMSE | 1.302686 +/- 0.165901 | [1.256701, 1.348672] | 50 |
| fixed weights | Cardinality | 0.090125 +/- 0.020982 | [0.084309, 0.095941] | 50 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.057400 +/- 0.012797 | [0.053853, 0.060947] | 50 |
| +structure-aware decoupled KLA | Cardinality | 0.071300 +/- 0.017489 | [0.066452, 0.076148] | 50 |
| +FID-FIA existence refinement | Cardinality | 0.048650 +/- 0.011146 | [0.045561, 0.051739] | 50 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | OSPA | 0.100054 +/- 0.041310 | [0.088604, 0.111505] | 6.12% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | OSPA | 0.207231 +/- 0.018901 | [0.201992, 0.212470] | 12.68% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | OSPA | 0.201425 +/- 0.044926 | [0.188972, 0.213878] | 12.32% | 50/50 | 1.776e-15 |
| Cao-Zhao FID-FIA baseline | RMSE | 0.048570 +/- 0.037032 | [0.038305, 0.058835] | 3.52% | 44/50 | 3.244e-08 |
| +structure-aware decoupled KLA | RMSE | 0.179599 +/- 0.023434 | [0.173103, 0.186095] | 13.00% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | RMSE | 0.078748 +/- 0.161489 | [0.033986, 0.123511] | 5.70% | 43/50 | 2.099e-07 |
| Cao-Zhao FID-FIA baseline | Cardinality | 0.032725 +/- 0.017871 | [0.027771, 0.037679] | 36.31% | 48/50 | 2.267e-12 |
| +structure-aware decoupled KLA | Cardinality | 0.018825 +/- 0.011722 | [0.015576, 0.022074] | 20.89% | 49/50 | 9.059e-14 |
| +FID-FIA existence refinement | Cardinality | 0.041475 +/- 0.022580 | [0.035216, 0.047734] | 46.02% | 49/50 | 9.059e-14 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 57.735435 +/- 5.526783 | 0.577354 | 1.000x | 50 |
| Cao-Zhao FID-FIA baseline | 151.611051 +/- 9.620478 | 1.516111 | 2.640x | 50 |
| +structure-aware decoupled KLA | 61.660102 +/- 3.726443 | 0.616601 | 1.073x | 50 |
| +FID-FIA existence refinement | 168.227822 +/- 14.272239 | 1.682278 | 2.928x | 50 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| Cao-Zhao FID-FIA baseline | 93.875616 +/- 9.125393 | 163.96% | 50/50 |
| +structure-aware decoupled KLA | 3.924667 +/- 4.854036 | 7.29% | 46/50 |
| +FID-FIA existence refinement | 110.492386 +/- 13.482375 | 192.80% | 50/50 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 2 | fixed weights | 54.893094 | 0.548931 | 1.000x |
| 1 | 2 | Cao-Zhao FID-FIA baseline | 143.096649 | 1.430966 | 2.607x |
| 1 | 2 | +structure-aware decoupled KLA | 59.182133 | 0.591821 | 1.078x |
| 1 | 2 | +FID-FIA existence refinement | 158.994263 | 1.589943 | 2.896x |
| 2 | 3 | fixed weights | 54.383254 | 0.543833 | 1.000x |
| 2 | 3 | Cao-Zhao FID-FIA baseline | 141.329094 | 1.413291 | 2.599x |
| 2 | 3 | +structure-aware decoupled KLA | 59.039204 | 0.590392 | 1.086x |
| 2 | 3 | +FID-FIA existence refinement | 157.802959 | 1.578030 | 2.902x |
| 3 | 4 | fixed weights | 55.457514 | 0.554575 | 1.000x |
| 3 | 4 | Cao-Zhao FID-FIA baseline | 147.572182 | 1.475722 | 2.661x |
| 3 | 4 | +structure-aware decoupled KLA | 59.906541 | 0.599065 | 1.080x |
| 3 | 4 | +FID-FIA existence refinement | 174.760190 | 1.747602 | 3.151x |
| 4 | 5 | fixed weights | 55.585876 | 0.555859 | 1.000x |
| 4 | 5 | Cao-Zhao FID-FIA baseline | 146.499062 | 1.464991 | 2.636x |
| 4 | 5 | +structure-aware decoupled KLA | 59.817795 | 0.598178 | 1.076x |
| 4 | 5 | +FID-FIA existence refinement | 190.342081 | 1.903421 | 3.424x |
| 5 | 6 | fixed weights | 79.163835 | 0.791638 | 1.000x |
| 5 | 6 | Cao-Zhao FID-FIA baseline | 166.378825 | 1.663788 | 2.102x |
| 5 | 6 | +structure-aware decoupled KLA | 65.805302 | 0.658053 | 0.831x |
| 5 | 6 | +FID-FIA existence refinement | 165.506861 | 1.655069 | 2.091x |
| 6 | 7 | fixed weights | 56.424827 | 0.564248 | 1.000x |
| 6 | 7 | Cao-Zhao FID-FIA baseline | 154.858276 | 1.548583 | 2.745x |
| 6 | 7 | +structure-aware decoupled KLA | 64.972924 | 0.649729 | 1.151x |
| 6 | 7 | +FID-FIA existence refinement | 170.335123 | 1.703351 | 3.019x |
| 7 | 8 | fixed weights | 56.678707 | 0.566787 | 1.000x |
| 7 | 8 | Cao-Zhao FID-FIA baseline | 153.725427 | 1.537254 | 2.712x |
| 7 | 8 | +structure-aware decoupled KLA | 62.594792 | 0.625948 | 1.104x |
| 7 | 8 | +FID-FIA existence refinement | 166.645628 | 1.666456 | 2.940x |
| 8 | 9 | fixed weights | 56.684196 | 0.566842 | 1.000x |
| 8 | 9 | Cao-Zhao FID-FIA baseline | 147.605290 | 1.476053 | 2.604x |
| 8 | 9 | +structure-aware decoupled KLA | 59.690773 | 0.596908 | 1.053x |
| 8 | 9 | +FID-FIA existence refinement | 161.857308 | 1.618573 | 2.855x |
| 9 | 10 | fixed weights | 55.300080 | 0.553001 | 1.000x |
| 9 | 10 | Cao-Zhao FID-FIA baseline | 155.140987 | 1.551410 | 2.805x |
| 9 | 10 | +structure-aware decoupled KLA | 65.200441 | 0.652004 | 1.179x |
| 9 | 10 | +FID-FIA existence refinement | 163.677281 | 1.636773 | 2.960x |
| 10 | 11 | fixed weights | 55.292036 | 0.552920 | 1.000x |
| 10 | 11 | Cao-Zhao FID-FIA baseline | 144.665268 | 1.446653 | 2.616x |
| 10 | 11 | +structure-aware decoupled KLA | 59.726056 | 0.597261 | 1.080x |
| 10 | 11 | +FID-FIA existence refinement | 158.544881 | 1.585449 | 2.867x |
| 11 | 12 | fixed weights | 55.619342 | 0.556193 | 1.000x |
| 11 | 12 | Cao-Zhao FID-FIA baseline | 149.654244 | 1.496542 | 2.691x |
| 11 | 12 | +structure-aware decoupled KLA | 60.006412 | 0.600064 | 1.079x |
| 11 | 12 | +FID-FIA existence refinement | 177.494031 | 1.774940 | 3.191x |
| 12 | 13 | fixed weights | 55.930300 | 0.559303 | 1.000x |
| 12 | 13 | Cao-Zhao FID-FIA baseline | 147.755448 | 1.477554 | 2.642x |
| 12 | 13 | +structure-aware decoupled KLA | 60.363006 | 0.603630 | 1.079x |
| 12 | 13 | +FID-FIA existence refinement | 159.507250 | 1.595073 | 2.852x |
| 13 | 14 | fixed weights | 55.898692 | 0.558987 | 1.000x |
| 13 | 14 | Cao-Zhao FID-FIA baseline | 150.782371 | 1.507824 | 2.697x |
| 13 | 14 | +structure-aware decoupled KLA | 60.551207 | 0.605512 | 1.083x |
| 13 | 14 | +FID-FIA existence refinement | 172.509645 | 1.725096 | 3.086x |
| 14 | 15 | fixed weights | 55.157296 | 0.551573 | 1.000x |
| 14 | 15 | Cao-Zhao FID-FIA baseline | 151.210337 | 1.512103 | 2.741x |
| 14 | 15 | +structure-aware decoupled KLA | 59.744914 | 0.597449 | 1.083x |
| 14 | 15 | +FID-FIA existence refinement | 161.770335 | 1.617703 | 2.933x |
| 15 | 16 | fixed weights | 56.076692 | 0.560767 | 1.000x |
| 15 | 16 | Cao-Zhao FID-FIA baseline | 147.821627 | 1.478216 | 2.636x |
| 15 | 16 | +structure-aware decoupled KLA | 60.288477 | 0.602885 | 1.075x |
| 15 | 16 | +FID-FIA existence refinement | 163.252297 | 1.632523 | 2.911x |
| 16 | 17 | fixed weights | 55.748473 | 0.557485 | 1.000x |
| 16 | 17 | Cao-Zhao FID-FIA baseline | 146.096886 | 1.460969 | 2.621x |
| 16 | 17 | +structure-aware decoupled KLA | 59.958303 | 0.599583 | 1.076x |
| 16 | 17 | +FID-FIA existence refinement | 157.206307 | 1.572063 | 2.820x |
| 17 | 18 | fixed weights | 55.175164 | 0.551752 | 1.000x |
| 17 | 18 | Cao-Zhao FID-FIA baseline | 142.884084 | 1.428841 | 2.590x |
| 17 | 18 | +structure-aware decoupled KLA | 59.459424 | 0.594594 | 1.078x |
| 17 | 18 | +FID-FIA existence refinement | 157.092930 | 1.570929 | 2.847x |
| 18 | 19 | fixed weights | 55.657346 | 0.556573 | 1.000x |
| 18 | 19 | Cao-Zhao FID-FIA baseline | 199.285739 | 1.992857 | 3.581x |
| 18 | 19 | +structure-aware decoupled KLA | 59.538574 | 0.595386 | 1.070x |
| 18 | 19 | +FID-FIA existence refinement | 156.919952 | 1.569200 | 2.819x |
| 19 | 20 | fixed weights | 55.714084 | 0.557141 | 1.000x |
| 19 | 20 | Cao-Zhao FID-FIA baseline | 146.801493 | 1.468015 | 2.635x |
| 19 | 20 | +structure-aware decoupled KLA | 60.012618 | 0.600126 | 1.077x |
| 19 | 20 | +FID-FIA existence refinement | 158.422416 | 1.584224 | 2.843x |
| 20 | 21 | fixed weights | 55.260750 | 0.552608 | 1.000x |
| 20 | 21 | Cao-Zhao FID-FIA baseline | 145.832133 | 1.458321 | 2.639x |
| 20 | 21 | +structure-aware decoupled KLA | 59.413471 | 0.594135 | 1.075x |
| 20 | 21 | +FID-FIA existence refinement | 157.057587 | 1.570576 | 2.842x |
| 21 | 22 | fixed weights | 55.604343 | 0.556043 | 1.000x |
| 21 | 22 | Cao-Zhao FID-FIA baseline | 145.548473 | 1.455485 | 2.618x |
| 21 | 22 | +structure-aware decoupled KLA | 59.836142 | 0.598361 | 1.076x |
| 21 | 22 | +FID-FIA existence refinement | 161.426928 | 1.614269 | 2.903x |
| 22 | 23 | fixed weights | 55.607076 | 0.556071 | 1.000x |
| 22 | 23 | Cao-Zhao FID-FIA baseline | 147.853745 | 1.478537 | 2.659x |
| 22 | 23 | +structure-aware decoupled KLA | 59.943408 | 0.599434 | 1.078x |
| 22 | 23 | +FID-FIA existence refinement | 159.128326 | 1.591283 | 2.862x |
| 23 | 24 | fixed weights | 56.062991 | 0.560630 | 1.000x |
| 23 | 24 | Cao-Zhao FID-FIA baseline | 147.466683 | 1.474667 | 2.630x |
| 23 | 24 | +structure-aware decoupled KLA | 60.710673 | 0.607107 | 1.083x |
| 23 | 24 | +FID-FIA existence refinement | 162.505378 | 1.625054 | 2.899x |
| 24 | 25 | fixed weights | 55.195492 | 0.551955 | 1.000x |
| 24 | 25 | Cao-Zhao FID-FIA baseline | 143.420150 | 1.434201 | 2.598x |
| 24 | 25 | +structure-aware decoupled KLA | 60.179592 | 0.601796 | 1.090x |
| 24 | 25 | +FID-FIA existence refinement | 211.296659 | 2.112967 | 3.828x |
| 25 | 26 | fixed weights | 55.857241 | 0.558572 | 1.000x |
| 25 | 26 | Cao-Zhao FID-FIA baseline | 150.974089 | 1.509741 | 2.703x |
| 25 | 26 | +structure-aware decoupled KLA | 60.240967 | 0.602410 | 1.078x |
| 25 | 26 | +FID-FIA existence refinement | 169.710880 | 1.697109 | 3.038x |
| 26 | 27 | fixed weights | 55.626018 | 0.556260 | 1.000x |
| 26 | 27 | Cao-Zhao FID-FIA baseline | 144.771263 | 1.447713 | 2.603x |
| 26 | 27 | +structure-aware decoupled KLA | 59.773041 | 0.597730 | 1.075x |
| 26 | 27 | +FID-FIA existence refinement | 156.005719 | 1.560057 | 2.805x |
| 27 | 28 | fixed weights | 55.517551 | 0.555176 | 1.000x |
| 27 | 28 | Cao-Zhao FID-FIA baseline | 149.574924 | 1.495749 | 2.694x |
| 27 | 28 | +structure-aware decoupled KLA | 59.857358 | 0.598574 | 1.078x |
| 27 | 28 | +FID-FIA existence refinement | 161.714809 | 1.617148 | 2.913x |
| 28 | 29 | fixed weights | 55.636151 | 0.556362 | 1.000x |
| 28 | 29 | Cao-Zhao FID-FIA baseline | 146.124847 | 1.461248 | 2.626x |
| 28 | 29 | +structure-aware decoupled KLA | 60.035072 | 0.600351 | 1.079x |
| 28 | 29 | +FID-FIA existence refinement | 160.372517 | 1.603725 | 2.883x |
| 29 | 30 | fixed weights | 55.728489 | 0.557285 | 1.000x |
| 29 | 30 | Cao-Zhao FID-FIA baseline | 147.849193 | 1.478492 | 2.653x |
| 29 | 30 | +structure-aware decoupled KLA | 60.002520 | 0.600025 | 1.077x |
| 29 | 30 | +FID-FIA existence refinement | 169.239995 | 1.692400 | 3.037x |
| 30 | 31 | fixed weights | 55.727143 | 0.557271 | 1.000x |
| 30 | 31 | Cao-Zhao FID-FIA baseline | 150.192113 | 1.501921 | 2.695x |
| 30 | 31 | +structure-aware decoupled KLA | 60.069258 | 0.600693 | 1.078x |
| 30 | 31 | +FID-FIA existence refinement | 178.044720 | 1.780447 | 3.195x |
| 31 | 32 | fixed weights | 68.153671 | 0.681537 | 1.000x |
| 31 | 32 | Cao-Zhao FID-FIA baseline | 155.950049 | 1.559500 | 2.288x |
| 31 | 32 | +structure-aware decoupled KLA | 63.096591 | 0.630966 | 0.926x |
| 31 | 32 | +FID-FIA existence refinement | 175.074338 | 1.750743 | 2.569x |
| 32 | 33 | fixed weights | 56.852126 | 0.568521 | 1.000x |
| 32 | 33 | Cao-Zhao FID-FIA baseline | 151.843619 | 1.518436 | 2.671x |
| 32 | 33 | +structure-aware decoupled KLA | 61.127307 | 0.611273 | 1.075x |
| 32 | 33 | +FID-FIA existence refinement | 170.662703 | 1.706627 | 3.002x |
| 33 | 34 | fixed weights | 61.696228 | 0.616962 | 1.000x |
| 33 | 34 | Cao-Zhao FID-FIA baseline | 156.202870 | 1.562029 | 2.532x |
| 33 | 34 | +structure-aware decoupled KLA | 64.875440 | 0.648754 | 1.052x |
| 33 | 34 | +FID-FIA existence refinement | 160.032818 | 1.600328 | 2.594x |
| 34 | 35 | fixed weights | 55.598160 | 0.555982 | 1.000x |
| 34 | 35 | Cao-Zhao FID-FIA baseline | 144.187124 | 1.441871 | 2.593x |
| 34 | 35 | +structure-aware decoupled KLA | 60.234762 | 0.602348 | 1.083x |
| 34 | 35 | +FID-FIA existence refinement | 159.617484 | 1.596175 | 2.871x |
| 35 | 36 | fixed weights | 55.371676 | 0.553717 | 1.000x |
| 35 | 36 | Cao-Zhao FID-FIA baseline | 147.482843 | 1.474828 | 2.664x |
| 35 | 36 | +structure-aware decoupled KLA | 59.791619 | 0.597916 | 1.080x |
| 35 | 36 | +FID-FIA existence refinement | 164.208111 | 1.642081 | 2.966x |
| 36 | 37 | fixed weights | 56.120776 | 0.561208 | 1.000x |
| 36 | 37 | Cao-Zhao FID-FIA baseline | 147.411558 | 1.474116 | 2.627x |
| 36 | 37 | +structure-aware decoupled KLA | 60.222987 | 0.602230 | 1.073x |
| 36 | 37 | +FID-FIA existence refinement | 162.455297 | 1.624553 | 2.895x |
| 37 | 38 | fixed weights | 55.230457 | 0.552305 | 1.000x |
| 37 | 38 | Cao-Zhao FID-FIA baseline | 158.267340 | 1.582673 | 2.866x |
| 37 | 38 | +structure-aware decoupled KLA | 69.736500 | 0.697365 | 1.263x |
| 37 | 38 | +FID-FIA existence refinement | 209.665938 | 2.096659 | 3.796x |
| 38 | 39 | fixed weights | 68.565630 | 0.685656 | 1.000x |
| 38 | 39 | Cao-Zhao FID-FIA baseline | 161.673716 | 1.616737 | 2.358x |
| 38 | 39 | +structure-aware decoupled KLA | 66.311898 | 0.663119 | 0.967x |
| 38 | 39 | +FID-FIA existence refinement | 223.386801 | 2.233868 | 3.258x |
| 39 | 40 | fixed weights | 81.568152 | 0.815682 | 1.000x |
| 39 | 40 | Cao-Zhao FID-FIA baseline | 158.706571 | 1.587066 | 1.946x |
| 39 | 40 | +structure-aware decoupled KLA | 66.274470 | 0.662745 | 0.813x |
| 39 | 40 | +FID-FIA existence refinement | 190.201378 | 1.902014 | 2.332x |
| 40 | 41 | fixed weights | 61.204450 | 0.612045 | 1.000x |
| 40 | 41 | Cao-Zhao FID-FIA baseline | 172.612641 | 1.726126 | 2.820x |
| 40 | 41 | +structure-aware decoupled KLA | 68.323182 | 0.683232 | 1.116x |
| 40 | 41 | +FID-FIA existence refinement | 172.328996 | 1.723290 | 2.816x |
| 41 | 42 | fixed weights | 64.156800 | 0.641568 | 1.000x |
| 41 | 42 | Cao-Zhao FID-FIA baseline | 161.926891 | 1.619269 | 2.524x |
| 41 | 42 | +structure-aware decoupled KLA | 80.865762 | 0.808658 | 1.260x |
| 41 | 42 | +FID-FIA existence refinement | 174.322228 | 1.743222 | 2.717x |
| 42 | 43 | fixed weights | 58.950518 | 0.589505 | 1.000x |
| 42 | 43 | Cao-Zhao FID-FIA baseline | 149.409648 | 1.494096 | 2.534x |
| 42 | 43 | +structure-aware decoupled KLA | 60.530081 | 0.605301 | 1.027x |
| 42 | 43 | +FID-FIA existence refinement | 162.856506 | 1.628565 | 2.763x |
| 43 | 44 | fixed weights | 56.460934 | 0.564609 | 1.000x |
| 43 | 44 | Cao-Zhao FID-FIA baseline | 169.907209 | 1.699072 | 3.009x |
| 43 | 44 | +structure-aware decoupled KLA | 62.534542 | 0.625345 | 1.108x |
| 43 | 44 | +FID-FIA existence refinement | 159.679837 | 1.596798 | 2.828x |
| 44 | 45 | fixed weights | 56.178258 | 0.561783 | 1.000x |
| 44 | 45 | Cao-Zhao FID-FIA baseline | 149.187248 | 1.491872 | 2.656x |
| 44 | 45 | +structure-aware decoupled KLA | 60.731472 | 0.607315 | 1.081x |
| 44 | 45 | +FID-FIA existence refinement | 163.270665 | 1.632707 | 2.906x |
| 45 | 46 | fixed weights | 55.672378 | 0.556724 | 1.000x |
| 45 | 46 | Cao-Zhao FID-FIA baseline | 146.709852 | 1.467099 | 2.635x |
| 45 | 46 | +structure-aware decoupled KLA | 60.053483 | 0.600535 | 1.079x |
| 45 | 46 | +FID-FIA existence refinement | 162.938921 | 1.629389 | 2.927x |
| 46 | 47 | fixed weights | 56.198316 | 0.561983 | 1.000x |
| 46 | 47 | Cao-Zhao FID-FIA baseline | 150.401185 | 1.504012 | 2.676x |
| 46 | 47 | +structure-aware decoupled KLA | 60.428446 | 0.604284 | 1.075x |
| 46 | 47 | +FID-FIA existence refinement | 163.510266 | 1.635103 | 2.910x |
| 47 | 48 | fixed weights | 55.832349 | 0.558323 | 1.000x |
| 47 | 48 | Cao-Zhao FID-FIA baseline | 146.119143 | 1.461191 | 2.617x |
| 47 | 48 | +structure-aware decoupled KLA | 60.123992 | 0.601240 | 1.077x |
| 47 | 48 | +FID-FIA existence refinement | 158.773618 | 1.587736 | 2.844x |
| 48 | 49 | fixed weights | 56.588418 | 0.565884 | 1.000x |
| 48 | 49 | Cao-Zhao FID-FIA baseline | 152.252265 | 1.522523 | 2.691x |
| 48 | 49 | +structure-aware decoupled KLA | 60.740917 | 0.607409 | 1.073x |
| 48 | 49 | +FID-FIA existence refinement | 171.292577 | 1.712926 | 3.027x |
| 49 | 50 | fixed weights | 55.825782 | 0.558258 | 1.000x |
| 49 | 50 | Cao-Zhao FID-FIA baseline | 146.406238 | 1.464062 | 2.623x |
| 49 | 50 | +structure-aware decoupled KLA | 60.366747 | 0.603667 | 1.081x |
| 49 | 50 | +FID-FIA existence refinement | 160.534384 | 1.605344 | 2.876x |
| 50 | 51 | fixed weights | 57.882631 | 0.578826 | 1.000x |
| 50 | 51 | Cao-Zhao FID-FIA baseline | 151.123657 | 1.511237 | 2.611x |
| 50 | 51 | +structure-aware decoupled KLA | 60.714511 | 0.607145 | 1.049x |
| 50 | 51 | +FID-FIA existence refinement | 167.432960 | 1.674330 | 2.893x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 1.908084 | 1.441941 | 0.281625 |
| Cao-Zhao FID-FIA baseline | 1.880574 | 1.499905 | 0.225500 |
| +structure-aware decoupled KLA | 1.838671 | 1.374600 | 0.270750 |
| +FID-FIA existence refinement | 1.765967 | 1.470315 | 0.195350 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 1.908084 +/- 0.054512 | [1.892974, 1.923194] | 50 |
| Cao-Zhao FID-FIA baseline | E-OSPA | 1.880574 +/- 0.040235 | [1.869422, 1.891727] | 50 |
| +structure-aware decoupled KLA | E-OSPA | 1.838671 +/- 0.055060 | [1.823409, 1.853933] | 50 |
| +FID-FIA existence refinement | E-OSPA | 1.765967 +/- 0.045748 | [1.753286, 1.778648] | 50 |
| fixed weights | RMSE | 1.441941 +/- 0.034792 | [1.432297, 1.451585] | 50 |
| Cao-Zhao FID-FIA baseline | RMSE | 1.499905 +/- 0.062373 | [1.482616, 1.517194] | 50 |
| +structure-aware decoupled KLA | RMSE | 1.374600 +/- 0.035674 | [1.364712, 1.384489] | 50 |
| +FID-FIA existence refinement | RMSE | 1.470315 +/- 0.137575 | [1.432181, 1.508449] | 50 |
| fixed weights | CardErr | 0.281625 +/- 0.032507 | [0.272614, 0.290636] | 50 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.225500 +/- 0.022446 | [0.219278, 0.231722] | 50 |
| +structure-aware decoupled KLA | CardErr | 0.270750 +/- 0.028531 | [0.262842, 0.278658] | 50 |
| +FID-FIA existence refinement | CardErr | 0.195350 +/- 0.019390 | [0.189975, 0.200725] | 50 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| Cao-Zhao FID-FIA baseline | E-OSPA | 0.027509 +/- 0.040134 | [0.016385, 0.038634] | 1.44% | 36/50 | 0.002602 |
| +structure-aware decoupled KLA | E-OSPA | 0.069413 +/- 0.009451 | [0.066793, 0.072033] | 3.64% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | E-OSPA | 0.142117 +/- 0.035008 | [0.132413, 0.151820] | 7.45% | 50/50 | 1.776e-15 |
| Cao-Zhao FID-FIA baseline | RMSE | -0.057964 +/- 0.060271 | [-0.074670, -0.041257] | -4.02% | 4/50 | 4.462e-10 |
| +structure-aware decoupled KLA | RMSE | 0.067341 +/- 0.004899 | [0.065983, 0.068698] | 4.67% | 50/50 | 1.776e-15 |
| +FID-FIA existence refinement | RMSE | -0.028374 +/- 0.131827 | [-0.064915, 0.008166] | -1.97% | 33/50 | 0.03284 |
| Cao-Zhao FID-FIA baseline | CardErr | 0.056125 +/- 0.020850 | [0.050346, 0.061904] | 19.93% | 50/50 | 1.776e-15 |
| +structure-aware decoupled KLA | CardErr | 0.010875 +/- 0.011468 | [0.007696, 0.014054] | 3.86% | 43/50 | 5.728e-08 |
| +FID-FIA existence refinement | CardErr | 0.086275 +/- 0.026780 | [0.078852, 0.093698] | 30.63% | 50/50 | 1.776e-15 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 1.958102 | 1.448123 | 0.345600 |
| 1 | Cao-Zhao FID-FIA baseline | 1.909700 | 1.508384 | 0.243000 |
| 1 | +structure-aware decoupled KLA | 1.873880 | 1.387617 | 0.297600 |
| 1 | +FID-FIA existence refinement | 1.793538 | 1.447922 | 0.210600 |
| 2 | fixed weights | 1.930062 | 1.451044 | 0.294600 |
| 2 | Cao-Zhao FID-FIA baseline | 1.907078 | 1.519590 | 0.248200 |
| 2 | +structure-aware decoupled KLA | 1.870632 | 1.381892 | 0.300400 |
| 2 | +FID-FIA existence refinement | 1.785525 | 1.461272 | 0.208400 |
| 3 | fixed weights | 1.951673 | 1.443968 | 0.343800 |
| 3 | Cao-Zhao FID-FIA baseline | 1.906330 | 1.518269 | 0.251600 |
| 3 | +structure-aware decoupled KLA | 1.863207 | 1.380122 | 0.292600 |
| 3 | +FID-FIA existence refinement | 1.775278 | 1.444366 | 0.207200 |
| 4 | fixed weights | 1.915241 | 1.452566 | 0.277600 |
| 4 | Cao-Zhao FID-FIA baseline | 1.897461 | 1.501598 | 0.235200 |
| 4 | +structure-aware decoupled KLA | 1.852503 | 1.379788 | 0.283800 |
| 4 | +FID-FIA existence refinement | 1.771536 | 1.513328 | 0.203800 |
| 5 | fixed weights | 1.891303 | 1.442694 | 0.256200 |
| 5 | Cao-Zhao FID-FIA baseline | 1.871835 | 1.508123 | 0.216400 |
| 5 | +structure-aware decoupled KLA | 1.823111 | 1.371393 | 0.255000 |
| 5 | +FID-FIA existence refinement | 1.756578 | 1.487655 | 0.190600 |
| 6 | fixed weights | 1.877376 | 1.437574 | 0.241600 |
| 6 | Cao-Zhao FID-FIA baseline | 1.849960 | 1.477645 | 0.198400 |
| 6 | +structure-aware decoupled KLA | 1.805413 | 1.368606 | 0.238000 |
| 6 | +FID-FIA existence refinement | 1.750021 | 1.452161 | 0.179800 |
| 7 | fixed weights | 1.884340 | 1.435728 | 0.255200 |
| 7 | Cao-Zhao FID-FIA baseline | 1.854740 | 1.471891 | 0.212600 |
| 7 | +structure-aware decoupled KLA | 1.822055 | 1.367642 | 0.258600 |
| 7 | +FID-FIA existence refinement | 1.755975 | 1.464531 | 0.187200 |
| 8 | fixed weights | 1.856573 | 1.423829 | 0.238400 |
| 8 | Cao-Zhao FID-FIA baseline | 1.847492 | 1.493737 | 0.198600 |
| 8 | +structure-aware decoupled KLA | 1.798567 | 1.359744 | 0.240000 |
| 8 | +FID-FIA existence refinement | 1.739286 | 1.491285 | 0.175200 |
