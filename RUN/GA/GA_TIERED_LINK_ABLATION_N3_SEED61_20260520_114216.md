# GA Tiered Link Ablation (2026-05-20 11:42:16)

Comparison order: fixed weights -> +structure-aware decoupled KLA

## Run Config
- Trials: 3
- baseSeed: 61 (fixed=1)
- trialSeeds: [62 63 64]
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
- Trial 1: [0.1 0.1 0 0.5 0.5 0.1 0.1 0.2]
- Trial 2: [0 0.5 0.1 0.1 0.1 0.2 0.1 0.5]
- Trial 3: [0 0.1 0.5 0.5 0.1 0.1 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 62 | fixed weights | 2.572633 | 2.638961 | 0.843750 |
| 1 | 62 | +structure-aware decoupled KLA | 1.662975 | 1.406415 | 0.175000 |
| 2 | 63 | fixed weights | 2.335380 | 2.403615 | 0.552500 |
| 2 | 63 | +structure-aware decoupled KLA | 1.796969 | 1.496802 | 0.176250 |
| 3 | 64 | fixed weights | 2.410837 | 2.415759 | 0.866250 |
| 3 | 64 | +structure-aware decoupled KLA | 1.799056 | 1.447059 | 0.227500 |

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.439616 | 2.486112 | 0.754167 |
| +structure-aware decoupled KLA | 1.753000 | 1.450092 | 0.192917 |

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | OSPA | 2.439616 +/- 0.121217 | [2.138473, 2.740760] | 3 |
| +structure-aware decoupled KLA | OSPA | 1.753000 +/- 0.077971 | [1.559294, 1.946706] | 3 |
| fixed weights | RMSE | 2.486112 +/- 0.132510 | [2.156912, 2.815312] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.450092 +/- 0.045270 | [1.337626, 1.562558] | 3 |
| fixed weights | Cardinality | 0.754167 +/- 0.175010 | [0.319382, 1.188952] | 3 |
| +structure-aware decoupled KLA | Cardinality | 0.192917 +/- 0.029957 | [0.118494, 0.267339] | 3 |

## Paired Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | OSPA | 0.686616 +/- 0.196612 | [0.198165, 1.175068] | 28.14% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 1.036020 +/- 0.172987 | [0.606262, 1.465777] | 41.67% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | Cardinality | 0.561250 +/- 0.160915 | [0.161482, 0.961018] | 74.42% | 3/3 | 0.25 |

## Computational Cost
Filter wall-clock time measures only the distributed LMB filtering/fusion call for each arm. Scenario generation, communication-model sampling, and metric evaluation are excluded.

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed weights | N |
|:----|-------------------:|-----------------:|------------------:|--:|
| fixed weights | 52.269640 +/- 3.387824 | 0.522696 | 1.000x | 3 |
| +structure-aware decoupled KLA | 54.492595 +/- 3.360075 | 0.544926 | 1.043x | 3 |

| Arm | Paired overhead (s) | Relative overhead | Slower trials |
|:----|--------------------:|------------------:|--------------:|
| +structure-aware decoupled KLA | 2.222955 +/- 0.566307 | 4.27% | 3/3 |

### Per-Trial Filter Runtime
| Trial | Seed | Arm | Runtime (s) | Runtime/step (s) | Relative to fixed weights |
|------:|-----:|:----|------------:|-----------------:|------------------:|
| 1 | 62 | fixed weights | 54.713806 | 0.547138 | 1.000x |
| 1 | 62 | +structure-aware decoupled KLA | 56.376601 | 0.563766 | 1.030x |
| 2 | 63 | fixed weights | 53.692722 | 0.536927 | 1.000x |
| 2 | 63 | +structure-aware decoupled KLA | 56.487938 | 0.564879 | 1.052x |
| 3 | 64 | fixed weights | 48.402393 | 0.484024 | 1.000x |
| 3 | 64 | +structure-aware decoupled KLA | 50.613247 | 0.506132 | 1.046x |

## Local Tracking Metrics (mean across sensors and trials)
| Arm | E-OSPA | RMSE | CardErr |
|:----|-------:|-----:|--------:|
| fixed weights | 2.919062 | 1.605059 | 1.568333 |
| +structure-aware decoupled KLA | 2.353297 | 1.586465 | 0.641250 |

## Local Tracking Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| fixed weights | E-OSPA | 2.919062 +/- 0.124169 | [2.610585, 3.227539] | 3 |
| +structure-aware decoupled KLA | E-OSPA | 2.353297 +/- 0.074179 | [2.169012, 2.537582] | 3 |
| fixed weights | RMSE | 1.605059 +/- 0.058389 | [1.460002, 1.750116] | 3 |
| +structure-aware decoupled KLA | RMSE | 1.586465 +/- 0.026638 | [1.520286, 1.652643] | 3 |
| fixed weights | CardErr | 1.568333 +/- 0.227556 | [1.003007, 2.133660] | 3 |
| +structure-aware decoupled KLA | CardErr | 0.641250 +/- 0.042993 | [0.534440, 0.748060] | 3 |

## Paired Local-Metric Improvements Relative to fixed weights
| Arm | Metric | Paired reduction | 95% CI | Reduction | Wins | Sign-test p |
|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|
| +structure-aware decoupled KLA | E-OSPA | 0.565766 +/- 0.050628 | [0.439990, 0.691542] | 19.38% | 3/3 | 0.25 |
| +structure-aware decoupled KLA | RMSE | 0.018594 +/- 0.031925 | [-0.060718, 0.097905] | 1.16% | 2/3 | 1 |
| +structure-aware decoupled KLA | CardErr | 0.927083 +/- 0.185360 | [0.466586, 1.387580] | 59.11% | 3/3 | 0.25 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | Arm | E-OSPA | RMSE | CardErr |
|------:|:----|-------:|-----:|--------:|
| 1 | fixed weights | 2.623744 | 1.581280 | 1.010000 |
| 1 | +structure-aware decoupled KLA | 2.263010 | 1.522034 | 0.573333 |
| 2 | fixed weights | 2.687562 | 1.543512 | 0.996667 |
| 2 | +structure-aware decoupled KLA | 2.348045 | 1.532172 | 0.653333 |
| 3 | fixed weights | 2.720567 | 1.545196 | 1.276667 |
| 3 | +structure-aware decoupled KLA | 2.301152 | 1.536891 | 0.636667 |
| 4 | fixed weights | 3.221483 | 1.562347 | 2.293333 |
| 4 | +structure-aware decoupled KLA | 2.478982 | 1.646107 | 0.783333 |
| 5 | fixed weights | 2.809956 | 1.582927 | 1.303333 |
| 5 | +structure-aware decoupled KLA | 2.283415 | 1.586341 | 0.586667 |
| 6 | fixed weights | 2.882744 | 1.708938 | 1.296667 |
| 6 | +structure-aware decoupled KLA | 2.362653 | 1.622473 | 0.590000 |
| 7 | fixed weights | 2.871107 | 1.662008 | 1.396667 |
| 7 | +structure-aware decoupled KLA | 2.351369 | 1.614216 | 0.620000 |
| 8 | fixed weights | 3.535337 | 1.654263 | 2.973333 |
| 8 | +structure-aware decoupled KLA | 2.437749 | 1.631486 | 0.686667 |
