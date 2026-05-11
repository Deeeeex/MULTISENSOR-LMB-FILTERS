# GA Original 4+4 + State-Dependent Sensor Quality + False Targets (2026-05-08 23:30:14)

Comparison order: fixed-weight GA -> adaptive GA

## Run Config
- Trials: 1
- baseSeed: 9 (fixed=1)
- trialSeeds: 10
- simulationLength: 100
- trueTargets: 10
- falseTargets: 3
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- comm level: 2
- globalMaxMeasurementsPerStep: 80.000
- linkModel: fixed
- pDrop: 0.200
- clutterRates: [3 3 3 3 3 3 3 3]
- baseDetectionProbabilities: [0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9]
- baseMeasurementStd: [3 3 3 3 3 3 3 3]

## Sensor Quality Model
- referenceRange: 180.000
- minDetectionProbability: 0.250
- detectionRangeDecay: 0.180
- edgeDetectionPenalty: 0.250
- rangeNoiseScale: 0.350
- edgeNoiseScale: 0.800
- maxCovarianceScale: 4.000

## Adaptive Config
- useCovariance: 1
- useExistenceConfidence: 1
- useDecoupledKla: 1
- useStructureAwareKla: 1
- useNIS: 1
- robustNIS: 1

## False Measurements
- Mean total per trial: 663.000
- Mean by sensor: [79 89 90 87 66 85 76 91]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 10 | fixed-weight GA | 2.698651 | 3.440289 | 0.533750 |
| 1 | 10 | adaptive GA | 2.429563 | 2.622745 | 0.261250 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 2.699 -> 2.430
- Position (RMSE): 3.440 -> 2.623
- Cardinality: 0.534 -> 0.261

## Paired Improvements Relative to Fixed-Weight GA
- Consensus OSPA: paired reduction 0.269088 (9.97%), wins 1/1
- Consensus RMSE: paired reduction 0.817544 (23.76%), wins 1/1
- Consensus cardinality disagreement: paired reduction 0.272500 (51.05%), wins 1/1

## Aggregated Local Metrics (mean across sensors and trials)
| Metric | Fixed-weight GA | Adaptive GA | Delta |
|:-------|----------------:|------------:|------:|
| E-OSPA | 3.690816 | 3.402625 | -0.288191 |
| H-OSPA | 0.500000 | 0.500000 | 0.000000 |
| RMSE | 2.802380 | 2.785226 | -0.017153 |
| CardErr | 2.081250 | 1.436250 | -0.645000 |

## Local Tracking Metrics By Sensor (mean across trials)
| Sensor | E-OSPA (base) | E-OSPA (adaptive) | H-OSPA (base) | H-OSPA (adaptive) | RMSE (base) | RMSE (adaptive) | CardErr (base) | CardErr (adaptive) |
|------:|---------------:|------------------:|--------------:|-----------------:|------------:|---------------:|---------------:|------------------:|
| 1 | 3.451 | 3.273 | 0.500 | 0.500 | 2.863 | 2.993 | 1.500 | 1.200 |
| 2 | 3.492 | 3.197 | 0.500 | 0.500 | 3.195 | 3.089 | 1.600 | 1.260 |
| 3 | 3.625 | 3.454 | 0.500 | 0.500 | 2.670 | 2.719 | 1.760 | 1.430 |
| 4 | 3.493 | 3.302 | 0.500 | 0.500 | 2.548 | 2.973 | 1.830 | 1.360 |
| 5 | 3.741 | 3.541 | 0.500 | 0.500 | 2.547 | 2.648 | 2.210 | 1.620 |
| 6 | 3.806 | 3.541 | 0.500 | 0.500 | 3.149 | 2.688 | 2.250 | 1.620 |
| 7 | 3.901 | 3.435 | 0.500 | 0.500 | 2.655 | 2.508 | 2.460 | 1.470 |
| 8 | 4.018 | 3.479 | 0.500 | 0.500 | 2.792 | 2.663 | 3.040 | 1.530 |
