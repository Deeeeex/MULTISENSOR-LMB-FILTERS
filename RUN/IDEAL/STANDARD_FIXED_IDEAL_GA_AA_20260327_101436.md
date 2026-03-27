# Standard Fixed Ideal GA/AA Comparison (2026-03-27 10:14:36)

This run approximates the ideal setup emphasized in Li et al. (2023): 4 synchronous sensors, ideal communication, uniform fusion weights, and full access to the common ROI. The target ground truth follows the repo's standard `Fixed` benchmark scenario inherited from the common LMB codebase.

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- numberOfSensors: 4
- scenarioType: Fixed
- simulationLength: 100
- fusionWeighting: Uniform
- clutterRates: [5 5 5 5]
- detectionProbabilities: [0.9 0.9 0.9 0.9]
- measurementNoiseStd: [3 3 3 3]
- ideal pDropBySensor: [0 0 0 0]

## Mean Metrics
| Filter | E-OSPA | H-OSPA | RMSE | Cardinality Error |
|:------|-------:|-------:|-----:|------------------:|
| GA | 1.931801 | 0.456857 | 1.515751 | 0.324000 |
| AA | 4.184006 | 0.499369 | 5.494451 | 0.198000 |

## Delta (GA - AA)
- E-OSPA: -2.252205
- H-OSPA: -0.042512
- RMSE: -3.978701
- Cardinality Error: 0.126000
