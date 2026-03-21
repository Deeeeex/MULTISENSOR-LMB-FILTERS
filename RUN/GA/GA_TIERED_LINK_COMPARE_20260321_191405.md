# GA Tiered Link Comparison (2026-03-21 19:14:05)

Comparison order: fixed weights -> adaptive robust NIS

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]
- adaptive useNIS: 1
- adaptive robustNIS: 1
- adaptive robustNISMin: 0.30
- adaptive useHistory: 0

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Mean pDropBySensor Across Trials
- [0.28 0.06 0.2 0.2 0.22 0.22 0.32 0.1]

## Per-Sensor Metrics (mean across trials)
| Sensor | Mean pDrop | E-OSPA (fixed) | E-OSPA (adaptive) | RMSE (fixed) | RMSE (adaptive) |
|:------:|-----------:|---------------:|------------------:|-------------:|----------------:|
| 1 | 0.280 | 2.800 | 2.384 | 1.584 | 1.575 |
| 2 | 0.060 | 2.562 | 2.336 | 1.556 | 1.566 |
| 3 | 0.200 | 2.771 | 2.397 | 1.540 | 1.540 |
| 4 | 0.200 | 2.791 | 2.386 | 1.670 | 1.573 |
| 5 | 0.220 | 3.334 | 2.646 | 1.749 | 1.709 |
| 6 | 0.220 | 2.920 | 2.469 | 1.602 | 1.605 |
| 7 | 0.320 | 3.153 | 2.490 | 1.621 | 1.648 |
| 8 | 0.100 | 3.231 | 2.425 | 1.702 | 1.658 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 2.586 -> 1.909
- Position (RMSE): 2.855 -> 2.980
- Cardinality: 0.850 -> 0.262
