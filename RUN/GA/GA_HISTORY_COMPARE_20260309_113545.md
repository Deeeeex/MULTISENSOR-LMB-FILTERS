# GA History Comparison (2026-03-09 11:35:45)

Comparison order: w/o history -> history

## Run Config
- Trials: 20
- baseSeed: 1 (fixed=1)
- useNIS: 0
- robustNISMin: 0.30
- sensorCommRange: 150
- fusionWeighting: Metropolis
- useHistory: 1
- historyEmaAlpha: 0.80
- historyScale: 2.00
- nisQuantileEnabled: 1
- nisQuantile: 0.70
- nisEmaEnabled: 1
- nisEmaAlpha: 0.70
- leaderSensor: 8

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (w/o history) | E-OSPA (history) | RMSE (w/o history) | RMSE (history) |
|:------:|---------------------:|-----------------:|-------------------:|---------------:|
| 1 | 2.406 | 2.420 | 1.514 | 1.523 |
| 2 | 2.398 | 2.409 | 1.505 | 1.514 |
| 3 | 2.426 | 2.435 | 1.513 | 1.523 |
| 4 | 2.494 | 2.501 | 1.558 | 1.567 |
| 5 | 2.526 | 2.538 | 1.616 | 1.614 |
| 6 | 2.492 | 2.490 | 1.592 | 1.599 |
| 7 | 2.417 | 2.425 | 1.602 | 1.609 |
| 8 | 2.384 | 2.392 | 1.595 | 1.601 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.811 -> 1.814
- Position (RMSE): 3.173 -> 3.158
- Cardinality: 0.214 -> 0.215
