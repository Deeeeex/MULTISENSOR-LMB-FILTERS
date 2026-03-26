# GA Ideal Communication Comparison (2026-03-26 18:45:08)

Comparison order: ordinary GA -> structure-aware decoupled KLA

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- comm level: 0
- linkModel: fixed
- pDrop target mean: 0.000
- adaptive useDecoupledKla: 1
- adaptive useStructureAwareKla: 1
- adaptive usePosteriorStructureConsistency: 0
- adaptive spatialStructureStrength: 0.450
- adaptive existenceStructureStrength: 0.080
- adaptive structureReliabilityPower: 0.300

## Mean pDropBySensor Across Trials
- [0 0 0 0 0 0 0 0]

## Per-Sensor Metrics (mean across trials)
| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.061 | 1.897 | 0.500 | 0.500 | 1.460 | 1.376 |
| 2 | 2.041 | 1.928 | 0.500 | 0.500 | 1.445 | 1.370 |
| 3 | 1.907 | 1.908 | 0.500 | 0.500 | 1.411 | 1.345 |
| 4 | 1.898 | 1.872 | 0.500 | 0.500 | 1.438 | 1.360 |
| 5 | 1.948 | 1.878 | 0.500 | 0.500 | 1.457 | 1.383 |
| 6 | 1.919 | 1.851 | 0.500 | 0.500 | 1.444 | 1.372 |
| 7 | 1.911 | 1.843 | 0.500 | 0.500 | 1.425 | 1.367 |
| 8 | 1.912 | 1.839 | 0.500 | 0.500 | 1.455 | 1.382 |

## Aggregated Local Metrics (mean across sensors and trials)
- E-OSPA: 1.950 -> 1.877
- H-OSPA: 0.500 -> 0.500
- RMSE: 1.442 -> 1.369

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.706 -> 1.494
- Position (RMSE): 1.526 -> 1.290
- Cardinality: 0.161 -> 0.139
