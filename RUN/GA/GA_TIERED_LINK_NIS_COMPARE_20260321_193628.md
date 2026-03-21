# GA Tiered Link NIS Comparison (2026-03-21 19:36:28)

Comparison order: w/o NIS -> robust NIS -> NIS

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- robustNISMin: 0.30
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.909 -> 1.909 -> 2.008
- Position (RMSE): 2.934 -> 2.980 -> 3.173
- Cardinality: 0.267 -> 0.262 -> 0.300
