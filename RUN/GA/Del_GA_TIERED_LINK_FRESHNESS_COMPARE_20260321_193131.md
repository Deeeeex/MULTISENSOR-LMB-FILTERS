# GA Tiered Link Freshness Comparison (2026-03-21 19:31:31)

Comparison order: robust NIS baseline -> robust NIS + freshness

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]
- control useNIS: 1
- control robustNIS: 1
- experiment useFreshness: 1

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.909 -> 1.910
- Position (RMSE): 2.980 -> 2.980
- Cardinality: 0.262 -> 0.263
