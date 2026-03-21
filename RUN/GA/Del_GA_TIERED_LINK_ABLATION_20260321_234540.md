# GA Tiered Link Ablation (2026-03-21 23:45:41)

Comparison order: fixed weights -> +covariance -> +link quality -> +cardinality consensus

## Run Config
- Trials: 1
- baseSeed: 1 (fixed=1)
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- linkModel: fixed
- pDrop target mean: 0.200
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

- finalArmMode: cardinality

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.590531 | 2.268101 | 0.868750 |
| +covariance | 2.243220 | 1.774557 | 0.608750 |
| +link quality | 1.909508 | 1.621662 | 0.242500 |
| +cardinality consensus | 1.954855 | 1.791287 | 0.285000 |
