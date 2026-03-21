# GA Tiered Link Ablation (2026-03-22 00:16:13)

Comparison order: fixed weights -> +covariance -> +link quality -> +existence confidence

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

- finalArmMode: existence

## Per-Trial pDropBySensor
- Trial 1: [0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]
- Trial 2: [0.5 0.1 0.5 0.2 0.1 0.1 0 0.1]
- Trial 3: [0.5 0 0.1 0.1 0.5 0.2 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.5 0 0.1 0.5 0.1]
- Trial 5: [0.1 0 0.1 0.1 0.5 0.2 0.5 0.1]

## Consensus Metrics (mean across trials)
| Arm | OSPA | RMSE | Cardinality |
|:----|-----:|-----:|------------:|
| fixed weights | 2.624065 | 2.702602 | 0.878750 |
| +covariance | 2.211513 | 2.410976 | 0.589500 |
| +link quality | 1.877771 | 1.800945 | 0.245250 |
| +existence confidence | 1.874840 | 1.779820 | 0.244500 |
