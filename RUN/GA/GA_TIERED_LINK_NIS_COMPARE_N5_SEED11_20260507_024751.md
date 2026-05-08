# GA Tiered Link NIS Comparison (2026-05-07 02:47:51)

Comparison order: w/o NIS -> robust NIS -> NIS

## Run Config
- Trials: 5
- baseSeed: 11 (fixed=1)
- trialSeeds: [12 13 14 15 16]
- robustNISMin: 0.30
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.1 0 0.1 0.1 0.5 0.1 0.2]
- Trial 2: [0 0.1 0.1 0.5 0.1 0.1 0.2 0.5]
- Trial 3: [0.1 0.1 0.2 0.5 0 0.5 0.1 0.1]
- Trial 4: [0.1 0.5 0 0.2 0.5 0.1 0.1 0.1]
- Trial 5: [0 0.1 0.5 0.1 0.1 0.2 0.1 0.5]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 12 | w/o NIS | 1.788364 | 2.650192 | 0.191250 |
| 1 | 12 | robust NIS | 1.801834 | 2.632149 | 0.196250 |
| 1 | 12 | plain NIS | 1.885569 | 4.130328 | 0.223750 |
| 2 | 13 | w/o NIS | 1.827029 | 3.992927 | 0.226250 |
| 2 | 13 | robust NIS | 1.823667 | 3.989898 | 0.223750 |
| 2 | 13 | plain NIS | 1.919115 | 3.613911 | 0.263750 |
| 3 | 14 | w/o NIS | 1.849595 | 3.515857 | 0.185000 |
| 3 | 14 | robust NIS | 1.874456 | 3.666023 | 0.193750 |
| 3 | 14 | plain NIS | 1.996938 | 3.477961 | 0.221250 |
| 4 | 15 | w/o NIS | 1.830029 | 2.206550 | 0.201250 |
| 4 | 15 | robust NIS | 1.861383 | 2.263294 | 0.215000 |
| 4 | 15 | plain NIS | 1.905981 | 2.866967 | 0.221250 |
| 5 | 16 | w/o NIS | 1.733531 | 2.831825 | 0.191250 |
| 5 | 16 | robust NIS | 1.776329 | 3.071819 | 0.196250 |
| 5 | 16 | plain NIS | 1.835651 | 2.723730 | 0.221250 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.806 -> 1.828 -> 1.909
- Position (RMSE): 3.039 -> 3.125 -> 3.363
- Cardinality: 0.199 -> 0.205 -> 0.230

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| w/o NIS | OSPA | 1.805710 +/- 0.046050 | [1.748540, 1.862879] | 5 |
| robust NIS | OSPA | 1.827534 +/- 0.040758 | [1.776934, 1.878134] | 5 |
| plain NIS | OSPA | 1.908651 +/- 0.058679 | [1.835803, 1.981499] | 5 |
| w/o NIS | RMSE | 3.039470 +/- 0.711413 | [2.156276, 3.922665] | 5 |
| robust NIS | RMSE | 3.124637 +/- 0.712218 | [2.240442, 4.008831] | 5 |
| plain NIS | RMSE | 3.362579 +/- 0.574387 | [2.649499, 4.075660] | 5 |
| w/o NIS | Cardinality | 0.199000 +/- 0.016308 | [0.178755, 0.219245] | 5 |
| robust NIS | Cardinality | 0.205000 +/- 0.013521 | [0.188214, 0.221786] | 5 |
| plain NIS | Cardinality | 0.230250 +/- 0.018758 | [0.206962, 0.253538] | 5 |
