# GA Tiered Link NIS Comparison (2026-05-07 02:47:49)

Comparison order: w/o NIS -> robust NIS -> NIS

## Run Config
- Trials: 5
- baseSeed: 16 (fixed=1)
- trialSeeds: [17 18 19 20 21]
- robustNISMin: 0.30
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Per-Trial pDropBySensor
- Trial 1: [0.5 0.1 0.2 0.5 0.1 0.1 0 0.1]
- Trial 2: [0 0.1 0.1 0.2 0.5 0.1 0.5 0.1]
- Trial 3: [0.2 0 0.5 0.5 0.1 0.1 0.1 0.1]
- Trial 4: [0.2 0.1 0.1 0.1 0 0.5 0.5 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.1 0.5 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 17 | w/o NIS | 1.872973 | 2.442355 | 0.268750 |
| 1 | 17 | robust NIS | 1.893446 | 2.390721 | 0.280000 |
| 1 | 17 | plain NIS | 1.983429 | 2.378081 | 0.321250 |
| 2 | 18 | w/o NIS | 1.852034 | 2.427182 | 0.256250 |
| 2 | 18 | robust NIS | 1.886697 | 2.530145 | 0.271250 |
| 2 | 18 | plain NIS | 1.981145 | 2.646021 | 0.313750 |
| 3 | 19 | w/o NIS | 1.738556 | 2.455229 | 0.220000 |
| 3 | 19 | robust NIS | 1.735874 | 2.464833 | 0.228750 |
| 3 | 19 | plain NIS | 1.920594 | 2.174576 | 0.348750 |
| 4 | 20 | w/o NIS | 1.830741 | 3.472765 | 0.205000 |
| 4 | 20 | robust NIS | 1.863835 | 3.262077 | 0.228750 |
| 4 | 20 | plain NIS | 1.994093 | 2.620193 | 0.280000 |
| 5 | 21 | w/o NIS | 1.863742 | 2.986476 | 0.252500 |
| 5 | 21 | robust NIS | 1.878649 | 2.946689 | 0.251250 |
| 5 | 21 | plain NIS | 1.969946 | 2.941487 | 0.311250 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.832 -> 1.852 -> 1.970
- Position (RMSE): 2.757 -> 2.719 -> 2.552
- Cardinality: 0.240 -> 0.252 -> 0.315

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| w/o NIS | OSPA | 1.831609 +/- 0.054361 | [1.764122, 1.899096] | 5 |
| robust NIS | OSPA | 1.851700 +/- 0.065683 | [1.770158, 1.933243] | 5 |
| plain NIS | OSPA | 1.969841 +/- 0.028835 | [1.934043, 2.005639] | 5 |
| w/o NIS | RMSE | 2.756801 +/- 0.464711 | [2.179878, 3.333724] | 5 |
| robust NIS | RMSE | 2.718893 +/- 0.372428 | [2.256537, 3.181249] | 5 |
| plain NIS | RMSE | 2.552072 +/- 0.290640 | [2.191252, 2.912892] | 5 |
| w/o NIS | Cardinality | 0.240500 +/- 0.026789 | [0.207242, 0.273758] | 5 |
| robust NIS | Cardinality | 0.252000 +/- 0.023645 | [0.222646, 0.281354] | 5 |
| plain NIS | Cardinality | 0.315000 +/- 0.024590 | [0.284472, 0.345528] | 5 |
