# GA Tiered Link NIS Comparison (2026-05-07 02:47:47)

Comparison order: w/o NIS -> robust NIS -> NIS

## Run Config
- Trials: 5
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6]
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

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 2 | w/o NIS | 1.945203 | 3.639272 | 0.252500 |
| 1 | 2 | robust NIS | 1.958061 | 3.884104 | 0.257500 |
| 1 | 2 | plain NIS | 1.984745 | 3.548526 | 0.286250 |
| 2 | 3 | w/o NIS | 2.044367 | 3.384011 | 0.342500 |
| 2 | 3 | robust NIS | 1.996604 | 3.575474 | 0.297500 |
| 2 | 3 | plain NIS | 2.038604 | 4.426048 | 0.293750 |
| 3 | 4 | w/o NIS | 1.896123 | 1.571932 | 0.272500 |
| 3 | 4 | robust NIS | 1.887506 | 1.664292 | 0.268750 |
| 3 | 4 | plain NIS | 2.065806 | 1.904033 | 0.335000 |
| 4 | 5 | w/o NIS | 1.758466 | 3.837666 | 0.191250 |
| 4 | 5 | robust NIS | 1.795198 | 3.695569 | 0.196250 |
| 4 | 5 | plain NIS | 1.943520 | 3.559789 | 0.237500 |
| 5 | 6 | w/o NIS | 1.902176 | 2.238702 | 0.276250 |
| 5 | 6 | robust NIS | 1.907466 | 2.080914 | 0.291250 |
| 5 | 6 | plain NIS | 2.005827 | 2.427713 | 0.350000 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.909 -> 1.909 -> 2.008
- Position (RMSE): 2.934 -> 2.980 -> 3.173
- Cardinality: 0.267 -> 0.262 -> 0.300

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| w/o NIS | OSPA | 1.909267 +/- 0.103086 | [1.781289, 2.037245] | 5 |
| robust NIS | OSPA | 1.908967 +/- 0.076643 | [1.813818, 2.004116] | 5 |
| plain NIS | OSPA | 2.007700 +/- 0.047384 | [1.948875, 2.066525] | 5 |
| w/o NIS | RMSE | 2.934317 +/- 0.981733 | [1.715530, 4.153104] | 5 |
| robust NIS | RMSE | 2.980071 +/- 1.027555 | [1.704397, 4.255744] | 5 |
| plain NIS | RMSE | 3.173222 +/- 1.003305 | [1.927654, 4.418790] | 5 |
| w/o NIS | Cardinality | 0.267000 +/- 0.054232 | [0.199673, 0.334327] | 5 |
| robust NIS | Cardinality | 0.262250 +/- 0.040325 | [0.212188, 0.312312] | 5 |
| plain NIS | Cardinality | 0.300500 +/- 0.044323 | [0.245475, 0.355525] | 5 |
