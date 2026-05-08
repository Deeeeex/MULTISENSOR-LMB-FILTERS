# GA Tiered Link NIS Comparison (2026-05-07 02:47:23)

Comparison order: w/o NIS -> robust NIS -> NIS

## Run Config
- Trials: 5
- baseSeed: 6 (fixed=1)
- trialSeeds: [7 8 9 10 11]
- robustNISMin: 0.30
- sensorCommRange: 150
- fusionWeighting: Metropolis
- leaderSensor: 8
- pDropLevels: [0 0.1 0.2 0.5]
- pDropLevelCounts: [1 4 1 2]

## Per-Trial pDropBySensor
- Trial 1: [0 0.2 0.1 0.5 0.1 0.5 0.1 0.1]
- Trial 2: [0.1 0.1 0.5 0 0.1 0.1 0.5 0.2]
- Trial 3: [0.5 0.1 0.2 0.1 0.5 0.1 0.1 0]
- Trial 4: [0.5 0.1 0 0.1 0.1 0.5 0.2 0.1]
- Trial 5: [0.1 0.5 0.1 0 0.5 0.1 0.1 0.2]

## Per-Trial Consensus Metrics
| Trial | Seed | Arm | OSPA | RMSE | Cardinality |
|------:|-----:|:----|-----:|-----:|------------:|
| 1 | 7 | w/o NIS | 1.806405 | 3.310892 | 0.213750 |
| 1 | 7 | robust NIS | 1.825052 | 3.309885 | 0.218750 |
| 1 | 7 | plain NIS | 1.981072 | 3.568248 | 0.272500 |
| 2 | 8 | w/o NIS | 1.794862 | 1.496891 | 0.223750 |
| 2 | 8 | robust NIS | 1.784817 | 1.457508 | 0.221250 |
| 2 | 8 | plain NIS | 1.865430 | 2.854386 | 0.223750 |
| 3 | 9 | w/o NIS | 1.770988 | 1.663427 | 0.233750 |
| 3 | 9 | robust NIS | 1.825059 | 1.524784 | 0.256250 |
| 3 | 9 | plain NIS | 1.874347 | 1.975384 | 0.256250 |
| 4 | 10 | w/o NIS | 1.828179 | 2.871089 | 0.215000 |
| 4 | 10 | robust NIS | 1.837535 | 3.009007 | 0.213750 |
| 4 | 10 | plain NIS | 1.966488 | 3.450749 | 0.230000 |
| 5 | 11 | w/o NIS | 1.876325 | 2.588116 | 0.242500 |
| 5 | 11 | robust NIS | 1.893202 | 2.626172 | 0.227500 |
| 5 | 11 | plain NIS | 1.922314 | 2.896097 | 0.246250 |

## Consensus Metrics (mean across trials)
- Comprehensive (OSPA): 1.815 -> 1.833 -> 1.922
- Position (RMSE): 2.386 -> 2.385 -> 2.949
- Cardinality: 0.226 -> 0.227 -> 0.246

## Consensus Metrics With Trial Variability
| Arm | Metric | Mean +/- Std | 95% CI | N |
|:----|:-------|-------------:|:-------|--:|
| w/o NIS | OSPA | 1.815352 +/- 0.039844 | [1.765887, 1.864817] | 5 |
| robust NIS | OSPA | 1.833133 +/- 0.039028 | [1.784681, 1.881585] | 5 |
| plain NIS | OSPA | 1.921930 +/- 0.052297 | [1.857006, 1.986854] | 5 |
| w/o NIS | RMSE | 2.386083 +/- 0.781699 | [1.415631, 3.356535] | 5 |
| robust NIS | RMSE | 2.385471 +/- 0.851935 | [1.327824, 3.443119] | 5 |
| plain NIS | RMSE | 2.948973 +/- 0.631447 | [2.165054, 3.732892] | 5 |
| w/o NIS | Cardinality | 0.225750 +/- 0.012330 | [0.210443, 0.241057] | 5 |
| robust NIS | Cardinality | 0.227500 +/- 0.016817 | [0.206622, 0.248378] | 5 |
| plain NIS | Cardinality | 0.245750 +/- 0.019737 | [0.221248, 0.270252] | 5 |
