# Mixture-aware heavy crossing validation

- Seeds: [7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26]
- Scenario: two crossing targets, high clutter, periodic delivery.
- Mixture-aware config: `minExistence=0.90`, `minEntropy=0.20`, `minAssociationAmbiguity=0.00`, `minDetectionMass=0.00`, `maxFusedEntropy=1.00`, `minFusedDominance=0.55`, `topComponents=2`, `predictionConsistency=0.20`.

## Mean E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy legacy fusion | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy mixture-aware fusion | 3.4105 | 0.3387 | 3.7856 | 4.1617 |

## Crossing-window E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy legacy fusion | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy mixture-aware fusion | 3.1509 | 0.4253 | 3.7656 | 3.9282 |

## Crossing-window cardinality error

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 0.1115 | 0.1253 | 0.3462 | 0.3462 |
| Periodic heavy legacy fusion | 0.1115 | 0.1253 | 0.3462 | 0.3462 |
| Periodic heavy mixture-aware fusion | 0.1192 | 0.1433 | 0.2731 | 0.5385 |

## Payload bytes

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 152224.0000 | 30937.0333 | 189875.2000 | 207424.0000 |
| Periodic heavy legacy fusion | 683221.6000 | 161573.6113 | 869576.8000 | 984496.0000 |
| Periodic heavy mixture-aware fusion | 679312.0000 | 148296.9286 | 848814.4000 | 956992.0000 |

## Relative crossing result

| Comparison | Mean change | P90 change | Wins |
|:--|--:|--:|--:|
| Mixture-aware heavy vs light | -4.26% | 2.63% | 16/20 |
| Mixture-aware heavy vs legacy heavy | -4.26% | 2.63% | 16/20 |

Negative percentages mean mixture-aware heavy has lower E-OSPA.
