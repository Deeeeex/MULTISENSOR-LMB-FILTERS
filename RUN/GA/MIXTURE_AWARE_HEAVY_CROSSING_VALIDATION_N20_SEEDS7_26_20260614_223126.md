# Mixture-aware heavy crossing validation

- Seeds: [7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26]
- Scenario: two crossing targets, high clutter, periodic delivery.
- Mixture-aware config: `minExistence=0.70`, `minEntropy=0.20`, `topComponents=2`.

## Mean E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy legacy fusion | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy mixture-aware fusion | 3.4877 | 0.2431 | 3.7585 | 3.8691 |

## Crossing-window E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy legacy fusion | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy mixture-aware fusion | 3.4546 | 0.4214 | 3.9440 | 4.1445 |

## Payload bytes

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 152224.0000 | 30937.0333 | 189875.2000 | 207424.0000 |
| Periodic heavy legacy fusion | 683221.6000 | 161573.6113 | 869576.8000 | 984496.0000 |
| Periodic heavy mixture-aware fusion | 769315.6000 | 107409.2027 | 894880.0000 | 986920.0000 |

## Relative crossing result

| Comparison | Mean change | P90 change | Wins |
|:--|--:|--:|--:|
| Mixture-aware heavy vs light | 5.76% | 19.64% | 6/20 |
| Mixture-aware heavy vs legacy heavy | 5.76% | 19.64% | 6/20 |

Negative percentages mean mixture-aware heavy has lower E-OSPA.
