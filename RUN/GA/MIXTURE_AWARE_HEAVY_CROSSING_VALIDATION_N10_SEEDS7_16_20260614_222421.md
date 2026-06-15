# Mixture-aware heavy crossing validation

- Seeds: [7 8 9 10 11 12 13 14 15 16]
- Scenario: two crossing targets, high clutter, periodic delivery.
- Mixture-aware config: `minExistence=0.70`, `minEntropy=0.20`, `topComponents=2`.

## Mean E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.5942 | 0.3975 | 3.9678 | 4.1932 |
| Periodic heavy legacy fusion | 3.5942 | 0.3975 | 3.9678 | 4.1932 |
| Periodic heavy mixture-aware fusion | 3.4987 | 0.2877 | 3.7651 | 3.8691 |

## Crossing-window E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.4106 | 0.4134 | 3.7895 | 4.0204 |
| Periodic heavy legacy fusion | 3.4106 | 0.4134 | 3.7895 | 4.0204 |
| Periodic heavy mixture-aware fusion | 3.4873 | 0.4542 | 3.9863 | 4.1445 |

## Payload bytes

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 162035.2000 | 27893.3117 | 190489.6000 | 207424.0000 |
| Periodic heavy legacy fusion | 721861.6000 | 131104.4034 | 838278.4000 | 934528.0000 |
| Periodic heavy mixture-aware fusion | 776500.0000 | 68438.4933 | 827994.4000 | 892816.0000 |

## Relative crossing result

| Comparison | Mean change | P90 change | Wins |
|:--|--:|--:|--:|
| Mixture-aware heavy vs light | 2.70% | 13.18% | 4/10 |
| Mixture-aware heavy vs legacy heavy | 2.70% | 13.18% | 4/10 |

Negative percentages mean mixture-aware heavy has lower E-OSPA.
