# Mixture-aware heavy crossing validation

- Seeds: [7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26]
- Scenario: two crossing targets, high clutter, periodic delivery.
- Mixture-aware config: `minExistence=0.90`, `minEntropy=0.20`, `minAssociationAmbiguity=0.00`, `topComponents=2`.

## Mean E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy legacy fusion | 3.5337 | 0.3968 | 3.9607 | 4.1932 |
| Periodic heavy mixture-aware fusion | 3.4490 | 0.2947 | 3.7660 | 3.9896 |

## Crossing-window E-OSPA

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy legacy fusion | 3.2983 | 0.4306 | 3.7861 | 4.0204 |
| Periodic heavy mixture-aware fusion | 3.2773 | 0.3753 | 3.8059 | 3.9725 |

## Crossing-window cardinality error

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 0.1115 | 0.1253 | 0.3462 | 0.3462 |
| Periodic heavy legacy fusion | 0.1115 | 0.1253 | 0.3462 | 0.3462 |
| Periodic heavy mixture-aware fusion | 0.1385 | 0.1457 | 0.3154 | 0.4615 |

## Payload bytes

| Arm | Mean | Std | P90 | Worst |
|:--|--:|--:|--:|--:|
| Periodic light posterior | 152224.0000 | 30937.0333 | 189875.2000 | 207424.0000 |
| Periodic heavy legacy fusion | 683221.6000 | 161573.6113 | 869576.8000 | 984496.0000 |
| Periodic heavy mixture-aware fusion | 748294.0000 | 124811.6120 | 931741.6000 | 948904.0000 |

## Relative crossing result

| Comparison | Mean change | P90 change | Wins |
|:--|--:|--:|--:|
| Mixture-aware heavy vs light | -0.10% | 8.68% | 10/20 |
| Mixture-aware heavy vs legacy heavy | -0.10% | 8.68% | 10/20 |

Negative percentages mean mixture-aware heavy has lower E-OSPA.
