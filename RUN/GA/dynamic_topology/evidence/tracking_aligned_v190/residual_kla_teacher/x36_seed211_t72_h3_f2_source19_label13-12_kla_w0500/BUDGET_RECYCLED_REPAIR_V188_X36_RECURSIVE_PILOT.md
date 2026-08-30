# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `2`
- Teacher-forced source: `19`
- Teacher-forced label: `[13 12]`
- Update operator: `label-kla`
- Source fusion weight: `0.500`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{2, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[1 0 0]`
- Repair bytes by page: `[60232 0 0]`

- Certified net saving by page: `[180952 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 82.834747 |
| E-OSPA gain | 0 | +2.802% | +3.647% |
| Mean RMSE | 57.902417 | 58.288297 | 54.306048 |
| RMSE gain | 0 | -0.666% | +6.211% |
| Consensus gain | 0 | +5.149% | +7.406% |
| Byte saving | 0 | +6.550% | +5.211% |

- Formation E-OSPA gains: `[1.62646 7.8714 2.39905 4.00729 5.65232 0]`
- Formation RMSE gains: `[-0.0689284 63.5639 -1.48871 1.81777 -0.999639 0]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `+0.870%`
- Mean RMSE: `+6.832%`
- Consensus: `+2.380%`
- Attempted bytes: `+142032 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page label-kla update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
