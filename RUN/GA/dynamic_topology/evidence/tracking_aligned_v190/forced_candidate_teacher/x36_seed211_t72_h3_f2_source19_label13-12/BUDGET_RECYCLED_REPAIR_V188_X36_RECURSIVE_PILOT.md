# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `2`
- Teacher-forced source: `19`
- Teacher-forced label: `[13 12]`
- Update operator: `hard-replacement`
- Source fusion weight: `1.000`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{2, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[1 0 0]`
- Repair bytes by page: `[60232 0 0]`

- Certified net saving by page: `[180952 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 82.840396 |
| E-OSPA gain | 0 | +2.802% | +3.641% |
| Mean RMSE | 57.902417 | 58.288297 | 54.371287 |
| RMSE gain | 0 | -0.666% | +6.098% |
| Consensus gain | 0 | +5.149% | +7.164% |
| Byte saving | 0 | +6.550% | +5.161% |

- Formation E-OSPA gains: `[1.62646 7.83613 2.39905 4.00729 5.65232 0]`
- Formation RMSE gains: `[-0.0689284 62.2899 -1.48871 1.81777 -0.999639 0]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `+0.863%`
- Mean RMSE: `+6.720%`
- Consensus: `+2.124%`
- Attempted bytes: `+147408 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page hard-replacement update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
