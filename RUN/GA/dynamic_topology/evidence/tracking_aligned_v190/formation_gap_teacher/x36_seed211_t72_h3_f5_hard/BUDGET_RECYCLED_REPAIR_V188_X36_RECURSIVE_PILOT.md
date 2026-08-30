# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `5`
- Teacher-forced source: `0`
- Teacher-forced label: `[0 0]`
- Update operator: `hard-replacement`
- Source fusion weight: `1.000`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{5, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[6 0 0]`
- Repair bytes by page: `[60232 0 0]`

- Certified net saving by page: `[180952 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 83.674247 |
| E-OSPA gain | 0 | +2.802% | +2.671% |
| Mean RMSE | 57.902417 | 58.288297 | 58.716549 |
| RMSE gain | 0 | -0.666% | -1.406% |
| Consensus gain | 0 | +5.149% | +3.973% |
| Byte saving | 0 | +6.550% | +5.908% |

- Formation E-OSPA gains: `[1.62646 3.33164 2.39905 4.00729 4.77626 0]`
- Formation RMSE gains: `[-0.0689284 -14.1975 -1.48871 1.81777 -14.6909 0]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `-0.135%`
- Mean RMSE: `-0.735%`
- Consensus: `-1.239%`
- Attempted bytes: `+68128 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page hard-replacement update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
