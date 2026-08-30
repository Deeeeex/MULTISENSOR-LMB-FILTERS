# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `1`
- Maximum repair formations per page: `1`
- Selected formations by page: `{1, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[6 0 0]`
- Repair bytes by page: `[60232 0 0]`

- Certified net saving by page: `[180952 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 83.532157 |
| E-OSPA gain | 0 | +2.802% | +2.836% |
| Mean RMSE | 57.902417 | 58.288297 | 58.120694 |
| RMSE gain | 0 | -0.666% | -0.377% |
| Consensus gain | 0 | +5.149% | +5.887% |
| Byte saving | 0 | +6.550% | +6.010% |

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `+0.035%`
- Mean RMSE: `+0.288%`
- Consensus: `+0.778%`
- Attempted bytes: `+57376 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page V188 action before state extraction and lets the exact complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
