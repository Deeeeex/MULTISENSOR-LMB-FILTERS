# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `2`
- Maximum repair formations per page: `1`
- Repair page offsets: `[0 1 2]`
- Selected formations by page: `{2, 2, 2}`
- Repair applied by page: `[1 1 1]`
- Feasible proposals by page: `[6 6 6]`
- Repair bytes by page: `[60232 60280 60112]`

- Certified net saving by page: `[180952 228640 124840]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 83.479177 |
| E-OSPA gain | 0 | +2.802% | +2.898% |
| Mean RMSE | 57.902417 | 58.288297 | 58.503546 |
| RMSE gain | 0 | -0.666% | -1.038% |
| Consensus gain | 0 | +5.149% | +4.045% |
| Byte saving | 0 | +6.550% | +5.039% |

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `+0.099%`
- Mean RMSE: `-0.369%`
- Consensus: `-1.164%`
- Attempted bytes: `+160320 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page V188 action before state extraction and lets the exact complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
