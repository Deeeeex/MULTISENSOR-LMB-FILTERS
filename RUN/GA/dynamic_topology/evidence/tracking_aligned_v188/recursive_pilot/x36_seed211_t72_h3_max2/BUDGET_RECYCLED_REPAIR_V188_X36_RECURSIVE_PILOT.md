# V188 X36 H=3 recursive repair pilot

- Teacher-forced formation: `0`
- Maximum repair formations per page: `2`
- Selected formations by page: `{[3 1], [], []}`
- Repair applied by page: `[2 0 0]`
- Feasible proposals by page: `[6 0 0]`
- Repair bytes by page: `[80840 0 0]`

- Certified net saving by page: `[160344 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 83.671892 |
| E-OSPA gain | 0 | +2.802% | +2.673% |
| Mean RMSE | 57.902417 | 58.288297 | 57.681418 |
| RMSE gain | 0 | -0.666% | +0.382% |
| Consensus gain | 0 | +5.149% | +5.712% |
| Byte saving | 0 | +6.550% | +5.530% |

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `-0.132%`
- Mean RMSE: `+1.041%`
- Consensus: `+0.594%`
- Attempted bytes: `+108224 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page V188 action before state extraction and lets the exact complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
