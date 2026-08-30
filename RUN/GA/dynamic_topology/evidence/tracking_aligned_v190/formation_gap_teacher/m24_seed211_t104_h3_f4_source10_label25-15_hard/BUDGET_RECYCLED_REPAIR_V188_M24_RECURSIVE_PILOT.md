# V188 M24 H=3 recursive repair pilot

- Teacher-forced formation: `4`
- Teacher-forced source: `10`
- Teacher-forced label: `[25 15]`
- Update operator: `hard-replacement`
- Source fusion weight: `1.000`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{4, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[1 0 0]`
- Repair bytes by page: `[38872 0 0]`

- Certified net saving by page: `[98544 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 71.664511 | 65.182920 | 65.936612 |
| E-OSPA gain | 0 | +9.044% | +7.993% |
| Mean RMSE | 41.837145 | 40.275111 | 40.999714 |
| RMSE gain | 0 | +3.734% | +2.002% |
| Consensus gain | 0 | +21.104% | +19.410% |
| Byte saving | 0 | +5.080% | +4.040% |

- Formation E-OSPA gains: `[21.6726 4.44092 0 1.73536]`
- Formation RMSE gains: `[60.9322 -0.145389 0 -158.439]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `-1.156%`
- Mean RMSE: `-1.799%`
- Consensus: `-2.148%`
- Attempted bytes: `+48952 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page hard-replacement update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
