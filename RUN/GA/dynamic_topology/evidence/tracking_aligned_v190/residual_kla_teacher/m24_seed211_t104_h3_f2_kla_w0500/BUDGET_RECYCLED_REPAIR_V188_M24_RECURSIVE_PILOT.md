# V188 M24 H=3 recursive repair pilot

- Teacher-forced formation: `2`
- Teacher-forced source: `0`
- Teacher-forced label: `[0 0]`
- Update operator: `label-kla`
- Source fusion weight: `0.500`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{2, [], []}`
- Repair applied by page: `[1 0 0]`
- Feasible proposals by page: `[4 0 0]`
- Repair bytes by page: `[34840 0 0]`

- Certified net saving by page: `[102576 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 71.664511 | 65.182920 | 67.947302 |
| E-OSPA gain | 0 | +9.044% | +5.187% |
| Mean RMSE | 41.837145 | 40.275111 | 41.569134 |
| RMSE gain | 0 | +3.734% | +0.641% |
| Consensus gain | 0 | +21.104% | +12.959% |
| Byte saving | 0 | +5.080% | +4.431% |

- Formation E-OSPA gains: `[21.6726 -8.40872 0 5.60865]`
- Formation RMSE gains: `[60.9322 -4.44022 0 -126.599]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `-4.241%`
- Mean RMSE: `-3.213%`
- Consensus: `-10.325%`
- Attempted bytes: `+30560 B`

- Development gate passed: `0`

## Evidence boundary

This H=3 opened recursive pilot applies the truth-free first-page label-kla update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
