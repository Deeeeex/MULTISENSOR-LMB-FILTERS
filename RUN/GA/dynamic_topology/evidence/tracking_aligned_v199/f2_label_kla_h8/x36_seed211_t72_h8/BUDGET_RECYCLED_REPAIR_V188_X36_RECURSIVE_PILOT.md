# V188 X36 H=8 recursive repair pilot

- Teacher-forced formation: `2`
- Teacher-forced source: `19`
- Teacher-forced label: `[13 12]`
- Update operator: `label-kla`
- Source fusion weight: `0.500`
- Maximum repair formations per page: `1`
- Repair page offsets: `0`
- Selected formations by page: `{2, [], [], [], [], [], [], []}`
- Repair applied by page: `[1 0 0 0 0 0 0 0]`
- Feasible proposals by page: `[1 0 0 0 0 0 0 0]`
- Repair bytes by page: `[60232 0 0 0 0 0 0 0]`

- Certified net saving by page: `[180952 0 0 0 0 0 0 0]` B

| Metric | Reference | V99 base | V188 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 79.451115 | 78.274634 |
| E-OSPA gain | 0 | +5.457% | +6.857% |
| Mean RMSE | 59.967347 | 62.172152 | 58.204481 |
| RMSE gain | 0 | -3.677% | +2.940% |
| Consensus gain | 0 | +8.803% | +11.348% |
| Byte saving | 0 | +5.423% | +4.870% |

- Formation E-OSPA gains: `[-0.931168 12.6338 7.71069 8.96969 12.4963 -0.021292]`
- Formation RMSE gains: `[1.84706 55.4856 -29.7994 1.78902 -2.56105 -0.748835]`

## Incremental effect of enabling repair over V99

- Mean E-OSPA: `+1.481%`
- Mean RMSE: `+6.382%`
- Consensus: `+2.790%`
- Attempted bytes: `+157896 B`

- Development gate passed: `0`

## Evidence boundary

This finite-horizon opened recursive pilot applies the truth-free selected-page label-kla update before state extraction and lets the resulting complete GM label propagate through later V99 pages. Ideal repair delivery is charged but not sampled, so the result is a recursive teacher mechanism screen, not deployable or cross-scene evidence.
