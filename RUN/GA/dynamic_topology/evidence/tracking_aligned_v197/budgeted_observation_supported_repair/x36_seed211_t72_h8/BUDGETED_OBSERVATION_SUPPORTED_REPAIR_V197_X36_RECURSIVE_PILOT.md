# V197 X36 H=8 budgeted set-repair pilot

- Requested omission sets: `{[1 2 4 5], [1 3 4 5], [1 3 4 5], [1 2 3 4 5 6], [1 3 4], [1 2 3 4 5 6], [1 2 3 4 5], [1 2 3 4 5 6]}`
- Projected omission sets: `{[1 4 5], [1 3 4 5], [1 3 4 5], [1 2 4 5 6], [1 3 4], [1 2 3 4 5 6], [1 2 3 4], [1 2 3 4 5 6]}`
- Automatic full-posterior releases: `{2, [], [], 3, [], [], 5, []}`
- Total release count: `3`
- Release cooldown: `3` pages

| Metric | Reference | V99 base | V197 budgeted repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 79.451115 | 79.652782 |
| E-OSPA gain | 0 | +5.457% | +5.217% |
| Mean RMSE | 59.967347 | 62.172152 | 61.870974 |
| RMSE gain | 0 | -3.677% | -3.174% |
| Consensus gain | 0 | +8.803% | +7.657% |
| Byte saving | 0 | +5.423% | +4.886% |

- Formation E-OSPA gains: `[-0.931168 4.24268 7.50547 8.96969 11.8234 -0.021292]`
- Formation RMSE gains: `[1.84706 -2.71963 -55.0062 1.78902 -2.59152 -0.748835]`
- Development gate passed: `0`

## Incremental effect over V99

- Mean E-OSPA: `-0.254%`
- Mean RMSE: `+0.484%`
- Consensus: `-1.257%`
- Attempted bytes: `+153344 B`

## Evidence boundary

V197 computes the current V99 omission proposal and the V194 observation-unsupported set-entry projection. If no full-posterior repair occurred during the preceding two completed pages, it restores at most one proposed formation: the formation with the largest current maximum receiver set-entry risk. The ordinary message builder and byte ledger execute and charge the projected payloads. The policy sees only current observable posteriors and its own past release decisions; it uses no truth, future measurement or future outcome. Opened anchors remain development evidence only.
