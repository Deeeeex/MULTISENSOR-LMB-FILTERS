# V198 X36 H=8 budgeted set-repair pilot

- Requested omission sets: `{[1 2 4 5], [1 3 4 5], [1 3 4 5], [1 2 3 4 5 6], [1 3 4], [1 2 3 4 5 6], [1 2 3 4 5], [1 2 3 4 5 6]}`
- Projected omission sets: `{[1 4 5], [1 3 4 5], [1 3 4 5], [1 2 3 4 5 6], [1 3 4], [1 2 3 4 5 6], [1 2 3 4 5], [1 2 3 4 5 6]}`
- Automatic full-posterior releases: `{2, [], [], [], [], [], [], []}`
- Total release count: `1`
- Release cooldown: `3` pages

| Metric | Reference | V99 base | V198 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 79.451115 | 79.535696 |
| E-OSPA gain | 0 | +5.457% | +5.357% |
| Mean RMSE | 59.967347 | 62.172152 | 60.832025 |
| RMSE gain | 0 | -3.677% | -1.442% |
| Consensus gain | 0 | +8.803% | +7.809% |
| Byte saving | 0 | +5.423% | +5.341% |

- Formation E-OSPA gains: `[-0.931168 4.24268 7.71069 8.96969 12.4963 -0.021292]`
- Formation RMSE gains: `[1.84706 -2.71963 -29.7994 1.78902 -2.56105 -0.748835]`
- Development gate passed: `0`

## Incremental effect over V99

- Mean E-OSPA: `-0.106%`
- Mean RMSE: `+2.156%`
- Consensus: `-1.090%`
- Attempted bytes: `+23384 B`

## Evidence boundary

V198 keeps the V197 top-one repair token and two-page post-release cooldown. Before spending an available token, it evaluates set-entry support using the current and immediately preceding local-posterior pages, including currently reachable cross-edge senders. A token is spent only when the entered set remains unsupported over this causal window. The policy uses no truth, future measurement, future outcome or numeric formation identifier. Opened anchors remain development evidence only.
