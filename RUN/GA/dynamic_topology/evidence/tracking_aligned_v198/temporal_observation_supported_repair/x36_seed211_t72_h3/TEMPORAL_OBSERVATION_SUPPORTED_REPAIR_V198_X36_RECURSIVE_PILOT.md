# V198 X36 H=3 budgeted set-repair pilot

- Requested omission sets: `{[1 2 4 5], [1 3 4 5], [1 3 4 5]}`
- Projected omission sets: `{[1 4 5], [1 3 4 5], [1 3 4 5]}`
- Automatic full-posterior releases: `{2, [], []}`
- Total release count: `1`
- Release cooldown: `3` pages

| Metric | Reference | V99 base | V198 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 84.095019 |
| E-OSPA gain | 0 | +2.802% | +2.181% |
| Mean RMSE | 57.902417 | 58.288297 | 57.561228 |
| RMSE gain | 0 | -0.666% | +0.589% |
| Consensus gain | 0 | +5.149% | +2.648% |
| Byte saving | 0 | +6.550% | +5.397% |

- Formation E-OSPA gains: `[1.62646 0 2.39905 4.00729 5.65232 0]`
- Formation RMSE gains: `[-0.0689284 0 -1.48871 1.81777 -0.999639 0]`
- Development gate passed: `0`

## Incremental effect over V99

- Mean E-OSPA: `-0.638%`
- Mean RMSE: `+1.247%`
- Consensus: `-2.636%`
- Attempted bytes: `+122384 B`

## Evidence boundary

V198 keeps the V197 top-one repair token and two-page post-release cooldown. Before spending an available token, it evaluates set-entry support using the current and immediately preceding local-posterior pages, including currently reachable cross-edge senders. A token is spent only when the entered set remains unsupported over this causal window. The policy uses no truth, future measurement, future outcome or numeric formation identifier. Opened anchors remain development evidence only.
