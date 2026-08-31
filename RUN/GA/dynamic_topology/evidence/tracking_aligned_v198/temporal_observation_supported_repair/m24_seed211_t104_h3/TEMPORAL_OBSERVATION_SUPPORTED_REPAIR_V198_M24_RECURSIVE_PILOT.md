# V198 M24 H=3 budgeted set-repair pilot

- Requested omission sets: `{[1 3 4], [1 2 3], [1 2 3]}`
- Projected omission sets: `{[1 3], [1 2 3], [1 2 3]}`
- Automatic full-posterior releases: `{4, [], []}`
- Total release count: `1`
- Release cooldown: `3` pages

| Metric | Reference | V99 base | V198 repair |
|:--|--:|--:|--:|
| Mean E-OSPA | 71.664511 | 65.182920 | 66.274291 |
| E-OSPA gain | 0 | +9.044% | +7.521% |
| Mean RMSE | 41.837145 | 40.275111 | 37.394018 |
| RMSE gain | 0 | +3.734% | +10.620% |
| Consensus gain | 0 | +21.104% | +17.429% |
| Byte saving | 0 | +5.080% | +3.562% |

- Formation E-OSPA gains: `[21.6726 4.44092 0 0]`
- Formation RMSE gains: `[60.9322 -0.145389 0 0]`
- Development gate passed: `0`

## Incremental effect over V99

- Mean E-OSPA: `-1.674%`
- Mean RMSE: `+7.154%`
- Consensus: `-4.659%`
- Attempted bytes: `+71448 B`

## Evidence boundary

V198 keeps the V197 top-one repair token and two-page post-release cooldown. Before spending an available token, it evaluates set-entry support using the current and immediately preceding local-posterior pages, including currently reachable cross-edge senders. A token is spent only when the entered set remains unsupported over this causal window. The policy uses no truth, future measurement, future outcome or numeric formation identifier. Opened anchors remain development evidence only.
