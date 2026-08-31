# V194 X36 H=3 observation-supported set-safe pilot

- Requested omission sets: `{[1 2 4 5], [1 3 4 5], [1 3 4 5]}`
- Projected omission sets: `{[1 4], [1 4], [1 4 5]}`
- Automatic full-posterior releases: `{[2 5], [3 5], 3}`
- Any automatic release applied: `1`

| Metric | Reference | V99 base | V194 set-safe |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 85.157065 |
| E-OSPA gain | 0 | +2.802% | +0.946% |
| Mean RMSE | 57.902417 | 58.288297 | 57.451224 |
| RMSE gain | 0 | -0.666% | +0.779% |
| Consensus gain | 0 | +5.149% | +1.978% |
| Byte saving | 0 | +6.550% | +3.448% |

- Formation E-OSPA gains: `[1.62646 0 0 4.00729 1.32065e-05 0]`
- Formation RMSE gains: `[-0.0689284 0 0 1.81777 0.00373561 0]`
- Development gate passed: `0`

## Evidence boundary

V194 first computes the current V99 positive-net omission proposal. For each proposed receiver formation, it constructs the observable counterfactual marginal LMB extraction and restores the ordinary full posterior whenever any entering candidate label lacks current receiver measurement-association support. The rule uses no formation or time identifier, truth, future measurement or future outcome. The existing message builder and byte ledger execute and charge the projected set. Opened anchors remain development evidence only.
