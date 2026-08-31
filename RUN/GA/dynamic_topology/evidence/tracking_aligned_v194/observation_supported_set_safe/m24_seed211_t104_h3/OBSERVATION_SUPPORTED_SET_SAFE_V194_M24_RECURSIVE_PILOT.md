# V194 M24 H=3 observation-supported set-safe pilot

- Requested omission sets: `{[1 3 4], [1 2 3], [1 2 3]}`
- Projected omission sets: `{[1 3], 3, 3}`
- Automatic full-posterior releases: `{4, [1 2], [1 2]}`
- Any automatic release applied: `1`

| Metric | Reference | V99 base | V194 set-safe |
|:--|--:|--:|--:|
| Mean E-OSPA | 71.664511 | 65.182920 | 69.842132 |
| E-OSPA gain | 0 | +9.044% | +2.543% |
| Mean RMSE | 41.837145 | 40.275111 | 37.355122 |
| RMSE gain | 0 | +3.734% | +10.713% |
| Consensus gain | 0 | +21.104% | +5.650% |
| Byte saving | 0 | +5.080% | +0.272% |

- Formation E-OSPA gains: `[8.90577 0 0 0]`
- Formation RMSE gains: `[60.8656 0 0 0]`
- Development gate passed: `0`

## Evidence boundary

V194 first computes the current V99 positive-net omission proposal. For each proposed receiver formation, it constructs the observable counterfactual marginal LMB extraction and restores the ordinary full posterior whenever any entering candidate label lacks current receiver measurement-association support. The rule uses no formation or time identifier, truth, future measurement or future outcome. The existing message builder and byte ledger execute and charge the projected set. Opened anchors remain development evidence only.
