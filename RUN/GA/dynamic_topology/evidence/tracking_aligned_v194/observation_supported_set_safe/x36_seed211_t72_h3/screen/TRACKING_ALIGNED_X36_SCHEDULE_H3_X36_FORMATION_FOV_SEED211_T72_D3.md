# Online positive-net payload H=3 return screen

- Contract: `tracking-aligned-x36-schedule-h3-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- Intervention duration: `3` step(s)
- Generation commit: `b947c4056ed6fd630d35f29f7a0d16af586f04cf`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `143.95 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Addressable / total network risk: `0.752% / 0.907%`
- Initial positive-net action / formations: `1 / [1 2 4 5]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 14.505% / 0.00643`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+0.946%`
- Best tail-safe mean gain: `+0.946%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v99-online-positive-net-f1-f2-f4-f5` | 1+2+4+5 | 3 | NaN | +0.139103 | 1 | +0.946% | +0.779% | +0.000% | +0.000% | +1.978% | +3.448% | 1 |

## Evidence boundary

V194 first computes the current V99 positive-net omission proposal. For each proposed receiver formation, it constructs the observable counterfactual marginal LMB extraction and restores the ordinary full posterior whenever any entering candidate label lacks current receiver measurement-association support. The rule uses no formation or time identifier, truth, future measurement or future outcome. The existing message builder and byte ledger execute and charge the projected set. Opened anchors remain development evidence only.
