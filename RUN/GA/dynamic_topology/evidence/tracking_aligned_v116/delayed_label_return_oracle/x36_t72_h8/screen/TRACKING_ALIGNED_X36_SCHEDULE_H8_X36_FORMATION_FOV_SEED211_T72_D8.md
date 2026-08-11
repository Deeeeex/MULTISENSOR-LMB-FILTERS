# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `2d2465a770e3c255d09177f6b25fa6aad8bb0924`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.81 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `3 / 3`
- Proxy TP / FP / FN: `3 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.706%`
- Best tail-safe mean gain: `+6.706%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v116-truth-ranked-top5-labels-p5-h8` | 2+4+5+3+6 | 2 | NaN | +0.346288 | 1 | +6.706% | +14.700% | +2.983% | +12.314% | +21.630% | +3.073% | 1 |
| `v116-truth-ranked-top10-labels-p5-h8` | 2+4+5+3+6 | 3 | NaN | +0.346288 | 1 | +6.634% | +13.339% | +2.983% | +12.032% | +20.708% | +2.843% | 1 |
| `v116-truth-ranked-top15-labels-p5-h8` | 2+4+5+3+6 | 4 | NaN | +0.346288 | 1 | +6.658% | +14.000% | +2.983% | +12.109% | +20.862% | +2.608% | 1 |

## Evidence boundary

V116 is a privileged opened-development X36 seed-211 t=72 H=8 action-space oracle. It keeps the V113 clockwise carrier and F2--F5 whole-source abstention fixed. On pages five through eight, sender-27 labels are matched one-to-one to current target states using privileged truth and ranked by Euclidean position error. The top 5, 10 or 15 complete labels are frozen before candidate outcomes are opened; omitted labels explicitly abstain. Future H=8 tracking outcomes choose only the reported oracle arm. V116 is not deployable, validation or generalization evidence.
