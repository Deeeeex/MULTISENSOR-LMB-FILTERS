# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `c63b880105f6d6f88207c8755ba574884013c3ba`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.20 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `3 / 3`
- Proxy TP / FP / FN: `3 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.613%`
- Best tail-safe mean gain: `+6.613%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v115-boundary-supported-labels-p5-h8` | 2+4+5+3+6 | 2 | NaN | +0.346288 | 1 | +6.613% | +12.718% | +2.983% | +11.850% | +20.879% | +2.456% | 1 |
| `v115-boundary-supported-or-high-r-p5-h8` | 2+4+5+3+6 | 3 | NaN | +0.346288 | 1 | +6.613% | +12.718% | +2.983% | +11.850% | +20.879% | +2.409% | 1 |
| `v115-boundary-receiver-need-p5-h8` | 2+4+5+3+6 | 4 | NaN | +0.346288 | 1 | +6.613% | +12.718% | +2.983% | +11.877% | +20.879% | +2.395% | 1 |

## Evidence boundary

V115 is an opened-development X36 seed-211 t=72 H=8 label-action screen. It keeps the V113 clockwise carrier and F2--F5 whole-source abstention fixed. Starting on page five, only the F6 boundary edge 27-to-32 changes from whole-posterior shielding to complete-label whitelisting. The three rules use current sender association support, sender existence, or current receiver need; omitted labels explicitly abstain and are not interpreted as negative evidence. Truth and future outcomes do not select labels at runtime. The action family and anchor were chosen after V114, so this is method-development evidence, not validation or cross-scene generalization.
