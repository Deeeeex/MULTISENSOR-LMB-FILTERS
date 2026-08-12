# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `39f09400c37fe42f14b6d0f6ace9da1f8fc20661`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.82 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `3 / 3`
- Proxy TP / FP / FN: `3 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.121%`
- Best tail-safe mean gain: `+6.121%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v117-entry27-to33-return34-to2-h8` | 2+4+5+3 | 2 | NaN | +0.342958 | 1 | +6.121% | +9.187% | +1.426% | +9.678% | +15.732% | +2.397% | 1 |
| `v117-entry27-to35-return36-to2-h8` | 2+4+5+3 | 3 | NaN | +0.342958 | 1 | +5.773% | +7.313% | +0.321% | +9.270% | +16.576% | +2.275% | 1 |
| `v117-entry27-to34-return35-to2-h8` | 2+4+5+3 | 4 | NaN | +0.342958 | 1 | +6.059% | +12.075% | +1.428% | +10.140% | +15.322% | +2.436% | 1 |

## Evidence boundary

V117 is a privileged opened-development X36 seed-211 t=72 H=8 gateway-location oracle. It keeps the clockwise formation cycle, the V113 F2--F5 abstention schedule, message count and fusion-weight multiset fixed. The original 27-to-32 F5-to-F6 residual is moved to receiver 33, 35 or 34; the displaced internal F6 source simultaneously takes over the original 33-to-2 F6-to-F1 return residual. This paired entry-return move preserves every sensor's outward influence path and both formation-cycle edges. These receivers were frozen from opened current tracking errors; future H=8 outcomes choose only the reported oracle arm. V117 is not deployable, validation or generalization evidence.
