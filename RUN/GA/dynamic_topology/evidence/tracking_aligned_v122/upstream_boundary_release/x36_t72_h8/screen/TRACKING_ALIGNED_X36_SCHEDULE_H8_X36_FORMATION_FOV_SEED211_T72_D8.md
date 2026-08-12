# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `1fea2d1c516f7e9dc03ebfabb8ad188441752670`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.28 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.297%`
- Best tail-safe mean gain: `+5.297%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v122-release-upstream-f5-h8` | 2+4+3 | 2 | NaN | +0.301821 | 1 | +5.297% | +7.958% | +1.471% | +6.952% | +16.143% | +0.729% | 1 |

## Evidence boundary

V122 is a single-arm privileged X36 seed-211 t=72 H=8 causal screen. It keeps the fixed clockwise carrier and the V113 explicit-abstention mechanism, but removes F5 from the protected set on every page while retaining F2--F4. This tests whether the delayed F6 loss is propagated by an altered F5 posterior rather than caused by F6 input geometry. V113 CCW full, CW full and F2--F5 abstention outcomes are reused. Measurements, delivery uniforms, filter RNG, topology, weights and communication accounting remain paired. The action was chosen after V121 opened outcomes and is development evidence only, not validation or generalization evidence.
