# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `e52d13e68efdafb1293a40a9568e214a3e45ce0e`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.08 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.378%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v110-opened-safe-formation-h8` | 2+4+5+3 | 2 | NaN | +0.342958 | 1 | +5.378% | +14.912% | -0.216% | +7.619% | +17.980% | +3.479% | 1 |

## Evidence boundary

V110 is a frozen retrospective action-space oracle. It keeps the matched static route and V109 explicit source-abstention semantics, but protects only F2--F5 because opened V105 outcomes identify F1 and F6 as the nonpositive formations. The complete F1/F6 outcomes are therefore used to define the action before this V110 rollout. Measurements, delivery uniforms, filter RNG, communication model and the frozen static full-payload outcome remain paired. V110 is not deployable and cannot support validation or generalization claims; it tests whether a perfect formation-risk gate has enough headroom to retain the aggregate gain without local harm.
