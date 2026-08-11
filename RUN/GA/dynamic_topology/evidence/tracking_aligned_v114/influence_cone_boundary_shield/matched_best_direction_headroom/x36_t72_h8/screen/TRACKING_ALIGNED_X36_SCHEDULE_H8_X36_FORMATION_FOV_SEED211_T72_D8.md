# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `be8ba6e2ba6671f0ce6d8b9fdc90abbd8afacb9b`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `138.80 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `2 / 2`
- Proxy TP / FP / FN: `2 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.804%`
- Best tail-safe mean gain: `+6.804%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v114-f6-shield-on-arrival-p6-h8` | 2+4+5+3+6 | 2 | NaN | +0.345455 | 1 | +6.758% | +15.497% | +2.983% | +12.561% | +23.235% | +3.071% | 1 |
| `v114-f6-shield-early-p5-h8` | 2+4+5+3+6 | 3 | NaN | +0.346288 | 1 | +6.804% | +16.734% | +2.983% | +12.849% | +23.170% | +3.289% | 1 |

## Evidence boundary

V114 is a frozen retrospective X36 seed-211 t=72 H=8 boundary-control screen. V113 opened outcomes identify the clockwise 27-to-32 cross-formation residual edge and the sixth page as the first F6 loss page. V114 therefore holds the V113 clockwise F2--F5 abstention mechanism fixed and adds F6 receiver-side abstention either on the predicted arrival page or one page earlier. The V113 CCW-full, CW-full and unshielded mechanism outcomes are reused; only the two boundary arms are executed. Measurements, delivery uniforms, filter RNG, carrier weights, physical edges and communication accounting remain paired, and rolling B3 is preserved. Because the boundary and timing were selected from opened V113 outcomes, V114 is an oracle action-space screen, not a deployable policy or validation claim.
