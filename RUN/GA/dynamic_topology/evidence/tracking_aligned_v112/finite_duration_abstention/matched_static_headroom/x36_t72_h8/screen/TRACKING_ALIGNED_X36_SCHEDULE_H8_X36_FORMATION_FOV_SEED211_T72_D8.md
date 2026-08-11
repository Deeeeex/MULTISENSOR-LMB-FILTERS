# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `abd898912a53aa0ec90f68ae7e747b758495a43f`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.33 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `4 / 4`
- Proxy TP / FP / FN: `4 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+4.300%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v112-abstain-3-then-full-h8` | 1+2+4+5+3 | 2 | NaN | +0.156245 | 1 | +2.703% | +5.389% | -0.928% | +1.000% | -2.509% | +1.103% | 1 |
| `v112-abstain-4-then-full-h8` | 1+2+4+5+3+6 | 3 | NaN | +0.212016 | 1 | +3.059% | +7.614% | -0.865% | +3.267% | +1.041% | +2.036% | 1 |
| `v112-abstain-5-then-full-h8` | 1+2+4+5+3+6 | 4 | NaN | +0.267787 | 1 | +3.927% | +8.808% | -0.865% | +5.216% | +2.194% | +2.729% | 1 |
| `v112-abstain-6-then-full-h8` | 1+2+4+5+3+6 | 5 | NaN | +0.323558 | 1 | +4.300% | +11.329% | -0.931% | +7.131% | +5.127% | +3.495% | 1 |

## Evidence boundary

V112 is a frozen retrospective H=8 action-family label screen. Each candidate follows the V105 formation schedule with explicit source abstention for exactly 3, 4, 5 or 6 pages and then restores the matched static full-payload route and fusion inputs for all remaining pages. Measurements, delivery uniforms, filter RNG, communication accounting and the frozen reference outcome remain paired. Truth and future outcomes are used only to label mean gain and worst downstream formation regret after execution. V112 is an action-space headroom study, not a deployable policy or a validation/generalization claim.
