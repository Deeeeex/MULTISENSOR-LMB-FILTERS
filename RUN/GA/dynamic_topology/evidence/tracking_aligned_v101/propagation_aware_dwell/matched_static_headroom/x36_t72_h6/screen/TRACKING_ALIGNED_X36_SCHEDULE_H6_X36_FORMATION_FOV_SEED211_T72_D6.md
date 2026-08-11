# Tracking-aligned X36 schedule H=6 return screen

- Contract: `tracking-aligned-x36-schedule-h6-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77]`
- Intervention duration: `6` step(s)
- Generation commit: `d72347dff95a8fc260f344fa88de1763cc847c6d`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.19 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+4.589%`
- Best tail-safe mean gain: `+4.589%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v101-propagation-aware-dwell-h6` | 1+2+4+5+3+6 | 2 | NaN | +0.323558 | 1 | +4.589% | +13.668% | +0.165% | +7.670% | +11.632% | +5.811% | 1 |

## Evidence boundary

V101 is a topology-calibrated headroom probe on the matched static carrier graph. It applies a three-step minimum dwell to the causal V100 receiver-formation selections, because the registered X36 within-formation dominant tree has gateway depth two. The only changed execution page is t=76, where formations 5 and 6 remain protected. The schedule is frozen from observable V100 decisions before V101 tracking outcomes are opened. It tests the delayed-action mechanism and is not yet an online deployable policy or validation claim.
