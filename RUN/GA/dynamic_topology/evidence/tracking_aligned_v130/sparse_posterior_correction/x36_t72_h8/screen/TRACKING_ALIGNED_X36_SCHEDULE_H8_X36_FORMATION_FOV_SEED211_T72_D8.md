# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `15006ff1b8335e397f4c8eaa49d834932ee92ad9`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `142.22 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.089%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v130-sparse-posterior-correction-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.089% | +14.841% | -0.878% | +8.825% | +19.545% | +6.115% | 1 |

## Evidence boundary

V130 is a paired X36 seed-211 t=72 H=8 sparse-action upper bound. It keeps the V105 control-only payload as the default, applies a 0.5-weight moment-compressed posterior to F1 on pages 6--8 and F6 on pages 7--8, and restores the full posterior to F2 on page 5. The formation-page correction support is inherited from the opened V126 failure mask and is therefore privileged; target truth and future measurements do not enter message construction itself. Actual control, light and full payload bytes are included and no parallel state is maintained. V130 tests hybrid action-space headroom only and is not deployable, validation, or generalization evidence.
