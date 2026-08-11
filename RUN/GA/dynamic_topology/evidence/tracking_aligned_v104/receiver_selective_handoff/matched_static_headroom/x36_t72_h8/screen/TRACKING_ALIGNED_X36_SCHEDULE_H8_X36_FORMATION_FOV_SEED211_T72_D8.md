# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `dd751c8e350f50066e08e120aeb4fd7a07864e13`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.11 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.333%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v104-receiver-selective-handoff-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.333% | +16.734% | -0.931% | +9.591% | +17.077% | +6.091% | 1 |

## Evidence boundary

V104 is a retrospective receiver-granularity headroom oracle. It keeps only V103 handoff receiver rows whose same-page paired E-OSPA gain was strictly positive; target truth and the opened V103 outcome therefore select the frozen receiver set. It is not deployable and cannot support a validation claim. The V103 protection schedule, maturity timing, physical carrier, row message counts, positive-weight multisets, cached inputs, filter randomness and H=8 horizon are unchanged. The already frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. V104 tests whether receiver granularity alone can remove the V103 local regressions before opening a label-wise action space.
