# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `76853ac46dcab92fd92c4efd605b1c14af394509`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.23 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.180%`
- Best tail-safe mean gain: `+6.180%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v131-intermittent-light-network-anchor-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +6.180% | +18.134% | +0.824% | +12.628% | +29.358% | +6.465% | 1 |

## Evidence boundary

V131 is a paired X36 seed-211 t=72 H=8 intermittent-anchor attribution. It retains the complete V105 working route, fusion weights and protection schedule, while maintaining a parallel network anchor at every node. The anchor uses local prediction and measurement updates on every page, but exchanges moment-compressed LMB posteriors only at t=72, 74, 76 and 78 over the same active edges and paired delivery outcomes. All auxiliary attempted and delivered bytes and runtime are charged directly; additional anchor memory is not yet byte-quantified. Rollback uses the opened V126 formation-time mask, so V131 remains a privileged mechanism test and cannot support online, validation, or generalization claims.
