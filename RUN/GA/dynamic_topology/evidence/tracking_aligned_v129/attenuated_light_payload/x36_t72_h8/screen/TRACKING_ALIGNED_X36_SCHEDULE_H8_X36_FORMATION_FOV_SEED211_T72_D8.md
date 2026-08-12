# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `090dc87150b99751505c3c7697f8df15f6f84a7e`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.96 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+1.614%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v129-attenuated-light-payload-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +1.614% | +6.982% | -0.559% | +2.878% | +6.404% | +7.865% | 1 |

## Evidence boundary

V129 is a paired X36 seed-211 t=72 H=8 soft-protection attribution. It retains the V105 static carrier, fusion weights and observable formation schedule, but replaces each protected control-only message with the sender's current per-label moment-compressed LMB posterior. The protected neighbor input uses a frozen 0.5 reliability factor before normalization. The factor is a single registered midpoint between V105 abstention and full static fusion, not an outcome-selected sweep. No parallel filter, target truth, future measurement or opened V126 rollback mask enters the candidate. Actual compressed payload bytes are included. This first opened case tests the mechanism and is not validation or a generalization claim.
