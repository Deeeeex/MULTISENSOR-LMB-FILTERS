# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `d5b7b8bbb83e7d9aed4672eddb9e94c7c2070f79`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `143.22 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `5 / 5`
- Proxy TP / FP / FN: `5 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.677%`
- Best tail-safe mean gain: `+6.677%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v118-source25-to32-h8` | 2+4+5+3 | 2 | NaN | +0.342958 | 1 | +6.459% | +12.004% | +2.832% | +11.189% | +19.559% | +2.248% | 1 |
| `v118-source26-to32-h8` | 2+4+5+3 | 3 | NaN | +0.342958 | 1 | +6.471% | +12.484% | +2.815% | +10.758% | +18.763% | +2.066% | 1 |
| `v118-source28-to32-h8` | 2+4+5+3 | 4 | NaN | +0.342958 | 1 | +6.677% | +15.309% | +2.849% | +11.901% | +20.130% | +2.340% | 1 |
| `v118-source29-to32-h8` | 2+4+5+3 | 5 | NaN | +0.342958 | 1 | +6.574% | +13.873% | +2.983% | +11.578% | +20.219% | +2.209% | 1 |
| `v118-source30-to32-h8` | 2+4+5+3 | 6 | NaN | +0.342958 | 1 | +6.559% | +12.143% | +3.257% | +11.169% | +20.906% | +1.940% | 1 |

## Evidence boundary

V118 is a privileged opened-development X36 seed-211 t=72 H=8 single-source oracle. It keeps receiver 32, the clockwise formation carrier, the V113 F2--F5 abstention schedule, the F6 internal and return paths, message count and fusion weights fixed. The original 27-to-32 residual is replaced by each other F5 sender, 25, 26, 28, 29 or 30. Because that cross edge is node 27's only outward influence, every candidate also replaces the fixed 25-to-26 intra-F5 edge by 27-to-26. This minimum paired role migration preserves connectivity without adding a message. All five alternatives are exhausted; future H=8 outcomes choose only the reported oracle arm. V118 is not deployable, validation or generalization evidence.
