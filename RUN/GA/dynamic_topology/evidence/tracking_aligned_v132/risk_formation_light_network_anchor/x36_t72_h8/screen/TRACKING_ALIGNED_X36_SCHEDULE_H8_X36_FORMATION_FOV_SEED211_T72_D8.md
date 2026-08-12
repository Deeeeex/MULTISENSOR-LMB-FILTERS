# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `bc0b2b760ba045c62b4372b4546abb999dd6c4d7`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.77 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.221%`
- Best tail-safe mean gain: `+6.221%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v132-risk-formation-light-network-anchor-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +6.221% | +17.849% | +0.506% | +12.652% | +30.318% | +7.309% | 1 |

## Evidence boundary

V132 is a paired X36 seed-211 t=72 H=8 message-reallocation mechanism test. It retains the V131 parallel moment-compressed network anchor and reallocates auxiliary directed messages on the receiver rows of opened risk formations F1, F2 and F6 at every page instead of refreshing all six formations on alternating pages. The working V105 route, payload and protection schedule remain unchanged. Actual auxiliary attempted/delivered bytes and runtime are charged, while extra anchor memory remains unquantified. The receiver formation set and rollback cells use opened V126/V131 outcomes, so V132 is privileged development evidence only and cannot support online, validation, or generalization claims.
