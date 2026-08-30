# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `4dd1119e6acf8a53ac14a24761747a1fbb9607e1`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.04 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.522%`
- Best tail-safe mean gain: `+6.522%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v157-positive-value-reference-label-k4` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +6.522% | +17.384% | +0.542% | +10.163% | +14.668% | +5.877% | 1 |

## Evidence boundary

V157 is a privileged paired X36 seed-211 t=72 H=8 label-value mechanism oracle. It retains the V105 carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it greedily tests complete GM Bernoulli reference-label edits against exact current-step truth E-OSPA, accepts only strictly positive marginal edits, and stops after at most four labels. The reference state, rollback cells and truth-valued selection are privileged, so V157 is not deployable and cannot support validation or generalization claims. Communication is conservatively charged 198144 bytes, equal to four maximum-size labels plus one header at every rollback cell.
