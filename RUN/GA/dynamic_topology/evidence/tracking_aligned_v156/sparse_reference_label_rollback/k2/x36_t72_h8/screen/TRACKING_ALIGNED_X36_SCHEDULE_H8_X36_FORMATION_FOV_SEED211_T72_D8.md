# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `b564614075b5cabe4926add3561a16ef1a7d3360`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.52 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.738%`
- Best tail-safe mean gain: `+5.738%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v156-sparse-reference-label-k2` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.738% | +14.069% | +0.126% | +8.923% | +15.836% | +6.149% | 1 |

## Evidence boundary

V156 K=2 is a privileged paired X36 seed-211 t=72 H=8 representational-sufficiency oracle. It retains the V105 static carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it compares the evolving working posterior with the paired static posterior, ranks labels without truth or future measurements, and copies at most 2 complete GM Bernoulli label states (including explicit tombstones). The paired reference and opened rollback cells are counterfactual information, so the arm is not deployable and cannot support validation or generalization claims. Communication accounting still contains only the V105 main path; reference-label transport is estimated separately.
