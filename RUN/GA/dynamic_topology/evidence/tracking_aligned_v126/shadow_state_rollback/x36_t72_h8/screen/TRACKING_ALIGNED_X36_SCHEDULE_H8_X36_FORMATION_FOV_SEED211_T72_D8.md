# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `170aa3aebfb6a957a7dfd247241b000e1abf2ccb`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.82 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.705%`
- Best tail-safe mean gain: `+5.705%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v126-shadow-state-rollback-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.705% | +14.717% | +0.125% | +9.205% | +16.183% | +6.032% | 1 |

## Evidence boundary

V126 is a privileged paired X36 seed-211 t=72 H=8 state-recovery upper bound. It retains the complete V105 static carrier, fusion weights and protection schedule. On only the formation-page cells whose opened V105 gain is negative, the post-fusion posterior is replaced before extraction by the same node posterior captured from the paired static full-payload arm. The intervention therefore tests whether an independently maintained safe shadow state can preserve V105 aggregate headroom while eliminating accumulated local regret. The rollback cells and counterfactual shadow states use opened alternative-arm outcomes, so V126 is not deployable and cannot support validation or generalization claims. Measurements, delivery uniforms, filter RNG, topology, weights and candidate communication accounting remain paired.
