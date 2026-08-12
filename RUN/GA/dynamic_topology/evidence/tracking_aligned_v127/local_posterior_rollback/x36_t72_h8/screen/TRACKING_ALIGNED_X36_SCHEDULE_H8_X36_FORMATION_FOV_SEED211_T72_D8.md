# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `d4c01e06563bae6352d772cf72218bc8393e2cbc`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `145.11 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.365%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v127-local-posterior-rollback-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.365% | +16.701% | -0.954% | +9.332% | +15.740% | +6.085% | 1 |

## Evidence boundary

V127 is a paired X36 seed-211 t=72 H=8 state-source attribution. It retains the complete V105 route, fusion weights and protection schedule, and retains the opened V126 rollback node-time mask. At those cells it replaces the post-fusion working posterior with the same receiver node's current measurement-updated local posterior. That state is already computed by the standard filter and requires no additional inter-node payload or parallel shadow filter. The rollback timing still comes from opened outcomes, so V127 tests state source feasibility only and is not an online policy, validation, or generalization claim. Measurements, delivery uniforms, filter RNG, topology, weights and communication accounting remain paired.
