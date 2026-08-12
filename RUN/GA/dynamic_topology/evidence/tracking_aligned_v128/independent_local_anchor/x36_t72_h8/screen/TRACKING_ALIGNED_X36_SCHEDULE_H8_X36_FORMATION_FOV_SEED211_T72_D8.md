# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `2fd5f0ae1c463bc49b83e5ad2fdca459fdbf4cd4`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.88 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+3.683%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v128-independent-local-anchor-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +3.683% | +10.530% | -3.005% | +4.585% | +3.968% | +6.874% | 1 |

## Evidence boundary

V128 is a paired X36 seed-211 t=72 H=8 independent-anchor attribution. It retains the complete V105 route, fusion weights, protection schedule and opened V126 rollback mask. Each node forks an anchor from the common t=72 local posterior and thereafter propagates it using only its own prediction and measurement update. Rollback reads that independent local anchor, which adds no posterior message or payload byte. Candidate runtime includes anchor filtering; additional anchor memory is not yet quantified. The rollback timing still comes from opened outcomes, so V128 tests state-source feasibility only and is not an online policy, validation, or generalization claim.
