# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention duration: `8` step(s)
- Generation commit: `3821c75fe193ce8487b0af1bbd1436197dfc9a11`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.47 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+5.334%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v103-matured-handoff-h8` | 1+2+4+5+3+6 | 2 | NaN | +0.435100 | 1 | +5.334% | +16.734% | -0.945% | +9.651% | +18.432% | +5.981% | 1 |

## Evidence boundary

V103 is a frozen matched-static causal headroom probe. It retains V101 protection for three complete fusion pages before a gateway posterior is eligible for one later within-formation handoff. Different formation activation times determine the handoff pages; reference-recovery pages separate all handoffs so rolling three-page sensor- and formation-level reachability is retained. Every changed row preserves its reference message count and positive-weight multiset. Static and candidate arms share the cached posterior, measurements, delivery uniforms, filter RNG, carrier graph, communication model and horizon. The schedule is frozen before V103 outcomes are opened and is not an online policy or validation claim.
