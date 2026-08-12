# Tracking-aligned X36 schedule H=8 return screen

- Contract: `tracking-aligned-x36-schedule-h8-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77 78 79]`
- Intervention: `explicit frozen H=8 sequence`
- Generation commit: `a1101fe3bf79f1a39fa8797f6ab163a908351f0e`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `139.80 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+6.767%`
- Best tail-safe mean gain: `+6.767%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload -> reference-full-payload -> reference-full-payload -> reference-full-payload -> reference-full-payload -> reference-full-payload -> reference-full-payload -> reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v125-reference-carrier-page-1-h8 -> v125-reference-carrier-page-2-h8 -> v125-reference-carrier-page-3-h8 -> v125-reference-carrier-page-4-h8 -> v125-reference-carrier-page-5-h8 -> v125-reference-carrier-page-6-h8 -> v125-reference-carrier-page-7-h8 -> v125-reference-carrier-page-8-h8` | 5+2+3+4 | 2 | NaN | +0.241207 | 1 | +6.767% | +12.705% | +3.281% | +10.959% | +19.899% | +1.447% | 1 |

## Evidence boundary

V125 is a privileged paired X36 seed-211 t=72 H=8 causal upper bound. It preserves every V124 graph row, fusion weight and receiver-side payload decision. Only the posterior transmitted on the persistent sensor-27-to-sensor-32 F5-to-F6 edge is replaced, page by page, by the same sender posterior captured from the paired clockwise full-payload arm. F5 therefore keeps the V124 protected local state while F6 receives the counterfactual static-carrier state. This requires an alternative-arm shadow trajectory and is not deployable; it only tests whether downstream state propagation is the remaining performance bottleneck. Measurements, delivery uniforms, filter RNG, topology, weights and byte accounting remain paired. No validation or generalization claim is authorized.
