# Online positive-net propagation H=6 return screen

- Contract: `tracking-aligned-x36-schedule-h6-opened-return-screen-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 211 / 72`
- Return times: `[72 73 74 75 76 77]`
- Intervention duration: `6` step(s)
- Generation commit: `d766b4d8e532ad78b554ad528a19ab59a4ab7e4c`
- Cache SHA-256: `33634cb7157e25a0ab7bb9b14e0ac51396b3fe8b9e973ad891f6cbaebc75ea80`
- Bank construction: `140.92 s`
- Observable formation ranking: `[2 1 4 3 5 6]`
- Support-weighted rescue scores: `[0.0206481 0.0109975 0.00957978 0.00857116 0.00514219 0.000832261]`
- Addressable / total network risk: `0.752% / 0.907%`
- Initial positive-net action / formations: `1 / [1 2 4 5]`
- Initial addressable coverage / useful-loss ratio / net benefit: `100.000% / 14.505% / 0.00643`
- Proxy positive / realized positive: `2 / 2`
- Proxy TP / FP / FN: `2 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+4.462%`
- Best tail-safe mean gain: `+4.462%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v97-positive-net-persistent-h6-f1-f2-f4-f5` | 1+2+4+5 | 2 | NaN | +0.278205 | 1 | +3.710% | +12.671% | -0.000% | +7.625% | +9.832% | +3.936% | 1 |
| `v100-online-positive-net-h6-f1-f2-f4-f5` | 1+2+4+5 | 3 | NaN | +0.278205 | 1 | +4.462% | +13.668% | +0.165% | +7.376% | +11.377% | +5.250% | 1 |

## Evidence boundary

V100 retains the exact V99 spatial selector and matched static carrier graph, but evaluates six fusion steps: one decision step plus the registered X36 formation-graph directed diameter of five. Static, fixed-set and online arms share the cached posterior, measurements, link uniforms, filter RNG, fusion weights and communication constraints. The probe tests whether the V99 gain is propagation-delayed or transient; it does not tune an outcome-based duration and remains opened development evidence only.
