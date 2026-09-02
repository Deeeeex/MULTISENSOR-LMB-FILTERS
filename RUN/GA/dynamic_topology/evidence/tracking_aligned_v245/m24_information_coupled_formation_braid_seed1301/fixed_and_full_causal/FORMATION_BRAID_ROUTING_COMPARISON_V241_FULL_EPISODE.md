# V241 formation-braid routing comparison

- Scene / seed: `m24-formation-fov-coupled-formation-braid / 1301`
- Source commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- Current execution commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- Fixed arm commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- Causal arm commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- Fixed no-rerouting baseline included: `1`
- Validation claim allowed: `0`

Structural preflight: static strong fraction `0.431`; always-replan changes `5`; causal changes `2`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Messages | Strong fraction | Routes |
|:--|--:|--:|--:|--:|--:|--:|:--:|--:|--:|
| Fixed formation tree | 126.724 | 124.560 | 9.052 | 8.823 | 135.624 | 34363568 | 46--48 | 0.431 | 13 |
| Always replan | — | — | — | — | — | — | — | — | — |
| Causal minimal edit | 125.394 | 123.428 | 8.947 | 8.662 | 133.109 | 38421672 | 48--48 | 1.000 | 22 |

## Causal over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+1.050%` |
| Focus E-OSPA | `+0.909%` |
| Full RMSE | `+1.161%` |
| Focus RMSE | `+1.823%` |
| Focus consistency | `+1.854%` |
| Attempted-byte saving | `-11.809%` |
| Weakest formation E-OSPA | `-0.088%` |
| Weakest formation RMSE | `-0.498%` |

- Mean direction passed: `1`
- Nonnegative formation tail passed: `0`

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim. A completed fixed-tree arm from the registered source commit may be reused only for a causal-only resume when the git diff contains the registered runtime-context compatibility files and no method, fusion, scenario, measurement, or metric change.
