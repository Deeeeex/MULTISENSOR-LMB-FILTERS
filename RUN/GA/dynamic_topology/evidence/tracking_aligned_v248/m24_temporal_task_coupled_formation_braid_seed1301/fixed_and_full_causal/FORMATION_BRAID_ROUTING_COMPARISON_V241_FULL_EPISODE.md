# V241 formation-braid routing comparison

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- Current execution commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- Fixed arm commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- Causal arm commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- Fixed no-rerouting baseline included: `1`
- Validation claim allowed: `0`

Structural preflight: static strong fraction `0.431`; always-replan changes `5`; causal changes `2`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Messages | Strong fraction | Routes |
|:--|--:|--:|--:|--:|--:|--:|:--:|--:|--:|
| Fixed formation tree | 125.478 | 124.413 | 22.640 | 17.292 | 133.599 | 40769168 | 46--48 | 0.431 | 13 |
| Always replan | — | — | — | — | — | — | — | — | — |
| Causal minimal edit | 122.380 | 121.666 | 14.081 | 13.536 | 131.913 | 44867136 | 48--48 | 1.000 | 22 |

## Causal over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+2.469%` |
| Focus E-OSPA | `+2.207%` |
| Full RMSE | `+37.805%` |
| Focus RMSE | `+21.719%` |
| Focus consistency | `+1.262%` |
| Attempted-byte saving | `-10.052%` |
| Weakest formation E-OSPA | `+1.515%` |
| Weakest formation RMSE | `-6.053%` |

- Mean direction passed: `1`
- Nonnegative formation tail passed: `0`

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim. A completed fixed-tree arm from the registered source commit may be reused only for a causal-only resume when the git diff contains the registered runtime-context compatibility files and no method, fusion, scenario, measurement, or metric change.
