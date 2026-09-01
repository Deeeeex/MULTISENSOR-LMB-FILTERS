# V241 formation-braid matched structural comparison

- Scene / seed: `x36-formation-fov-formation-braid / 1301`
- Scene valid: `1`
- Structural gate passed: `1`
- Tracking outcome authorized: `0`

| Arm | Messages min--max | Strong fraction | Physical | Tree changes | Reselection times | First fixed-tree dropout | Dropped route-messages |
|:--|:--:|--:|:--:|--:|:--|--:|--:|
| static-dropout | 66--72 | 0.338 | 1 | 0 | [] | 55 | 382 |
| always-replan | 72--72 | 1.000 | 1 | 7 | every step (160) | NaN | 0 |
| causal | 72--72 | 1.000 | 1 | 4 | [55 95 109 155] | NaN | 0 |

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim.
