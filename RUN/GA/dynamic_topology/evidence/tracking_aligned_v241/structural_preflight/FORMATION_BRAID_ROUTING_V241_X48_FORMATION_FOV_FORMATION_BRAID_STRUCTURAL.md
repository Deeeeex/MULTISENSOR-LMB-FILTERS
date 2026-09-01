# V241 formation-braid matched structural comparison

- Scene / seed: `x48-formation-fov-formation-braid / 1301`
- Scene valid: `1`
- Structural gate passed: `1`
- Tracking outcome authorized: `0`

| Arm | Messages min--max | Strong fraction | Physical | Tree changes | Reselection times | First fixed-tree dropout | Dropped route-messages |
|:--|:--:|--:|:--:|--:|:--|--:|--:|
| static-dropout | 90--96 | 0.281 | 1 | 0 | [] | 46 | 532 |
| always-replan | 96--96 | 1.000 | 1 | 10 | every step (160) | NaN | 0 |
| causal | 96--96 | 1.000 | 1 | 6 | [46 71 101 104 139 149] | NaN | 0 |

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim.
