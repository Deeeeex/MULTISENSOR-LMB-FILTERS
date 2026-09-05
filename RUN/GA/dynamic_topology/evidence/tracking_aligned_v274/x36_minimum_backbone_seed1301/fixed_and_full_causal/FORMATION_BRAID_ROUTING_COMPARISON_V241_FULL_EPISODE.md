# V241 formation-braid routing comparison

- Scene / seed: `x36-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Current execution commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Fixed arm commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Causal arm commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Fixed no-rerouting baseline included: `1`
- Validation claim allowed: `0`

Structural preflight: static strong fraction `0.338`; always-replan changes `7`; causal changes `4`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Messages | Strong fraction | Routes |
|:--|--:|--:|--:|--:|--:|--:|:--:|--:|--:|
| Fixed formation tree | 132.680 | 131.588 | 36.925 | 37.377 | 139.407 | 76871008 | 66--72 | 0.338 | 13 |
| Always replan | — | — | — | — | — | — | — | — | — |
| Causal minimal edit | 131.795 | 130.685 | 19.586 | 24.689 | 139.004 | 81258696 | 72--72 | 1.000 | 38 |

## Causal over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.667%` |
| Focus E-OSPA | `+0.686%` |
| Full RMSE | `+46.958%` |
| Focus RMSE | `+33.947%` |
| Focus consistency | `+0.289%` |
| Attempted-byte saving | `-5.708%` |
| Weakest formation E-OSPA | `-1.079%` |
| Weakest formation RMSE | `+8.389%` |

- Mean direction passed: `1`
- Nonnegative formation tail passed: `0`

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim. A completed fixed-tree arm from the registered source commit may be reused only for a causal-only resume when the git diff contains the registered runtime-context compatibility files and no method, fusion, scenario, measurement, or metric change.
