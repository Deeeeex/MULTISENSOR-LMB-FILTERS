# V241 formation-braid routing comparison

- Scene / seed: `m24-formation-fov-formation-braid / 1301`
- Source commit: `ea6c3f2be053d9daff5c121187882c3f70cdd065`
- Current execution commit: `fce9f40191fc00cb1fbd49b20ffefdade4cd7704`
- Compatible fixed-arm checkpoint reused: `ea6c3f2be053d9daff5c121187882c3f70cdd065`
- Fixed arm commit: `ea6c3f2be053d9daff5c121187882c3f70cdd065`
- Causal arm commit: `fce9f40191fc00cb1fbd49b20ffefdade4cd7704`
- Fixed no-rerouting baseline included: `1`
- Validation claim allowed: `0`

Structural preflight: static strong fraction `0.431`; always-replan changes `5`; causal changes `2`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Messages | Strong fraction | Routes |
|:--|--:|--:|--:|--:|--:|--:|:--:|--:|--:|
| Fixed formation tree | 123.211 | 121.650 | 8.906 | 8.528 | 138.025 | 35469296 | 46--48 | 0.431 | 13 |
| Always replan | — | — | — | — | — | — | — | — | — |
| Causal minimal edit | 121.074 | 119.717 | 8.794 | 8.314 | 131.664 | 39171480 | 48--48 | 1.000 | 22 |

## Causal over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+1.734%` |
| Focus E-OSPA | `+1.589%` |
| Full RMSE | `+1.264%` |
| Focus RMSE | `+2.503%` |
| Focus consistency | `+4.609%` |
| Attempted-byte saving | `-10.438%` |
| Weakest formation E-OSPA | `-0.931%` |
| Weakest formation RMSE | `-2.620%` |

- Mean direction passed: `1`
- Nonnegative formation tail passed: `0`

## Evidence boundary

V241 is an opened-seed development comparison on exploratory formation-braid scenes. The fixed-tree arm never changes its initial formation pairs, but reassigns sensor gateways from the current physical graph while those pairs remain feasible. Only after a fixed formation pair becomes infeasible are unavailable route inputs omitted and their KLA mass returned to the receiver self weight. The always-replan and causal arms retain exactly two current physical inputs per receiver and the same V227 weight multiset. V241 can establish a paired development effect on the executed scene and seed only; it does not establish held-out generalization or a paper claim. A completed fixed-tree arm from the registered source commit may be reused only for a causal-only resume when the git diff contains the registered runtime-context compatibility files and no method, fusion, scenario, measurement, or metric change.
