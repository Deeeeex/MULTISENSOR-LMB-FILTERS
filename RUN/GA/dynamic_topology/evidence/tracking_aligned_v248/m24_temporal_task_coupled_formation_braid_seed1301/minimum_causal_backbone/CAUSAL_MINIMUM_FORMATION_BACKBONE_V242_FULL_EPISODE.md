# V242 causal minimum formation backbone full episode

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- V241 reference commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- V241 mechanism gate passed: `1`
- Goal direction passed: `1`
- Paper threshold passed: `0`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages | Strong fraction |
|:--|--:|--:|--:|--:|:--:|--:|
| V241 fixed formation tree | 125.478 | 22.640 | 133.599 | 40769168 | 46--48 | 0.431 |
| V241 causal 2N | 122.380 | 14.081 | 131.913 | 44867136 | 48--48 | 1.000 |
| V242 causal minimum backbone | 122.462 | 12.183 | 131.664 | 36675624 | 30--30 | 1.000 |

## V242 over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+2.403%` |
| Full RMSE | `+46.190%` |
| Focus consistency | `+1.449%` |
| Attempted-byte saving | `+10.041%` |
| Weakest formation E-OSPA | `+0.272%` |
| Weakest formation RMSE | `-24.085%` |

## V242 over V241 causal 2N

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `-0.067%` |
| Full RMSE | `+13.482%` |
| Focus consistency | `+0.189%` |
| Attempted-byte saving | `+18.257%` |
| Weakest formation E-OSPA | `-1.906%` |
| Weakest formation RMSE | `-17.003%` |

## Evidence boundary

V242 is a structural development policy on the opened formation-braid seed. It inherits the causal V240 formation-tree repair, keeps one current physical directed cycle inside each formation, and keeps only the two directed cross-formation gateway messages associated with every undirected formation-tree edge. Omitted local residual mass returns to receiver self weight. Its N+2(F-1) message count is minimal only under this local-cycle plus bidirectional-tree architecture; it is not a global directed-graph lower bound. Structural results do not establish tracking gain, held-out generalization or a paper claim.
