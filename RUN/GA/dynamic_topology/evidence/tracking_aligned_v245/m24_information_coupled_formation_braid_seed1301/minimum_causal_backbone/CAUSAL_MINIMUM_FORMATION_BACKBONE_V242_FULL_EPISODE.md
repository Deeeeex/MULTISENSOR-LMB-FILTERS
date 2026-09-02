# V242 causal minimum formation backbone full episode

- Scene / seed: `m24-formation-fov-coupled-formation-braid / 1301`
- Source commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- V241 reference commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- V241 mechanism gate passed: `1`
- Goal direction passed: `0`
- Paper threshold passed: `0`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages | Strong fraction |
|:--|--:|--:|--:|--:|:--:|--:|
| V241 fixed formation tree | 126.724 | 9.052 | 135.624 | 34363568 | 46--48 | 0.431 |
| V241 causal 2N | 125.394 | 8.947 | 133.109 | 38421672 | 48--48 | 1.000 |
| V242 causal minimum backbone | 125.991 | 9.579 | 132.815 | 26465520 | 30--30 | 1.000 |

## V242 over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.578%` |
| Full RMSE | `-5.826%` |
| Focus consistency | `+2.071%` |
| Attempted-byte saving | `+22.984%` |
| Weakest formation E-OSPA | `-0.191%` |
| Weakest formation RMSE | `-9.886%` |

## V242 over V241 causal 2N

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `-0.476%` |
| Full RMSE | `-7.069%` |
| Focus consistency | `+0.221%` |
| Attempted-byte saving | `+31.118%` |
| Weakest formation E-OSPA | `-1.886%` |
| Weakest formation RMSE | `-12.626%` |

## Evidence boundary

V242 is a structural development policy on the opened formation-braid seed. It inherits the causal V240 formation-tree repair, keeps one current physical directed cycle inside each formation, and keeps only the two directed cross-formation gateway messages associated with every undirected formation-tree edge. Omitted local residual mass returns to receiver self weight. Its N+2(F-1) message count is minimal only under this local-cycle plus bidirectional-tree architecture; it is not a global directed-graph lower bound. Structural results do not establish tracking gain, held-out generalization or a paper claim.
