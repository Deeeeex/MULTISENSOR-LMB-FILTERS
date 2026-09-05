# V242 causal minimum formation backbone full episode

- Scene / seed: `x36-formation-fov-temporal-coupled-formation-braid / 1301`
- Source commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- V241 reference commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- V241 mechanism gate passed: `1`
- Goal direction passed: `0`
- Paper threshold passed: `0`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages | Strong fraction |
|:--|--:|--:|--:|--:|:--:|--:|
| V241 fixed formation tree | 132.680 | 36.925 | 139.407 | 76871008 | 66--72 | 0.338 |
| V241 causal 2N | 131.795 | 19.586 | 139.004 | 81258696 | 72--72 | 1.000 |
| V242 causal minimum backbone | 132.192 | 19.329 | 140.489 | 60090416 | 46--46 | 1.000 |

## V242 over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.368%` |
| Full RMSE | `+47.655%` |
| Focus consistency | `-0.776%` |
| Attempted-byte saving | `+21.830%` |
| Weakest formation E-OSPA | `-0.818%` |
| Weakest formation RMSE | `+4.444%` |

## V242 over V241 causal 2N

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `-0.301%` |
| Full RMSE | `+1.313%` |
| Focus consistency | `-1.068%` |
| Attempted-byte saving | `+26.050%` |
| Weakest formation E-OSPA | `-2.246%` |
| Weakest formation RMSE | `-27.753%` |

## Evidence boundary

V242 is a structural development policy on the opened formation-braid seed. It inherits the causal V240 formation-tree repair, keeps one current physical directed cycle inside each formation, and keeps only the two directed cross-formation gateway messages associated with every undirected formation-tree edge. Omitted local residual mass returns to receiver self weight. Its N+2(F-1) message count is minimal only under this local-cycle plus bidirectional-tree architecture; it is not a global directed-graph lower bound. Structural results do not establish tracking gain, held-out generalization or a paper claim.
