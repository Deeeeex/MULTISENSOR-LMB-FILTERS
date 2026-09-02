# V242 causal minimum formation backbone full episode

- Scene / seed: `m24-formation-fov-formation-braid / 1301`
- Source commit: `36b4efb803876c890eb8c8674c8a4e0d839995b5`
- V241 reference commit: `ea6c3f2be053d9daff5c121187882c3f70cdd065`
- V241 mechanism gate passed: `1`
- Goal direction passed: `1`
- Paper threshold passed: `0`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages | Strong fraction |
|:--|--:|--:|--:|--:|:--:|--:|
| V241 fixed formation tree | 123.211 | 8.906 | 138.025 | 35469296 | 46--48 | 0.431 |
| V241 causal 2N | 121.074 | 8.794 | 131.664 | 39171480 | 48--48 | 1.000 |
| V242 causal minimum backbone | 122.621 | 8.237 | 134.841 | 30371232 | 30--30 | 1.000 |

## V242 over fixed formation tree

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+0.479%` |
| Full RMSE | `+7.513%` |
| Focus consistency | `+2.307%` |
| Attempted-byte saving | `+14.373%` |
| Weakest formation E-OSPA | `-2.036%` |
| Weakest formation RMSE | `+2.866%` |

## V242 over V241 causal 2N

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `-1.277%` |
| Full RMSE | `+6.329%` |
| Focus consistency | `-2.413%` |
| Attempted-byte saving | `+22.466%` |
| Weakest formation E-OSPA | `-3.893%` |
| Weakest formation RMSE | `-3.850%` |

## Evidence boundary

V242 is a structural development policy on the opened formation-braid seed. It inherits the causal V240 formation-tree repair, keeps one current physical directed cycle inside each formation, and keeps only the two directed cross-formation gateway messages associated with every undirected formation-tree edge. Omitted local residual mass returns to receiver self weight. Its N+2(F-1) message count is minimal only under this local-cycle plus bidirectional-tree architecture; it is not a global directed-graph lower bound. Structural results do not establish tracking gain, held-out generalization or a paper claim.
