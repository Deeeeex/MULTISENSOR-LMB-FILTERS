# V275 KLA consensus-matrix balance diagnostic

- Generation commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Structural seed: `1301`
- Numerical tolerance: `1.0e-12`
- Diagnostic complete: `1`
- Standard unweighted-KLA condition established: `0`

The standard consensus result for an unweighted collective KLA requires more than instantaneous strong connectivity: the consensus matrix must also satisfy the relevant stochasticity conditions. This report checks row and column sums of the matrices actually emitted by V240 and V242.

| Scene | Arm | Messages | Strong / physical | Max row dev | Max / mean column dev | Non-double pages |
|:--|:--|:--:|:--:|--:|:--|:--|
| m24-formation-fov-temporal-coupled-formation-braid | V240 full causal | 48--48 | 1 / 1 | 0 | 0.1 / 0.05313 | 160/160 `all` |
| m24-formation-fov-temporal-coupled-formation-braid | V242 minimum backbone | 30--30 | 1 / 1 | 0 | 0.05 / 0.00531 | 17/160 `[53 54 69 137 138 139 140 141 142 143 144 145 146 147 148 149 150]` |
| x36-formation-fov-temporal-coupled-formation-braid | V240 full causal | 72--72 | 1 / 1 | 0 | 0.1 / 0.05188 | 160/160 `all` |
| x36-formation-fov-temporal-coupled-formation-braid | V242 minimum backbone | 46--46 | 1 / 1 | 0 | 0.05 / 0.00688 | 22/160 `[53 54 87 88 93 94 95 96 97 98 99 100 101 102 103 104 105 106 151 152 153 154]` |

## Symmetry of the available-link process

| Scene | Physical adjacency max asymmetry | Drop schedule max asymmetry |
|:--|--:|--:|
| m24-formation-fov-temporal-coupled-formation-braid | 0 | 0 |
| x36-formation-fov-temporal-coupled-formation-braid | 0 | 0 |

## Interpretation

V240 is row stochastic but is not doubly stochastic on any of the 160 pages in either scale. V242 removes most of that imbalance, but its independently directed cross-formation gateways still leave intermittent non-double pages. Therefore current V242 supports claims about physical feasibility, instantaneous strong connectivity and message count, but not a direct appeal to the standard unbiased collective-KLA convergence theorem.

Because the available physical graph and drop-probability schedule are symmetric, a same-pair reciprocal gateway is a plausible next construction. Coupling both directions of every formation-tree edge, while assigning distinct incident edges to sensors within each formation, preserves N+2(F-1) messages, the one-or-two-input receiver bound and equal incoming/outgoing residual weight. That construction still needs a feasibility algorithm and paired tracking evidence; it is not implemented by this diagnostic.

## Evidence boundary

V275 replays only the physical graph, link-drop schedule and causal routing policies. It reads no posterior, measurement, truth, realized delivery or tracking outcome. Row/column sums diagnose the consensus matrix used by the current implementation; they do not establish finite-round tracking benefit or convergence by themselves.
