# V153 X36 closure: graph diversity does not transfer by scale

## Decision

V153 is closed before multi-seed expansion or GNN training.  On the opened
`x36-formation-fov` seed-83 continuation, none of the six complete safe graph
generators survives the registered tracking--tail--consensus--byte gate.  The
best mean candidate, rank 5, improves mean E-OSPA by only `0.088%`; it worsens
the worst sensor by `1.169%`, the weakest formation by `4.372%`, consensus by
`1.290%`, and attempted bytes by `1.300%`.  This is far below the frozen `5%`
mean-gain threshold.

The M24 bright spot does not transfer.  Rank 4 improves the opened M24 mean by
`13.013%`, but the same rank worsens the X36 mean by `1.546%` and also regresses
every registered tail criterion.  One positive M24 development seed therefore
cannot support a cross-scale method claim.

## Frozen X36 result

- Preset / seed / analysis window: `x36-formation-fov / 83 / 60:67`.
- Better static reference: clockwise spliced residual cycle.
- Exact directed transmission opportunities per page: `60` for every arm.
- Full mixture-aware LMB posterior on every selected edge.
- Every dynamic candidate passes physicality, row-stochastic-weight and
  rolling-B3 structure checks.

| Rank | Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Byte saving | Retained |
|--:|--:|--:|--:|--:|--:|:--:|
| 1 | -2.319% | -1.742% | -5.643% | -3.528% | -1.649% | no |
| 2 | -1.767% | -1.742% | -6.908% | -2.992% | -1.874% | no |
| 3 | -1.837% | -1.150% | -4.093% | -3.135% | -0.942% | no |
| 4 | -1.546% | -1.742% | -4.233% | -2.568% | -1.239% | no |
| 5 | +0.088% | -1.169% | -4.372% | -1.290% | -1.300% | no |
| 6 | -2.350% | -0.020% | -6.764% | -3.024% | -1.268% | no |

Negative byte saving means that the candidate sends more serialized bytes
than the paired static reference despite using the same number of transmission
opportunities.

## Scene diagnosis

The failure is not explained by malformed X36 sensing geometry.  A fresh
geometry-only check under seed 83 passes the registered scene-validity gates.
The scale-matched focus load is almost identical:

| Metric | M24 | X36 |
|:--|--:|--:|
| Mean visible targets per sensor-time | 13.415 | 13.363 |
| Mean visible sensors per active target | 20.123 | 20.045 |
| Focus blackout fraction | 0.000 | 0.000 |
| Multi-formation visibility fraction | 0.773 | 0.785 |

Both scenes use the same 120-degree, 300-m sensor hardware, detection
probability, measurement noise and per-sensor clutter rate.  X36 also clears
its handover and blackout requirements.  Its higher absolute tracking error
can reflect the larger recursive fusion problem, but it is not caused by a
lower registered per-sensor observation load.

## Mechanism diagnosis

V152/V153 did not create six task-level graph macro-actions.  Every candidate
keeps the same clockwise or counter-clockwise formation ring and differs only
in the sensor gateways selected by an M24-pinned edge score plus cumulative
edge-reuse penalties.  The reuse penalty guarantees different graphs; it does
not guarantee different or useful finite-horizon tracking effects.

Moreover, a rank is regenerated from the current posterior on every page.
The selected M24 rank-4 and X36 rank-5 trajectories each use eight distinct
gateway maps in eight pages.  In X36, rank 5 is mildly positive in several
middle pages but becomes worse than static at the end of the window.  The
evidence therefore rejects two assumptions:

1. edge-set diversity is a sufficient proxy for task-value diversity; and
2. a memoryless page-wise graph selector scales without representing the
   information-flow path it has already established.

This does not reject complete safe dynamic graphs in general.  It closes the
registered M24-score-plus-reuse codebook and the V153 evaluation contract.

## Successor boundary

The next bounded test changes the temporal decision object, not the payload,
KLA weights or graph-safety rules.  A candidate complete graph becomes an
option with a minimum physical dwell: it is selected once, retained while it
remains physically executable, and otherwise falls back through the exact
projector.  The first X36 screen compares these coherent graph options with
the already paired static reference under the unchanged eight-page budget.

This differs from the closed V49 cycle objective and V56--V59 local actions:
it does not optimize graph contraction, omit a posterior, or alter one
formation independently.  It asks whether the complete gateway assignment
has task headroom when its induced information path is allowed to persist.
Learning remains forbidden until the complete option bank clears the same
cross-scale `5%` and no-regression gate.

## Reporting boundary

This is opened-development negative evidence.  It remains in the repository
experiment record and is not a paper result or a main-progress-document
number.  No held-out seed, richer scene, GNN, or generalization claim is
authorized by this closure.
