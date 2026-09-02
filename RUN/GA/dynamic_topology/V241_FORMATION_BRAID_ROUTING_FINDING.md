# V241 formation-braid routing finding

## Paired M24 result

V241 compares a fixed initial formation tree with a causal policy that keeps
the incumbent tree while it remains feasible and repairs it only at physical
failure.  Both arms use the same M24 formation-braid scene, seed 1301, local
filter, two-input fusion-weight multiset, measurements, and 160-step horizon.

| Metric | Fixed formation tree | Causal minimal repair | Relative gain |
|:--|--:|--:|--:|
| Full E-OSPA | 123.211 | 121.074 | +1.734% |
| Focus E-OSPA | 121.650 | 119.717 | +1.589% |
| Full position RMSE | 8.906 | 8.794 | +1.264% |
| Focus position RMSE | 8.528 | 8.314 | +2.503% |
| Focus inter-formation disagreement | 138.025 | 131.664 | +4.609% |
| Terminal inter-formation disagreement | reference | candidate | +19.448% |
| Attempted posterior bytes | 35,469,296 | 39,171,480 | -10.438% |
| Instantaneous strong-connectivity fraction | 0.431 | 1.000 | -- |

The fixed route drops 182 directed messages beginning at `t=70`.  The causal
route changes its formation tree only at `t=[70,151]`, keeps 48 directed
messages at every step, and remains physically feasible and strongly
connected throughout.

## Causal attribution

The two trajectories are identical before the first tree repair.  After the
first repair (`t=70..150`), E-OSPA, RMSE, and inter-formation disagreement
improve by 2.350%, 3.794%, and 6.346%.  In the final interval (`t=151..160`),
E-OSPA and disagreement improve by 8.939% and 17.801%, while RMSE degrades by
10.875%.  This temporal alignment supports a real information-flow mechanism
rather than an initial-route confound.

The mean-direction gate passes, but the local tail does not.  Per-formation
E-OSPA gains are `[+4.368,+2.652,-0.931,+0.537]%`; per-formation RMSE gains
are `[+6.466,-2.620,+0.325,-0.042]%`.  The worst-sensor E-OSPA and RMSE still
improve by 0.415% and 1.559%, respectively.

## Decision

V241 establishes development-level value for causal topology repair, but it
does not yet meet the joint paper objective: the average gains are modest,
two formation-level RMSE tails regress, and restoring all dropped messages
adds 10.438% attempted bytes.

The registered next step is V242.  It retains the same causal formation-tree
repair but replaces the two-input-per-receiver route with one local directed
cycle per formation plus one bidirectional gateway for every formation-tree
edge.  This reduces the structural message count from 48 to 30 in M24 while
preserving instantaneous strong connectivity.  V242 tests whether removing
redundant residual inputs can retain the V241 estimation/consensus gain,
eliminate the communication overhead, and reduce the final-interval RMSE
failure.

Separately, V244 preserves the method and sensor geometry but couples target
handoffs to every initial formation-tree cut.  This addresses a scene confound
in the original braid, where both target handoffs and platform overtakes are
partitioned into independent pairs and a failed inter-module bridge may carry
little task-relevant information.  V244 is a scene control, not a replacement
for the V241 result.

## Evidence boundary

This is one opened M24 development seed on an exploratory scene.  It supports
the V242/V244 method decision only.  It is not an X36 result, a held-out
generalization result, or a paper-level performance claim.
